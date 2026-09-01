# ADR-0033: Outbound BLE writes are serialized by the app — a busy transport is not a device failure

- **Status:** Accepted
- **Date:** 2026-09-01
- **Deciders:** Frank Schroer
- **Related issues:** none filed — reported from the phone 2026-09-01 ("sometimes a channel search says the locator is not available, and searching again works")

## Context

Android's BLE stack permits **exactly one outstanding GATT operation per connection**. A `writeCharacteristic` issued while another write is still in flight is not queued by the stack — it is refused on the spot, returning `ERROR_GATT_WRITE_REQUEST_BUSY` on API 33+ and `false` below it.

`BluetoothConnectionManager.sendData` passed that refusal straight to the caller, and `onCharacteristicWrite` only logged. Nothing serialized writes, nothing retried, and the busy return was not even logged — so a refused write was indistinguishable, from every layer above, from a device that had not answered.

The app has a background writer that runs *only* in the state the affected features exist for. Under [ADR-0019](0019-channel-interference-detection.md) the app polls `ReceiverInfoRequest` every 2 s once the locator has been unheard for 5 s; [ADR-0012](0012-app-ble-connection-health-probe.md)'s health probe writes the same message on its own 10 s cadence. Both are running precisely when the operator reaches for **Find a locator** ([ADR-0029](0029-locator-search-candidate-channels.md)) — which is a feature for a locator you cannot hear.

The result reported from the phone: pressing **Search N channels** while one of those polls had a write outstanding produced *"No response from the receiver. If its firmware predates locator search, update it"* **instantly**, with nothing ever transmitted. Pressing it again a moment later worked. The message sent the operator to reflash a receiver that was healthy, current, and had never been asked anything.

Two things made this hard to see rather than merely intermittent:

- **The failure is silent at the layer that causes it.** No log line, no wire traffic, no receiver-side trace — the receiver's `[search]` console shows nothing at all, because no request reached it.
- **The symptom names the wrong device.** Every string on this path blames the receiver or its firmware, so the evidence points away from the phone.

The same defect reached every other send path. `changeLocatorArmedState` reported `SendFailure` for an **Arm** that was never attempted; `requestChannelSurvey` had the identical instant-failure path as the search. And fragmentation was broken outright: `sendData` wrote chunks back to back, so every chunk after the first hit the one-operation limit and was refused — a message longer than `negotiatedMtu - 3` could not have arrived intact.

## Decision

We will **serialize all outbound GATT writes in the app**, and treat a busy transport as a condition the transport absorbs rather than one the operator is told about.

1. **One queue, drained one write at a time.** `sendData` enqueues; a pump issues the head only when nothing is outstanding; `onCharacteristicWrite` — previously log-only — is what advances the queue. Fragments of one message are enqueued together and go out in order, each waiting for the previous one's acknowledgment.

2. **A refused write is re-offered, not reported.** Backoff of one connection interval (20 ms), up to 5 attempts, before the write is dropped and logged. A caller cannot act on "the stack was momentarily busy", so it is not told about it.

3. **`sendData` returns *accepted for transmission*, not *transmitted*.** The two cannot be the same thing on a link that carries one operation at a time. `false` now means only: there is no connection, or the queue is full because the link has stopped draining. Every flow that needs to know a message actually landed already confirms it by reading something back — [ADR-0011](0011-locator-lora-channel-from-app.md)'s recognition cycle, the search's own terminator, the survey's response.

4. **A lost write acknowledgment must not wedge the queue.** Android BLE stacks do drop `onCharacteristicWrite`. A 2 s per-write watchdog retires the head and continues, because a queue that stops forever on one lost callback is strictly worse than the unserialized writes this replaces.

5. **The queue is discarded when the link goes away.** A queued write belongs to the connection it was made on. Carrying it across a reconnect is the late-delivery hazard [ADR-0011](0011-locator-lora-channel-from-app.md) documents — an undelivered request firing minutes later, out of the flow that queued it, against whatever the receiver has since been pointed at.

## Consequences

- **The instant false failure is gone**, and with it the false firmware advice. Every string on these paths is now reachable only in the state it describes — see the string audit below.
- **`search_failed` and `survey_failed` now mean what they say.** Both are reachable only via their own silence timeouts (8 s and the survey's) or a genuinely absent link. The receiver-firmware advice they carry is once again literal, which is what `UserManual.md` §3.3 has always claimed it was.
- **Fragmented messages can now arrive.** Nothing routine exceeds `negotiatedMtu - 3` (244 bytes at the negotiated MTU of 247), so this fixed a latent defect rather than an observed one.
- **A new background writer is now safe to add.** Before this, any added poll raised the collision rate on every operator action; that coupling is gone. This is the property most worth not silently contradicting.
- **Decision 3 is load-bearing for callers.** Anything that reads `sendData`'s result as proof of delivery is wrong and was always wrong — the queue only makes it obvious. Confirmation comes from a read-back, never from the send.
- **Not conclusively confirmed.** Initial testing on the phone 2026-09-01 looked good, but the defect is sporadic by nature and the operator is still observing. Absence of the symptom over one session is weak evidence; the strong evidence would be a logcat capture showing the busy return being absorbed.
- **Revisit if:** the queue depth ceiling (32) is ever reached in normal use, which would mean a writer is producing faster than the link drains and the right fix is upstream; or if Android ever exposes a stack-level write queue that makes this redundant.

### iOS is exposed to the same hazard by a different mechanism

CoreBluetooth does not return a refusal — `writeValue(_:for:type:)` returns `Void` — so this defect cannot present on iOS the way it did on Android. It is **not** therefore absent. `BluetoothTransport.send` writes with `.withoutResponse`, and a `.withoutResponse` write issued while `peripheral.canSendWriteWithoutResponse` is `false` is **silently discarded by the framework** — no error, no callback, no return value, and no boolean the app could even have noticed. That is the same defect with the diagnostic surface removed.

**Ported 2026-09-01.** `OutboundWriteQueue` holds the portable half — fragmentation, the all-or-nothing admission rule, the ceiling, FIFO — as a pure type beside `ConnectionHealthMonitor`, with 12 tests. `BluetoothTransport` drains it while `canSendWriteWithoutResponse` allows, resumes on `peripheralIsReady(toSendWriteWithoutResponse:)`, and discards the queue on disconnect and on a receiver switch.

Two claims in this section were written from reading the iOS source and **implementing it corrected both**:

- **Decision 4 does port, for a different hazard.** "No acknowledgment to lose, so nothing for a watchdog to guard" was the wrong reading. The stall it must guard is not a lost completion but a `canSendWriteWithoutResponse` that never comes back — reported `false` on a freshly connected peripheral until something is written to it, which is a deadlock by construction: nothing is written because the flag is false, and the flag never clears because nothing is written. A stalled queue therefore writes its head anyway after 250 ms, best-effort. On a rocket-recovery app the failure mode being guarded against is "no message ever reaches the receiver", which is worse than any dropped write.
- **Decision 2's retry has no counterpart and needs none.** Android retries because the stack hands back a refusal; iOS has no refusal to retry, only a flag to wait on, and `peripheralIsReady` is the wait. What iOS owes instead is *visibility*, which Android got free from logcat.

That visibility is the third thing implementing it surfaced. **This app has no logging at all** — no `os_log`, no `print` — by convention: diagnostics reach a screen instead. A silently discarded write would therefore have been as invisible after this change as before it, which would have left the ADR's central complaint unaddressed on the platform that suffers from it worst. `BluetoothTransport.droppedWriteChunks` counts every chunk not cleanly delivered — refused for a full queue, discarded with a link, or forced past a stalled ready-callback — and the diagnostics screen shows it as **writes lost**, the outbound counterpart of its existing **bad CRC**.

Decision 3 was already true there: `send` returned a link-state check and nothing more.

## Alternatives considered

- **Retry at each call site.** Puts transport knowledge in every feature, and each site would have to invent its own backoff. It also cannot fix fragmentation, which is inside `sendData`.
- **Report the busy return honestly with a different string** ("the phone's Bluetooth is busy, try again"). Truthful, and still asks the operator to do the retry the transport can do for them — on a screen reached because something is already wrong.
- **Write with `WRITE_TYPE_NO_RESPONSE` to avoid the acknowledgment round trip.** Removes the very callback the queue is paced by, and trades a visible refusal for a silent drop — which is precisely the iOS failure mode described above.
- **Serialize by funnelling every send through one coroutine on a single-threaded dispatcher.** Equivalent for ordering, but it would have to block that thread on the write callback to preserve the one-operation rule, and it offers nowhere to put the lost-callback watchdog that decision 4 requires.
