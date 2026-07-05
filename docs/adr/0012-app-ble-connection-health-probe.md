# ADR-0012: App BLE connection health — probe the receiver before declaring a phantom link

- **Status:** Accepted
- **Date:** 2026-07-05
- **Deciders:** Frank Schroer
- **Related issues:** #16 (radio-layer CRC discard removed — the paired firmware change)

## Context

The app connects to the receiver over BLE (GATT) and consumes the LoRa traffic the receiver relays. Android's BLE stack can report a GATT link as `Connected`/`Ready` while the link is in fact dead — an OS-cached "phantom" connection that never delivers a byte. To catch that, `BluetoothConnectionManager` runs a **health watchdog** armed when the connection reaches `Ready`.

The original watchdog was a **one-shot, tear-down-on-silence** timer: `DATA_TIMEOUT_MS` (10 s) after `Ready`, if no GATT notification had arrived, it declared the link phantom and forced a reconnect (`onAclDisconnected(source = "health watchdog")`). Reconnect re-armed the same watchdog on the next `Ready`.

The flaw surfaced on the bench: **GATT silence is not the same as a dead link.** A perfectly healthy receiver has nothing to relay whenever the locator is not transmitting — locator powered off, sitting on the pad between sessions, or out of LoRa range. In every such case the watchdog fired, tore down a good connection, reconnected, reached `Ready`, saw 10 s more silence, and tore down again — an endless disconnect/reconnect loop reported by the user as "when the app isn't receiving messages from a locator, it repeatedly disconnects and reconnects the receiver."

The distinguishing fact the watchdog ignored: **the receiver answers `ReceiverInfoRequest` on its own behalf** (receiver-only message IDs 15–16), independent of any locator activity. A live receiver replies; a phantom link does not. Passive silence conflates the two; an active probe separates them.

This is the app-side counterpart to the locator radio-layer CRC change (#16): the locator now admits CRC-mismatched frames at the radio layer, relying on the app's per-frame CRC-16 as the sole integrity gate, and the app must in turn keep its BLE link to the receiver alive through long stretches of legitimate locator silence.

## Decision

We replace the tear-down-on-silence watchdog with an **active liveness probe**, under these invariants:

1. **Silence triggers a probe, not a disconnect.** The watchdog runs as a repeating loop. Each `DATA_TIMEOUT_MS` (10 s) window with **no** inbound GATT data invokes an `onHealthProbe` callback, wired in `BluetoothService` to `requestReceiverInfo()` (a `ReceiverInfoRequest`). Any inbound byte — relayed locator data *or* a probe response — resets the miss counter, because `onDataReceived` calls `recordDataReceived()` for all inbound traffic.

2. **Only repeated unanswered probes mean phantom.** The link is declared phantom and reconnected only after `MAX_MISSED_HEALTH_PROBES` (3) **consecutive** silent windows in which the probe drew no response (~30 s). A receiver that answers any probe keeps the connection indefinitely, regardless of locator activity.

3. **When the locator is transmitting, no probe is sent at all.** Active telemetry/PreLaunchData fills every window with data, the miss counter never leaves zero, and the watchdog adds no BLE chatter during real use. Probes exist only to keep an *idle* link warm.

## Consequences

- A connected-but-idle receiver stays connected through arbitrary locator silence; the disconnect/reconnect loop is gone. Genuine phantom links are still recycled, within ~30 s instead of ~10 s.
- **Invariant 1 is load-bearing — do not revert to declaring a phantom on GATT silence alone.** That was the original bug. Silence must trigger a probe first; a disconnect is warranted only when the receiver itself stops answering.
- **Cross-component dependency:** the design assumes the receiver answers `ReceiverInfoRequest` while idle (FR-R2). If a future receiver change gates receiver-info replies behind locator activity, the probe stops distinguishing phantom from idle and this watchdog breaks. Keep receiver-info replies unconditional.
- Idle connections cost one probe round-trip per ~10 s window (observed on the bench as app⇄receiver traffic roughly every ~20 s: probe, reply, then the reply counts as data for the next window). To trade warmth for less chatter, raise `DATA_TIMEOUT_MS`; to trade phantom-detection latency for fewer false teardowns, raise `MAX_MISSED_HEALTH_PROBES`.
- **No wire-format change.** `ReceiverInfoRequest`/receiver-info response already exist; the change is confined to the app (`BluetoothConnectionManager` watchdog loop + `onHealthProbe` callback, `BluetoothService` wiring).
- **Revisit if:** receiver-info replies become conditional, a lower-level BLE keepalive/supervision-timeout signal becomes available and sufficient, or the probe cadence proves too costly for battery/airtime.

## Alternatives considered

- **Keep the one-shot tear-down but lengthen `DATA_TIMEOUT_MS`.** Only widens the silence window before the loop resumes; any locator silence longer than the timeout still tears down a good link. Doesn't address the root confusion of silence with death.
- **Trust `Ready` and drop the watchdog entirely.** Loses phantom-connection detection — the reason the watchdog exists. A cached link would sit dead forever with no recovery.
- **Rely on Android's ACL-disconnect broadcast / GATT supervision timeout alone.** Real phantom links are exactly the case where the OS does *not* raise a timely disconnect; the app needs its own liveness signal.
- **Probe with a locator-bound message (e.g. version or a locator config read).** Those depend on the locator being powered and in range — the very thing that may be absent. Only a receiver-answered message (`ReceiverInfoRequest`) proves BLE liveness without a live locator.
