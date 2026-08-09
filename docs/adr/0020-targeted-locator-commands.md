# ADR-0020: Address app→locator commands to a locator, so a broadcast command cannot arm somebody else's rocket

- **Status:** Accepted
- **Date:** 2026-08-05
- **Deciders:** fschroer
- **Related issues:** [#34](https://github.com/fschroer/steam-pigeon-locator/issues/34)
- **Relates to:** [ADR-0006](0006-locator-connect-password.md) (the recognition gate this completes), [ADR-0011](0011-locator-lora-channel-from-app.md) (the channel-change path that exposed it)

## Context

A bench test with two locators powered on channel 0 — one connected and password-known, one unknown — sent a channel change to the connected locator. **Both moved.**

The mechanism is that **no app→locator command carries a target.** The receiver forwards each command over LoRa, which is a broadcast medium, and every locator on the channel in an accepting state applies it. There is no identity check anywhere in the locator's command handling:

| Command | Accepted when | Effect on a bystander locator |
|---|---|---|
| `LocatorCfgChgRequest` | Disarmed | **entire `RocketPersistentSettings` overwritten** |
| `ArmRequest` | Disarmed | **it arms** |
| `DisarmRequest` | WaitingLaunch / Landed | it disarms |
| `DeploymentTestRequest` | Armed | it runs a deployment test |
| `FlightMetadataRequest` / `FlightDataRequest` / `FlightDataAck` / `VersionRequest` | various | it responds, colliding with the intended locator |

Two things make this more serious than the incident that surfaced it.

**The config payload is the whole settings struct.** `LocatorCfgChgRequest` carries `RocketPersistentSettings`, so the bystander received the target's deployment channel modes, drogue delays, main deploy altitudes, deploy signal duration, launch detect altitude, channel *and* device name. That is pyro configuration silently rewritten on a rocket the user never connected to.

**`ArmRequest` is in the same class.** At a launch with several fliers sharing a channel, pressing Arm arms every disarmed locator on that channel. This is the consequence that motivates the ADR; the channel change is merely how it was noticed.

### This is the gap in ADR-0006, not a new problem

[ADR-0006](0006-locator-connect-password.md) states its purpose as "preventing accidental cross-connection among many users at a launch." Its gate governs which locator's *telemetry the app acts on*, and Decision 5 says plainly that "the locator keeps accepting well-formed commands." So the gate never touched the question of which locators *receive* a command. The advertised goal and the delivered behavior have differed since that ADR was written; the survey work in [ADR-0019](0019-channel-interference-detection.md) simply made it trivial to trigger, because "move to a clean channel" is the first command anyone sends with two locators powered.

The soft/hard-gate distinction in ADR-0006 is also the wrong axis for this. That framing is about resisting a *modified app*. This is an *honest* app, doing exactly what its user asked, reaching rockets it never intended to address. No amount of app-side gating fixes it, because the fault is that the wire format has no addressee.

## Decision

**1. Every app→locator command carries `uint32_t target_locator_id`, immediately after the `PacketHeader`.** The value is the MCU UID the app already uses for recognition and already receives in every broadcast ([ADR-0006](0006-locator-connect-password.md) Decision 1), so no new identity concept is introduced and the app needs no new state.

Eight commands are addressed: `LocatorCfgChgRequest`, `ArmRequest`, `DisarmRequest`, `FlightMetadataRequest`, `FlightDataRequest`, `FlightDataAck`, `DeploymentTestRequest`, `VersionRequest`.

**2. The locator ignores any command not addressed to its own UID, and `0` matches nothing.** A locator whose UID does not match discards the frame before any state change. Zero is not a wildcard: on a path that includes Arm, the failure direction must be *do nothing*. This makes an old app unable to arm or configure a new locator at all, which is the correct trade — a command that silently does nothing is recoverable, one that arms the wrong rocket is not.

**The check is unconditional, not driven by a list of "addressed" message types.** Every LoRa frame reaching the locator's handler *is* an app command: locator→app messages never come back to it, and the receiver's startup string fails CRC long before. An allowlist would be correct today and is how this was first written, but it must be hand-updated for every new command, and forgetting would silently restore the broadcast behavior this ADR exists to remove. Requiring an address from everything means a new command is protected by default, and the cost of forgetting anything is that it does nothing rather than that it reaches every rocket on the channel. This project has been bitten repeatedly by hand-synchronized lists drifting; one fewer is worth the trade.

The receiver still sizes each command individually, but that is a per-type length, not a policy list — a wrong length there makes a command fail to parse, which is visible immediately, rather than making it silently unaddressed.

**3. Receiver-directed messages are unaffected.** `ReceiverCfgChgRequest`, `ReceiverInfoRequest` and `ChannelSurveyRequest` travel point-to-point over BLE to the one receiver the app is connected to and are never forwarded, so they have no addressing problem to solve.

**4. The check lives on the locator, not the receiver.** The receiver is a password-agnostic relay ([ADR-0006](0006-locator-connect-password.md) context 1) and must not need to know which locator a command is for. It only length-validates and forwards. Filtering at the receiver would also be worthless: the hazard is other people's receivers relaying their own users' commands onto a shared channel.

**5. The app fills the field at the send choke point,** from the connection established in [ADR-0019](0019-channel-interference-detection.md)'s single-holder rule. `BluetoothService` already gates locator-directed sends on being connected; that flag becomes the id itself, so the thing that authorizes the send is the same thing that addresses it. They cannot drift apart.

## Consequences

- **Breaking wire change across all three components.** Locator, receiver and app must be updated together. Unlike previous breaks this one fails *safe*: a mismatched pair does nothing rather than doing the wrong thing.
- Command frames grow 4 bytes each. Irrelevant — they are ground-range, disarmed-state, and not periodic.
- **This closes the gap between ADR-0006's stated purpose and its behavior.** ADR-0006 remains correct about telemetry recognition; this ADR supplies the half it never had.
- The locator gains a genuine notion of "commands for me", which is the precondition for any future hard gate (ADR-0006's "Triggers to revisit"). This ADR deliberately does **not** authenticate the target id — a spoofed id is a *hostile app* problem, still out of scope. This fixes accidents, which is the actual observed failure.
- **Addressing commands does not make the app's derived state safe, and the distinction is worth stating.** Once commands are addressed, every *response* is implicitly addressed too: `FlightMetadata`, `FlightData`, `VersionInfo` and the deployment-test countdown can only arrive from the locator that was asked. **Unsolicited broadcasts are the exception** — `PreLaunchData` and `TelemetryData` arrive from every locator on the channel, so anything the app derives from them must be gated on the sender.

  This was not hypothetical. With commands correctly addressed and only one locator arming, the app still flipped between armed and disarmed: `BluetoothService.computeExpectedPacketLength` derived armed state from the message *type* alone, in the framing path — before parsing, before authentication, before anything knew whose packet it was. The armed locator's `TelemetryData` and the bystander's `PreLaunchData` fought over the flag at the broadcast rate, while the device name and telemetry on screen, which *are* gated, stayed correctly on the connected locator. [ADR-0006](0006-locator-connect-password.md) named this watcher's type-only behavior during the armed-startup work and did not fix it. Armed state is now derived inside the connected-locator gate in `RocketViewModel`.

  **The rule for anything added later: state derived from an unsolicited broadcast must be gated on `locator_id`; state derived from a response need not be.**

- **Revisit if:** a deliberate broadcast-to-all command is ever wanted (it would need its own MsgType, not a wildcard id — reusing 0 would reintroduce exactly this hazard); or if the threat model hardens to include a modified app, at which point the target id wants authenticating alongside the existing `auth_tag`.

## Alternatives considered

- **Filter in the receiver.** Rejected: the receiver is a shared, password-agnostic relay, and the hazard is other people's receivers on the same channel. Filtering there protects nobody.
- **Keep it app-side — refuse to send when more than one locator is audible.** Rejected: it fails exactly when it matters (the other locator is silent at that instant, or is somebody else's a field away), and it leaves the wire format defective for every other client.
- **`0` as a broadcast wildcard for backward compatibility.** Rejected. It would let an un-updated app keep arming every locator on the channel, which is the defect. Old apps failing closed is the point.
- **Put the target in `PacketHeader` for all messages.** Rejected for the same reason ADR-0006 rejected putting identity there: only one direction needs it, and it would add bytes to `TelemetryData` — which is range-sensitive and periodic — for no gain.
- **Address by device name rather than UID.** Rejected: names are user-editable, not unique, and are themselves part of the config payload a command can overwrite.
