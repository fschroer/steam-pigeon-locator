# ADR-0028: The app does not transmit a setting it cannot read back

- **Status:** Accepted
- **Date:** 2026-08-21
- **Deciders:** fschroer
- **Related ADRs:** [0011](0011-locator-lora-channel-from-app.md) (the receiver
  follows a locator channel change by reading `lora_channel` at a fixed offset
  in the relayed frame), [0020](0020-targeted-locator-commands.md) (config is
  addressed to one locator), [0016](0016-ios-port-corebluetooth-and-platform-parity.md)
  (one user manual serves both platforms, so a control on one is a control on both)
- **Requirements:** FR-A3 (configure the locator from the app), FR-L2 (persistent
  locator settings)

## Context

`LocatorCfgChgRequest` carries the whole `RocketPersistentSettings` struct — the
app sends every field at once and the locator `memcpy`s the lot into the settings
it is about to write to flash.

Two of those fields, `launch_detect_altitude` and `deploy_signal_duration`, are
not carried by `PreLaunchData`. The app therefore has no way to learn what the
locator actually holds for either.

That matters because of how a config change is confirmed. There is no
acknowledgement message: the app compares the whole settings object it sent
against one rebuilt from the next broadcast, and reports success only on
equality. For the two fields the broadcast does not carry, it had to substitute
something — and what it substituted was the firmware defaults, 30 m and 1.0 s.

Three consequences, none of them visible from any single place in the code:

1. **Editing either field always reported failure, while succeeding.** Set launch
   detect altitude to 50 and press Update: the locator receives it, saves it, and
   goes on broadcasting. The rebuilt config still says 30, the comparison never
   matches, the app reports *"Update not acknowledged"*, and the display reverts
   to 30 m on the next broadcast. The change took effect and the app said it had
   not. A `// To do: remove from UI` had sat beside both constants for months.

2. **Every config change overwrote both fields**, because the message carries all
   of them. A plain LoRa channel move — which has nothing to do with either —
   wrote the app's guess over whatever the locator held. `deploy_signal_duration`
   is pyro firing time.

3. **The console never set them either.** `UserInteraction`'s config save assigns
   eight fields by name into the settings it read from the archive, and neither of
   these is among them. So the app was the only writer, and it was writing a value
   it had invented. Believing the console could set them (the parity notes said
   so) was itself part of why this looked survivable.

## Decision

**A setting the app cannot read back is not a setting the app transmits.**

`launch_detect_altitude` and `deploy_signal_duration` become **reserved** fields
of `LocatorCfgChgRequest`:

- The app fills both slots with the firmware defaults and offers no control for
  either. The controls are removed from Locator Settings on both platforms.
- The locator copies the message as before, then **restores its own values for
  both** from `archive_.GetLocatorSettings()` before the flash write.
- The receiver is unchanged. It relays the frame and reads `lora_channel` by
  `offsetof`, and that offset does not move.

**The slots stay occupied rather than being removed from the wire.** Deleting
them would shift `lora_channel`, which is the one byte the receiver reads out of
a relayed frame to follow a channel change (ADR-0011). It would also change
`sizeof(LocatorSettings)` from 45, which three `static_assert`s and the app's
`WireLayoutTest` pin, and — because there is no version negotiation — would mean
app, receiver and locator could only ever be flashed together. Keeping the layout
means each side can be updated independently, and an app predating this change
also stops being able to reset the two fields the moment the locator is flashed.

**The filler is the firmware defaults, not zero.** Since the layout is unchanged,
a locator running older firmware still adopts whatever arrives in those slots. For
it, zero would mean launch detected at 0 m AGL — true on the pad — and a pyro
signal held for 0 s, which is a charge that never fires. The defaults leave such a
locator exactly where it already was, which is what makes the app safe to ship
ahead of the firmware.

## Consequences

**Both fields are now fixed at their defaults on every device**, 30 m and 1.0 s.
Nothing can change them: not the app, not the console. This is a real reduction in
capability, and it is recorded as one rather than presented as a pure fix — but
what it removes is a control that could never report success and a write the app
had no business making. The defaults are the right values for ordinary flying.

**The two save paths now agree.** The console has always assigned settings field
by field and therefore always left these two alone; the app path was the odd one
out because it copied the message whole.

**Every other field on the screen becomes confirmable.** Omitting these two is
precisely what makes the whole-object comparison honest — the app now sends only
fields it can verify, so a reported success is a real one.

**This closes when the firmware carries both fields in a broadcast.** That is a
change across three binaries — add them to `PreLaunchData`, resize it, update the
`static_assert`s, the Kotlin offsets and the iOS copy — at which point the app can
read them back, confirm them, and offer the controls again. Nothing here should be
read as saying the fields are unimportant, only that a control the app cannot
confirm is worse than no control.

**iOS reached the same place first, from the same evidence.** The controls were
omitted there on 2026-08-20 as the port's one deliberate UI divergence from
Android, with a note that it would close when the firmware carried the fields.
This ADR makes it the system's decision rather than one platform's workaround, and
the divergence closes because Android moved, not because iOS did.
