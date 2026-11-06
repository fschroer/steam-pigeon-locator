# ADR-0043: Times are stored in UTC and shown with their offset; the app tells the locator its zone

- **Status:** Proposed. Asked for by Frank Schroer 2026-11-06 ("the locator time adjusted to the proper time zone based on the app's time zone, and all dates/times carry the time zone information"). Built 2026-11-06 in the locator, receiver, Android, iOS (wire copy only) and web. Host tests only: **nothing compiled for Android, nothing on hardware.**
- **Date:** 2026-11-06
- **Deciders:** Frank Schroer
- **Related ADRs:** "Addressed app→locator commands" (0020), whose address rule this message follows; "The radio download becomes the primary data path" (0040), for the receiver's forward slots and the Firebase metadata; "iOS port" (0016), for the parity rule; "Flyers read their flights in a browser" (0042), for the web viewer.

## Context

Audit, 2026-11-06, of where a wall-clock time starts and where it is printed:

- **The only clock is GPS.** The locator stores `FlightTimestampS`, Unix seconds in UTC from NAV-PVT, once per record. Everything else in a record is milliseconds since launch onset, which has no zone. That stored value is correct and unambiguous.
- **The locator printed it with no zone.** The console's `data` list and the USB export's `Flight time:` line used `FormatUnixUtc`, "2026-11-06 07:36:49". It was UTC, which only the manual said. A flyer reading it at the field took it for local time.
- **The apps** convert to the phone's zone for the Flight Profiles list, but printed no zone either. The app flight log (`FlightLog`) already wrote ISO 8601 with an offset on every row.
- **Firebase** carries `record_ts` (Unix seconds) in the object path and metadata, which is unambiguous but unreadable in the console.
- **The web** printed `toLocaleString` with no zone, and the notes log stamped "[2026-11-06 21:30]" in the writer's local time with no zone. A note written in California and read in Florida was three hours off with nothing to say so.
- **The receiver** handles no times.

## Decision

### 1. Store UTC; convert only when printing; print the offset every time

Nothing stored changes: the archive, the wire, the record keys, `record_ts` stay UTC seconds. Every place a person reads a wall-clock time prints it with its zone. The form is ISO-style local time plus the numeric offset, `2026-11-05 23:36:49 -08:00`, everywhere it fits. A numeric offset is unambiguous, sorts with the time, and parses back. Zone abbreviations ("PST") are not unique and are not used in files; the web's list uses the browser's short zone name, since that is display only.

### 2. The app sends the phone's offset to the locator: `TimeZoneSet` (MsgType 32)

| Offset | Field | Type |
|---|---|---|
| 0 | `PacketHeader` | 6 B |
| 6 | `target_locator_id` | u32, ADR-0020 |
| 10 | `utc_offset_min` | i16 LE, minutes east of UTC, DST included (−480 PST, −420 PDT) |

12 bytes. The locator accepts −840 … +840 and ignores the rest, in any state (it changes printing only). It keeps the offset **in RAM**. After a reset it prints UTC, labeled `UTC`, until the app sends again. It is not persisted, because the only journal with room for it (`RocketRuntimeMetadata`) has no version field, and any change to its layout re-defaults it once, password included (ADR-0006).

**Why the offset and not a zone name.** The locator has no zone rules and no room for them. The cost is DST: the console prints every flight with **today's** offset. A July flight read in November shows −08:00, an hour off the wall clock that day but correct as an instant, because the offset is printed beside it. The app, which has the rules, prints each flight with the offset of its own date.

### 3. When the app sends it

From the version loop, which already runs only while the link is up:
- after the locator has answered a VersionRequest since the link came up (never before, so it cannot land on the VersionRequest in the receiver's single forward slot);
- again when the phone's offset changes (DST, travel);
- again every 10 minutes while the link stays up, which repairs a lost frame or a locator reset.

The rule is `TimeZoneWire.shouldSend` (Android), pinned by `TimeZoneWireTest`.

### 4. The receiver relays it as housekeeping

The receiver sizes it (12 B) and forwards it in `pending_tx_`. It is the app's own initiative, like the version poll, so:
- `IsOperatorCommand` excludes it. A time-zone frame never ends a channel survey or a locator search;
- it **never displaces an operator command** already waiting in `pending_tx_` (an Arm, a deployment-test cancel). It is dropped instead, and the next send repairs that.

An older receiver does not know type 32, fails its CRC and resyncs, as for every new type. An older locator drops it at the parser. Neither is harmed; the console just stays in UTC.

### 5. What prints what

| Where | Before | Now |
|---|---|---|
| Locator console `data` list, USB export `Flight time:` | `2026-11-06 07:36:49` (UTC, unlabeled) | `2026-11-05 23:36:49 -08:00`, or `2026-11-06 07:36:49 UTC` with no offset from the app. A slot with no GPS time stays `1970-01-01 00:00:00 UTC` |
| Android and iOS Flight Profiles | `2026-11-05 23:36:49` | `2026-11-05 23:36:49 -08:00`, with the offset of the flight's own date |
| App flight log rows | ISO 8601 with offset | unchanged |
| Firebase metadata | `record_ts` | adds `flight_time`, ISO 8601 UTC (`2026-11-06T07:36:49Z`) |
| Web flight list and titles | `Nov 5, 2026, 11:36 PM` | `Nov 5, 2026, 11:36 PM PST` (viewer's zone, named) |
| Web notes log stamp | `[2026-11-06 21:30]` | `[2026-11-06 21:30 -08:00]` |
| Web reading a USB export's `Flight time:` | bare = UTC | bare = UTC (older exports); `UTC`/`Z` = UTC; `±HH:MM` = local with that offset |

Kept as they are, on purpose:
- The USB export's sample rows are unchanged (time_ms has no zone), so the app's full-record CSV stays byte-identical to the export (ADR-0040 decision 8).
- The app flight log's **file name** stays `yyyy-MM-dd_HHmmss`: it is a name, and every row inside carries the offset.
- Notes already written keep their old unzoned stamps.

## Consequences

- A locator talking only to iOS prints UTC, labeled UTC, until iOS sends `TimeZoneSet` (UI_PARITY "iOS OWES THIS" row 8). That is correct, just not local.
- The console `data` header gains a `Zone` column; any script that parses the list by position needs to know.
- Wire sync: locator `MessageProtocol.hpp` asserts (`sizeof(TimeZoneSet) == 12`, offset 10), the receiver's copy, `WireLayoutTest.kt`, `WireLayoutTests.swift`.
