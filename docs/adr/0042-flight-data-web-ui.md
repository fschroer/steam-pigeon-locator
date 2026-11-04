# ADR-0042: Flyers read their flights in a browser, from a static web app over the same Firebase store

- **Status:** Accepted. Decided by Frank Schroer 2026-11-04, who took the plan as proposed (new repo, static site on Firebase Hosting, storage rules move into that repo). Not yet built: the repo `fschroer/steam-pigeon-web` has to be created first.
- **Date:** 2026-11-04
- **Deciders:** Frank Schroer
- **Related ADRs:** "The radio download becomes the primary data path" (0040), whose decisions 8 (the Firebase web store) and 9 (sharing by link token) this builds on; "iOS port" (0016), for the parity rule this does **not** extend to the web.

## Context

Since ADR-0040, every finished full record goes up to Cloud Storage for Firebase (`gs://steam-pigeon.firebasestorage.app`). Each record is two files under `users/{uid}/locators/{locator_id}/{record_ts}_{crc}.{bin,csv}`, the raw v8 bytes and the CSV that the USB export prints. The only way to reach them today is the phone app.

Frank (2026-11-04) wants users to reach the store from a browser. They should be able to sign in, export a flight's full data as CSV, and chart it in the page.

## Decision

### 1. A new repo, `steam-pigeon-web`

The web UI has its own toolchain (Node, TypeScript) and its own deploy cadence, and it shares no code with the firmware or the Kotlin app. So it gets its own repo, like the iOS app.

Like the app repos, it carries only a short `CLAUDE.md` that points back here. System docs stay in this repo's `docs/`.

### 2. A static single-page app on Firebase Hosting, with no server of its own

- TypeScript, built with Vite, served by Firebase Hosting in the `steam-pigeon` project.
- The browser talks to Firebase Auth and Cloud Storage directly through the Firebase JS SDK, so there is no backend to run, patch or pay for. The Storage rules are the security boundary, exactly as they are for the phone.
- Charts use uPlot. A record is about 9,000 rows of 38 columns, and uPlot draws that without lag on a phone browser.

### 3. Same accounts as the app

The web app uses the same Firebase Auth project and the same providers, Google and Email/Password, so a flyer's phone login is their web login. What a user sees on the web is exactly what their phone uploaded.

Who may create an account is unchanged: it is whatever the Auth project allows today. Restricting sign-up (invite-only, an allow-list) would be a project-wide decision, because it would bind the app too, and it is not taken here.

### 4. The web app reads the CSV, and does not decode the `.bin`

The uploaded `.csv` matches the USB export character for character (ADR-0040 decision 8), so:
- **Export** is a download of the stored `.csv` as it is. Nothing is re-encoded, so it cannot differ from what the locator would print.
- **Charts** parse that same CSV.

The v8 decoder is deliberately **not** ported to TypeScript. It would be one more hand-synced copy of the wire format (the firmware `MessageProtocol.hpp`, `WireLayoutTest.kt`, `WireLayoutTests.swift`), and nothing in this UI needs it. If a record without a `.csv` ever has to be shown, revisit this rather than port the decoder quietly.

### 5. The Storage rules move into the web repo and are deployed, not pasted

`firebase/storage.rules` moves from the Android repo into `steam-pigeon-web`, next to the `firebase.json` that Hosting needs anyway. Then one `firebase deploy` publishes both the site and the rules, and the reference copy cannot drift from what is live, the way a console paste can.

The Android repo keeps a one-line pointer where the file was.

The rules themselves do not change: a signed-in user reads and writes only `users/{their uid}/**`, and everything else is closed.

### 6. v1 scope

1. Sign in (Google, email/password) and sign out.
2. The flight list: the user's locators and records, from `listAll` over `users/{uid}/locators/`, with the custom metadata the app sets (`record_ts`, `samples`, `slot`, `record_crc`).
3. A flight view: a uPlot chart with selectable series, and markers where `flight_state` changes.
4. CSV download.
5. Share link: copy the record's Firebase download URL (ADR-0040 decision 9). Named-user sharing stays deferred.

### 7. Setup outside the code

- **Bucket CORS.** Browsers can read Storage objects only with a CORS entry for the site's origins (`gsutil cors set`). The JSON lives in the web repo.
- **Authorized domains.** Firebase Auth accepts sign-in only from listed domains. `*.web.app` and `*.firebaseapp.com` are listed by default, and a custom domain would have to be added.
- **The web config stays out of git.** The Firebase web `apiKey` begins `AIza`, which the project's pre-commit secret scan flags by design. It is not a secret, because the rules guard the data, but the scan should keep its teeth. So the config comes from a gitignored `.env.local` at build time, or from a CI secret.

## Consequences

- **The web is not a parity platform.** The Android⇄iOS parity matrix (`SteamPigeon_SystemSummary.md` §4.4) does not grow a web column. The web app is a viewer of the store, and live telemetry, BLE and settings stay phone-only.
- **Known gaps, not blockers for v1:**
  - **The stored metadata carries no flight name, rocket or apogee**, so the list shows only the locator id, the flight time and the sample count. Adding those is an app-side metadata change.
  - **Simulated records (ADR-0041) are not flagged** in the CSV or the metadata, so the web cannot mark them. The app knows (FlightEvents `flags` bit 0) and could add a `simulated` metadata key.
- **Cost.** Hosting and Storage reads stay inside the Blaze no-cost quota at this scale.
- **A deploy now publishes the rules too,** so a rules mistake reaches production with the site. The rules are short; review them like code.
