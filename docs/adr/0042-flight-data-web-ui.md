# ADR-0042: Flyers read their flights in a browser, from a static web app over the same Firebase store

- **Status:** Accepted. Decided by Frank Schroer 2026-11-04, who took the plan as proposed (new repo, static site on Firebase Hosting, storage rules move into that repo). v1 built 2026-11-04: `steam-pigeon-web` `4678440`, rules moved out of the Android repo in `rocket-flight-manager` `80a3f5f`. Checked by type-check, unit tests and a browser render of a synthetic flight. **Not yet deployed or run against the real Firebase project.**
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
- **One more hand-copied name table:** the web labels flight states from its own copy of `FlightStates` (`src/channels.ts`). It is a display table, not wire layout, but it changes with the enum like every other copy.
- **Cost.** Hosting and Storage reads stay inside the Blaze no-cost quota at this scale.
- **A deploy now publishes the rules too,** so a rules mistake reaches production with the site. The rules are short; review them like code.

## Amendment 2026-11-05: dark theme, deleting a flight, and the path in Google Earth

Frank asked for three things after the first run against the real store.

1. **Dark theme by default.** The viewer can switch to light, or follow the system setting. The choice is kept per browser.
2. **Delete a flight from the web store.** This removes the record's `.csv` and `.bin` from `users/{uid}/locators/{locator_id}/`, after a confirmation that names the flight. The existing rules already allow the owner to delete.
   - **The phone is not touched.** Its copy stays, already marked uploaded, so the app does not upload it again. The history order (phone → cloud → locator, ADR-0040 decision 6b) still finds it.
   - **The locator is not touched.** Its copy stays in flash until the archive overwrites it.
   - **A share link to the record stops working.**
   - **No app change, so no parity question.** The web is not a parity platform (Consequences above), and neither app has a cloud delete. If one is wanted later, the Android app is the reference: it would remove the cloud copy the same way, and decide separately whether to clear its own.
   - A record deleted here comes back only if the phone loses its copy and downloads the record from the locator again.
3. **The flight path in Google Earth, as a KML download.**
   - Google Earth has had no embeddable web API since the Earth API was retired in 2015. So the web exports a KML file instead, which opens in Google Earth on the web, desktop or mobile.
   - The path is built from the CSV, like everything else here:
     - GPS lat/lon where `fix_type` ≥ 2;
     - consecutive repeats dropped;
     - the altitude is `fused_agl_m`, relative to the ground and extruded;
     - a pin marks each `flight_state` change.
   - The CSV carries no GPS altitude, so the height is the locator's own AGL estimate.
   - An in-page map is a separate decision. Google's photorealistic 3D map needs a Maps Platform API key with billing; a 2D satellite map would follow the app's MapLibre ADR and its open tile-licensing question.

Implemented in `steam-pigeon-web` `33c0e65`.

### Amendment 2026-11-05 (later): locator names, and a fly-around

- **Uploads carry `locator_name`.** The name the locator calls itself (the app's remembered `KnownLocator.label`) is added to the upload metadata, and only when one is known. The web titles each locator with the name from its newest upload, with the id beside it. Uploads made before this change have no name, so they show the id alone. This is in Android `7dbbd14`, not compiled, and iOS does not upload yet.
- **Fly-around tour: tried and withdrawn (2026-11-05).** A `gx:Tour` orbit was added, but Google Earth on the web rejects any file that contains one ("Unsupported element: gx:Tour"), and a separate Earth-Pro-only file was not wanted. A Google Earth Studio project (`.esp`) is parked: its format is undocumented, and Studio access is limited. The KML is plain again. It carries an angled view of the whole flight: a `LookAt` at a 70° tilt, aimed at half apogee, on the Document and on the path. Earth Pro honors the tilt. Earth on the web opens top-down whatever the file says. In Frank's test of 2026-11-05, a file with a `LookAt` kept Earth web in 3D, while files with no view or with a `Camera` dropped it to 2D, so the `LookAt` stays. Its path runs from the pad to the first fix after Landed, the same extent as the app's map. The landing fix is kept only if it is reachable at 50 m/s or less, which drops a simulated flight's snap home.

### Amendment 2026-11-05 (evening): an in-page 3D view on Google's 3D map

Frank chose this on 2026-11-05, over CesiumJS and over staying with KML only. Google Earth on the web ignores a KML file's tilt, and only the page itself can control the camera.

- **What it is.** The flight page gets a 3D view on the Maps JavaScript API's photorealistic 3D map (`maps3d`: `Map3DElement`, `Polyline3DElement`, `Marker3DElement`), in satellite mode.
  - It opens angled (67.5°) on the whole flight.
  - The path is drawn at `fused_agl_m`, relative to the ground, over the same extent as the KML.
  - Each flight event gets an extruded pin.
  - **Fly around** (`flyCameraAround`) makes one 30 s circle, and a click on the map stops it. **Reset view** returns to the opening view.
- **Heights.** The path and pins are relative to the ground, so they need no elevation data. The camera aims at an absolute altitude, so the pad's height is estimated from the median baro pressure on the pad, using the standard atmosphere. Weather shifts that estimate by tens of meters, which only moves the aim point slightly.
- **Cost and the key.**
  - Each map load is billed under the "Immersive Maps" SKU: 5,000 free a month as of 2026-11, then about $7 per 1,000. So the map loads **only on an explicit "Show 3D view" click**.
  - The key is a Maps Platform browser key, restricted to the site's origins and to the Maps JavaScript API. It is read from `VITE_GOOGLE_MAPS_API_KEY` at build time. It ends up in the public bundle, as every browser Maps key does, and the restrictions are what protect it.
  - Without a key, the section says the view is unavailable.
- **Licensing.** This uses Google's own map in Google's own JS API, within its terms. It does not run into the tile-licensing question of issue #26, which is about the app's offline MapLibre tiles.
- In `steam-pigeon-web` `6b464bf`. Not yet rendered with a real key.

### Amendment 2026-11-06: sharing a flight with named people

Frank asked for sharing that works like Google Drive: share a file with one or more people, and it shows in their "Shared with you" list. ADR-0040 decision 9 had deferred named-user sharing; this takes it up for the web only.

- **A share is a Firestore document.** The project gains a Firestore database that holds one collection, `shares`. Each document lets one email address read one record. Its id is `{ownerUid}_{locatorId}_{recordKey}_{recipientEmail}`, where `recordKey` is `{record_ts}_{crc}`. The fields are the same values, plus the owner's email, the locator's name and a server timestamp, so the recipient's list needs no Storage reads.
- **The share itself is the permission.** `storage.rules` gains one match. A signed-in reader whose email is verified may `get` `users/{owner}/locators/{locator}/{record}.csv` when `firestore.exists()` finds the share whose id the rule rebuilds from that path and the reader's email.
  - Recipients get **read only, the CSV only**. They cannot list the folder, read the `.bin`, write or delete.
  - The owner's own rule is unchanged. Everything else stays closed.
- **`firestore.rules`** let only the owner create a share, with a fixed field set, an id that must match its fields, a lowercase recipient address that is not the owner's own, and the owner's real email if one is given. Nobody updates a share. The owner and the recipient may read or delete it. Everything else in Firestore is closed.
- **Identity is the email address, and it must be verified** (the token's `email_verified`). A share may name an address that has no account yet; it works once someone signs up and verifies that address. Google sign-ins are verified. An email/password account made in the app may not be, so the web offers to send the verification link.
- **No notification.** There is no server to send email, so the owner tells the person. Firebase's own email sending is for auth only.
- **Revoking.** Removing the share closes access at once. Deleting the flight removes its shares too. A recipient may remove a share from their own list. Copy share link (ADR-0040 decision 9, the download-URL token) is unchanged and separate.
- **Not in the apps.** The web is not a parity platform. If the apps later show "Shared with you", they read the same collection, and Android is the reference.
- **Setup:** Frank creates the Firestore database (production mode). On the first deploy he grants the Storage service permission to read Firestore for its rules. Deploys now publish `hosting,storage,firestore`.
- In `steam-pigeon-web` .
- **Checked.** In the Firebase emulators, 28 of 29 rule checks pass: who may create, list, read and delete a share, and that recipients cannot read the `.bin`, another record, the folder listing, or anything without a verified email or after a revoke. The one positive check, a recipient reading the shared CSV, could not run in the cloud sandbox, because its proxy blocks the Storage emulator's call to the Firestore emulator. It still needs a real two-account test.
