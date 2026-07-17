# ADR-0014: MapLibre for the app map — offline satellite caching, and the tile-licensing constraint

- **Status:** Accepted (map stack); tile provider for release **unresolved — see Consequences**
- **Date:** 2026-07-16
- **Deciders:** fschroer
- **Related issues:** —

## Context

Recovery happens where there is no cell signal. The app map is the tool for walking to a rocket that may be kilometres downrange, so an online-only map fails at the exact moment it is needed. `DownloadMapScreen` had existed as a UI stub — buttons, no download logic — and the live map was a plain `GoogleMap` with `MapType.SATELLITE`, i.e. online tile streaming. Offline was an intended-but-unbuilt feature.

The **Google Maps SDK exposes no offline API** on Android or iOS. (The consumer Google Maps app has offline areas; the SDK does not expose them, and those areas contain vector road data, not satellite imagery — see "Alternatives".) Offline satellite was therefore unreachable without replacing the map stack.

A native iOS port is under consideration. MapLibre Native runs on Android *and* iOS from the same style JSON and the same offline-pack model, so the map layer is one of the few pieces that can be designed once for both.

## Decision

We will render the app map with **MapLibre Native** (`org.maplibre.gl:android-sdk`) instead of Google Maps, and cache satellite tiles for a launch site as MapLibre **offline regions** so the live map renders with no connectivity.

Specific enough to check code against:

1. **The map lives behind `app/.../ui/MapLibreCompat.kt`.** `MapLibreCameraState` is a Compose-observable camera mirroring maps-compose's `CameraPositionState` (read `position`, observe `moveStartedReason`, assign `position` to move the map). `MapLibreMapView` hosts the `MapView` via `AndroidView`; the rocket marker, GPS-accuracy ring and flight path are **GeoJSON style layers**, not declarative children.
2. **Bounds fitting uses MapLibre's pure `getCameraForLatLngBounds(bounds, padding, 0.0, 0.0)` query — never a `moveCamera` "probe".** A probe moves the camera mid-frame; MapLibre's GL thread renders continuously and draws that intermediate state, which appears as a persistent wobble whenever auto-zoom is on. Fit **north-up and flat** (bearing 0, tilt 0): tilt is already compensated separately by `zoomCorrection`, so letting the SDK also account for it corrects twice.
3. **Offline regions are created by `ui/OfflineMapManager.kt`.** MapLibre's offline downloader accepts **only an `http(s)` style URL** — it rejects `asset://`, `data:` and `file://`, stalling the region at "0/1 tiles". The app therefore serves the style from a short-lived `LocalStyleServer` bound to `127.0.0.1` for the duration of a download. **`res/xml/network_security_config.xml` (localhost-scoped cleartext) is required by this, not leftover scaffolding.**
4. **The live map and the downloader must resolve the same tile source** (`SatelliteProvider` + `MapProviderPrefs`). MapLibre keeps one app-wide offline database and serves any tile whose URL matches, so a provider mismatch silently yields a blank offline map.
5. **MapLibre's camera zoom runs ~1 level deeper than Google's** (512- vs 256-px tile convention). Any zoom/scale math must use the MapLibre convention — the scale bar uses `78271.51696 * cos(lat) / 2^z` m/px.

## Consequences

**Easier**

- Offline satellite at a launch site — verified on-device, air-gapped (airplane mode **and Wi-Fi off**, ambient cache cleared): a downloaded region renders as a hard-edged rectangle of imagery with nothing outside it.
- One map stack and one style JSON shared with a future iOS port.
- No Google Maps API key. `maps-compose` (×3), `play-services-maps` and the `com.google.android.geo.API_KEY` meta-data are removed. `play-services-location` stays — it provides the fused location provider.

**Harder / risks**

- **RELEASE BLOCKER — no wired tile provider permits what this feature does.** Verified against current terms 2026-07-16:
  - **Mapbox** — Product Terms **§2.9.1**: "Customer shall use the Mobile SDKs as Customer's exclusive means of accessing the Service Offerings in mobile applications" (rendering Mapbox tiles via MapLibre violates this by itself); **§2.8.1**: caching "limited to thirty (30) days", and only "up to the limits set in the Mobile SDKs… shall not circumvent or change those limits"; **§1.9**: no bulk/automated queries, no systematic download.
  - **Esri World Imagery** — bulk caching/redistribution restricted.
  - **MapTiler Cloud** — **§5.7** "temporary personal cache… single end-user only"; **§6.3** no "batch or excessive bulk download of map tiles"; **§6.2** no export "for usage outside the Service".

  Both currently wired providers (Esri, Mapbox) are therefore **evaluation-only**. Shipping as-is would breach their terms.
- **Identified clean path: NAIP** — `s3://naip-visualization` (requester-pays), **"Public Domain with Attribution"**, 30–100 cm, CONUS-only, so tiles may be cached and redistributed freely. Requires a GDAL tiling pipeline plus an on-device MBTiles tile server (a natural extension of `LocalStyleServer`). USGS's ready-made `USGSImageryOnly` service was evaluated and **rejected: it caps at z16 (~2 m/px)** — 404s above z16 at all four launch sites, urban and remote alike; adequate for terrain context, too coarse to spot a rocket.
- **Satellite imagery is inherently large.** In a raster pyramid the deepest zoom is ~75% of all tiles (the series sums to 4/3 of the top level), so each level dropped saves ~75%. Tile size also varies ~5× by zoom, so size estimates are measured **per zoom** (`SatelliteProvider.avgTileBytes(z)`), not tiles × one constant.
- **Detail beyond the imagery's native resolution is free but worthless.** Measured Mapbox bytes/tile collapse past z19 (z20 9.5 KB, z21 5.5, z22 4.0 vs ~20 KB at z17–18) — the signature of upscaled blur. Native resolution varies by site, so the useful max zoom is a per-site judgement; the download screen shows a live detail inset for exactly this.

**Revisit when:** a provider grants written permission for permanent offline caching (MapTiler §6.3 and Mapbox §1.9 both say "unless otherwise agreed in writing" — worth asking for a hobby-scale use); or the NAIP pipeline lands; or MapLibre gains a non-`http` offline style path (which would retire `LocalStyleServer`).

## Alternatives considered

- **Stay on Google Maps.** Rejected: the SDK has no offline API on either platform, so the core requirement is unreachable. (Google Maps' own offline areas are small because they store **vector road data and no satellite imagery at all** — not a like-for-like comparison.)
- **Mapbox Maps SDK** (their own, satisfying §2.9.1). Still caps caching at 30 days and at the SDK's tile limits (~6,000 default vs the 23,014 tiles one test region needed), and abandons the MapLibre layer shared with the prospective iOS port. Fights the "download a site and keep it" use case.
- **MapTiler Server/Data self-hosting.** §4.1.1 permits "download and deploy pre-rendered tilesets… for self-hosting", but bundling onto end-user devices sits ambiguously between §5.2 (display to end users) and §6.4/§3.3 (redistribution needs a custom agreement), and it carries MAU caps (100 free / 500 standard). Would need written confirmation.
- **USGS `USGSImageryOnly` tiles.** Public domain and a drop-in URL swap (same ArcGIS `/tile/{z}/{y}/{x}` scheme as the Esri source). Rejected on resolution: z16 ceiling.
