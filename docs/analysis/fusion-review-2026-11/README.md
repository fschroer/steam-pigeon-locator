# Fusion review prototype scripts (2026-11)

Offline Python used for [`docs/flight-analysis-2026-11-fusion-review.md`](../../flight-analysis-2026-11-fusion-review.md).
They are analysis tools, not firmware: ADR-0004 asks for any promotion to be replayed on
the locator's own code (`SP_BENCH_REPLAY` or `Tests/EkfReplay`).

Set `FLIGHT_DATA` to the flight CSV folder (default `G:\My Drive\Rocketry\Flight Data`),
then run from this folder with `python3 <script>.py`. Needs numpy.

| Script | What it does |
|---|---|
| `load.py` | CSV loader (header lines, then the `time_ms,...` table) |
| `rotm.py` | Quaternion rotate and angle wrap helpers |
| `vkf2.py` | 2-state vertical filter (altitude, vertical speed) |
| `ev2.py` | Scores the vertical filter against raw baro, the EKF and GPS |
| `gps.py` | Per-phase GPS quality survey (fix loss, low satellites, `h_acc`, vertical-speed disagreement, position jumps) |
| `dr.py` | Ascent tilt dead reckoning over simulated outages, against hold-last-fix and constant velocity |
| `wind1hz.py` | #72 descent wind coast at the app's inputs (1 Hz telemetry, packet loss), and the error-radius growth rate |
| `tc.py` | Strapdown replay with the #44 canopy tilt correction (firmware blend or roll/pitch only) |
| `v1.py`–`v5.py` | #44 replay checks: dead-reckoned replay vs logged tilt, gate variants, gain sweep, coast gate pass rate, summary table |
