#pragma once
// Super-loop rate (TIM17 period = 1 s / SAMPLES_PER_SECOND).  50 Hz since
// ADR-0039; it was 20 Hz before.  Overridable so a host suite that replays the
// 20 Hz flight corpus can build the real code at the rate the corpus was
// recorded at (Tests/FlightReplay, Tests/DescentSlowdown).
#ifndef SAMPLES_PER_SECOND
#define SAMPLES_PER_SECOND 50
#endif
