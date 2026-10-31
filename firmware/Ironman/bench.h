// Full-system bench test, key 'T' (capital). Stages, PASS/FAIL/SKIP each:
//   1 I2C scan (both buses)   2 palm chain   3 forearm chain   4 servo cascade with current logging
//   5 LEDs at the cap + cascade together   6 static outputs: each puck group, both lasers, the vibrator, one at a time
//   7 the display link   8 the flight stack's sensors (IMUs, ADS1115, LiDAR, ELRS module) - reported, not failed
// Hard floor: if the pack drops below BENCH_ABORT_V at any point the test aborts, every servo
// detaches and every LED goes off. Needs the pack (a NOPACK reading skips the floor).
#pragma once
#include <Arduino.h>
void   benchTestRun();
bool   benchTestRunning();
String benchTestReport();   // the last summary
