// The self-test of an HW_FAKE build: every module runs on scripted sensors and recorded actuators
// with the clock stepped by hand, so the safety rules, the state machines and the gestures are
// exercised in a second with no hardware at all. Runs at boot (SELFTEST_AT_BOOT); the report is
// kept for GET :8080/selftest and the summary printed to the console.
#pragma once
#include <Arduino.h>
#include "config.h"                 // HW_FAKE comes from here (flash.sh writes it into variant.h)
#if HW_FAKE
void   selftestRun();
String selftestReport();     // every test, PASS / FAIL with its note
String selftestSummary();    // one line: "selftest: 23 passed, 0 failed"
bool   selftestPassed();
#endif
