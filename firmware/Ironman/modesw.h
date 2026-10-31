// The three-position mode switch on GPIO 36 / 39 (ON-OFF-ON, common to ground, 10 k pull-ups and 100 nF at the
// ESP32): the MIDDLE = both high = SAFE, the GPIO 36 end = pole A low = SHOW, the GPIO 39 end = pole B low = FULL
// (Toby, 2026-10-08 - open is SAFE, so a broken wire or a lost common lands in SAFE); both low is impossible on this
// switch and reads SAFE. POLLED every MODE_POLL_MS with a
// MODE_DEBOUNCE_MS stable requirement - never an interrupt: GPIO 36 and 39 glitch low while ADC1 samples (ESP32
// errata), and the thumb and battery are on ADC1. Power-up stays SAFE until the switch has been seen in the middle
// (SAFE) once (MODE_POWERUP_NEEDS_SAFE): switching on in SHOW or FULL never goes live.
//   SAFE  idle lights only; gestures detected and reported to the puck, nothing acts; nothing moves or emits
//   SHOW  plates, palm burst, finger LEDs, ring changes, vibrator, the thermal camera; no lasers, LiDAR or drone
//   FULL  everything (code name Mode::Flight): SHOW plus the lasers, the LiDAR and drone arming
// A broken wire to the SAFE end reads as the middle (SHOW): bring-up checks all three positions (05 step 2).
#pragma once
#include <Arduino.h>
#include "config.h"

void        modeswBegin();
void        modeswPoll();
Mode        modeNow();
const char* modeName(Mode m);                // "SAFE" / "SHOW" / "FULL"
const char* modeNowName();
bool        modeAllowsShow();              // SHOW or FULL: plates, LED groups, ring changes, vibrator, thermal camera
bool        modeAllowsFull();              // FULL: the lasers, the LiDAR, drone arming
bool        modeAllowsArming();            // FULL (the same as modeAllowsFull)
bool        modeWaitingForSafe();          // power-up: the switch is not yet seen in the middle (SAFE), so the arm holds SAFE
void        modeForce(Mode m);             // key M / GET :8080/mode?force=safe|show|full: overrides the switch until the switch moves again
void        modeFollowSwitch();
bool        modeForced();
void        modeOnChange(void (*fn)(Mode from, Mode to));
uint32_t    modeChanges();                 // debounced changes since boot (a floating pin shows as hundreds a second)
String      modeJson();                    // {"mode":"SHOW","forced":0,"a":1,"b":1,"switch":"SHOW","waiting":0,"changes":3}
String      modeStatusLine();
#if HW_FAKE
void        modeFakePowerUp();             // the self-test: a power-up with the switch where the fake pins are (the latch armed again)
#endif
