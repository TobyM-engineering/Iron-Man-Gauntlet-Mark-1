// The CRITICAL shutdown (Toby, 2026-10-07; the audit's over-discharge finding): when the pack reaches CRITICAL the loads
// are already off (goSafe) and a SILENT grace of SHUTDOWN_GRACE_MS lets a sag that recovers cancel it; then everything
// the firmware controls goes off - the servos released and the PCA9685 asleep, both LED chains dark, the LED groups, both
// lasers, the vibrator, the accent LEDs, the drone disarmed, the IMUs, the INA219 and the LiDAR asleep - and the puck is
// told to blank its screen ($S flags bit 5) for SHUTDOWN_TELL_MS. Then the ESP32 goes into deep sleep; it wakes every
// SHUTDOWN_CHIRP_EVERY_MS (5 s), chirps once (SHUTDOWN_CHIRP_MS) and sleeps again - only the master switch (a power-off)
// ends it. That chirp is the battery's ONLY sound (Toby, 2026-10-08: "no 5 s continuous beep anywhere"; the 5 s alarm
// and the 30 s chirp went). What still draws from the pack after that (the regulators, the ELRS module, the strip's idle
// current, the puck) is in design/01-power.md section 15.
#pragma once
#include <Arduino.h>

void    shutdownArm(const char* why);   // the pack is CRITICAL (onPower): the silent grace starts; a second call does nothing
void    shutdownCancel();               // the pack is back above CRITICAL during the grace (a sag, not an empty pack)
void    shutdownPoll();                 // every loop(): the grace, then everything off, then deep sleep
bool    shutdownActive();               // armed or later - nothing may switch an output back on
uint8_t shutdownPhase();                // 0 none, 1 grace, 2 everything off + telling the puck, 3 asleep (a fake build stops here)
void    shutdownReset();                // the fake build's self-test: back to normal
