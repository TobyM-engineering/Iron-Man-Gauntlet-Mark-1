// The small outputs. v3 (2026-10-06): the red laser is the TTL #1056 on PCA channel 7 (laser.cpp's
// gesture drives it through laserRedSet), the green laser's channel 6 is open (00 #22: the LPA on hand
// has no input), the vibration motor hangs on the DRV2605L over I2C (haptic.cpp, no PCA channel), and
// the three accent LEDs are on GPIO 4. Mode SAFE and a forced idle refuse them all; the green laser
// also needs the plates open.
#pragma once
#include <Arduino.h>

void        outputsBegin();
void        outputsPoll();                        // the vibrator's timer, the green laser's guard
bool        laserRedSet(bool on, bool closeAfter = true);    // ON: the hand plate lifts first, fires once it is up; OFF: then the plate closes if nothing needs it (closeAfter false = a mode change: it stays). false = refused (FULL only)
bool        laserRedWanted();                     // asked for (its plate may still be lifting)
bool        laserGreenWanted();
bool        laserGreenSet(bool on, bool closeAfter = true);  // the same with plate 1
bool        lidarAloneSet(bool on, bool closeAfter = true);  // (2026-10-20) the LiDAR on its own: the hand plate lifts, then it measures (no laser); off = it stops, then the plate closes if nothing holds it. The lasers' rules (FULL, not in flight)
bool        lidarAloneWanted();                  // asked for (its plate may still be lifting)
bool        lidarAloneOn();                      // measuring on its own (lidar.cpp keeps measuring while this is true)
uint32_t    laserSettleMs();                     // a laser fires this long after its plate's move has ended (LASER_PLATE_SETTLE_MS, NVS "lset")
void        laserSettleSet(uint32_t ms);         // live (/outputs?settle=MS), saved; 0..2000
bool        laserRedOn();
bool        laserGreenOn();
bool        vibPulse(uint16_t ms, const char* why);   // one buzz of at most VIB_PULSE_MAX_MS
void        vibStop();
bool        vibOn();
void        hapticFeedback(uint8_t n);           // (2026-10-21) the gestures' answer: 1 buzz = done, 2 = refused (any mode)
uint32_t    hapticFeedbackCount();
bool        accentSet(bool on);                   // the three red LEDs on PIN_ACCENT (false = refused)
bool        accentOn();
void        outputsAllOff(const char* why);       // goSafe, OTA
const char* outputsBlockText();                   // why a laser / the motor is refused right now ("" = allowed)
const char* outputsGreenBlockText();              // why the green one is refused now (the mode, the battery, the drone)
bool        lasersFlightHeld();                   // the drone is armed or flipping, or stopped less than LASER_AFTER_FLIGHT_MS ago: no laser fires
uint32_t    outputsLaserShots();
String      outputsJson();                        // {"red":0,"green":0,"vib":0,"block":""}
String      outputsStatusLine();
