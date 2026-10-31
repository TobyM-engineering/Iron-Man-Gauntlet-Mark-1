// The thermal camera (Toby, 2026-10-07 night): SHOW and FULL, by a gesture only (the gesture is in the map he has to
// approve, design/00 #31; until then the bench key I). ON = the hand plate lifts (servosMovePlate(THERMAL_PLATE)) and,
// once it has landed open, the THERMAL bit goes into $S: the puck starts reading the MLX90640 - powered all the time
// from the puck's own 3.3 V, 15 / 20 / 25 mA (Melexis datasheet rev 12 Table 5), no switch - and shows its image full
// screen. OFF = the bit clears first (the puck stops reading, back to its 3D view), then the hand plate closes.
// SAFE, goSafe and the battery's idle / CRITICAL only clear the bit: the plate freezes where it is - a mode change never
// closes a plate. While on it HOLDS the hand plate (servos.cpp): a cascade close leaves it up, and its off closes the
// plate only if nothing else needs it (plate 2 / 3, the red laser) - the plates' order, Toby 2026-10-07 night.
#pragma once
#include <Arduino.h>

void        thermalPoll();                    // every loop(): lifting -> on once the plate is up, closing -> off
bool        thermalOn(const char* why);      // false = blocked (mode) or refused (a move running, the battery); reported to the puck as $E
bool        thermalOff(const char* why);     // the bit off, then the hand plate closes (SHOW / FULL)
void        thermalToggle(const char* why);
void        thermalStop(const char* why);    // the mode left SHOW / FULL or goSafe: the bit off, the plate stays where it is
bool        thermalIsOn();                   // deployed and on: the $S THERMAL bit
const char* thermalStateName();              // off / lifting / on / closing
String      thermalJson();
