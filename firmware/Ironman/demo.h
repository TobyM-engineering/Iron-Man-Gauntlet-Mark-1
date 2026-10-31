// The fake build's demo (HW_FAKE only): after the self-test the fake world keeps moving so the puck has something
// real to draw over the link - a moving forearm and hand (the two BNO085s, 50 Hz), fingers flexing (the five flex
// sensors), the plates cycling (a cascade, then plate 1 alone, then the hand plate alone), the ring, the strip, the
// LED groups, the accent LEDs and both lasers changing, a thumb triple-crunch and two finger dips through the REAL
// gesture detector, the mode switch going SAFE and back, and the pack draining from 8.6 V to 6.95 V over six
// minutes, then a fresh pack. Every value goes through the real modules - only the sensors are scripted - so the
// $G / $S frames on the wire are exactly what the real arm sends. One cycle = DEMO_CYCLE_MS.
#pragma once
#include <Arduino.h>
#include "config.h"
#if HW_FAKE
void   demoPoll();               // every loop()
void   demoSet(bool on);         // GET :8080/demo?on=1|0, key Z
bool   demoRunning();
String demoJson();               // {"on":1,"t":12.3,"cycle":2,"step":"plate 1 alone","volts":8.12}
#endif
