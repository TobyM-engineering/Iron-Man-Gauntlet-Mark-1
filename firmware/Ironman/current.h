// INA219 at the head of the servo branch (0x41; 0x40 is the PCA): its VIN- feeds the PCA9685's V+ terminal, and
// everything on the PCA9685 passes it - the six servos, the palm ring (ch 12's header) and the red laser (ch 7's).
// The shunt is INA219_SHUNT_OHM (the board's stock R100, 0.1 ohm). servos.cpp uses currentServoA() for stall detection.
// Optional: without it everything works, the cascade's time-based release still bounds a stall, and the
// bench test reports "current logging skipped".
#pragma once
#include <Arduino.h>

bool  currentBegin();          // after Wire is up
bool  currentPresent();
void  currentLogStart();       // reset the peak, start printing every 500 ms
void  currentLogStop();        // stop printing (peak kept)
void  currentSample();         // call often; reads every 50 ms, tracks the peak
float currentNowA();           // the whole servo branch
float currentServoA();         // the servos alone: the branch less the palm ring's own estimate
float currentPeakA();
float currentBusV();
void  currentSleep();          // the CRITICAL shutdown: the INA219 powered down
String currentJson();          // {"ok":1,"a":0.12,"servo":0.10,"peak":1.40,"v":4.98,"shunt":0.10}
#if HW_FAKE
void  currentFakeA(float a);   // the self-test: what the "INA219" reads from now on
#endif
