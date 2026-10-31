// haptic - the DRV2605L haptic driver (Adafruit #2305) on I2C bus 2 at DRV2605_ADDR drives the #1201
// coin motor on its own OUT pins: no PCA channel, no transistor, no flyback diode, no series resistor
// (v3, 2026-10-06). Open-loop ERM, real-time-playback: write an amplitude and the motor runs, write 0
// and it stops; outputs.cpp keeps the timer. HW_FAKE touches no bus and just keeps the state.
#pragma once
#include <Arduino.h>

void hapticBegin();        // probe and configure; absent = reported once, every call becomes a no-op
void hapticStart();        // motor on at HAPTIC_RTP_LEVEL
void hapticStop();         // motor off, chip to standby
bool hapticPresent();
bool hapticOn();
