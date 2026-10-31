// Passive buzzer module (MusRock, low-level trigger, its own PNP) on PIN_BUZZER: LEDC tones with the output inverted, a small non-blocking pattern player.
#pragma once
#include <Arduino.h>

// No low-battery, critical or LAND NOW pattern (Toby, 2026-10-08: the battery's only sound is the shutdown's chirp every
// 5 s, played by hal.cpp from deep sleep; the latched monitor FAULT keeps its own buzz)
enum class Buzz : uint8_t { Boot, Mode, Arm, Disarm, Touch, Refused, Test, Fault, Low };   // Low: the low-battery chirp (2026-10-20)

void     buzzerBegin();
void     buzzerPoll();
void     buzzerPlay(Buzz b);          // starts a pattern (a running one is replaced)
void     buzzerTone(uint16_t hz, uint16_t ms);   // one tone
void     buzzerOff();
bool     buzzerSounding();
uint32_t buzzerPlays();
