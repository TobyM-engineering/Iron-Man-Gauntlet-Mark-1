#include "buzzer.h"
#include "config.h"
#include "hal.h"
#include <GauntletBenchIO.h>

struct Step { uint16_t hz, ms; };                        // hz 0 = silence
static const Step P_BOOT[]     = { { 880, 60 }, { 0, 40 }, { 1320, 60 }, { 0, 40 }, { 1760, 90 } };
static const Step P_MODE[]     = { { 1320, 50 } };
static const Step P_ARM[]      = { { 880, 80 }, { 0, 40 }, { 1320, 80 }, { 0, 40 }, { 1760, 120 } };
static const Step P_DISARM[]   = { { 1760, 80 }, { 0, 40 }, { 1320, 80 }, { 0, 40 }, { 880, 120 } };
static const Step P_TOUCH[]    = { { 2200, 25 } };
static const Step P_REFUSED[]  = { { 440, 120 }, { 0, 40 }, { 440, 120 } };
static const Step P_TEST[]     = { { 440, 150 }, { 880, 150 }, { 1320, 150 }, { 1760, 150 }, { 2200, 150 } };
static const Step P_LOW[]      = { { SHUTDOWN_CHIRP_HZ, SHUTDOWN_CHIRP_MS } };   // (2026-10-20) the low-battery chirp - the same note as the shutdown's
static const Step P_FAULT[]    = { { 440, 90 }, { 0, 50 }, { 1760, 90 }, { 0, 50 }, { 440, 90 }, { 0, 50 }, { 1760, 90 } };   // (2026-10-08) the battery monitor FAULT: low-high-low-high, unlike any other

static const Step* pat = nullptr; static uint8_t patN = 0, patI = 0;
static uint32_t stepStart = 0, plays = 0;
static bool     sounding = false;

static void start(const Step* p, uint8_t n) { pat = p; patN = n; patI = 0; stepStart = millis(); plays++; if (p[0].hz) { halToneStart(PIN_BUZZER, p[0].hz); sounding = true; } else { halToneStop(PIN_BUZZER); sounding = false; } }

void buzzerBegin() { halPinMode(PIN_BUZZER, OUTPUT); halDigitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW); Bench.printf("buzzer: GPIO %u, LEDC tones, %s\n", PIN_BUZZER, BUZZER_ACTIVE_LOW ? "low-level trigger module (idles HIGH)" : "active high"); }
void buzzerPlay(Buzz b) {
  switch (b) {
    case Buzz::Boot:     start(P_BOOT, 5); break;
    case Buzz::Mode:     start(P_MODE, 1); break;
    case Buzz::Arm:      start(P_ARM, 5); break;
    case Buzz::Disarm:   start(P_DISARM, 5); break;
    case Buzz::Touch:   start(P_TOUCH, 1); break;
    case Buzz::Refused:  start(P_REFUSED, 3); break;
    case Buzz::Test:     start(P_TEST, 5); break;
    case Buzz::Fault:    start(P_FAULT, 7); break;
    case Buzz::Low:      start(P_LOW, 1); break;
  }
}
static Step one;
void buzzerTone(uint16_t hz, uint16_t ms) { one = { hz, ms }; start(&one, 1); }
void buzzerOff() { pat = nullptr; halToneStop(PIN_BUZZER); sounding = false; }
bool buzzerSounding() { return sounding; }
uint32_t buzzerPlays() { return plays; }

void buzzerPoll() {
  if (!pat) return;
  uint32_t now = millis();
  if (now - stepStart < pat[patI].ms) return;
  if (++patI >= patN) { buzzerOff(); return; }
  stepStart = now;
  if (pat[patI].hz) { halToneStart(PIN_BUZZER, pat[patI].hz); sounding = true; } else { halToneStop(PIN_BUZZER); sounding = false; }
}
