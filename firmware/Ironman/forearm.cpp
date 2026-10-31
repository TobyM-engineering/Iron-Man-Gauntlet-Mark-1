#include "forearm.h"
#include "config.h"
#include "ledramp.h"
#include "hal.h"
#include <GauntletBenchIO.h>

static LedChain strip(FOREARM_COUNT, PIN_ARM_LED);

uint16_t forearmCount() { return FOREARM_COUNT; }

void forearmBegin() {
  strip.begin();
  strip.setBrightness(FOREARM_BRIGHTNESS);      // the cap; never raised
  forearmOff();
}

void forearmOff() {
  strip.clear();
  strip.show();
}

void forearmSolidRamp(uint8_t r, uint8_t g, uint8_t b) { forearmSolidRamp(r, g, b, [] { return true; }); }
void forearmSolidRamp(uint8_t r, uint8_t g, uint8_t b, std::function<bool()> tick) { ledFillRamp(strip, r, g, b, tick); }

void forearmBounce(uint8_t trips, uint16_t msPerPixel, std::function<bool()> tick) {
  for (uint8_t t = 0; t < trips; t++) {
    for (int i = 0; i < (int)FOREARM_COUNT * 2 - 1; i++) {
      int px = i < (int)FOREARM_COUNT ? i : (int)FOREARM_COUNT * 2 - 2 - i;   // 0..89 then 88..0
      strip.clear();
      strip.setPixelColor(px, 60, 60, 60);
      strip.show();
      if (px == 0 || px == FOREARM_COUNT - 1) Bench.printf("  at pixel %d (%s end)\n", px, px == 0 ? "start" : "far");
      uint32_t t0 = millis();
      while (millis() - t0 < msPerPixel) { if (!tick()) { forearmOff(); return; } halWdtFeed(); delay(5); }
    }
  }
  forearmOff();
}

void forearmWalk(uint16_t msPerPixel, std::function<bool()> tick) {
  for (uint16_t i = 0; i < FOREARM_COUNT; i++) {
    strip.clear();
    strip.setPixelColor(i, 60, 60, 60);
    strip.show();
    if (i % 10 == 0) Bench.printf("  pixel %u\n", i);
    uint32_t t0 = millis();
    while (millis() - t0 < msPerPixel) { if (!tick()) { forearmOff(); return; } halWdtFeed(); delay(5); }
  }
  forearmOff();
}
float   forearmMilliamps()  { return strip.milliamps(); }
uint8_t forearmBrightness() { return strip.getBrightness(); }
uint32_t forearmShows()     { return strip.shows(); }

// Non-blocking: one span of the strip in one colour, the rest dark, written at once (the fake build's demo and the
// puck link's checks; the show patterns above stay the bench test's).
void forearmFill(uint16_t first, uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  strip.clear();
  for (uint16_t i = first; i < first + n && i < FOREARM_COUNT; i++) strip.setPixelColor(i, r, g, b);
  strip.show();
}
// What the strip holds now, for the puck link's $G: the first lit pixel's colour (as set), the brightness, how many
// are lit and where the first one is (-1 = dark).
void forearmSummary(uint32_t& rgb, uint8_t& lvl, uint8_t& lit, int8_t& first) {
  rgb = 0; lit = 0; first = -1; lvl = strip.getBrightness();
  for (uint16_t i = 0; i < FOREARM_COUNT; i++) {
    uint32_t c = strip.getPixelColor(i) & 0xFFFFFF;
    if (!c) continue;
    if (first < 0) { first = (int8_t)i; rgb = c; }
    lit++;
  }
}

// The idle look (config FOREARM_IDLE_*): a few dim red pixels whenever the arm is on.
void forearmIdle() {
  strip.clear();
  uint8_t r = (FOREARM_IDLE_RGB >> 16 & 255) * FOREARM_IDLE_LEVEL / 255, g = (FOREARM_IDLE_RGB >> 8 & 255) * FOREARM_IDLE_LEVEL / 255, b = (FOREARM_IDLE_RGB & 255) * FOREARM_IDLE_LEVEL / 255;
  for (uint16_t k = 0; k < FOREARM_IDLE_COUNT; k++) { uint16_t i = FOREARM_IDLE_FIRST + k * FOREARM_IDLE_SPACING; if (i < FOREARM_COUNT) strip.setPixelColor(i, r, g, b); }
  strip.show();
}

// A chain walk that never blocks (the chain test under load, key U): one dim white pixel end to end, `passes` times.
static int16_t  walkPx = -1; static uint8_t walkPass = 0, walkPasses = 0; static uint16_t walkMs = 40; static uint32_t walkT = 0;
void forearmWalkStart(uint16_t msPerPixel, uint8_t passes) { walkMs = msPerPixel; walkPasses = passes; walkPass = 0; walkPx = 0; walkT = 0; }
bool forearmWalking() { return walkPx >= 0; }
void forearmWalkPoll() {
  if (walkPx < 0) return;
  uint32_t now = millis();
  if (walkT && now - walkT < walkMs) return;
  walkT = now;
  if (walkPx >= (int16_t)FOREARM_COUNT) { walkPx = 0; if (++walkPass >= walkPasses) { walkPx = -1; forearmOff(); return; } }
  strip.clear(); strip.setPixelColor(walkPx, 60, 60, 60); strip.show();
  if (walkPx == 0) Bench.printf("forearm walk: pass %u of %u\n", walkPass + 1, walkPasses);
  walkPx++;
}
