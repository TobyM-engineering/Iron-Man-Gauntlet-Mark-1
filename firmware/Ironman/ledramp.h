// Shared LED helper: fill a chain with a colour by ramping up over LED_RAMP_MS
// instead of switching on instantly (inrush on the shared servo rail). The
// strip's brightness stays at the cap; only the colour is scaled.
#pragma once
#include <Arduino.h>
#include "hal.h"
#include "config.h"

// tick() is called between steps so the caller can keep the network alive
// and bail out; return false from it to stop the ramp early.
template <typename Tick>
inline void ledFillRamp(LedChain& strip, uint8_t r, uint8_t g, uint8_t b, Tick tick, uint16_t ms = LED_RAMP_MS) {
  const int steps = 10;
  for (int s = 1; s <= steps; s++) {
    float k = s / (float)steps;
    uint32_t c = strip.Color((uint8_t)(r * k), (uint8_t)(g * k), (uint8_t)(b * k));
    for (uint16_t i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, c);
    strip.show();
    if (!tick()) return;
    delay(ms / steps);
  }
}
