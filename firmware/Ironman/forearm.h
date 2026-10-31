// Forearm strip (WS2812B, ~90 px on GPIO 13). Same brightness cap as the palm.
#pragma once
#include <Arduino.h>
#include <functional>

void forearmBegin();
void forearmOff();
void forearmSolidRamp(uint8_t r, uint8_t g, uint8_t b);              // ramps up over LED_RAMP_MS
void forearmSolidRamp(uint8_t r, uint8_t g, uint8_t b, std::function<bool()> tick);
void forearmWalk(uint16_t msPerPixel, std::function<bool()> tick);  // one dot end to end; tick false = stop
void forearmBounce(uint8_t trips, uint16_t msPerPixel, std::function<bool()> tick);   // end to end and back, trips times
uint16_t forearmCount();
float    forearmMilliamps();   // what the strip draws with what it shows now (estimate)
uint8_t  forearmBrightness();  // never above LED_BRIGHTNESS_CAP
uint32_t forearmShows();
void     forearmFill(uint16_t first, uint16_t n, uint8_t r, uint8_t g, uint8_t b);   // one span lit, the rest dark, at once (non-blocking)
void     forearmSummary(uint32_t& rgb, uint8_t& lvl, uint8_t& lit, int8_t& first);   // the first lit pixel's colour, the brightness, how many lit, where (-1 = dark)
void     forearmIdle();                                   // the idle look: FOREARM_IDLE_COUNT pixels at FOREARM_IDLE_LEVEL in FOREARM_IDLE_RGB
void     forearmWalkStart(uint16_t msPerPixel, uint8_t passes);   // a non-blocking chain walk (forearmWalkPoll every loop)
void     forearmWalkPoll();
bool     forearmWalking();
