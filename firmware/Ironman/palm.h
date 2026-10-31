// Palm ring: PALM_COUNT WS2812 pixels on PIN_PALM, 5 V straight from the
// PowerBoost. One colour on every pixel, brightness in tenths of PALM_CAP and
// never above it - the cap is the PowerBoost's 1 A budget. The ring goes dark
// the moment the servo attaches and comes back when it lets go (servoctl calls
// palmHold), so the two loads never add up. Every change eases in over
// PALM_RAMP_MS (no current step for the boost) and is logged with the
// estimated draw. Boots dark; nothing is saved across a reboot on purpose.
// Driven from the console (keys l . ,) and from the sim page (GET /ring).
#pragma once
#include <Arduino.h>

void     palmBegin();          // ring dark, first thing in setup (WS2812s power up in a random state)
void     palmPoll();           // every loop(): advances a running ramp
void     palmNextColour();     // key l: off -> on at the level it had before the last off (PALM_DEFAULT_LEVEL the first time), else the next colour
void     palmBrighter();       // key . : one tenth of the cap up, stops at the cap
void     palmDimmer();         // key , : one tenth down, 0 = off
void     palmHold(bool hold);  // true: dark now without forgetting the setting (servo attached); false: restore
void     palmOff();            // level 0, dark now (OTA start) - the SAFE glow, if on, eases back under it
void     palmGlow(bool on);    // SAFE's dim palm glow: PALM_GLOW_COLOUR at PALM_GLOW_LEVEL while the setting is off (the setting untouched)
bool     palmGlowing();        // the glow is what shows right now
void     palmGlowHide();       // (2026-10-21) RING OFF in SAFE: the glow off until the switch leaves SAFE
void     palmWalk();           // key t / the page's "test": one pixel at a time in chain order (PALM_COUNT + PALM_WALK_EXTRA steps) - where does the chain stop?
void     palmWarning(bool on); // battery: true = blink white LBO_BLINK_MS on / off whatever is set (servo hold still wins); false = back to the setting
void     palmAlertRate(uint16_t ms);   // the alert's blink period: 700 ms = the drone's battery getting low, 300 = LOW
void     palmAlert(bool on);   // the DRONE's battery low (fly.cpp): blink RED 300 ms on / off at 6/10 whatever is set; the glove's own white LBO blink wins over it; false = back to the setting
String   palmGesture(bool fist);   // the thumb triple-crunch: fist = off; open hand: off -> on (white, at the level it had before the last off), white -> red, red -> white, other colours left alone. Returns the log line
String   palmDip(bool up);         // a finger dip: up (pinky) = one level brighter, down (index) = one level dimmer but never off; nothing while off. Returns the log line
void     palmStatus();         // key m
String   palmStatusLine();
uint16_t palmMilliamps();      // estimated draw of what the ring shows right now
String   palmJson();           // {"i":colour,"lvl":n,"max":10,"held":0|1,"ma":estimate,"rgb":[r,g,b]} - the feed's "ring"
String   palmSet(const String& query);   // GET :8080/ring?c=NAME|INDEX &lvl=N &up=1 &down=1 &off=1 -> that JSON plus "cap", "name", "colours" (the palette) and "msg" (the console line it produced)
#include <functional>
uint8_t  palmColourIndex();    // for the display link's status frame
uint8_t  palmColourCount();    // the palette's size (white 0, repulsor 1, red 2, orange 3, amber 4, green 5, cyan 6, blue 7, magenta 8)
uint8_t  palmLevel();          // 0 = off .. PALM_LEVELS
float    palmMilliampsNow();   // the chain's own estimate of what it shows now
void     palmSolidRamp(uint8_t r, uint8_t g, uint8_t b, std::function<bool()> tick);   // bench test: a colour at the cap, ramped up
void     palmWalkBlocking(uint16_t msPerPixel, std::function<bool()> tick);            // bench test: one dot end to end (blocking)
uint32_t palmShownRgb();       // the drive every pixel gets right now, RRGGBB (brightness and the cap included) - the puck link
uint32_t palmSetRgb();         // the colour that is set, full scale RRGGBB
uint8_t  palmMode();           // 0 normal, 1 held dark (servo), 2 glove battery blink, 3 drone battery blink, 4 chain test (GauntletLink $G ringmode)
