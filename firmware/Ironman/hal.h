// hal - the thin hardware layer every sensor and actuator goes through. A real build calls the
// Arduino API; an HW_FAKE build touches no GPIO, no bus and no UART at all: every output is
// recorded, every input is scripted, and the clock the modules read (millis() is mapped to
// halMillis() in config.h) is one the self-test moves forward in steps instead of waiting.
#pragma once
#include <Arduino.h>
#include "config.h"

// ---- time ----
uint32_t halMillis();                       // real: millis(); fake: millis() + whatever the test has advanced
void     halDelay(uint32_t ms);             // real: delay(); fake: a real delay too (the loop must yield), the clock follows
#if HW_FAKE
void     halAdvance(uint32_t ms);           // the fake clock jumps forward by ms (the test's way of waiting)
#endif

// ---- digital and analog pins ----
void     halPinMode(uint8_t pin, uint8_t mode);
void     halDigitalWrite(uint8_t pin, uint8_t level);
int      halDigitalRead(uint8_t pin);
uint32_t halAnalogReadMv(uint8_t pin);      // eFuse-calibrated millivolts, 11 dB attenuation (0 .. ~3100 mV)
void     halAnalogSetup(uint8_t pin);       // 12-bit, 11 dB, once per analog pin
void     halDriveStrong(uint8_t pin);       // the pin's highest drive setting (the accent LEDs' 14 mA)
#if HW_FAKE
void     halFakeSetInput(uint8_t pin, int level);     // what the next digital reads return
void     halFakeSetMv(uint8_t pin, uint32_t mv);      // what the next analog reads return
int      halFakeOutput(uint8_t pin);                  // the last level written to the pin (-1 = never written)
uint32_t halFakeWrites(uint8_t pin);                  // how many writes the pin has seen
uint8_t  halFakeMode(uint8_t pin);                    // the pinMode it was given (255 = never)
#endif

// ---- deep sleep (the CRITICAL shutdown, 2026-10-07) ----
// Every output the firmware owns is set to its safe level and HELD through the sleep (gpio_hold_en + the deep-sleep
// hold): both LED data lines LOW, the accent LEDs LOW, the buzzer idle (HIGH, the module is active low), the PCA9685
// OE HIGH (its 1 k pull-up holds it anyway). No wake source: only the master switch (or EN) starts the arm again.
// A fake build records the call and returns.
void     halDeepSleep();
void     halReleaseHolds();                     // first thing in setup(): lets go of any pad still held (belt and braces - a reset clears them)
// (2026-10-08, the audits) the shutdown sleep wakes on a timer: halDeepSleepFor(ms) = halDeepSleep with a timer wake,
// and the boot it wakes into asks halWokeFromShutdown() (a timer wake + the RTC flag halDeepSleepFor set) before
// anything else - the pads are still held, nothing is re-initialised: one chirp, then asleep again.
void     halDeepSleepFor(uint32_t ms);
bool     halWokeFromShutdown();
void     halShutdownChirpAndSleep(uint16_t hz, uint32_t chirpMs, uint32_t sleepMs);   // the wake's only job (does not return on a real board)
#if HW_FAKE
bool     halFakeSlept();                      // halDeepSleep() / halDeepSleepFor() was called
uint32_t halFakeSleepMs();                    // the timer wake it asked for (0 = none)
uint32_t halFakeChirps();                     // halShutdownChirpAndSleep() calls
void     halFakeWake();                       // forget it (the self-test carries on)
void     halFakeTimerWake();                  // as if the shutdown sleep's timer had just fired
#endif

// ---- the task watchdog (2026-10-08, the audits) ----
// The loop task is watched: if loop() (or a bounded blocking job feeding it) stops calling halWdtFeed() for the timeout,
// the chip panics and resets. GPIO 23 then floats, the 1 k pull-up takes the PCA9685's OE high and - with MODE2 OUTNE =
// 00 (servos.cpp writes 0x04 at every boot) - every PCA output goes LOW: servos, both lasers, the LED-driver TRIGs.
void     halWdtBegin(uint32_t timeoutMs);     // subscribe the calling task (loop), panic on expiry
void     halWdtFeed();                        // every loop() and inside every bounded blocking job (sweeps, the bench test)
void     halWdtPause(bool pause);             // an OTA write (the flash erase can stall the loop): unsubscribed meanwhile
bool     halWdtOn();
uint32_t halWdtTimeoutMs();
uint32_t halWdtFeeds();

// ---- tones (the buzzer): one LEDC channel ----
void     halToneStart(uint8_t pin, uint16_t hz);
void     halToneStop(uint8_t pin);
#if HW_FAKE
uint16_t halFakeTone(uint8_t pin);                    // the frequency sounding now, 0 = silent
#endif

// ---- a WS2812 chain, so the palm and forearm modules compile and run without the RMT driver ----
// Same calls as Adafruit_NeoPixel, plus the brightness cap: setBrightness() never goes above
// LED_BRIGHTNESS_CAP, whatever it is asked.
#if !HW_FAKE
#include <Adafruit_NeoPixel.h>
#endif
class LedChain {
 public:
  LedChain(uint16_t n, uint8_t pin);
  void     begin();
  void     show();
  void     clear();
  void     setBrightness(uint8_t b);
  uint8_t  getBrightness() const;
  void     setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b);
  void     setPixelColor(uint16_t i, uint32_t c);
  uint32_t getPixelColor(uint16_t i) const;           // as set (before the brightness scaling)
  static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }
  uint16_t numPixels() const { return count; }
  uint8_t  pin() const { return gpio; }
  uint32_t shows() const { return nShows; }           // frames written since boot
  float    milliamps() const;                          // what the chain draws with what it shows now: 20 mA per channel per pixel at full, 1 mA idle each
 private:
  uint16_t count; uint8_t gpio; uint32_t nShows = 0;
#if !HW_FAKE
  Adafruit_NeoPixel strip;
#else
  uint8_t   bright = 255;
  uint32_t* px;
#endif
};
