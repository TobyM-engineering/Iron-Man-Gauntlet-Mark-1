#include "hal.h"
#include <GauntletBenchIO.h>

// config.h maps millis()/delay() to the hal in HW_FAKE builds; here the real ones are wanted, and a
// function-like macro is not expanded when the name is written in parentheses.
#if HW_FAKE
static uint32_t fakeOffset = 0;
uint32_t halMillis()           { return (millis)() + fakeOffset; }
void     halDelay(uint32_t ms) { (delay)(ms); }
void     halAdvance(uint32_t ms) { fakeOffset += ms; }
#else
uint32_t halMillis()           { return millis(); }
void     halDelay(uint32_t ms) { delay(ms); }
#endif

#if HW_FAKE
static constexpr uint8_t NPINS = 40;
static uint8_t  fMode[NPINS];
static int8_t   fOut[NPINS];
static int8_t   fIn[NPINS];
static uint32_t fMv[NPINS];
static uint32_t fWrites[NPINS];
static uint16_t fTone[NPINS];
static bool     fInit = false;
static void fakeInit() { if (fInit) return; fInit = true; for (uint8_t i = 0; i < NPINS; i++) { fMode[i] = 255; fOut[i] = -1; fIn[i] = HIGH; fMv[i] = 0; fWrites[i] = 0; fTone[i] = 0; } }
void     halPinMode(uint8_t pin, uint8_t mode)      { fakeInit(); if (pin < NPINS) fMode[pin] = mode; }
void     halDigitalWrite(uint8_t pin, uint8_t level) { fakeInit(); if (pin < NPINS) { fOut[pin] = level ? HIGH : LOW; fWrites[pin]++; } }
int      halDigitalRead(uint8_t pin)                { fakeInit(); return pin < NPINS ? fIn[pin] : LOW; }
uint32_t halAnalogReadMv(uint8_t pin)               { fakeInit(); return pin < NPINS ? fMv[pin] : 0; }
void     halAnalogSetup(uint8_t)                    { fakeInit(); }
void     halFakeSetInput(uint8_t pin, int level)    { fakeInit(); if (pin < NPINS) fIn[pin] = level ? HIGH : LOW; }
void     halFakeSetMv(uint8_t pin, uint32_t mv)     { fakeInit(); if (pin < NPINS) fMv[pin] = mv; }
int      halFakeOutput(uint8_t pin)                 { fakeInit(); return pin < NPINS ? fOut[pin] : -1; }
uint32_t halFakeWrites(uint8_t pin)                 { fakeInit(); return pin < NPINS ? fWrites[pin] : 0; }
uint8_t  halFakeMode(uint8_t pin)                   { fakeInit(); return pin < NPINS ? fMode[pin] : 255; }
void     halDriveStrong(uint8_t)                    {}
void     halToneStart(uint8_t pin, uint16_t hz)     { fakeInit(); if (pin < NPINS) fTone[pin] = hz; }
void     halToneStop(uint8_t pin)                   { fakeInit(); if (pin < NPINS) fTone[pin] = 0; }
uint16_t halFakeTone(uint8_t pin)                   { fakeInit(); return pin < NPINS ? fTone[pin] : 0; }
#else
void     halPinMode(uint8_t pin, uint8_t mode)      { pinMode(pin, mode); }
void     halDigitalWrite(uint8_t pin, uint8_t level) { digitalWrite(pin, level); }
int      halDigitalRead(uint8_t pin)                { return digitalRead(pin); }
uint32_t halAnalogReadMv(uint8_t pin)               { return analogReadMilliVolts(pin); }
void     halAnalogSetup(uint8_t pin)                { analogReadResolution(12); pinMode(pin, INPUT); analogSetPinAttenuation(pin, ADC_11db); }
void     halDriveStrong(uint8_t pin)                { gpio_set_drive_capability((gpio_num_t)pin, GPIO_DRIVE_CAP_3); }
static bool toneAttached = false;
void     halToneStart(uint8_t pin, uint16_t hz) {
  if (!toneAttached) { ledcAttachChannel(pin, hz ? hz : 1000, 10, BUZZER_LEDC_CH); ledcOutputInvert(pin, BUZZER_ACTIVE_LOW); toneAttached = true; }   // the module sounds on LOW: duty 0 = pin HIGH = quiet
  ledcWriteTone(pin, hz);
}
void     halToneStop(uint8_t pin)                   { if (toneAttached) ledcWriteTone(pin, 0); }
#endif

// ---- LedChain ----
#if !HW_FAKE
LedChain::LedChain(uint16_t n, uint8_t pin) : count(n), gpio(pin), strip(n, pin, NEO_GRB + NEO_KHZ800) {}
void     LedChain::begin()                 { strip.begin(); }
void     LedChain::show()                  { strip.show(); nShows++; }
void     LedChain::clear()                 { strip.clear(); }
void     LedChain::setBrightness(uint8_t b) { strip.setBrightness(b > LED_BRIGHTNESS_CAP ? LED_BRIGHTNESS_CAP : b); }
uint8_t  LedChain::getBrightness() const   { return const_cast<Adafruit_NeoPixel&>(strip).getBrightness(); }
void     LedChain::setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) { strip.setPixelColor(i, r, g, b); }
void     LedChain::setPixelColor(uint16_t i, uint32_t c) { strip.setPixelColor(i, c); }
uint32_t LedChain::getPixelColor(uint16_t i) const { return const_cast<Adafruit_NeoPixel&>(strip).getPixelColor(i); }
#else
LedChain::LedChain(uint16_t n, uint8_t pin) : count(n), gpio(pin) { px = new uint32_t[n](); }
void     LedChain::begin()                 { halPinMode(gpio, OUTPUT); halDigitalWrite(gpio, LOW); }
void     LedChain::show()                  { nShows++; }
void     LedChain::clear()                 { for (uint16_t i = 0; i < count; i++) px[i] = 0; }
void     LedChain::setBrightness(uint8_t b) { bright = b > LED_BRIGHTNESS_CAP ? LED_BRIGHTNESS_CAP : b; }
uint8_t  LedChain::getBrightness() const   { return bright; }
void     LedChain::setPixelColor(uint16_t i, uint8_t r, uint8_t g, uint8_t b) { if (i < count) px[i] = Color(r, g, b); }
void     LedChain::setPixelColor(uint16_t i, uint32_t c) { if (i < count) px[i] = c; }
uint32_t LedChain::getPixelColor(uint16_t i) const { return i < count ? px[i] : 0; }
#endif
float LedChain::milliamps() const {
  // The NeoPixel library stores brightness 0 as "255" internally (0 = no scaling); treat both as full.
  uint8_t b = getBrightness(); float k = (b == 0 ? 255 : b) / 255.0f;
  float ma = 0;
  for (uint16_t i = 0; i < count; i++) {
    uint32_t c = getPixelColor(i);
    ma += 1.0f + ((c >> 16 & 255) + (c >> 8 & 255) + (c & 255)) / 255.0f * 20.0f * k;
  }
  return ma;
}

// ---- the task watchdog: real in both builds (the fake build runs on a real devkit) ----
#include "esp_task_wdt.h"
static bool     wdtOn = false;
static uint32_t wdtMs = 0, wdtFeeds = 0;
void halWdtBegin(uint32_t timeoutMs) {
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = timeoutMs; cfg.idle_core_mask = 0; cfg.trigger_panic = true;   // only the loop task is watched; on expiry: panic = reset
  if (esp_task_wdt_reconfigure(&cfg) != ESP_OK) esp_task_wdt_init(&cfg);           // the core may or may not have started it
  esp_task_wdt_add(NULL);
  wdtOn = true; wdtMs = timeoutMs;
}
void     halWdtFeed()      { if (wdtOn) { esp_task_wdt_reset(); wdtFeeds++; } }
void     halWdtPause(bool pause) {
  if (pause && wdtOn) { esp_task_wdt_delete(NULL); wdtOn = false; }
  else if (!pause && !wdtOn && wdtMs) { esp_task_wdt_add(NULL); wdtOn = true; }
}
bool     halWdtOn()        { return wdtOn; }
uint32_t halWdtTimeoutMs() { return wdtMs; }
uint32_t halWdtFeeds()     { return wdtFeeds; }

// ---- deep sleep ----
#if HW_FAKE
static bool fSlept = false;
static uint32_t fSleepMs = 0, fChirps = 0, fTimerWake = 0;
void halDeepSleep() { fSlept = true; fSleepMs = 0; for (uint8_t p : { PIN_ARM_LED, PIN_PALM_LED, PIN_ACCENT }) halDigitalWrite(p, LOW); halDigitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW); halDigitalWrite(PIN_PCA_OE, HIGH); }
void halDeepSleepFor(uint32_t ms) { halDeepSleep(); fSleepMs = ms; }
bool halWokeFromShutdown() { return fTimerWake != 0; }
void halShutdownChirpAndSleep(uint16_t hz, uint32_t chirpMs, uint32_t sleepMs) {   // the fake: the tone is recorded, the sleep again too
  fTimerWake = 0; fChirps++;
  halToneStart(PIN_BUZZER, hz); halAdvance(chirpMs); halToneStop(PIN_BUZZER);
  halDeepSleepFor(sleepMs);
}
bool     halFakeSlept()     { return fSlept; }
uint32_t halFakeSleepMs()   { return fSleepMs; }
uint32_t halFakeChirps()    { return fChirps; }
void halReleaseHolds() {}
void halFakeWake()      { fSlept = false; fSleepMs = 0; fTimerWake = 0; }
void halFakeTimerWake() { if (fSlept && fSleepMs) { fSlept = false; fTimerWake = 1; halAdvance(fSleepMs); } }
#else
#include "esp_sleep.h"
#include "driver/gpio.h"
static uint32_t shutdownSleepMs = 0;                         // halDeepSleepFor's timer wake (0 = none: only the master switch)
// The shutdown's timed sleep: an RTC flag says "this timer wake is the shutdown's" (RTC slow memory survives deep sleep,
// not a power-off - so turning the master switch off and on again is a normal boot; a normal boot clears it too).
static constexpr uint32_t SHUTDOWN_RTC_MAGIC = 0x5D0C1A55;
RTC_DATA_ATTR static uint32_t shutdownRtc = 0;
void halReleaseHolds() {
  shutdownRtc = 0;
  gpio_deep_sleep_hold_dis();
  for (uint8_t p : { PIN_ARM_LED, PIN_PALM_LED, PIN_ACCENT, PIN_BUZZER, PIN_PCA_OE }) gpio_hold_dis((gpio_num_t)p);
}
void halDeepSleep() {
  ledcDetach(PIN_BUZZER);                                    // the tone channel lets go of the pin first
  const uint8_t lows[] = { PIN_ARM_LED, PIN_PALM_LED, PIN_ACCENT };
  for (uint8_t p : lows) { pinMode(p, OUTPUT); digitalWrite(p, LOW); gpio_hold_en((gpio_num_t)p); }
  pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW); gpio_hold_en((gpio_num_t)PIN_BUZZER);
  pinMode(PIN_PCA_OE, OUTPUT); digitalWrite(PIN_PCA_OE, HIGH); gpio_hold_en((gpio_num_t)PIN_PCA_OE);
  gpio_deep_sleep_hold_en();                                 // the digital pads keep those levels through deep sleep
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if (shutdownSleepMs) esp_sleep_enable_timer_wakeup((uint64_t)shutdownSleepMs * 1000ULL);
  Serial.flush();
  esp_deep_sleep_start();                                    // does not return
}
void halDeepSleepFor(uint32_t ms) { shutdownRtc = SHUTDOWN_RTC_MAGIC; shutdownSleepMs = ms; halDeepSleep(); }
bool halWokeFromShutdown() { return shutdownRtc == SHUTDOWN_RTC_MAGIC && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER; }
void halShutdownChirpAndSleep(uint16_t hz, uint32_t chirpMs, uint32_t sleepMs) {
  // every other pad stays held (OE high, both LED data lines and the accent LEDs low): only the buzzer is let go
  gpio_hold_dis((gpio_num_t)PIN_BUZZER);
  halToneStart(PIN_BUZZER, hz); delay(chirpMs); halToneStop(PIN_BUZZER);
  halDeepSleepFor(sleepMs);                                  // holds the buzzer idle again and sleeps: does not return
}
#endif
