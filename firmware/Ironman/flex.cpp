#include "config.h"
#include "flex.h"
#include "hal.h"
#include "buses.h"
#include <Wire.h>
#include <math.h>
#include <GauntletBenchIO.h>

static float    mv[5]  = { 0, 0, 0, 0, 0 };   // smoothed, for the feed and the stream
static uint16_t raw[5] = { 0, 0, 0, 0, 0 };   // the last sample as read - the gesture detector wants the real dynamics, not the lag of the smoothing
static uint32_t lastSampleMs[5] = { 0, 0, 0, 0, 0 };   // when each finger last got a sample (the thumb every FLEX_SAMPLE_MS, the others from the ADS1115)
static uint32_t lastThumbMs = 0;
static bool     adsPresent = false;
static uint32_t adsReads = 0, adsFails = 0, adsProbeMs = 0, adsStartMs = 0;
static uint8_t  adsChan = 0;                  // the AIN a conversion is running on
static bool     adsBusy = false;
static uint32_t rateWinMs = 0, rateWinN = 0, gapWinMax = 0, gapMax = 0; static float rateHz = 0;
// the faults (2026-10-08)
static bool     faulted[5] = { false, false, false, false, false }, reported[5] = { false, false, false, false, false };
static uint32_t badSince[5] = { 0, 0, 0, 0, 0 }, goodSince[5] = { 0, 0, 0, 0, 0 }, faultsTotal = 0;
static uint16_t pollsSince[5] = { 0, 0, 0, 0, 0 };   // flexPoll() calls since the finger's last sample: a stall of the loop itself (a flash write) is not a silent sensor
static bool     staleOnly[5] = { false, false, false, false, false };   // the fault is "no samples" only: it clears with the first good sample
static char     why[5][48];
#if HW_FAKE
static uint16_t fakeMv[5] = { 800, 800, 800, 800, 800 };   // a plausible open hand until a test says otherwise
static bool     fakeGone = false, fakeFastClear = true;      // the self-test: a plausible sample clears a fault at once (t_flex_fault turns this off to time the real clear)
#endif

bool gestureCalMv(uint8_t flexIdx, uint16_t& flat, uint16_t& closed);   // gesture.cpp: the calibrated flat / closed millivolts (false = none yet)

const char* flexName(uint8_t i)  { return i < 5 ? FLEX_NAMES[i] : "?"; }
uint16_t    flexMv(uint8_t i)    { return i < 5 ? (uint16_t)(mv[i] + 0.5f) : 0; }
uint16_t    flexRawMv(uint8_t i) { return i < 5 ? raw[i] : 0; }
bool        flexAdsPresent()     { return adsPresent; }
uint32_t    flexAdsReads()       { return adsReads; }
bool        flexFault(uint8_t i) { return i < 5 && faulted[i]; }
bool        flexAnyFault()       { for (uint8_t i = 0; i < 5; i++) if (faulted[i]) return true; return false; }
uint8_t     flexFaultMask()      { uint8_t m = 0; for (uint8_t i = 0; i < 5; i++) if (faulted[i]) m |= (1 << i); return m; }
const char* flexFaultWhy(uint8_t i) { return i < 5 && faulted[i] ? why[i] : ""; }
// (2026-10-21) broken for real - open, shorted or silent - without the calibrated-range rule: a NEW sensor reads outside the
// old sensor's range, so judged by flexFault a replaced thumb could never be calibrated (Toby's new thumb: 2600 mV flat
// against the old 800 / 600 - "above its flat", faulted, and every calibration step refused it)
bool flexBroken(uint8_t i) {
  if (i >= 5) return true;
  bool fresh = lastSampleMs[i] && !(pollsSince[i] >= FLEX_STALE_POLLS && millis() - lastSampleMs[i] > FLEX_STALE_MS);
  return !fresh || raw[i] < FLEX_FAULT_LOW_MV || raw[i] > FLEX_FAULT_HIGH_MV;
}
uint32_t    flexFaultsTotal()    { return faultsTotal; }
float       flexAdsRateHz()      { return rateHz; }
uint32_t    flexAdsMaxGapMs()    { return gapMax; }

// ---- the judgement, after every sample and on every poll (a finger that stops getting samples is faulted too) ----
// A working #182 on 47 k reads about 2.2 V flat and stays above ~0.6 V even bent past a fist; an open sensor or a broken
// wire pulls the input to ground (~0 V), a shorted sensor to 3.3 V. Beyond the calibrated range: above flat by
// FLEX_FAULT_FLAT_MARGIN_MV or half the swing, or below closed by FLEX_FAULT_CLOSED_MARGIN_MV or the whole swing (a hard
// squeeze reads past the calibrated fist). The thumb's calibrated "closed" is the fist-limited one (step 6), and the
// thumb alone closes further, so its low side is judged by the absolute line only.
static bool plausible(uint8_t i, uint16_t v, char* w, size_t n) {
  if (v < FLEX_FAULT_LOW_MV)  { snprintf(w, n, "%u mV: open sensor or broken wire", v); return false; }
  if (v > FLEX_FAULT_HIGH_MV) { snprintf(w, n, "%u mV: shorted sensor", v); return false; }
  uint16_t flat, closed;
  if (gestureCalMv(i, flat, closed) && flat > closed) {
    uint16_t sw = flat - closed;
    uint16_t hi = sw / 2 > FLEX_FAULT_FLAT_MARGIN_MV ? sw / 2 : FLEX_FAULT_FLAT_MARGIN_MV;
    uint16_t lo = sw > FLEX_FAULT_CLOSED_MARGIN_MV ? sw : FLEX_FAULT_CLOSED_MARGIN_MV;
    if (v > flat + hi) { snprintf(w, n, "%u mV: above its flat %u by more than %u", v, flat, hi); return false; }
    if (i != 0 && closed > lo && v < closed - lo) { snprintf(w, n, "%u mV: below its closed %u by more than %u", v, closed, lo); return false; }
  }
  return true;
}
static void judge(uint8_t i, uint32_t now) {
  char w[48];
  // stale = no sample for FLEX_STALE_MS AND for FLEX_STALE_POLLS polls in a row: time alone would fault every finger
  // whenever the loop itself stalls (an NVS write at a disarm takes tens of ms - it did, in the self-test)
  bool fresh = lastSampleMs[i] && !(pollsSince[i] >= FLEX_STALE_POLLS && now - lastSampleMs[i] > FLEX_STALE_MS);
  bool ok = fresh && plausible(i, raw[i], w, sizeof w);
  if (!fresh) snprintf(w, sizeof w, i ? "no samples (the ADS1115 silent?)" : "no samples");
  if (!ok) {
    goodSince[i] = 0;
    if (!faulted[i]) { faulted[i] = true; reported[i] = false; badSince[i] = now; strlcpy(why[i], w, sizeof why[i]); staleOnly[i] = !fresh; }
    else if (fresh) staleOnly[i] = false;                // a bad reading after the silence: the full clear time applies
    if (!reported[i] && now - badSince[i] >= FLEX_FAULT_REPORT_MS) {
      reported[i] = true; faultsTotal++;
      Bench.printf("flex: %s FAULT - %s. The gestures read it as OPEN (never bent)%s until it reads right for %lu ms\n", FLEX_NAMES[i], why[i],
                   i == 0 ? ", its clicks are ignored" : ", the throttle leaves it out", (unsigned long)FLEX_FAULT_CLEAR_MS);
    }
  } else if (faulted[i]) {
    if (!goodSince[i]) goodSince[i] = now;
    if (staleOnly[i]) goodSince[i] = now - FLEX_FAULT_CLEAR_MS;   // the samples are back and they read right: nothing was wrong with the sensor
#if HW_FAKE
    if (fakeFastClear) goodSince[i] = now - FLEX_FAULT_CLEAR_MS;
#endif
    if (now - goodSince[i] >= FLEX_FAULT_CLEAR_MS) {
      faulted[i] = false; goodSince[i] = 0;
      if (reported[i]) Bench.printf("flex: %s reads right again (%u mV) - back in the gestures\n", FLEX_NAMES[i], raw[i]);
      reported[i] = false;
    }
  }
}
static void sample(uint8_t i, float v, uint32_t now) {   // one new sample: the time-constant smoothing, then the judgement
  raw[i] = (uint16_t)(v + 0.5f);
  float dt = lastSampleMs[i] ? (float)(now - lastSampleMs[i]) : (float)FLEX_SAMPLE_MS;
  if (i && lastSampleMs[i]) { uint32_t gap = now - lastSampleMs[i]; if (gap > gapWinMax) gapWinMax = gap; }
  float a = 1.0f - expf(-dt / FLEX_SMOOTH_TAU_MS);
  if (!lastSampleMs[i] || faulted[i]) mv[i] = v; else mv[i] += a * (v - mv[i]);   // a fresh start (or a faulted finger) follows at once
  lastSampleMs[i] = now; pollsSince[i] = 0;
  judge(i, now);
}

// ---- ADS1115, the minimum: config register 0x01, conversion register 0x00 -------------------------------
// Config: OS=1 (start) | MUX 100..111 (AINx vs GND) | PGA 001 (+-4.096 V) | MODE 1 (single shot) | DR 111 (860 SPS) | comparator off (0x03)
static bool adsWrite16(uint8_t reg, uint16_t v) {
#if !HW_FAKE
  Wire1.beginTransmission(ADS1115_ADDR); Wire1.write(reg); Wire1.write((uint8_t)(v >> 8)); Wire1.write((uint8_t)v);
  return Wire1.endTransmission() == 0;
#else
  (void)reg; (void)v; return !fakeGone;
#endif
}
static bool adsRead16(uint8_t reg, uint16_t& v) {
#if !HW_FAKE
  Wire1.beginTransmission(ADS1115_ADDR); Wire1.write(reg);
  if (Wire1.endTransmission() != 0) return false;
  if (Wire1.requestFrom((int)ADS1115_ADDR, 2) != 2) { while (Wire1.available()) Wire1.read(); return false; }
  uint8_t hi = Wire1.read(), lo = Wire1.read();
  v = (uint16_t)((hi << 8) | lo);
  return true;
#else
  if (fakeGone) return false;
  if (reg == 0x01) { v = 0x8000; return true; }                        // OS bit set = not busy
  int32_t counts = (int32_t)fakeMv[adsChan + 1] * 1000 / 125;          // 0.125 mV per LSB at +-4.096 V
  if (counts > 32767) counts = 32767;
  v = (uint16_t)counts; return true;
#endif
}
static bool adsStart(uint8_t ain) {
  uint16_t cfg = 0x8000 | ((uint16_t)(4 + ain) << 12) | (1 << 9) | (1 << 8) | (7 << 5) | 0x03;
  return adsWrite16(0x01, cfg);
}

static bool adsProbe() {
#if HW_FAKE
  return !fakeGone;                                          // the fake ADS1115 answers from flexFakeSetMv
#else
  return busAck(2, ADS1115_ADDR);
#endif
}

static void adsFound() {
  adsPresent = true; adsBusy = false; adsFails = 0;
  Bench.printf("flex: ADS1115 at 0x%02X on bus 2 - index, middle, ring, pinky on AIN0..3, +-4.096 V, 860 SPS single-shot round robin, the next conversion started as one is read\n", ADS1115_ADDR);
}

void flexBegin() {
  halAnalogSetup(PIN_FLEX_THUMB);
#if HW_FAKE
  halFakeSetMv(PIN_FLEX_THUMB, fakeMv[0]);
#endif
  mv[0] = raw[0] = (uint16_t)halAnalogReadMv(PIN_FLEX_THUMB);
  lastSampleMs[0] = millis();
  Bench.printf("flex: thumb on GPIO %u (ADC1), 3V3 through the sensor, 47k to ground - voltage falls as the finger bends; a reading under %u or over %u mV is a FAULT\n",
               PIN_FLEX_THUMB, FLEX_FAULT_LOW_MV, FLEX_FAULT_HIGH_MV);
  adsProbeMs = millis();
  if (adsProbe()) adsFound();
  else Bench.printf("flex: no ADS1115 at 0x%02X on bus 2 - the four fingers are FAULTED (no samples) until it appears (re-probed every second)\n", ADS1115_ADDR);
  flexPrint();
}

void flexPoll() {
  uint32_t now = millis();
  if (now - rateWinMs >= 1000) { rateHz = rateWinMs ? rateWinN * 1000.0f / (now - rateWinMs) : 0; gapMax = gapWinMax; gapWinMax = 0; rateWinN = 0; rateWinMs = now; }
  if (now - lastThumbMs >= FLEX_SAMPLE_MS) {
    lastThumbMs = now;
    sample(0, halAnalogReadMv(PIN_FLEX_THUMB), now);
  }
  for (uint8_t i = 1; i < 5; i++) {                    // no samples coming: faulted (and logged) all the same
    if (pollsSince[i] < 0xFFFF) pollsSince[i]++;
    if (!lastSampleMs[i] || (pollsSince[i] >= FLEX_STALE_POLLS && now - lastSampleMs[i] > FLEX_STALE_MS)) judge(i, now);
  }
  if (!adsPresent) {
    if (now - adsProbeMs >= 1000) { adsProbeMs = now; if (adsProbe()) adsFound(); }
    return;
  }
  if (!adsBusy) {
    if (adsStart(adsChan)) { adsBusy = true; adsStartMs = now; }
    else if (++adsFails >= 20) { adsPresent = false; adsProbeMs = now; Bench.println("flex: the ADS1115 stopped answering - treating it as gone (power? plug?), re-probing every second"); }
    return;
  }
  if (now - adsStartMs < ADS_CONV_MS) return;
  uint16_t cfg;
  if (!adsRead16(0x01, cfg)) { adsBusy = false; adsFails++; return; }
  if (!(cfg & 0x8000)) { if (now - adsStartMs > 50) { adsBusy = false; adsFails++; } return; }   // still converting (polled again next loop; a stuck one is restarted and counts as a fail)
  uint16_t c;
  if (!adsRead16(0x00, c)) { adsBusy = false; adsFails++; return; }
  int16_t counts = (int16_t)c;
  float v = counts <= 0 ? 0.0f : counts * 0.125f;                      // mV
  sample(adsChan + 1, v, now);
  adsReads++; rateWinN++; adsFails = 0;
  adsChan = (adsChan + 1) & 3;
  if (adsStart(adsChan)) { adsBusy = true; adsStartMs = now; }        // the next finger's conversion at once (2026-10-08: one loop later before)
  else { adsBusy = false; adsFails++; }
}

void flexPrint() {
  Bench.print("flex mV ");
  for (uint8_t i = 0; i < 5; i++) Bench.printf(" %s %u%s", FLEX_NAMES[i], flexMv(i), faulted[i] ? " FAULT" : "");
  Bench.printf("   (ADS1115 %s)\n", adsPresent ? "ok" : "ABSENT");
  for (uint8_t i = 0; i < 5; i++) if (faulted[i]) Bench.printf("  %s FAULT: %s - read as open\n", FLEX_NAMES[i], why[i]);
}

String flexRateLine() {
  char b[200];
  float perFinger = rateHz / 4;
  snprintf(b, sizeof b, "flex: ADS1115 %.0f conversions/s - each finger every %.1f ms (%.0f Hz, the longest wait %lu ms), the thumb every %u ms; HandBench read all five every 10 ms; smoothing %.0f ms both",
           rateHz, perFinger > 0 ? 1000.0f / perFinger : 0.0f, perFinger, (unsigned long)gapMax, (unsigned)FLEX_SAMPLE_MS, FLEX_SMOOTH_TAU_MS);
  return String(b);
}

String flexJson() { return "{\"ads\":" + String(adsPresent ? 1 : 0) + ",\"n\":" + String(adsReads) + ",\"hz\":" + String(rateHz, 0) + ",\"fault\":" + String(flexFaultMask()) + "}"; }

#if HW_FAKE
void flexFakeSetMv(uint8_t i, uint16_t v) {
  if (i >= 5) return;
  fakeMv[i] = v;
  if (i == 0) halFakeSetMv(PIN_FLEX_THUMB, v);
}
void flexFakeAdsGone(bool gone) { fakeGone = gone; if (gone) adsBusy = false; }
void flexFakeFastClear(bool on) { fakeFastClear = on; }
#endif
