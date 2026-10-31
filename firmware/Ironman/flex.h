// Five flex sensors, one per finger: 3V3 -> sensor -> input -> 47 kOhm (FLEX_DIVIDER_OHM, 10 k until 2026-10-07) -> sensor ground, so the
// reading FALLS as the finger bends. The thumb is on the ESP32's own ADC (GPIO 34, raw 10 ms
// samples - the flick detector lives on them); index, middle, ring and pinky are AIN0..3 of the
// ADS1115 on bus 2 (0x48), single-shot round robin at 860 SPS. Millivolts either way; the sim
// page calibrates flat/closed per finger in the browser and sends the numbers to gesture.cpp.
//
// (2026-10-08, the audits) Every finger is JUDGED on every sample: a reading no working sensor can give (under
// FLEX_FAULT_LOW_MV = open / broken, over FLEX_FAULT_HIGH_MV = shorted, beyond the calibrated range by a margin) or no
// fresh sample for FLEX_STALE_MS (the ADS1115 silent) is a FAULT from that sample on, until the finger has read right
// for FLEX_FAULT_CLEAR_MS. A faulted finger reads OPEN for every gesture (gesture.cpp) - never as a bent finger - and
// the flight's throttle leaves it out (fly.cpp). The fingers' smoothing is a time constant (FLEX_SMOOTH_TAU_MS =
// HandBench's 0.25 per 10 ms), so a faster ADS1115 changes the latency, not the feel.
#pragma once
#include <Arduino.h>

void        flexBegin();              // the thumb's ADC pin, the ADS1115 probe (absent = those four read 0 and say so)
void        flexPoll();               // every loop(): the thumb every FLEX_SAMPLE_MS, one ADS conversion in flight at a time
uint16_t    flexMv(uint8_t i);        // smoothed millivolts, 0 thumb .. 4 pinky
uint16_t    flexRawMv(uint8_t i);     // the last sample as read, no smoothing - for the thumb gesture's timing
const char* flexName(uint8_t i);
bool        flexAdsPresent();         // the ADS1115 answered at boot (or since)
uint32_t    flexAdsReads();           // conversions collected
bool        flexFault(uint8_t i);     // finger i is faulted now (0 thumb .. 4 pinky)
bool        flexAnyFault();           // any of the five
uint8_t     flexFaultMask();          // bit i = finger i faulted
const char* flexFaultWhy(uint8_t i);  // the reading that made it ("12 mV: open sensor or broken wire")
bool        flexBroken(uint8_t i);    // (2026-10-21) open, shorted or silent - the calibration's test (not the old calibrated range)
uint32_t    flexFaultsTotal();        // faults logged since boot
float       flexAdsRateHz();          // ADS1115 conversions per second over the last second (each finger gets a quarter)
uint32_t    flexAdsMaxGapMs();        // the longest a finger waited for its next sample over the last second
void        flexPrint();              // one line with all five
String      flexJson();               // {"ads":1,"n":123,"hz":640,"fault":0}
String      flexRateLine();           // "flex: ADS1115 640 conversions/s - each finger every 6.3 ms ..."
#if HW_FAKE
void        flexFakeSetMv(uint8_t i, uint16_t mv);   // what the next samples of finger i read (the thumb goes through the hal's pin table as well)
void        flexFakeAdsGone(bool gone);              // the ADS1115 stops answering (true) / comes back
void        flexFakeFastClear(bool on);              // true (default): a plausible sample clears a fault at once - the other tests need not wait; false: the real FLEX_FAULT_CLEAR_MS
#endif
