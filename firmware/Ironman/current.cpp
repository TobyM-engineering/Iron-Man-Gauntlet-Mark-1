#include "current.h"
#include "config.h"
#include "palm.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <GauntletBenchIO.h>

static Adafruit_INA219 ina(INA219_ADDR);
static bool     present = false, logging = false;
static uint32_t lastSample = 0, lastPrint = 0;
static float    nowA = 0, peakA = 0, busV = 0;
// The library's 32V_2A calibration assumes a 0.1 ohm shunt; with another shunt the same register value reads
// 0.1 / R of the true current (the calibration register is the same number for 0.1 ohm at 0.1 mA / LSB and
// 0.01 ohm at 1 mA / LSB), so one factor puts it right.
static constexpr float SHUNT_SCALE = 0.1f / INA219_SHUNT_OHM;
#if HW_FAKE
static float fakeA = 0;
void  currentFakeA(float a) { fakeA = a; }
#endif

bool  currentPresent() { return present; }
float currentNowA()    { return nowA; }
float currentPeakA()   { return peakA; }
float currentBusV()    { return busV; }
float currentServoA() {
  float a = nowA - palmMilliampsNow() / 1000.0f;   // the ring rides the same palm feed; its own estimate is good to ~20 %
  return a > 0 ? a : 0;
}

bool currentBegin() {
#if HW_FAKE
  present = true;
  Bench.printf("current: fake INA219 at 0x%02X (shunt %.3f ohm)\n", INA219_ADDR, INA219_SHUNT_OHM);
  return true;
#else
  Wire.beginTransmission(INA219_ADDR);
  if (Wire.endTransmission() != 0) { Bench.printf("current: no INA219 at 0x%02X - stall detection off (the cascade's timed release still applies)\n", INA219_ADDR); return false; }
  if (!ina.begin(&Wire)) { Bench.println("current: INA219 found but did not init"); return false; }
  ina.setCalibration_32V_2A();                   // gain /8 = 320 mV full scale: 3.2 A on the stock 0.1 ohm
  present = true;
  Bench.printf("current: INA219 at 0x%02X, shunt %.3f ohm (%.1f A full scale), servo branch %.2f V\n", INA219_ADDR, INA219_SHUNT_OHM, 0.32f / INA219_SHUNT_OHM, ina.getBusVoltage_V());
  return true;
#endif
}

void currentLogStart() { peakA = 0; logging = true; lastPrint = 0; }
void currentLogStop()  { logging = false; }

void currentSample() {
  if (!present) return;
  uint32_t now = millis();
  if (now - lastSample < 50) return;
  lastSample = now;
#if HW_FAKE
  nowA = fakeA; busV = 5.0f;
#else
  nowA = ina.getCurrent_mA() * SHUNT_SCALE / 1000.0f;
  busV = ina.getBusVoltage_V();
#endif
  if (nowA > peakA) peakA = nowA;
  if (logging && now - lastPrint >= 500) {
    lastPrint = now;
    Bench.printf("  I %.2f A (servos %.2f A, peak %.2f A)  rail %.2f V\n", nowA, currentServoA(), peakA, busV);
  }
}

String currentJson() {
  char b[120];
  snprintf(b, sizeof b, "{\"ok\":%u,\"a\":%.3f,\"servo\":%.3f,\"peak\":%.3f,\"v\":%.2f,\"shunt\":%.3f}",
           present ? 1u : 0u, nowA, currentServoA(), peakA, busV, INA219_SHUNT_OHM);
  return String(b);
}

// The CRITICAL shutdown: the INA219's power-down mode (about 15 uA instead of 1 mA).
void currentSleep() {
#if !HW_FAKE
  if (present) ina.powerSave(true);
#endif
}
