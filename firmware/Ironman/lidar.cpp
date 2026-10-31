#include "config.h"
#include "lidar.h"
#include "config.h"
#include "buses.h"
#include "laser.h"
#include "outputs.h"
#include <Wire.h>
#include <GauntletBenchIO.h>

// LIDAR-Lite v3 registers (Garmin operating manual, v3)
static constexpr uint8_t REG_ACQ_COMMAND   = 0x00;   // write 0x04: measure with receiver bias correction
static constexpr uint8_t REG_STATUS        = 0x01;   // bit 0 = busy
static constexpr uint8_t REG_DISTANCE      = 0x8f;   // 0x0f | 0x80: two bytes, HIGH then low, centimetres (auto-increment)
static constexpr uint8_t REG_POWER_CONTROL = 0x65;   // bit 0 = receiver off (saves ~40 mA), bit 2 = sleep until the next I2C transaction
static constexpr uint8_t POWER_IDLE        = 0x05;   // sleep + receiver off between shows
static constexpr uint8_t POWER_AWAKE       = 0x00;

static bool     present  = false;
static bool     busy     = false;                   // a measurement is running in the sensor
static bool     asleep   = false;                   // POWER_IDLE has been written since the last measurement
static uint32_t nextMs   = 0, busySince = 0, lastStatusMs = 0, lastProbeMs = 0, lastGoodMs = 0, lastLogMs = 0, lastOkMs = 0;
static uint32_t count    = 0, fails = 0, losses = 0;
static uint16_t mm       = 0;
static bool     logging  = LIDAR_LOG_DEFAULT;
#if HW_FAKE
static bool     fakePresent = true; static uint16_t fakeCm = 150; static uint8_t fakeLastReadReg = 0, fakeLastPower = 0xFF; static uint8_t fakeBusyPolls = 0;
#endif

static bool laserWantsIt() {
  return !LIDAR_WITH_LASER_ONLY || laserRedOn() || lidarAloneOn();   // the laser actually firing (Ironman: after the hand plate has lifted), or the puck's LIDAR button (2026-10-20)
}

static bool writeReg(uint8_t reg, uint8_t val) {
#if HW_FAKE
  if (!fakePresent) return false;
  if (reg == REG_POWER_CONTROL) fakeLastPower = val;
  if (reg == REG_ACQ_COMMAND) fakeBusyPolls = 2;        // busy for two status polls
  return true;
#else
  TwoWire& w = busWire(2);
  w.beginTransmission(LIDAR_ADDR); w.write(reg); w.write(val);
  return w.endTransmission() == 0;
#endif
}
// The v3 has no repeated START: set the register pointer with a STOP, then read.
static bool readRegs(uint8_t reg, uint8_t* buf, uint8_t n) {
#if HW_FAKE
  if (!fakePresent) return false;
  if (reg == REG_STATUS) { buf[0] = fakeBusyPolls ? 0x01 : 0x00; if (fakeBusyPolls) fakeBusyPolls--; return true; }
  if (reg == REG_DISTANCE && n == 2) { fakeLastReadReg = reg; buf[0] = (uint8_t)(fakeCm >> 8); buf[1] = (uint8_t)fakeCm; return true; }
  fakeLastReadReg = reg; return false;
#else
  TwoWire& w = busWire(2);
  w.beginTransmission(LIDAR_ADDR); w.write(reg);
  if (w.endTransmission() != 0) return false;
  uint8_t got = w.requestFrom((int)LIDAR_ADDR, (int)n);
  if (got != n) { while (w.available()) w.read(); return false; }
  for (uint8_t i = 0; i < n; i++) buf[i] = w.read();
  return true;
#endif
}

static bool probe() {
#if HW_FAKE
  return fakePresent;
#else
  return busAck(2, LIDAR_ADDR);
#endif
}

static void sleepNow() {
  if (!LIDAR_SLEEP_IDLE || asleep) return;
  asleep = writeReg(REG_POWER_CONTROL, POWER_IDLE);
}

static void found(uint32_t now) {
  present = true; busy = false; fails = 0; nextMs = now; lastOkMs = now; asleep = false;
  Bench.printf("lidar: LIDAR-Lite v3 at 0x%02X on bus 2 - a measurement every %u ms while the laser is on, distances in mm (the sensor counts cm)%s%s\n",
               LIDAR_ADDR, (unsigned)LIDAR_PERIOD_MS, logging ? ", logged here (q toggles)" : "", LIDAR_SLEEP_IDLE ? "; asleep with the receiver off between shows" : "");
  sleepNow();
}
// A failed transaction: try again shortly. Only a run of them with nothing getting through for
// LIDAR_LOST_MS means the sensor is really gone (a NACK or two mid-measurement is not "gone").
static void lost(uint32_t now, const char* what) {
  fails++; busy = false; nextMs = now + 50;
  if (fails < 3 || now - lastOkMs < LIDAR_LOST_MS) return;
  present = false; losses++; lastProbeMs = now; mm = 0;
  Bench.printf("lidar: %s, nothing got through for %u ms - treating it as gone (5 V? the 680 uF? plug?), re-probing every %u s; the feed says ok:0\n", what, (unsigned)(now - lastOkMs), (unsigned)(LIDAR_PROBE_MS / 1000));
}

void lidarBegin() {
  uint32_t now = millis();
  lastProbeMs = now;
  if (probe()) found(now);
  else Bench.printf("lidar: nothing at 0x%02X on bus 2 - absent (5 V on? SDA 16 / SCL 17?), re-probing every %u s; the feed says ok:0\n", LIDAR_ADDR, (unsigned)(LIDAR_PROBE_MS / 1000));
}

void lidarPoll() {
  uint32_t now = millis();
  if (!present) {
    if (now - lastProbeMs >= LIDAR_PROBE_MS) { lastProbeMs = now; if (probe()) found(now); }
    return;
  }
  if (!laserWantsIt() && !busy) {                        // idle: no triggers, the sensor asleep with its receiver off
    mm = 0; nextMs = now;
    if (!asleep && now - lastProbeMs >= 1000) { lastProbeMs = now; sleepNow(); }
    return;
  }
  if (!busy) {
    if ((int32_t)(now - nextMs) < 0) return;
    if (asleep) { if (!writeReg(REG_POWER_CONTROL, POWER_AWAKE)) { lost(now, "the wake-up was not acknowledged"); return; } asleep = false; }   // the first write after a sleep wakes it; it may NACK once
    if (!writeReg(REG_ACQ_COMMAND, 0x04)) { lost(now, "the trigger was not acknowledged"); return; }
    busy = true; busySince = now; lastStatusMs = now; lastOkMs = now; nextMs = now + LIDAR_PERIOD_MS;
    return;
  }
  if (now - lastStatusMs < LIDAR_STATUS_MS) return;      // do not hog the bus the hand BNO085 shares
  lastStatusMs = now;
  uint8_t st = 0;
  if (!readRegs(REG_STATUS, &st, 1)) { lost(now, "the status read failed"); return; }
  lastOkMs = now;
  if (st & 0x01) {
    if (now - busySince > LIDAR_BUSY_MAX_MS) { busy = false; lost(now, "a measurement never finished"); }
    return;
  }
  uint8_t d[2];
  if (!readRegs(REG_DISTANCE, d, 2)) { lost(now, "the distance read failed"); return; }
  busy = false; fails = 0; lastOkMs = now;
  uint16_t cm = ((uint16_t)d[0] << 8) | d[1];            // v3: high byte first
  bool valid = cm >= LIDAR_MIN_CM && cm <= LIDAR_MAX_CM;
  if (valid) { mm = cm * 10; lastGoodMs = now; count++; } else mm = 0;
  if (logging && now - lastLogMs >= LIDAR_PERIOD_MS - 5) {
    lastLogMs = now;
    if (valid) Bench.printf("lidar: %u mm\n", mm); else Bench.printf("lidar: no return (%u cm raw)\n", cm);
  }
  if (!laserWantsIt()) sleepNow();                        // the last one of a show: back to sleep
}

bool     lidarPresent()  { return present; }
uint16_t lidarMm()       { return mm; }
uint32_t lidarAgeMs()    { return lastGoodMs ? millis() - lastGoodMs : 0; }
uint32_t lidarCount()    { return count; }
bool     lidarLogging()  { return logging; }
bool     lidarAsleep()   { return asleep; }
void     lidarLog(bool on) { logging = on; Bench.printf("lidar: console log %s\n", on ? "on - one line per measurement" : "off (the feed still carries it)"); }

String lidarJson() {
  char b[96];
  snprintf(b, sizeof b, "{\"ok\":%d,\"mm\":%u,\"age\":%u,\"n\":%u,\"idle\":%d}", present ? 1 : 0, mm, (unsigned)lidarAgeMs(), (unsigned)count, (present && !laserWantsIt()) ? 1 : 0);
  return String(b);
}

String lidarStatusLine() {
  if (!present) return "lidar: absent at 0x" + String(LIDAR_ADDR, HEX) + " on bus 2 (re-probing every " + String(LIDAR_PROBE_MS / 1000) + " s" + (losses ? ", lost " + String(losses) + "x since boot" : "") + ")";
  return String("lidar: v3, ") + (!laserWantsIt() ? (asleep ? "asleep with the receiver off - it measures only while the laser is on; " : "idle (sleep bits not acknowledged yet); ") : "") + (mm ? String(mm) + " mm" : String("no return")) + ", " + String(count) + " readings, every " + String(LIDAR_PERIOD_MS) + " ms, log " + (logging ? "on" : "off") + " (q)" + (losses ? ", lost and found " + String(losses) + "x" : "");
}
#if HW_FAKE
void    lidarFakeSetCm(uint16_t cm) { fakeCm = cm; }
void    lidarFakePresent(bool p)    { fakePresent = p; }
uint8_t lidarFakeLastReadReg()      { return fakeLastReadReg; }
uint8_t lidarFakeLastPower()        { return fakeLastPower; }
#endif

