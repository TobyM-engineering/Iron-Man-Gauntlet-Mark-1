#include "power.h"
#include "config.h"
#include "hal.h"
#include <GauntletBenchIO.h>
#include <Preferences.h>

// The meter's word on the day (Toby, 2026-10-13: no recompile for a calibration): /power?meter=7.62 with the pack on
// scales the reading to what the meter says, kept in NVS ("bk"); 1 = none. The two-point BATT_A / BATT_B in config.h
// stays under it.
static float battK = 1.0f;

static float      volts = 0, pinV = 0;
static PowerState state = PowerState::Unknown;
static uint32_t   lastSampleMs = 0, belowSince[4] = { 0, 0, 0, 0 }, lastWarnMs = 0, faultSince = 0, okSince = 0, settleSince = 0;
static float      avg[10]; static uint8_t avgN = 0, avgI = 0;   // 1 s average at 100 ms
static bool       faultLatched = false;                          // (2026-10-08) a FAULT stays until power-off
static void (*onChange)(PowerState, PowerState) = nullptr;
#if HW_FAKE
static float fakeV = 8.0f;
#endif

static const float THRESH[4] = { BATT_ARM_FLOOR_V, BATT_LOW_V, BATT_IDLE_V, BATT_CRITICAL_V };
static const PowerState LEVELS[4] = { PowerState::ArmFloor, PowerState::Low, PowerState::Idle, PowerState::Critical };

float       powerVolts()       { return volts; }
PowerState  powerState()       { return state; }
bool        powerArmOk()       { return state == PowerState::Ok || state == PowerState::NoPack || state == PowerState::Unknown; }
bool        powerShowOk()      { return state == PowerState::Ok || state == PowerState::ArmFloor || state == PowerState::NoPack; }
bool        powerIdleForced()  { return state == PowerState::Idle || state == PowerState::Critical; }
bool        powerCritical()    { return state == PowerState::Critical; }
bool        powerFaultLatched() { return faultLatched; }
void        powerOnChange(void (*fn)(PowerState, PowerState)) { onChange = fn; }
const char* powerStateName() {
  switch (state) {
    case PowerState::Ok: return "ok"; case PowerState::ArmFloor: return "ARM FLOOR"; case PowerState::Low: return "LOW";
    case PowerState::Idle: return "IDLE"; case PowerState::Critical: return "CRITICAL"; case PowerState::Fault: return "MONITOR FAULT";
    case PowerState::NoPack: return "no pack (USB)"; default: return "unknown";
  }
}
const char* powerBlockText() {
  static char b[96];
  switch (state) {
    case PowerState::ArmFloor: snprintf(b, sizeof b, "glove battery %.2f V under the %.1f V arm floor", volts, BATT_ARM_FLOOR_V); return b;
    case PowerState::Low:      snprintf(b, sizeof b, "glove battery LOW %.2f V", volts); return b;
    case PowerState::Idle:     snprintf(b, sizeof b, "glove battery %.2f V - forced idle", volts); return b;
    case PowerState::Critical: snprintf(b, sizeof b, "glove battery CRITICAL %.2f V", volts); return b;
    case PowerState::Fault:    snprintf(b, sizeof b, "battery monitor FAULT (%.2f V read - the divider?) - latched until power-off", volts); return b;
    default: return "";
  }
}

static float sampleVolts() {
#if HW_FAKE
  pinV = fakeV / DIV_RATIO_NOMINAL;
  return fakeV * battK;                         // the meter factor too, so the self-test can see it (1 = none)
#else
  pinV = halAnalogReadMv(PIN_VBAT) / 1000.0f;
  float raw = pinV * DIV_RATIO_NOMINAL;
  return (raw * BATT_A + BATT_B) * battK;
#endif
}

void powerBegin() {
  { Preferences p; if (p.begin("ironman", true)) { float k = p.getFloat("bk", 1.0f); if (k > 0.8f && k < 1.2f) battK = k; p.end(); } }
  halAnalogSetup(PIN_VBAT);
  for (uint8_t i = 0; i < 10; i++) avg[i] = sampleVolts();
  avgN = 10; volts = avg[0];
  Bench.printf("power: pack on GPIO %u, divider %.0fk/%.0fk (x%.3f), calibration %.4f x + %.4f; rules: arm >= %.1f V, show >= %.1f V, idle < %.1f V, shutdown < %.1f V; first reading %.2f V\n",
               PIN_VBAT, DIV_R_TOP / 1000, DIV_R_BOTTOM / 1000, DIV_RATIO_NOMINAL, BATT_A, BATT_B, BATT_ARM_FLOOR_V, BATT_LOW_V, BATT_IDLE_V, BATT_CRITICAL_V, volts);
}

static void setState(PowerState s) {
  if (s == state) return;
  PowerState from = state; state = s;
  Bench.printf("power: battery %s (%.2f V)\n", powerStateName(), volts);
  if (onChange) onChange(from, s);
}

void powerPoll() {
  uint32_t now = millis();
  if (now - lastSampleMs < 100) return;
  lastSampleMs = now;
  avg[avgI] = sampleVolts(); avgI = (avgI + 1) % 10; if (avgN < 10) avgN++;
  float sum = 0; for (uint8_t i = 0; i < avgN; i++) sum += avg[i];
  volts = sum / avgN;
  if (faultLatched) { if (state != PowerState::Fault) setState(PowerState::Fault); return; }   // latched: nothing the reading does now is trusted (a flapping leg reads plausible half the time)

  // the monitor itself
  if (volts < BATT_FAULT_LOW_V || volts > BATT_FAULT_HIGH_V) {
    if (!faultSince) faultSince = now;
    if (now - faultSince >= BATT_PERSIST_MS) { faultLatched = true; setState(PowerState::Fault); }
    for (int8_t k = 0; k < 4; k++) belowSince[k] = 0;              // the level timers mean nothing while the reading is not a pack's (a stale one fired CRITICAL as the average climbed back through 6 V)
    return;
  }
  faultSince = 0;
  if (volts >= BATT_FAULT_LOW_V && volts <= 5.8f) {              // USB / a 5 V bench feed back-feeding the junction: not a pack
    if (!okSince) okSince = now;
    if (now - okSince >= BATT_PERSIST_MS) setState(PowerState::NoPack);
    for (int8_t k = 0; k < 4; k++) belowSince[k] = 0;
    return;
  }
  okSince = 0;
  if (state == PowerState::NoPack || state == PowerState::Fault) {   // coming back from USB / a fault: the 1 s average climbs through every threshold - let it settle, then take the level it lands on
    if (!settleSince) settleSince = now;
    if (now - settleSince < BATT_PERSIST_MS) return;
    settleSince = 0;
    int8_t k = 3; while (k >= 0 && volts >= THRESH[k]) k--;
    setState(k < 0 ? PowerState::Ok : LEVELS[k]);
    return;
  }
  settleSince = 0;
  // the four thresholds: a level is entered after BATT_PERSIST_MS below it and left only BATT_HYST_V above it
  int8_t deepest = -1;
  for (int8_t k = 0; k < 4; k++) {
    if (volts < THRESH[k]) { if (!belowSince[k]) belowSince[k] = now; if (now - belowSince[k] >= BATT_PERSIST_MS) deepest = k; }
    else belowSince[k] = 0;
  }
  int8_t cur = -1; for (int8_t k = 0; k < 4; k++) if (state == LEVELS[k]) cur = k;
  if (deepest > cur) setState(LEVELS[deepest]);                   // worse: at once (after the persistence)
  else if (cur >= 0) {                                            // better only with hysteresis: the level is left once the pack is BATT_HYST_V above its threshold ...
    if (volts >= THRESH[cur] + BATT_HYST_V) { int8_t k = cur - 1; while (k >= 0 && volts >= THRESH[k]) k--; setState(k < 0 ? PowerState::Ok : LEVELS[k]); }   // ... to the deepest level the pack is still under, or Ok
  } else if (state != PowerState::Ok) setState(PowerState::Ok);
  if (state != PowerState::Ok && now - lastWarnMs >= 15000) { lastWarnMs = now; Bench.printf("power: battery %s (%.2f V)\n", powerStateName(), volts); }
}

float powerCalFactor() { return battK; }
const char* powerCalMeter(float meter) {           // 0 = forget; else the meter's reading of the pack right now
  if (meter == 0) { battK = 1.0f; }
  else {
    if (meter < 5.0f || meter > 9.6f) return "a 2S pack reads 5-9.6 V on the meter - check the number";
    float plain = volts / battK;                    // the 1 s average without the old factor
    if (plain < 4.6f) return "the reading is a monitor fault or no pack - the battery must be on";
    float k = meter / plain;
    if (k < 0.8f || k > 1.2f) return "more than 20 % from the reading - check the divider (100 k / 22 k) and the meter first";
    battK = k;
  }
  Preferences p; if (p.begin("ironman", false)) { if (battK == 1.0f) p.remove("bk"); else p.putFloat("bk", battK); p.end(); }
  for (uint8_t i = 0; i < avgN; i++) avg[i] = sampleVolts();   // the average starts again at the new scale
  volts = avg[0];
  Bench.printf("battery: meter calibration %s (factor %.4f)\n", battK == 1.0f ? "cleared" : "set", battK);
  return "";
}
String powerJson() {
  char b[128];
  snprintf(b, sizeof b, "{\"v\":%.2f,\"state\":\"%s\",\"pin\":%.3f,\"armok\":%d,\"showok\":%d,\"k\":%.4f}", volts, powerStateName(), pinV, powerArmOk() ? 1 : 0, powerShowOk() ? 1 : 0, battK);
  return String(b);
}
String powerStatusLine() {
  char b[160];
  snprintf(b, sizeof b, "power: %.2f V (%s), pin %.3f V; arm %s, show %s; thresholds %.1f / %.1f / %.1f / %.1f V", volts, powerStateName(), pinV,
           powerArmOk() ? "ok" : "REFUSED", powerShowOk() ? "ok" : "REFUSED", BATT_ARM_FLOOR_V, BATT_LOW_V, BATT_IDLE_V, BATT_CRITICAL_V);
  return String(b);
}
#if HW_FAKE
void powerFakeVolts(float v) { fakeV = v; }
void powerFakeFaultReset() { faultLatched = false; faultSince = 0; settleSince = 0; state = PowerState::Unknown; }
#endif
