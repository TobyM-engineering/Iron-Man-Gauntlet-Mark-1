#include "demo.h"
#if HW_FAKE
#include "config.h"
#include "hal.h"
#include "servos.h"
#include "palm.h"
#include "forearm.h"
#include "power.h"
#include "modesw.h"
#include "pucks.h"
#include "outputs.h"
#include "current.h"
#include "imu.h"
#include "flex.h"
#include "gesture.h"
#include "laser.h"
#include "thermal.h"
#include "lidar.h"
#include <GauntletBenchIO.h>

static bool     on = DEMO_AT_BOOT, primed = false;
static uint32_t t0 = 0, lastImuMs = 0, lastFlexMs = 0, lastStripMs = 0, cycles = 0, drainT0 = 0;
static uint64_t firedMask = 0;
static const char* step = "starting";
static float    volts = DEMO_PACK_FULL_V;
static int32_t  thumbScriptAt = -1, dipAt = -1; static uint8_t dipFinger = 0;   // ms into the cycle a scripted gesture starts (-1 = none)

bool demoRunning() { return on && primed; }      // started (the first loop after the self-test), not just enabled
void demoSet(bool o) {
  on = o; primed = false;
  if (!on) { servosAbort(); pucksAllOff("demo off"); laserSetForce(LaserForce::Auto); forearmOff(); palmOff(); accentSet(false); laserGreenSet(false); modeFollowSwitch(); }
  Bench.printf("demo: %s\n", on ? "ON - the fake arm moves on its own (key Z or /demo?on=0 stops it)" : "off");
}

struct Q { float w, x, y, z; };
static Q qmul(const Q& a, const Q& b) {
  return { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
           a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
static Q euler(float rollDeg, float pitchDeg, float yawDeg) {   // ZYX (aerospace), the imu.cpp convention: pitch + = fingers down, yaw + = left
  float r = rollDeg * DEG_TO_RAD / 2, p = pitchDeg * DEG_TO_RAD / 2, y = yawDeg * DEG_TO_RAD / 2;
  Q qz = { cosf(y), 0, 0, sinf(y) }, qy = { cosf(p), 0, sinf(p), 0 }, qx = { cosf(r), sinf(r), 0, 0 };
  return qmul(qz, qmul(qy, qx));
}
static float wave(float t, float periodS, float phase = 0) { return sinf(2 * PI * (t / periodS) + phase); }

// the fingers' millivolts for a curl 0..1 (flat DEMO_FLAT_MV, closed DEMO_CLOSED_MV; the thumb's closed is the fist-limited one)
static uint16_t mvOf(uint8_t f, float c) {
  if (c < 0) c = 0; if (c > 1) c = 1;
  uint16_t flat = f ? DEMO_FLAT_MV : DEMO_THUMB_FLAT_MV, closed = f ? DEMO_CLOSED_MV : DEMO_THUMB_CLOSED_MV;
  return (uint16_t)(flat - c * (flat - closed));
}

// one-shot actions, each at its time in the cycle (ms) - bit i of firedMask = action i done this cycle
static bool at(uint8_t i, uint32_t ms, uint32_t ct) { if (ct < ms || (firedMask >> i) & 1) return false; firedMask |= (1ULL << i); return true; }

static void begin() {
  primed = true; t0 = millis(); firedMask = 0; drainT0 = millis(); volts = DEMO_PACK_FULL_V;
  modeForce(Mode::Flight);                      // FULL: the demo shows the lasers too (they need FULL since 2026-10-07 night)
  powerFakeVolts(volts); currentFakeA(0.08f); lidarFakePresent(true); lidarFakeSetCm(150);
  { const float id[4] = { 1, 0, 0, 0 }, x[3] = { 1, 0, 0 }; imuSetRelZero(0, id, x); imuSetRelZero(1, id, x); }   // the step-1 pose = identity, so the frames carry the script's angles exactly
  char q[96]; snprintf(q, sizeof q, "tf=%u&tc=%u&fl=%u,%u,%u,%u&fx=%u,%u,%u,%u", DEMO_THUMB_FLAT_MV, DEMO_THUMB_CLOSED_MV,
                       DEMO_FLAT_MV, DEMO_FLAT_MV, DEMO_FLAT_MV, DEMO_FLAT_MV, DEMO_CLOSED_MV, DEMO_CLOSED_MV, DEMO_CLOSED_MV, DEMO_CLOSED_MV);
  gestureSet(String(q));                        // the page's finger calibration, as the sim page would send it
  palmSet("c=repulsor&lvl=6");
  Bench.println("demo: started - cycle " + String(DEMO_CYCLE_MS / 1000) + " s; the pack drains " + String(DEMO_PACK_FULL_V, 2) + " -> " + String(DEMO_PACK_EMPTY_V, 2) + " V over " + String(DEMO_DRAIN_MS / 60000) + " min");
}

void demoPoll() {
  if (!on) return;
  if (!primed) begin();
  uint32_t now = millis(), ct = (now - t0) % DEMO_CYCLE_MS;
  uint32_t cyc = (now - t0) / DEMO_CYCLE_MS;
  if (cyc != cycles) { cycles = cyc; firedMask = 0; }
  float t = (now - t0) / 1000.0f;
  // the scripted gestures need the other fingers still: blend into "open and still" over 17-19 s and back out over 28-30 s,
  // slowly - a jump from open to a fist inside 400 ms would read as the laser's "fist closed fast"
  float still = ct < 17000 ? 0.0f : ct < 19000 ? (ct - 17000) / 2000.0f : ct < 28000 ? 1.0f : ct < 30000 ? (30000 - ct) / 2000.0f : 0.0f;

  // --- the battery: a straight drain, then a fresh pack
  float f = (now - drainT0) / (float)DEMO_DRAIN_MS;
  if (f >= 1) { drainT0 = now; f = 0; Bench.println("demo: a fresh pack"); }
  volts = DEMO_PACK_FULL_V - f * (DEMO_PACK_FULL_V - DEMO_PACK_EMPTY_V);
  powerFakeVolts(volts);
  currentFakeA(servosBusy() ? 0.45f : 0.08f);   // under SERVO_HOLD_STALL_A: a healthy cascade

  // --- the two orientation sensors at 50 Hz: the forearm sways, the wrist flexes, turns and twists
  if (now - lastImuMs >= 20) {
    lastImuMs = now;
    float k = 1.0f - 0.8f * still;
    Q fq = euler(8 * wave(t, 13), 12 * wave(t, 9), 25 * wave(t, 17));
    Q wq = euler(15 * k * wave(t, 11, 1), 28 * k * wave(t, 5), 12 * k * wave(t, 7, 2));
    Q hq = qmul(fq, wq);
    imuFakeInject(0, fq.w, fq.x, fq.y, fq.z); imuFakeInject(1, hq.w, hq.x, hq.y, hq.z);
  }

  // --- the fingers every 10 ms: a slow wave across the hand, open and still while the gestures play
  if (now - lastFlexMs >= 10) {
    lastFlexMs = now;
    for (uint8_t i = 1; i <= 4; i++) {
      float c = (1 - still) * (0.45f - 0.40f * cosf(2 * PI * t / 9.0f + i * 0.5f)) + still * 0.08f;   // a 9 s wave: never a fast fist
      if (dipAt >= 0 && i == dipFinger) {      // a dip as the self-test makes one: down 35 % of the swing in 60 ms, held 150 ms, back up in 60 ms
        int32_t d = (int32_t)ct - dipAt;
        if (d >= 0 && d < 60) c += 0.35f * d / 60.0f; else if (d >= 60 && d < 210) c += 0.35f; else if (d >= 210 && d < 270) c += 0.35f * (270 - d) / 60.0f; else if (d >= 270) dipAt = -1;
      }
      flexFakeSetMv(i, mvOf(i, c));
    }
    float tc = (1 - still) * (0.30f - 0.25f * cosf(2 * PI * t / 8.0f)) + still * 0.05f;
    if (thumbScriptAt >= 0) {                  // the triple crunch as the self-test makes it: three 90 mV presses 380 ms apart (30 ms down, 60 held, 30 up)
      int32_t d = (int32_t)ct - thumbScriptAt;
      if (d >= 0 && d < 1140) { int32_t p = d % 380; tc = 0.05f + (p < 30 ? 0.45f * p / 30.0f : p < 90 ? 0.45f : p < 120 ? 0.45f * (120 - p) / 30.0f : 0.0f); }
      else if (d >= 1140) thumbScriptAt = -1;
    }
    flexFakeSetMv(0, mvOf(0, tc));
  }

  // --- the forearm strip: a bar filling from the elbow in the ring's colour, then a chase (5 Hz)
  if (now - lastStripMs >= 200) {
    lastStripMs = now;
    uint32_t c = palmSetRgb(); uint8_t r = c >> 16, g = (c >> 8) & 0xFF, b = c & 0xFF;
    uint32_t s = ct % 20000;
    if (s < 10000) forearmFill(0, (uint16_t)(s * FOREARM_COUNT / 10000), r, g, b);
    else           forearmFill((uint16_t)(((s - 10000) / 100) % FOREARM_COUNT), 10, r, g, b);
  }
  lidarFakeSetCm((uint16_t)(150 + 100 * wave(t, 4)));

  // --- the script (ms into the cycle)
  if (at(0, 1000, ct))  { step = "cascade open"; servosOpen(); accentSet(true); }
  if (at(1, 3200, ct))  { step = "green laser (plates open)"; laserGreenSet(true); }
  if (at(2, 4000, ct))  { step = "palm LED groups 1 + 2 (repulsor)"; laserGreenSet(false); pucksShow("repulsor"); }
  if (at(3, 6500, ct))  { pucksAllOff("demo"); palmSet("c=red&lvl=8"); }
  if (at(4, 8000, ct))  { step = "cascade close"; servosClose(); }
  if (at(5, 11000, ct)) { step = "finger LEDs (group 3)"; pucksShow("fingers"); }
  if (at(6, 12500, ct)) { pucksAllOff("demo"); }
  if (at(7, 13000, ct)) { step = "plate 1 alone: open"; servosMovePlate(1, true); vibPulse(200, "demo"); }
  if (at(8, 15500, ct)) { step = "plate 1 alone: close"; servosMovePlate(1, false); accentSet(false); }
  if (at(9, 18000, ct)) { step = "thumb triple-crunch (hand open: ring white -> red)"; palmSet("c=white&lvl=5"); }
  if (at(10, 20000, ct)) { thumbScriptAt = 20000; }
  if (at(11, 23500, ct)) { step = "pinky dip (ring brighter)"; dipAt = 23500; dipFinger = 4; }
  if (at(12, 26500, ct)) { step = "index dip (ring dimmer)"; dipAt = 26500; dipFinger = 1; }
  if (at(13, 30500, ct)) { step = "thermal camera on (the hand plate lifts, THERMAL bit)"; thermalOn("demo"); palmSet("c=cyan&lvl=7"); }
  if (at(14, 33000, ct)) { step = "thermal camera off (the hand plate closes)"; thermalOff("demo"); }
  if (at(15, 35500, ct)) { step = "red laser + LiDAR"; laserSetForce(LaserForce::On); }
  if (at(16, 38500, ct)) { laserSetForce(LaserForce::Auto); palmSet("c=amber&lvl=4"); }
  if (at(17, 40000, ct)) { step = "open, pause, close"; servosOpenThenClose(); }
  if (at(18, 46000, ct)) { step = "mode SAFE"; modeForce(Mode::Safe); }
  if (at(19, 48000, ct)) { step = "mode FULL"; modeForce(Mode::Flight); palmSet("c=repulsor&lvl=6"); }
}

String demoJson() {
  char b[200];
  snprintf(b, sizeof b, "{\"on\":%d,\"t\":%.1f,\"cycle\":%lu,\"step\":\"%s\",\"volts\":%.2f}", on ? 1 : 0, on && primed ? ((millis() - t0) % DEMO_CYCLE_MS) / 1000.0f : 0.0f, (unsigned long)cycles, step, volts);
  return String(b);
}
#endif
