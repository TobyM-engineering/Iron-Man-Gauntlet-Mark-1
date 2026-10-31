// calib.cpp - the puck's CALIBRATE screen ($C ACT_CAL, 2026-10-21). See calib.h.
#include "calib.h"
#include "config.h"
#include "flex.h"
#include "gesture.h"
#include "imu.h"
#include "fly.h"
#include "s3link.h"
#include <GauntletLink.h>
#include <GauntletBenchIO.h>

enum class St : uint8_t { Idle, Settle, Fingers, Zero, Axis };
static St       st = St::Idle;
static uint8_t  step = 0;
static uint32_t t0 = 0, lastSample = 0, n = 0;
static double   sum[5];
static const char* const STEP_NAME[7] = { "?", "fingers flat", "fingers closed", "thumb closed alone", "arm straight out", "arm hanging down", "arm straight up" };
static constexpr uint8_t STEPS = 6;

bool calibBusy() { return st != St::Idle; }
static void answer(bool ok, const char* why) {
  if (ok) Bench.printf("calibrate: step %u (%s) DONE - saved\n", step, STEP_NAME[step]);
  else    Bench.printf("calibrate: step %u (%s) REFUSED - %s\n", step, step <= STEPS ? STEP_NAME[step] : "?", why);
  s3linkAct(glink::ACT_CAL, ok ? glink::RES_DONE : glink::RES_REFUSED, 0);
  st = St::Idle;
}

void calibStart(uint8_t s) {
  if (st != St::Idle) {   // one step at a time: this press is answered, the running one carries on
    Bench.printf("calibrate: step %u refused - step %u is still running\n", s, step);
    s3linkAct(glink::ACT_CAL, glink::RES_REFUSED, 0);
    return;
  }
  step = s;
  if (s < 1 || s > STEPS) { step = 0; answer(false, "no such step (1-6)"); return; }
  if (flyArmed()) { answer(false, "the drone is armed"); return; }
  st = St::Settle; t0 = millis();
  Bench.printf("calibrate: step %u (%s) - hold still, %u ms then %u ms of readings\n", s, STEP_NAME[s], (unsigned)CAL_SETTLE_MS, (unsigned)CAL_AVG_MS);
}

void calibPoll() {
  if (st == St::Idle) return;
  uint32_t now = millis();
  if (flyArmed()) { answer(false, "the drone armed during the step"); return; }
  switch (st) {
    case St::Settle:
      if (now - t0 < CAL_SETTLE_MS) return;
      if (step <= 3) { for (uint8_t i = 0; i < 5; i++) sum[i] = 0; n = 0; lastSample = 0; t0 = now; st = St::Fingers; return; }
      if (step == 4) {
        if (imuCount() < 2) { answer(false, "an IMU is absent (both are needed for the zero)"); return; }
        imuZeroAsync(CAL_AVG_MS); st = St::Zero; return;
      }
      if (!imuAxisStart(step == 6, CAL_AVG_MS)) { const char* w = ""; imuAxisState(&w); answer(false, w); return; }
      st = St::Axis; return;
    case St::Fingers:
      if (now - lastSample >= FLEX_SAMPLE_MS) { lastSample = now; for (uint8_t i = 0; i < 5; i++) sum[i] += flexMv(i); n++; }
      if (now - t0 < CAL_AVG_MS) return;
      {
        uint16_t mv[5]; for (uint8_t i = 0; i < 5; i++) mv[i] = (uint16_t)(sum[i] / (n ? n : 1) + 0.5);
        const char* why = step == 3 ? gestureCalThumbAlone(mv[0]) : gestureCalPose(step == 2, mv);
        answer(!why[0], why);
      }
      return;
    case St::Zero:
      if (imuZeroRunning()) return;
      if (imuLastZeroCount() >= 2) answer(true, ""); else answer(false, "an IMU sent no samples during the hold");
      return;
    case St::Axis: {
      const char* why = ""; int8_t s = imuAxisState(&why);
      if (s < 0) return;
      answer(s == 1, why);
      return;
    }
    default: st = St::Idle; return;
  }
}
