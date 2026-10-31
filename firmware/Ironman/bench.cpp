#include "bench.h"
#include "config.h"
#include "hal.h"
#include "palm.h"
#include "forearm.h"
#include "servos.h"
#include "current.h"
#include "power.h"
#include "pucks.h"
#include "outputs.h"
#include "modesw.h"
#include "s3link.h"
#include "buses.h"
#include "flex.h"
#include "imu.h"
#include "lidar.h"
#include "crsf.h"
#include <GauntletBenchIO.h>

static constexpr float BENCH_ABORT_V = BATT_LOW_V;      // the hard floor for the test
static constexpr uint8_t STAGES = 8;

enum class R : uint8_t { Pass, Fail, Skip };
static const char* rname(R r) { return r == R::Pass ? "PASS" : r == R::Fail ? "FAIL" : "SKIP"; }
static R    results[STAGES];
static char notes[STAGES][80];
static bool aborted = false, running = false;
static uint8_t stageNo = 0;
static String report;
static const char* titles[STAGES] = { "I2C scan", "palm chain", "forearm chain", "servo cascade", "LEDs + cascade", "static outputs", "display link", "flight sensors" };

bool   benchTestRunning() { return running; }
String benchTestReport()  { return report; }

static void stage(uint8_t n, const char* title) { stageNo = n; Bench.printf("\n--- stage %u: %s ---\n", n, title); }
static void result(R r, const char* note) {
  results[stageNo - 1] = r;
  strncpy(notes[stageNo - 1], note, sizeof(notes[0]) - 1);
  Bench.printf("stage %u: %s  %s\n", stageNo, rname(r), note);
}

static void everythingOff() {
  servosAbort();
  servosParkAll();
  pucksAllOff("bench test");
  outputsAllOff("bench test");
  palmOff();
  forearmOff();
}

// The hard floor. Called from every loop in the test.
static bool checkAbort() {
  if (aborted) return true;
  powerPoll();
  if (powerState() == PowerState::NoPack) return false;         // USB: no floor to check
  float v = powerVolts();
  if (v < BENCH_ABORT_V) {
    aborted = true;
    everythingOff();
    Bench.printf("\n!! ABORT: pack %.2f V < %.1f V - every servo detached, LEDs and outputs off\n", v, BENCH_ABORT_V);
    return true;
  }
  return false;
}

static bool tick() {
  Bench.poll();
  servosPoll();
  pucksPoll();
  outputsPoll();
  currentSample();
  return !checkAbort();
}

static bool idle(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { if (!tick()) return false; halWdtFeed(); delay(10); }
  return true;
}

static bool runCascade(bool open) {
  if (open) servosOpen(); else servosClose();
  if (!servosBusy()) return false;
  while (servosBusy()) { if (!tick()) return false; halWdtFeed(); delay(5); }
  return true;
}

static void stage1_i2c() {
  stage(1, "I2C scan");
  busesScan();
  bool pca = busAck(1, 0x40);
  char note[80];
  snprintf(note, sizeof note, "PCA %s, INA219 %s, ADS1115 %s", pca ? "yes" : "NO", busAck(1, INA219_ADDR) ? "yes" : "no", busAck(2, ADS1115_ADDR) ? "yes" : "no");
  result(pca ? R::Pass : R::Fail, note);
}

static void stage2_palm() {
  stage(2, "palm chain (19 px)");
  Bench.println("  walking one pixel - count them");
  palmWalkBlocking(150, tick); if (aborted) return;
  const char* names[3] = { "RED", "GREEN", "BLUE" }; uint8_t c[3][3] = { {255,0,0}, {0,255,0}, {0,0,255} };
  for (int i = 0; i < 3; i++) {
    Bench.printf("  all %s (ramped)\n", names[i]);
    palmSolidRamp(c[i][0], c[i][1], c[i][2], tick); if (aborted) return;
    if (!idle(800)) return;
  }
  palmOff();
  result(R::Pass, "ran; confirm 19 px and R,G,B order by eye");
}

static void stage3_forearm() {
  stage(3, "forearm chain");
  Bench.printf("  walking one pixel over %u - the last lit pixel should be the end of the strip\n", forearmCount());
  forearmWalk(40, tick); if (aborted) return;
  const char* names[3] = { "RED", "GREEN", "BLUE" }; uint8_t c[3][3] = { {255,0,0}, {0,255,0}, {0,0,255} };
  for (int i = 0; i < 3; i++) {
    Bench.printf("  all %s (ramped)\n", names[i]);
    forearmSolidRamp(c[i][0], c[i][1], c[i][2], tick); if (aborted) return;
    if (!idle(800)) return;
  }
  forearmOff();
  result(R::Pass, "ran; confirm the dot reaches the last LED and R,G,B order by eye");
}

static void stage4_cascade() {
  stage(4, "servo cascade ch0-5");
  if (!servosReady()) { result(R::Fail, "PCA9685 not ready"); return; }
  if (!currentPresent()) Bench.println("  (no INA219 - current logging skipped)");
  currentLogStart();
  bool ok = runCascade(true) && idle(800) && runCascade(false);
  currentLogStop();
  servosParkAll();
  if (aborted) return;
  char note[80];
  if (currentPresent()) snprintf(note, sizeof note, "open+close done; peak %.2f A", currentPeakA());
  else                  snprintf(note, sizeof note, "open+close done; no current data");
  result(ok ? R::Pass : R::Fail, ok ? note : "cascade did not complete");
}

static void stage5_combined() {
  stage(5, "LEDs at cap + cascade");
  if (!servosReady()) { result(R::Fail, "PCA9685 not ready"); return; }
  Bench.printf("  both chains to white at the cap (%u/255), ramped\n", LED_BRIGHTNESS_CAP);
  palmSolidRamp(255, 255, 255, tick);    if (aborted) return;
  forearmSolidRamp(255, 255, 255, tick); if (aborted) return;
  if (!idle(500)) return;
  currentLogStart();
  bool ok = runCascade(true) && idle(800) && runCascade(false);
  currentLogStop();
  servosParkAll();
  palmOff(); forearmOff();
  if (aborted) return;
  char note[80];
  if (currentPresent()) snprintf(note, sizeof note, "done under load; peak %.2f A", currentPeakA());
  else                  snprintf(note, sizeof note, "done under load; no current data");
  result(ok ? R::Pass : R::Fail, ok ? note : "cascade did not complete");
}

static void stage6_outputs() {
  stage(6, "static outputs, one at a time");
  if (!servosReady()) { result(R::Fail, "PCA9685 not ready"); return; }
  if (!modeAllowsShow()) { result(R::Skip, "mode SAFE - flip the switch to SHOW (or key M) and run again"); return; }
  uint8_t lit = 0;
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) {
    Bench.printf("  puck group %u (%s, ch %u) for 2 s - meter one LED lead: 0.90-1.10 A; its D4184's OUT- to DC- under 0.1 V\n", g + 1, PUCK_GROUP_NAME[g], PCA_CH_PUCK_GROUP[g]);
    if (puckGroupOn(g, "bench")) { lit++; if (!idle(2000)) return; puckGroupOff(g); if (!idle(500)) return; }
    else Bench.printf("  (group %u: %s)\n", g + 1, pucksLastRefusal());
  }
  if (!modeAllowsFull()) Bench.println("  lasers skipped - they need FULL (the switch at its GPIO 39 end); run T there to include them");
  else {
  Bench.println("  red laser 1 s"); laserRedSet(true); if (!idle(1000)) return; laserRedSet(false);
  Bench.println("  green laser 1 s (only with the plates open)"); laserGreenSet(true); if (!idle(1000)) return; laserGreenSet(false);
  }
  Bench.println("  vibrator 500 ms"); vibPulse(500, "bench"); if (!idle(800)) return;
  char note[96]; snprintf(note, sizeof note, "%u of %u puck groups lit (the rest refused - see the log), lasers and vibrator pulsed", lit, PUCK_GROUPS);
  result(lit == PUCK_GROUPS ? R::Pass : R::Fail, note);
}

static void stage7_display() {
  stage(7, "display link");
  uint32_t hb = s3linkHeartbeats();
  Bench.println("  waiting 2.5 s for a heartbeat from the round display");
  if (!idle(2500)) return;
  char note[80];
  if (s3linkUp() && s3linkHeartbeats() > hb) { snprintf(note, sizeof note, "display up, %lu heartbeats", (unsigned long)s3linkHeartbeats()); result(R::Pass, note); }
  else result(R::Skip, "no heartbeat from the display (not fitted, or its UART wires 18/19 are crossed)");
}

static void stage8_flight() {
  stage(8, "flight sensors (reported, not failed)");
  char note[80];
  snprintf(note, sizeof note, "IMUs %u/2, ADS1115 %s, LiDAR %s, ELRS module %s", imuCount(), flexAdsPresent() ? "ok" : "absent", lidarPresent() ? "ok" : "absent", crsfModuleHeard() ? "heard" : "silent");
  Bench.printf("  %s\n", note);
  result(R::Pass, note);
}

void benchTestRun() {
  aborted = false; running = true;
  for (auto& r : results) r = R::Skip;
  for (auto& n : notes) n[0] = 0;
  Bench.println("\n=============== FULL-SYSTEM BENCH TEST ===============");
  Bench.printf("pack %.2f V (%s)  floor %.1f V  LED cap %u/255  INA219 %s  mode %s\n",
               powerVolts(), powerStateName(), BENCH_ABORT_V, LED_BRIGHTNESS_CAP, currentPresent() ? "present" : "absent", modeNowName());
  if (!checkAbort()) {
    stage1_i2c();
    if (!aborted) stage2_palm();
    if (!aborted) stage3_forearm();
    if (!aborted) stage4_cascade();
    if (!aborted) stage5_combined();
    if (!aborted) stage6_outputs();
    if (!aborted) stage7_display();
    if (!aborted) stage8_flight();
  }
  everythingOff();
  report = "";
  Bench.println("\n--------------- summary ---------------");
  for (int i = 0; i < STAGES; i++) {
    char line[120];
    if (aborted && i + 1 > stageNo) snprintf(line, sizeof line, "  %u %-16s  --    not run", i + 1, titles[i]);
    else if (aborted && i + 1 == stageNo) snprintf(line, sizeof line, "  %u %-16s  ABORT battery floor", i + 1, titles[i]);
    else snprintf(line, sizeof line, "  %u %-16s  %s  %s", i + 1, titles[i], rname(results[i]), notes[i]);
    Bench.println(line); report += line; report += "\n";
  }
  if (currentPresent()) Bench.printf("peak current seen: %.2f A\n", currentPeakA());
  Bench.printf("pack now %.2f V. %s\n", powerVolts(), aborted ? "ABORTED - servos detached, LEDs off." : "done - servos released, LEDs off.");
  Bench.println("=======================================================\n");
  running = false;
}
