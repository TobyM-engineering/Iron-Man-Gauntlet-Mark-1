#include "shutdown.h"
#include "config.h"
#include "hal.h"
#include "servos.h"
#include "palm.h"
#include "forearm.h"
#include "pucks.h"
#include "outputs.h"
#include "buzzer.h"
#include "current.h"
#include "s3link.h"
#include "imu.h"
#include "laser.h"
#include "fly.h"
#include <GauntletBenchIO.h>

void goSafe(const char* why);   // Ironman.ino

static uint8_t     phase = 0;
static uint32_t    t0 = 0;
static const char* reason = "";

bool    shutdownActive() { return phase != 0; }
uint8_t shutdownPhase()  { return phase; }

// (Toby, 2026-10-08: "no 5 s continuous beep anywhere") the grace before everything goes off is SILENT - the buzzer's
// only battery sound is the chirp every SHUTDOWN_CHIRP_EVERY_MS once the ESP32 is asleep
void shutdownArm(const char* why) {
  if (phase) return;
  phase = 1; t0 = millis(); reason = why;
  buzzerOff();
  Bench.printf("SHUTDOWN: %s - everything off in %lu s (silent; a recovered sag cancels it), the puck blanked, then the ESP32 asleep, chirping every %lu s. Switch the arm off and charge the pack.\n",
               why, (unsigned long)(SHUTDOWN_GRACE_MS / 1000), (unsigned long)(SHUTDOWN_CHIRP_EVERY_MS / 1000));
}

void shutdownCancel() {
  if (phase != 1) return;
  phase = 0;
  Bench.println("SHUTDOWN cancelled - the pack is back above CRITICAL (a sag under load, not an empty pack)");
}

static void everythingOff() {
  goSafe(reason);                     // drone disarmed, laser off, plates parked, LED groups, outputs (lasers, vibrator, accent), ring and strip off
  laserOff(reason);
  pucksAllOff(reason);
  outputsAllOff(reason);
  palmAlert(false); palmWarning(false);   // the CRITICAL red blink would light the ring again
  palmOff(); forearmOff();
  servosSleep();                      // every channel released, OE high, the PCA9685's oscillator stopped
  currentSleep();
  imuSleep();
  buzzerOff();
  s3linkSetFlag(5, true);             // $S flags bit 5: "blank the screen, the arm is going to sleep"
  Bench.printf("SHUTDOWN: everything off (%s); telling the puck for %lu ms\n", reason, (unsigned long)SHUTDOWN_TELL_MS);
}

void shutdownPoll() {
  if (phase == 0 || phase == 3) return;
  uint32_t now = millis();
  if (phase == 1) {
    if (now - t0 >= SHUTDOWN_GRACE_MS) { everythingOff(); phase = 2; t0 = now; }
    return;
  }
  if (phase == 2 && now - t0 >= SHUTDOWN_TELL_MS) {
    Bench.printf("SHUTDOWN: deep sleep now - a chirp every %lu s until the master switch is off (the regulators, the ELRS module and the puck still drain the pack)\n", (unsigned long)(SHUTDOWN_CHIRP_EVERY_MS / 1000));
    phase = 3;
    Bench.wifiStop();                           // (2026-10-08, WiFi on at boot again) the radio off before the sleep - each wake only chirps, it never starts WiFi
    halDeepSleepFor(SHUTDOWN_CHIRP_EVERY_MS);   // a real build does not come back from this: each timer wake chirps and sleeps again (setup())
  }
}

void shutdownReset() {
  phase = 0;
  s3linkSetFlag(5, false);
  servosWake();
#if HW_FAKE
  halFakeWake();
#endif
}
