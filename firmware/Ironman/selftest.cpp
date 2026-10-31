#include "selftest.h"
#if HW_FAKE
#include "config.h"
#include "hal.h"
#include "servos.h"
#include "palm.h"
#include "forearm.h"
#include "power.h"
#include "modesw.h"
#include "buzzer.h"
#include "haptic.h"
#include "pucks.h"
#include "outputs.h"
#include "s3link.h"
#include "current.h"
#include "shutdown.h"
#include "imu.h"
#include "flex.h"
#include "gesture.h"
#include "laser.h"
#include "lidar.h"
#include "crsf.h"
#include "fly.h"
#include "thermal.h"
#include "panel.h"
#include "calib.h"
#include "gesturemap.h"
#include <GauntletBenchIO.h>
#include <GauntletLink.h>
#include <Preferences.h>

void goSafe(const char* why);   // Ironman.ino
void batteryAlarmsPoll();       // Ironman.ino: the FAULT buzz, LAND NOW (the vibrator)

static String report;
static uint16_t passed = 0, failed = 0;
static char note[520];

static void result(bool ok, const char* name) {
  if (ok) passed++; else failed++;
  report += ok ? "PASS  " : "FAIL  ";
  report += name; report += " - "; report += note; report += "\n";
  note[0] = 0;
}
#define NOTE(...) snprintf(note, sizeof note, __VA_ARGS__)

// One loop() iteration of the fake world, then the clock moves. Every poll the real loop runs.
static void tick(uint32_t ms = 5) {
  powerPoll(); modeswPoll(); currentSample(); servosPoll(); pucksPoll(); outputsPoll(); buzzerPoll(); thermalPoll(); s3linkPoll(); palmPoll(); batteryAlarmsPoll();
  imuPoll(); flexPoll(); gesturePoll(); lidarPoll(); laserPoll(); crsfPoll(); flyPoll(); gmapPoll(); calibPoll();   // gmapPoll after flyPoll, as in loop()
  halAdvance(ms);
}
static void run(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 5) tick(5); }
static void runNoFly(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 5) { crsfPoll(); halAdvance(5); } }   // the CRSF task alone: flyPoll would feed the sticks every 20 ms

// ---- the fake world's inputs ----
static void handLevel() { imuFakeInject(0, 1, 0, 0, 0); imuFakeInject(1, 1, 0, 0, 0); }   // both sensors at their step-1 pose: hand level, wrist in line
static void fingers(uint16_t mv) { for (uint8_t i = 1; i <= 4; i++) flexFakeSetMv(i, mv); }
static void thumb(uint16_t mv) { flexFakeSetMv(0, mv); }
static void injectFrame(uint8_t type, const uint8_t* payload, uint8_t n) {
  uint8_t f[64]; if (n > 60) n = 60; f[0] = 0xEA; f[1] = n + 2; f[2] = type; memcpy(f + 3, payload, n); f[3 + n] = crsfCrc8(f + 2, n + 1);
  crsfFakeInject(f, n + 4);
}
static void droneSays(bool linked, const char* mode, float volts, float rollDeg, float pitchDeg) {
  if (linked) { uint8_t ls[10] = { 50, 0, 100, 10, 0, 6, 1, 50, 100, 10 }; injectFrame(0x14, ls, 10); }
  uint8_t fm[8]; strncpy((char*)fm, mode, 7); fm[7] = 0; injectFrame(0x21, fm, strlen(mode) + 1);
  uint16_t v = (uint16_t)(volts * 10 + 0.5f); uint8_t bat[8] = { (uint8_t)(v >> 8), (uint8_t)v, 0, 0, 0, 0, 0, 50 }; injectFrame(0x08, bat, 8);
  int16_t p = (int16_t)(pitchDeg / 57.29578f * 10000), r = (int16_t)(rollDeg / 57.29578f * 10000);
  uint8_t att[6] = { (uint8_t)(p >> 8), (uint8_t)p, (uint8_t)(r >> 8), (uint8_t)r, 0, 0 }; injectFrame(0x1E, att, 6);
}
// the world for a flight: level hand every 20 ms, the drone's telemetry every 200 ms
static void flyWorld(uint32_t ms, bool linked = true, const char* mode = "ANGL*", float roll = 0, float pitch = 0) {
  for (uint32_t t = 0; t < ms; t += 5) {
    if (t % 20 == 0) handLevel();
    if (t % 200 == 0) droneSays(linked, mode, 3.8f, roll, pitch);
    tick(5);
  }
}
// a thumb press: down 80 mV in 30 ms, hold, back up, rest
static void thumbPress(uint16_t flat, uint16_t depth, uint32_t holdMs, uint32_t restMs, bool world) {
  for (int k = 1; k <= 3; k++) { thumb(flat - depth * k / 3); if (world) flyWorld(10); else run(10); }
  if (world) flyWorld(holdMs); else run(holdMs);
  for (int k = 2; k >= 0; k--) { thumb(flat - depth * k / 3); if (world) flyWorld(10); else run(10); }
  if (world) flyWorld(restMs); else run(restMs);
}

// =====================================================================================================
static void t_boot_state() {
  bool oeHigh = halFakeOutput(PIN_PCA_OE) == HIGH, branchLow = halFakeOutput(PIN_ACCENT) == LOW, buzzLow = halFakeOutput(PIN_BUZZER) == (BUZZER_ACTIVE_LOW ? HIGH : LOW) && halFakeTone(PIN_BUZZER) == 0;
  bool servosOff = true; for (uint8_t c = 0; c < SERVO_COUNT; c++) if (servoChannelUs(c)) servosOff = false;
  bool digOff = pcaDigitalOnCount() == 0;
  bool chainsDark = palmLevel() == 0 && palmMilliampsNow() < PALM_COUNT * 1.5f && forearmMilliamps() < FOREARM_COUNT * 1.5f;
  bool wifiAtBoot = NET_WIFI_AT_BOOT;                                        // (Toby, 2026-10-08) WiFi on at boot, one build, no key or switch position
  NOTE("OE %s, accent %s, buzzer %s, servos %s, static outputs %s, chains %s (palm %.0f mA, forearm %.0f mA); WiFi at boot %s", oeHigh ? "high" : "LOW", branchLow ? "low" : "HIGH", buzzLow ? "quiet" : "SOUNDING",
       servosOff ? "released" : "PULSING", digOff ? "off" : "ON", chainsDark ? "dark" : "LIT", palmMilliampsNow(), forearmMilliamps(), wifiAtBoot ? "on" : "OFF");
  result(oeHigh && branchLow && buzzLow && servosOff && digOff && chainsDark && wifiAtBoot, "boot state: every output off, WiFi set to start at boot");
}

static void t_tables() {
  NOTE("%u pins, %u I2C devices, %u PCA channels checked at compile time (static_assert): no double booking, no strap/flash pin, no input-only output", PIN_COUNT, I2C_DEVICE_COUNT, PCA_CHANNEL_COUNT);
  result(pinsValid() && i2cValid() && pcaValid(), "config tables");
}

static void t_led_cap() {
  bool fb = forearmBrightness() <= LED_BRIGHTNESS_CAP && forearmBrightness() > 0;
  palmSet("c=white&lvl=10");
  run(400);
  float ma = palmMilliampsNow();
  float capMa = (PALM_COUNT + PALM_WALK_EXTRA) * 1.0f + PALM_COUNT * 3 * 20.0f * LED_BRIGHTNESS_CAP / 255.0f + 2;   // every pixel's 1 mA idle (the chain test's five extras included) + the lit ones at the cap
  bool pc = ma <= capMa && ma > capMa * 0.8f;
  palmOff();
  NOTE("forearm brightness %u/255 (cap %u); palm white 10/10 = %.0f mA (the cap allows %.0f)", forearmBrightness(), LED_BRIGHTNESS_CAP, ma, capMa);
  result(fb && pc, "LED brightness cap");
}

static void t_servo_envelope() {
  servosPower(true);
  servoWriteUs(0, 3000); uint16_t hi = servoChannelUs(0);
  servoWriteUs(0, 100);  uint16_t lo = servoChannelUs(0);
  servoWriteUs(9, 1500); bool staticRefused = servoChannelUs(9) == 0 && !pcaDigitalState(9);
  servoRelease(0); servosPower(false);
  NOTE("3000 -> %u us, 100 -> %u us (envelope %u-%u); a pulse on a static channel is ignored: %s", hi, lo, SERVO_US_MIN, SERVO_US_MAX, staticRefused ? "yes" : "NO");
  result(hi == SERVO_US_MAX && lo == SERVO_US_MIN && staticRefused, "servo pulse envelope");
}

// a pair is "in step" when the two pulses are mirror images about 1500 us (same start, same travel, same duration, one mirrored)
// a pair in one mirrored step (2026-10-20): both servos the same fraction of their own travel, turning opposite ways - the
// hand plate's pair, tuned on the arm, is no longer symmetric about 1500 us
static bool mirrored(uint8_t chA, uint8_t chB) {
  int ia = -1, ib = -1; for (uint8_t i = 0; i < PLATE_COUNT; i++) { if (PLATES[i].ch == chA) ia = i; if (PLATES[i].ch == chB) ib = i; }
  int a = servoChannelUs(chA), b = servoChannelUs(chB);
  if (ia < 0 || ib < 0 || !a || !b) return false;
  float ca = servoEndUs((uint8_t)ia, false), oa = servoEndUs((uint8_t)ia, true), cb = servoEndUs((uint8_t)ib, false), ob = servoEndUs((uint8_t)ib, true);
  if (oa == ca || ob == cb) return false;
  float fa = (a - ca) / (oa - ca), fb = (b - cb) / (ob - cb);
  return (oa - ca) * (ob - cb) < 0 && fabsf(fa - fb) <= 0.01f && fa > 0.005f;
}
static void t_cascade() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(100);
  servosOpen();
  run(20);                                                      // the sequencer writes every 10 ms
  bool firstHand = servoChannelUs(PCA_CH_PALM_A) != 0 && servoChannelUs(PCA_CH_PLATE1) == 0 && servoChannelUs(PCA_CH_PLATE2) == 0 && servoChannelUs(PCA_CH_PLATE3) == 0;   // (2026-10-20) the hand plate alone first; plate 1 goes with plate 2
  bool handPair = mirrored(PCA_CH_PALM_A, PCA_CH_PALM_B);
  bool oeLive = halFakeOutput(PIN_PCA_OE) == LOW;
  run(PLATES[4].moveMs / 2);                                    // the hand plate past 30 % open (its smoothstep: ~36 % of its time): plate 2 and plate 1 start
  bool secondWave = servoChannelUs(PCA_CH_PLATE2) != 0 && servoChannelUs(PCA_CH_PLATE1) != 0 && servoChannelUs(PCA_CH_PLATE3) == 0 && servoChannelUs(PCA_CH_PLATE4) == 0;   // plate 3's pair waits for plate 2
  run(PLATES[1].moveMs);
  bool third = servoChannelUs(PCA_CH_PLATE3) != 0 && servoChannelUs(PCA_CH_PLATE4) != 0;
  auto onPath = []() { int a = servoChannelUs(PCA_CH_PLATE3), b = servoChannelUs(PCA_CH_PLATE4); return a && b && abs(b - (int)servosFollowerUs((uint16_t)a)) <= 1; };
  bool pair3 = onPath();                                        // (2026-10-16) 3B follows 3A along PLATE3_PATH, not mirrored
  run(PLATES[2].moveMs / 2); bool pair3mid = onPath();          // still on the path half way
  run(PLATES[2].moveMs / 2 + CASCADE_HOLD_MS + 100);
  bool done = !servosBusy() && servosPlatesState() == PlatesState::Open;
  bool released = true; for (uint8_t c = 0; c < SERVO_COUNT; c++) if (servoChannelUs(c)) released = false;
  bool oeOff = halFakeOutput(PIN_PCA_OE) == HIGH;
  servosClose(); run(2500);
  bool closed = servosPlatesState() == PlatesState::Closed && !servosBusy();
  NOTE("open: the hand plate first %s (its pair mirrored in step %s), then plate 2 + plate 1 %s, then plate 3's pair %s (on its path %s, half way %s); done %s, all released %s, OE back high %s; close -> %s",
       firstHand ? "yes" : "NO", handPair ? "yes" : "NO", secondWave ? "yes" : "NO", third ? "yes" : "NO", pair3 ? "yes" : "NO", pair3mid ? "yes" : "NO",
       done ? "yes" : "NO", released ? "yes" : "NO", oeOff ? "yes" : "NO", closed ? "closed" : servosPlatesStateName());
  result(firstHand && handPair && oeLive && secondWave && third && pair3 && pair3mid && done && released && oeOff && closed, "armor cascade: order, the two pairs in one mirrored step, release, OE");
}

static void t_cascade_abort() {
  servosOpen(); run(100);
  goSafe("test");
  bool released = true; for (uint8_t c = 0; c < SERVO_COUNT; c++) if (servoChannelUs(c)) released = false;
  NOTE("goSafe mid-cascade: busy %s, released %s, OE %s, state %s", servosBusy() ? "YES" : "no", released ? "yes" : "NO", halFakeOutput(PIN_PCA_OE) == HIGH ? "high" : "LOW", servosPlatesStateName());
  result(!servosBusy() && released && halFakeOutput(PIN_PCA_OE) == HIGH, "goSafe parks a running cascade");
}

static bool parseLine(const char* line, glink::Parser& p, const char** f, uint8_t& nf);
// (Toby, 2026-10-08) the arm floor makes no sound; no battery state blinks the ring; the puck and the OLED are told ($S).
// (2026-10-20, Toby) LOW and IDLE chirp every BATT_LOW_CHIRP_EVERY_MS.
static uint32_t quietTones = 0, ringBlinks = 0;
static void runQuiet(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 5) { tick(5); if (halFakeTone(PIN_BUZZER)) quietTones++; if (palmMode() == 2) ringBlinks++; } }
static void t_battery_rules() {
  modeForce(Mode::Show);
  powerFakeVolts(8.0f); run(3000);
  bool ok0 = powerState() == PowerState::Ok;
  uint32_t q0 = buzzerPlays(); quietTones = ringBlinks = 0;
  powerFakeVolts(7.4f); runQuiet(3500);
  bool armfloor = powerState() == PowerState::ArmFloor && !powerArmOk() && powerShowOk();
  bool floorQuiet = buzzerPlays() == q0 && quietTones == 0;                    // the arm floor: no sound
  // (2026-10-21) at 7.4 V the start floor scales with the load: the palm 20 mm (3 LEDs, 7.21 V) and the palm 10 mm on top
  // (5, 7.36 V) start, the fingers on top of those (all 7, 7.5 V) do not - for the battery
  bool g1on = puckGroupOn(0, "test"), g2on = puckGroupOn(1, "test"), g3no = !puckGroupOn(2, "test") && pucksRefusedForBattery();
  pucksAllOff("test");
  bool lightStart = g1on && g2on && g3no;
  uint32_t lowPlays0 = buzzerPlays();
  powerFakeVolts(6.9f); runQuiet(3500);
  bool low = powerState() == PowerState::Low && !powerShowOk() && !powerIdleForced();
  glink::Parser lp; const char* lf[glink::MAX_FIELDS]; uint8_t lnf; glink::Status ls;
  bool told = parseLine(s3linkLastStatusLine(), lp, lf, lnf) && glink::statusParse(lf, lnf, ls) == glink::Result::Ok && ls.bstate == 2 && (ls.flags & 1);   // the puck and the OLED: LOW, the warning bit
  uint32_t q1 = buzzerPlays();
  bool refused = !puckGroupOn(0, "test");
  uint32_t q2 = buzzerPlays();                                                // a refused burst may sound its own "refused" - not a battery sound
  powerFakeVolts(6.7f); runQuiet(BATT_LOW_CHIRP_EVERY_MS + 1500);
  bool idle = powerState() == PowerState::Idle && powerIdleForced();
  uint32_t chirps = (q1 - lowPlays0) + (buzzerPlays() - q2);
  bool silent = floorQuiet && chirps >= 2 && ringBlinks == 0;                  // (2026-10-20) silent at the arm floor, a chirp every 5 s at LOW / IDLE, never a ring blink
  powerFakeVolts(6.5f); run(3500);
  bool crit = powerState() == PowerState::Critical && powerCritical();
  powerFakeVolts(6.7f); run(3500);
  bool stillCrit = powerState() == PowerState::Critical;                       // 6.7 < 6.6 + 0.15: hysteresis holds it
  powerFakeVolts(8.0f); run(3500);
  bool back = powerState() == PowerState::Ok;
  powerFakeVolts(5.1f); run(3500);
  bool nopack = powerState() == PowerState::NoPack && powerArmOk();
  powerFakeVolts(8.0f); run(3500);
  servosClose(); run(3000);                                                 // every plate down: the FAULT must refuse an OPEN
  uint32_t plays0 = buzzerPlays();
  powerFakeVolts(3.0f); run(3500);                                          // the divider broken: a FAULT, latched
  bool fault = powerState() == PowerState::Fault && !powerCritical() && powerFaultLatched();
  powerFakeVolts(8.0f); run(3500);
  bool latched = powerState() == PowerState::Fault && !powerArmOk() && !powerShowOk();
  bool blocked = !puckGroupOn(0, "test") && !vibPulse(100, "test") && !thermalOn("test") && !servosMovePlate(1, true) && strstr(servosLastRefusal(), "FAULT");
  uint32_t plays1 = buzzerPlays(); run(BATT_FAULT_BUZZ_EVERY_MS + 300);
  bool buzz = plays1 > plays0 && buzzerPlays() > plays1;
  powerFakeFaultReset(); run(3500);
  bool cleared = powerState() == PowerState::Ok;
  NOTE("8.0 ok %s; 7.4 arm-floor (7.5) %s; 7.4: 3 then 5 LEDs start, all 7 (floor %.1f V) refused for the battery %s; 6.9 LOW %s (the puck told LOW + the warning bit %s, LED burst refused %s); 6.7 IDLE %s; arm floor silent, LOW / IDLE chirping (%lu chirps), no ring blink %s; 6.5 CRITICAL %s; 6.7 still critical %s; 8.0 ok %s; 5.1 no-pack %s; 3.0 FAULT not critical %s, latched at 8.0 again %s (show, vibrator, camera, plate opening refused %s), its buzz again 30 s later %s; reset (a power-off) %s",
       ok0 ? "y" : "N", armfloor ? "y" : "N", LIGHT_FULL_FLOOR_V, lightStart ? "y" : "N", low ? "y" : "N", told ? "y" : "N", refused ? "y" : "N", idle ? "y" : "N", (unsigned long)chirps, silent ? "y" : "N", crit ? "y" : "N", stillCrit ? "y" : "N", back ? "y" : "N", nopack ? "y" : "N",
       fault ? "y" : "N", latched ? "y" : "N", blocked ? "y" : "N", buzz ? "y" : "N", cleared ? "y" : "N");
  result(ok0 && armfloor && lightStart && low && told && refused && idle && silent && crit && stillCrit && back && nopack && fault && latched && blocked && buzz && cleared, "battery rules 7.5 / 7.0 / 6.8 / 6.6 + hysteresis, a chirp every 5 s from LOW, the FAULT latched");
}

// The switch (Toby, 2026-10-08): the MIDDLE = both high = SAFE, the GPIO 36 end = A low = SHOW, the GPIO 39 end = B low =
// FULL, both low (impossible on the switch) = SAFE; a 10 ms glitch (the ADC1 errata) is ignored; a broken wire to either
// end or a lost common reads both high = SAFE, never a live mode.
static void t_mode_debounce() {
  modeFollowSwitch();
  halFakeSetInput(PIN_MODE_A, HIGH); halFakeSetInput(PIN_MODE_B, HIGH); run(200);
  bool safe = modeNow() == Mode::Safe;
  halFakeSetInput(PIN_MODE_A, LOW); run(100);
  bool show = modeNow() == Mode::Show;
  halFakeSetInput(PIN_MODE_A, HIGH); run(10); halFakeSetInput(PIN_MODE_A, LOW); run(100);   // a 10 ms glitch
  bool glitchIgnored = modeNow() == Mode::Show;
  halFakeSetInput(PIN_MODE_A, HIGH); run(100);                           // SHOW, the 36 wire breaks (A floats high on its pull-up)
  bool brokenA = modeNow() == Mode::Safe;
  halFakeSetInput(PIN_MODE_B, LOW); run(100);
  bool full = modeNow() == Mode::Flight && strcmp(modeNowName(), "FULL") == 0;
  halFakeSetInput(PIN_MODE_B, HIGH); run(100);                           // FULL, the 39 wire breaks (or the common is lost)
  bool brokenB = modeNow() == Mode::Safe;
  halFakeSetInput(PIN_MODE_A, LOW); halFakeSetInput(PIN_MODE_B, LOW); run(100);
  bool bothLow = modeNow() == Mode::Safe;
  halFakeSetInput(PIN_MODE_B, HIGH); run(100);
  bool show2 = modeNow() == Mode::Show;
  halFakeSetInput(PIN_MODE_A, HIGH); run(100);
  bool safe2 = modeNow() == Mode::Safe;
  NOTE("the middle (both high) = SAFE %s; the 36 end (A low) = SHOW %s; a 10 ms high ignored %s; SHOW with the 36 wire broken = SAFE %s; the 39 end (B low) = FULL %s; FULL with the 39 wire broken / the common lost = SAFE %s; both low = SAFE %s; SHOW again %s; the middle SAFE %s",
       safe ? "y" : "N", show ? "y" : "N", glitchIgnored ? "y" : "N", brokenA ? "y" : "N", full ? "y" : "N", brokenB ? "y" : "N", bothLow ? "y" : "N", show2 ? "y" : "N", safe2 ? "y" : "N");
  result(safe && show && glitchIgnored && brokenA && full && brokenB && bothLow && show2 && safe2, "mode switch: middle SAFE / 36 SHOW / 39 FULL, a broken wire SAFE, both low SAFE, debounce");
}

// Power-up stays SAFE until the switch has been in the middle (SAFE) once: switching on in SHOW or FULL never goes live.
static void t_mode_powerup() {
  halFakeSetInput(PIN_MODE_A, LOW); halFakeSetInput(PIN_MODE_B, HIGH);    // switched on at the 36 end (SHOW)
  modeFakePowerUp(); run(200);
  bool held1 = modeNow() == Mode::Safe && modeWaitingForSafe();
  halFakeSetInput(PIN_MODE_A, HIGH); halFakeSetInput(PIN_MODE_B, LOW); run(200);   // on to FULL (too quick through the middle to count): still SAFE
  bool held2 = modeNow() == Mode::Safe && modeWaitingForSafe();
  halFakeSetInput(PIN_MODE_B, HIGH); run(200);                           // the middle once
  bool seen = modeNow() == Mode::Safe && !modeWaitingForSafe();
  halFakeSetInput(PIN_MODE_A, LOW); run(200);
  bool live = modeNow() == Mode::Show;
  halFakeSetInput(PIN_MODE_A, HIGH); halFakeSetInput(PIN_MODE_B, LOW); run(200);
  bool full = modeNow() == Mode::Flight;
  halFakeSetInput(PIN_MODE_B, HIGH); run(200);
  NOTE("switched on in SHOW: held SAFE %s; moved to FULL: still SAFE %s; the middle seen %s; then SHOW live %s, FULL live %s", held1 ? "y" : "N", held2 ? "y" : "N", seen ? "y" : "N", live ? "y" : "N", full ? "y" : "N");
  result(held1 && held2 && seen && live && full, "power-up holds SAFE until the switch has been in SAFE (the middle) once");
}

static void t_safe_mode_refuses() {
  halFakeSetInput(PIN_MODE_A, HIGH); halFakeSetInput(PIN_MODE_B, HIGH); modeFollowSwitch(); run(100);   // the middle = SAFE
  bool p = !puckGroupOn(0, "test"), g = !laserGreenSet(true), r = !laserRedSet(true), v = !vibPulse(100, "test"), t = !thermalOn("test");
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(100);
  bool rShow = !laserRedSet(true), gShow = !laserGreenSet(true), vShow = vibPulse(100, "test"); run(200);
  modeForce(Mode::Safe); run(100);
  NOTE("mode SAFE: LED group %s, green laser %s, red laser %s, vibrator %s, thermal camera %s all refused; SHOW: red %s and green %s refused (they need FULL), vibrator allowed %s",
       p ? "y" : "N", g ? "y" : "N", r ? "y" : "N", v ? "y" : "N", t ? "y" : "N", rShow ? "y" : "N", gShow ? "y" : "N", vShow ? "y" : "N");
  result(p && g && r && v && t && rShow && gShow && vShow, "SAFE refuses every output, SHOW refuses the lasers");
}

// The INA219's stall check (servos.cpp): a healthy cascade is left alone, a servo pressing on a stop is caught
// while holding, a pair working against each other is caught while moving.
static void t_servo_stall() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); palmOff(); run(500);
  uint32_t s0 = servosStallCount();
  currentFakeA(0.4f); servosClose(); uint32_t t0 = millis();          // healthy: under both limits all the way
  while (servosBusy() && millis() - t0 < 8000) run(20);
  bool healthy = !servosBusy() && servosStallCount() == s0 && servosPlatesState() == PlatesState::Closed;
  currentFakeA(1.0f); servosOpen(); t0 = millis();                    // fine while moving (< 2.5 A), a stall once every move has landed
  while (servosBusy() && millis() - t0 < 8000) run(20);
  bool holdStall = servosStallCount() == s0 + 1 && strstr(servosLastStall(), "holding") != nullptr && servosPlatesState() == PlatesState::Open;
  currentFakeA(3.0f); servosClose(); run(SERVO_STALL_MS + 150);       // over the moving limit: released within a sample of SERVO_STALL_MS
  bool moveStall = !servosBusy() && servosStallCount() == s0 + 2 && strstr(servosLastStall(), "moving") != nullptr && servosPlatesState() == PlatesState::Unknown;
  currentFakeA(0.0f);
  NOTE("0.4 A cascade left alone %s; 1.0 A caught while holding %s; 3.0 A caught while moving %s (last: %s)", healthy ? "y" : "N", holdStall ? "y" : "N", moveStall ? "y" : "N", servosLastStall());
  result(healthy && holdStall && moveStall, "servo stall detection on the INA219 (holding / moving)");
}

static void t_pucks() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C); run(3000);
  bool on = puckGroupOn(0, "test");
  bool branch = pucksBranchOn(), ch = pcaDigitalState(PCA_CH_PUCK_GROUP[0]), oe = halFakeOutput(PIN_PCA_OE) == LOW;
  // the whole group and nothing else: pucks 1-3 lit, the other two groups' channels low
  bool group = puckIsOn(0) && puckIsOn(1) && puckIsOn(2) && !puckIsOn(3) && !puckIsOn(5) && pucksOnCount() == 3
               && !pcaDigitalState(PCA_CH_PUCK_GROUP[1]) && !pcaDigitalState(PCA_CH_PUCK_GROUP[2]);
  run(PUCK_ON_MAX_MS_PALM + 200);                                          // the heat model (or the backstop) ends it
  bool timedOff = !puckGroupIsOn(0) && !pcaDigitalState(PCA_CH_PUCK_GROUP[0]) && !pucksBranchOn();
  bool cooling = !puckGroupOn(0, "test");
  run(PUCK_OFF_MIN_MS_PALM); pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C);   // rested, the model cold: the rest rule alone
  bool again = puckGroupOn(0, "test"); puckGroupOff(0);
  // the duty limit on the finger group (2026-10-20: PUCK_DUTY_MAX of the last window, counting the time already on):
  // 50 s bursts with the shortest rests - the third in two minutes is over 75 %
  const uint32_t dutyBurstMs = 50000; uint8_t fg = 2; bool dutyRefused = false; uint8_t bursts = 0;
  for (int k = 0; k < 4; k++) { pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C); if (puckGroupOn(fg, "duty")) {   // the model kept cold: the duty rule alone (t_plate_cap tests the heat)
                                 bursts++; run(dutyBurstMs); puckGroupOff(fg); run(PUCK_OFF_MIN_MS_FINGER + 100); } else { dutyRefused = true; break; } }
  // refused during a cascade
  servosOpen(); tick(); bool duringCascade = !puckGroupOn(1, "test"); run(3000);
  NOTE("group 1 on %s (pucks 1-3 only %s, branch %s, ch%u %s, OE %s); off by itself %s; cooling refused %s; rested and cold again %s; finger group duty: %u bursts of 50 s then refused %s; refused during a cascade %s",
       on ? "y" : "N", group ? "y" : "N", branch ? "y" : "N", PCA_CH_PUCK_GROUP[0], ch ? "y" : "N", oe ? "y" : "N", timedOff ? "y" : "N", cooling ? "y" : "N", again ? "y" : "N", bursts, dutyRefused ? "y" : "N", duringCascade ? "y" : "N");
  result(on && group && branch && ch && oe && timedOff && cooling && again && dutyRefused && bursts == 2 && duringCascade, "BuckPuck group burst, rest, duty, cascade lock-out");
  pucksAllOff("test");
}

static void t_outputs() {
  modeForce(Mode::Flight); run(100);                   // the green laser needs FULL (t_safe_mode_refuses has SHOW refusing it)
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(3500); }         // the plates shut first
  bool gStarted = laserGreenSet(true); run(40);
  bool gWaits = !laserGreenOn() && servosMovingPlate() == LASER_GREEN_PLATE && !pcaDigitalState(PCA_CH_LASER_GREEN);   // plate 1 opens first
  run(PLATES[0].moveMs + laserSettleMs() + 200);        // it lands, then the laser waits for the plate to get there
  bool gOk = laserGreenOn() && pcaDigitalState(PCA_CH_LASER_GREEN);
  laserGreenSet(false); run(PLATES[0].moveMs + CASCADE_HOLD_MS + 300);
  bool gClosed = !laserGreenOn() && !pcaDigitalState(PCA_CH_LASER_GREEN) && servoPosPermille(0) <= 50;
  bool v = vibPulse(300, "test") && hapticOn(); run(400); bool vOff = !vibOn() && !hapticOn();
  bool oeOff = halFakeOutput(PIN_PCA_OE) == HIGH;
  NOTE("green laser: plate 1 opens first %s, then it fires %s, off and plate 1 closed again %s; vibrator 300 ms on %s then off %s; OE released after %s", gWaits ? "y" : "N", gOk ? "y" : "N", gClosed ? "y" : "N", v ? "y" : "N", vOff ? "y" : "N", oeOff ? "y" : "N");
  result(gStarted && gWaits && gOk && gClosed && v && vOff && oeOff, "green laser opens plate 1 first, vibrator timer, OE release");
}

static bool parseLine(const char* line, glink::Parser& p, const char** f, uint8_t& nf) {
  bool got = false; for (const char* c = line; *c; c++) if (p.feed(*c)) got = true;
  nf = got ? p.fields(f, glink::MAX_FIELDS) : 0;
  return got;
}

static void t_link_protocol() {
  glink::Status s; s.seq = 42; s.mode = 2; s.mv = 7860; s.bstate = 1; s.plates = 5; s.ringi = 3; s.ringlvl = 7; s.armed = 1; s.lq = 99; s.cv100 = 381; strcpy(s.fcmode, "ANGL*"); s.lidar = 1234; s.laser = 1; s.pucks = 0x87; s.flags = 5; s.page = 2;
  char body[glink::MAX_LINE], line[glink::MAX_LINE]; glink::statusBody(body, sizeof body, s); size_t n = glink::frame(line, sizeof line, body);
  glink::Parser p; const char* f[glink::MAX_FIELDS]; uint8_t nf; bool got = parseLine(line, p, f, nf);
  glink::Status r; bool parsed = got && glink::statusParse(f, nf, r) == glink::Result::Ok;
  bool same = parsed && r.seq == 42 && r.mode == 2 && r.mv == 7860 && r.plates == 5 && r.ringlvl == 7 && r.armed == 1 && r.lq == 99 && r.cv100 == 381 && strcmp(r.fcmode, "ANGL*") == 0 && r.lidar == 1234 && r.pucks == 0x87 && r.page == 2;
  line[5] ^= 0x01; glink::Parser p2; bool got2 = false; for (size_t i = 0; i < n; i++) if (p2.feed(line[i])) got2 = true;   // one bit wrong: dropped
  // a v1 line (no version field, two-digit CRC8): recognised, counted, never accepted
  const char* v1body = "S,1,1,8000,0,2,0,5,0,0,0,-,0,0,0,0,0"; uint8_t c8 = 0; for (const char* c = v1body; *c; c++) c8 = glink::crc8(c8, (uint8_t)*c);
  char v1[96]; snprintf(v1, sizeof v1, "$%s*%02X\n", v1body, c8); glink::Parser p3; const char* f3[glink::MAX_FIELDS]; uint8_t n3; bool got3 = parseLine(v1, p3, f3, n3);
  // another version: the CRC is good, the frame is refused by its version, never read field by field
  String b9 = String(body); int lastComma = b9.lastIndexOf(','); b9 = b9.substring(0, lastComma + 1) + "9";
  char l9[glink::MAX_LINE]; glink::frame(l9, sizeof l9, b9.c_str()); glink::Parser p4; const char* f4[glink::MAX_FIELDS]; uint8_t n4; bool got4 = parseLine(l9, p4, f4, n4);
  bool v9 = got4 && glink::statusParse(f4, n4, r) == glink::Result::BadVersion;
  NOTE("v%u round trip %s (%u fields), one bit wrong dropped %s, a v1 line counted as v1 and refused %s, a v9 frame refused by its version %s", glink::VERSION, same ? "ok" : "MISMATCH", nf, !got2 && p2.bad == 1 ? "y" : "N", !got3 && p3.v1 == 1 ? "y" : "N", v9 ? "y" : "N");
  result(same && !got2 && p2.bad == 1 && !got3 && p3.v1 == 1 && v9, "puck link frames: CRC-16, version, v1 refused");
}

static void heartbeatIn(uint16_t seq, int version) {
  char body[80], line[glink::MAX_LINE];
  if (version == glink::VERSION) { glink::Heartbeat h; h.seq = seq; h.uptimeS = 12; h.page = 1; h.mlx = 1; h.oled = 1; h.fps = 30; h.heap = 180000; glink::heartbeatBody(body, sizeof body, h); }
  else snprintf(body, sizeof body, "H,%u,12,1,1,1,30,180000,%d", seq, version);
  glink::frame(line, sizeof line, body); s3linkInject(line);
}

static void t_s3link() {
  uint32_t sent0 = s3linkStatusSent(), live0 = s3linkLiveSent();
  run(1000);
  uint32_t perSec = s3linkStatusSent() - sent0, livePerSec = s3linkLiveSent() - live0;
  heartbeatIn(1, glink::VERSION);
  bool up = s3linkUp() && s3linkTheirVersion() == glink::VERSION;
  uint32_t touches0 = s3linkTouches();
  char body[64], line[glink::MAX_LINE]; snprintf(body, sizeof body, "T,2,UP,120,40"); glink::frame(line, sizeof line, body); palmSet("c=white&lvl=5"); run(300); s3linkInject(line); run(300);
  bool touched = s3linkTouches() == touches0 + 1 && palmLevel() == 5;    // counted, and nothing on the arm (touch is view-only)
  snprintf(body, sizeof body, "T,3,UP,500,40"); glink::frame(line, sizeof line, body); s3linkInject(line);   // x out of range: refused
  bool badTouch = s3linkTouches() == touches0 + 1;
  palmOff();
  run(S3_HEARTBEAT_LOST_MS + 500);
  uint32_t sentDown = s3linkStatusSent(); run(500);
  bool down = !s3linkUp() && s3linkStatusSent() > sentDown;              // down, and the frames keep going out
  uint32_t vr0 = s3linkVersionRejects(); heartbeatIn(2, 3);
  bool v3 = !s3linkUp() && s3linkTheirVersion() == 3 && s3linkVersionRejects() == vr0 + 1;
  heartbeatIn(3, glink::VERSION); bool back = s3linkUp();
  const char* last = s3linkLastStatusLine(); bool shape = last[0] == '$' && last[1] == 'S' && strchr(last, '*');
  NOTE("%lu $S and %lu $G in 1 s (want 9-11 / 19-21), v%u heartbeat -> up %s, touch UP counted, ring untouched %s, touch x=500 refused %s, 3.5 s silence -> down with frames still going out %s, a v3 heartbeat refused %s, v%u again -> up %s, shape %s",
       (unsigned long)perSec, (unsigned long)livePerSec, glink::VERSION, up ? "y" : "N", touched ? "y" : "N", badTouch ? "y" : "N", down ? "y" : "N", v3 ? "y" : "N", glink::VERSION, back ? "y" : "N", shape ? "ok" : "BAD");
  result(perSec >= 9 && perSec <= 11 && livePerSec >= 19 && livePerSec <= 21 && up && touched && badTouch && down && v3 && back && shape, "puck link: 10 Hz $S, 20 Hz $G, heartbeat + version, touch, link down");
}

// The $G the arm actually sent, parsed back with the puck's own parser, against the modules' state.
static void t_link_live() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(200);
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(2500); }
  servosMovePlate(1, true); run(PLATES[0].moveMs + CASCADE_HOLD_MS + 100);
  palmSet("c=red&lvl=5"); run(400);
  accentSet(true);
  gestureSet("tf=800&tc=600&fl=1200,1200,1200,1200&fx=600,600,600,600");
  thumb(780); flexFakeSetMv(1, 900); flexFakeSetMv(2, 1100); flexFakeSetMv(3, 700); flexFakeSetMv(4, 1200);
  run(800);                                                     // the four ADS1115 fingers start from 0 here (the finger tests come later): ~30 updates each to settle
  for (int k = 0; k < 10; k++) { imuFakeInject(0, 0.966f, 0, 0, 0.259f); imuFakeInject(1, 0.940f, 0, 0.342f, 0); run(20); }
  run(200);
  glink::Parser p; const char* f[glink::MAX_FIELDS]; uint8_t nf; glink::Live g;
  bool got = parseLine(s3linkLastLiveLine(), p, f, nf);
  glink::Result res = got ? glink::liveParse(f, nf, g) : glink::Result::BadFields;
  bool ok = res == glink::Result::Ok;
  bool servos = ok && g.servo[0] == 1000 && g.servo[1] == 0 && g.servo[2] == 0 && g.servo[4] == 0 && g.plates == (uint8_t)PlatesState::Mixed && g.drv == 0;
  bool ring = ok && g.ringset == 0xFF0000 && g.ringlvl == 5 && g.ring == palmShownRgb() && g.ringmode == 0;
  bool misc = ok && g.accent == 1 && g.mode == 1 && abs((int)g.mv - (int)(powerVolts() * 1000)) <= 2 && g.bstate == 0 && g.lasers == 0 && g.groups == 0 && g.ma >= 0;
  bool ev = ok && g.gev == glink::GEV_PLATE;
  bool imu = true, fingersOk = true;
  { float fq[4], hq[4]; uint8_t bits = imuSegPair(fq, hq);
    imu = ok && (g.imu & 3) == 3 && (bits & 3) == 3;
    for (int i = 0; i < 4; i++) if (abs(g.fq[i] - glink::q10k(fq[i])) > 1 || abs(g.hq[i] - glink::q10k(hq[i])) > 1) imu = false; }
  for (uint8_t i = 0; i < 5; i++) { float c = gestureFingerCurl(i); int16_t want = isnan(c) ? -1 : (int16_t)lroundf((c < 0 ? 0 : c > 1 ? 1 : c) * 1000); if (!ok || abs(g.curl[i] - want) > 5 || abs((int)g.fmv[i] - (int)flexMv(i)) > 2) fingersOk = false; }
  if (!ok || g.curl[1] <= 0 || g.curl[1] >= 1000 || g.curl[4] < 0 || g.curl[4] > 5) fingersOk = false;     // index part way, pinky about flat (the smoothed mV climbs to its flat value from below and truncates: 1 per mille is "flat")
  accentSet(false); servosMovePlate(1, false); run(PLATES[0].moveMs + CASCADE_HOLD_MS + 100); palmOff();
  NOTE("$G %s (%u fields, %s): servos %s (plate 1 %d, plate 2 %d, state %u), ring %s (%06lX shown %06lX lvl %u), accent/mode/battery %s, event 'plate alone' %s, quaternions = the arm's segments %s, curls + mV = gesture/flex %s (curls %d %d %d %d %d, mV %u %u %u %u %u)",
       got ? "parsed" : "NOT PARSED", nf, glink::resultName(res), servos ? "y" : "N", g.servo[0], g.servo[1], g.plates, ring ? "y" : "N", (unsigned long)g.ringset, (unsigned long)g.ring, g.ringlvl, misc ? "y" : "N", ev ? "y" : "N", imu ? "y" : "N", fingersOk ? "y" : "N", g.curl[0], g.curl[1], g.curl[2], g.curl[3], g.curl[4], g.fmv[0], g.fmv[1], g.fmv[2], g.fmv[3], g.fmv[4]);
  result(ok && servos && ring && misc && ev && imu && fingersOk, "puck link $G = the arm's state, field by field");
}

// The link can never stall the arm: frames far faster than the line, garbage on RX, no puck at all.
static void t_link_nonblocking() {
  uint32_t maxUs = 0, drops = 0, accepted = 0; bool uart = s3linkUartOn();
  if (uart) { s3linkFloodTest(200, maxUs, drops, accepted); delay(200); }   // then the ring drains by itself (1 KB in 89 ms)
  bool flood = !uart || (maxUs < 2000 && drops > 0 && accepted >= 2 && accepted + drops == 200);
  // 20 000 bytes of garbage (with '$', '*' and newlines in it) through the parser, S3_RX_MAX_PER_POLL at a time
  uint32_t good0 = s3linkParser().good, hb0 = s3linkHeartbeats(), t0 = 0, worst = 0; uint32_t seed = 12345;
  char chunk[S3_RX_MAX_PER_POLL + 1];
  for (int c = 0; c < 20000 / S3_RX_MAX_PER_POLL; c++) {
    for (uint16_t i = 0; i < S3_RX_MAX_PER_POLL; i++) { seed = seed * 1103515245u + 12345u; uint8_t b = (seed >> 16) & 0xFF; chunk[i] = b ? (char)b : '$'; }
    chunk[S3_RX_MAX_PER_POLL] = 0;
    t0 = micros(); s3linkInject(chunk); uint32_t us = micros() - t0; if (us > worst) worst = us;
  }
  bool garbage = s3linkParser().good == good0 && s3linkHeartbeats() == hb0 && worst < 5000;
  heartbeatIn(9, glink::VERSION); bool recovers = s3linkUp();             // a good frame right after the garbage is read
  // no puck, in REAL time on the REAL UART: 3 s of polling at the loop's pace, the frames go out on the wire at their
  // rates (nothing comes back unless a puck is plugged in) - the slowest poll, the frames sent, nothing dropped
  s3linkTestMode(false);
  uint32_t live0 = s3linkLiveSent(), sent0 = s3linkStatusSent(), drop0 = s3linkTxDrops(), linkMax = 0, r0 = millis();
  while (millis() - r0 < 3000) { uint32_t a = micros(); s3linkPoll(); uint32_t us = micros() - a; if (us > linkMax) linkMax = us; delay(1); }
  s3linkTestMode(true);
  uint32_t liveN = s3linkLiveSent() - live0, sentN = s3linkStatusSent() - sent0, dropN = s3linkTxDrops() - drop0;
  bool noPuck = liveN >= 57 && liveN <= 61 && sentN >= 28 && sentN <= 31 && dropN == 0 && linkMax < 2000;
  NOTE("UART flood: %s (200 widest $G back to back: %lu accepted, %lu dropped, slowest write %lu us); 20 000 garbage bytes: 0 accepted %s, slowest %u-byte poll %lu us; a heartbeat right after is read %s; 3 s real time on the %s: %lu $G + %lu $S out, %lu dropped, slowest poll %lu us",
       uart ? "ran" : "SKIPPED (no UART on this board)", (unsigned long)accepted, (unsigned long)drops, (unsigned long)maxUs, garbage ? "y" : "N", S3_RX_MAX_PER_POLL, (unsigned long)worst, recovers ? "y" : "N",
       uart ? "real UART" : "link (no UART here)", (unsigned long)liveN, (unsigned long)sentN, (unsigned long)dropN, (unsigned long)linkMax);
  result(flood && garbage && recovers && noPuck, "puck link never blocks (TX full = drop, RX garbage bounded, no puck = nothing)");
}

// One plate alone, a pair together, and open-pause-close without a delay() in the loop.
static void t_plate_alone() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(100);
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(2500); }
  uint32_t m0 = servosPlateMoves();
  bool badRefused = !servosMovePlate(0, true) && !servosMovePlate(PLATE_GROUPS + 1, true);
  bool started = servosMovePlate(1, true); run(20);
  bool onlyOne = servoChannelUs(PCA_CH_PLATE1) != 0 && servoChannelUs(PCA_CH_PLATE2) == 0 && servoChannelUs(PCA_CH_PLATE3) == 0 && servoChannelUs(PCA_CH_PALM_A) == 0 && servosMovingPlate() == 1;
  bool busyRefused = !servosMovePlate(2, true);
  run(PLATES[0].moveMs + CASCADE_HOLD_MS + 100);
  bool alone = !servosBusy() && servoPosPermille(0) == 1000 && servoPosPermille(1) == 0 && servosPlatesState() == PlatesState::Mixed && servosPlateMoves() == m0 + 1 && servoChannelUs(PCA_CH_PLATE1) == 0;
  servosMovePlate(4, true); run(20);                                     // the hand plate: its pair together (plate 3 needs the hand plate and plate 2 first - t_plate_deps)
  bool pair = servoChannelUs(PCA_CH_PALM_A) != 0 && servoChannelUs(PCA_CH_PALM_B) != 0 && servoChannelUs(PCA_CH_PLATE2) == 0 && mirrored(PCA_CH_PALM_A, PCA_CH_PALM_B);
  run(PLATES[4].moveMs + CASCADE_HOLD_MS + 100);
  servosClose(); run(2500);
  bool closed = servosPlatesState() == PlatesState::Closed;
  // open-pause-close: every tick stays short through the pause (it was a delay(1500) inside servosPoll)
  servosOpenThenClose(); uint32_t tickMax = 0; bool sawPause = false;
  for (int k = 0; k < 1400 && (servosBusy() || k < 10); k++) { uint32_t a = micros(); tick(5); uint32_t us = micros() - a; if (us > tickMax) tickMax = us; if (servosBusy() && servosPlatesState() == PlatesState::Open) sawPause = true; }
  bool opc = sawPause && !servosBusy() && servosPlatesState() == PlatesState::Closed && tickMax < 300000;   // the console's own prints (UART0, blocking) cost a few ms a line; the old pause was a 1 500 000 us delay()
  NOTE("bad plate numbers refused %s; plate 1 alone: only ch%u driven %s, a second move refused while busy %s, lands open, others closed, state mixed %s; the hand plate: both servos together, mirrored %s; close -> closed %s; open-pause-close with no blocking (slowest loop %lu us) %s",
       badRefused ? "y" : "N", PCA_CH_PLATE1, onlyOne ? "y" : "N", busyRefused ? "y" : "N", alone ? "y" : "N", pair ? "y" : "N", closed ? "y" : "N", (unsigned long)tickMax, opc ? "y" : "N");
  result(badRefused && started && onlyOne && busyRefused && alone && pair && closed && opc, "one plate alone, a pair together, open-pause-close never blocks");
}

static void t_buzzer() {
  buzzerPlay(Buzz::Arm); tick();
  bool sounding = halFakeTone(PIN_BUZZER) == 880 && buzzerSounding();
  run(500); bool quiet = halFakeTone(PIN_BUZZER) == 0 && !buzzerSounding();
  NOTE("arm pattern starts at 880 Hz %s and is silent after 500 ms %s", sounding ? "y" : "N", quiet ? "y" : "N");
  result(sounding && quiet, "buzzer patterns");
}

static void t_crsf_frame() {
  crsfSafe();
  uint8_t buf[26]; crsfBuildRcFrame(buf);
  uint16_t ch[16]; uint32_t bit = 0; const uint8_t* p = buf + 3;
  for (uint8_t i = 0; i < 16; i++) { uint16_t v = 0; for (uint8_t b = 0; b < 11; b++) { if (p[bit >> 3] & (1u << (bit & 7))) v |= (1u << b); bit++; } ch[i] = v; }
  bool hdr = buf[0] == 0xC8 && buf[1] == 24 && buf[2] == 0x16, crc = crsfCrc8(buf + 2, 23) == buf[25];
  bool safe = ch[0] == CRSF_MID && ch[1] == CRSF_MID && ch[2] == CRSF_MIN && ch[3] == CRSF_MID && ch[4] == CRSF_MIN;
  crsfSetSticks(1200, 1300, 1000, 900); crsfBuildRcFrame(buf);
  bit = 0; for (uint8_t i = 0; i < 16; i++) { uint16_t v = 0; for (uint8_t b = 0; b < 11; b++) { if (p[bit >> 3] & (1u << (bit & 7))) v |= (1u << b); bit++; } ch[i] = v; }
  bool sticks = ch[0] == 1200 && ch[1] == 1300 && ch[2] == 1000 && ch[3] == 900;
  runNoFly(CRSF_FEED_STALE_MS + 100);                              // nobody feeds the sticks: the task's watchdog
  bool wd = crsfChannel(2) == CRSF_MIN && crsfWatchdogTripped();
  crsfSafe();
  NOTE("header %s, CRC %s, safe frame = centred/throttle min/AUX low %s, sticks packed %s, watchdog drops the throttle after %lu ms %s", hdr ? "ok" : "BAD", crc ? "ok" : "BAD", safe ? "y" : "N", sticks ? "y" : "N", (unsigned long)CRSF_FEED_STALE_MS, wd ? "y" : "N");
  result(hdr && crc && safe && sticks && wd, "CRSF RC frame packing, CRC, watchdog");
}

static void t_telemetry_parse() {
  droneSays(true, "ANGL*", 3.8f, 2.0f, -3.0f); run(50);
  float r, p, y; uint32_t age; bool att = crsfAttitude(r, p, y, age) && fabsf(r - 2.0f) < 0.1f && fabsf(p + 3.0f) < 0.1f;
  bool link = crsfLinkUp() && crsfLQ() == 100, mode = crsfCraftReady() && !crsfCraftBlocked(), v = fabsf(crsfCraftVolts() - 3.8f) < 0.05f;
  droneSays(true, "ANGL!", 3.8f, 0, 0); run(50); bool blocked = crsfCraftBlocked();
  run(CRSF_LINKSTATS_LOST_MS + 500); bool lost = !crsfLinkUp();
  NOTE("link LQ 100 %s, mode ANGL* ready %s, 3.8 V %s, attitude roll 2 pitch -3 %s, ANGL! blocked %s, no statistics 2.5 s -> link lost %s", link ? "y" : "N", mode ? "y" : "N", v ? "y" : "N", att ? "y" : "N", blocked ? "y" : "N", lost ? "y" : "N");
  result(link && mode && v && att && blocked && lost, "CRSF telemetry parsing + link-statistics timeout");
}

static void t_power_lock() {
  uint8_t idx0 = crsfPowerMaxIdx();                               // before the module has introduced its options: the fallback
  // the module's SETTINGS_ENTRY for "Max Power" (TEXT_SELECTION, type 9): dest, origin, idx, chunks, parent, type, name\0, options\0, value, min, max, default, units\0
  uint8_t p[64]; uint8_t n = 0;
  p[n++] = 0xEA; p[n++] = 0xEE; p[n++] = 7; p[n++] = 0; p[n++] = 0; p[n++] = 9;
  const char* name = "Max Power"; memcpy(p + n, name, strlen(name) + 1); n += strlen(name) + 1;
  const char* opts = "10;25;50;100;250;500"; memcpy(p + n, opts, strlen(opts) + 1); n += strlen(opts) + 1;
  p[n++] = 0; p[n++] = 0; p[n++] = 5; p[n++] = 0; p[n++] = 'm'; p[n++] = 'W'; p[n++] = 0;
  injectFrame(0x2B, p, n); run(50);
  uint8_t idx = crsfPowerMaxIdx();
  // and the "Dynamic" field (ELRS 3.x TEXT_SELECTION "Off;Dyn;AUX9;AUX10;AUX11;AUX12"): Dynamic power is the "Dyn" option
  uint8_t didx0 = crsfDynamicIdx();
  n = 0; p[n++] = 0xEA; p[n++] = 0xEE; p[n++] = 8; p[n++] = 0; p[n++] = 0; p[n++] = 9;
  const char* dname = "Dynamic"; memcpy(p + n, dname, strlen(dname) + 1); n += strlen(dname) + 1;
  const char* dopts = "Off;Dyn;AUX9;AUX10;AUX11;AUX12"; memcpy(p + n, dopts, strlen(dopts) + 1); n += strlen(dopts) + 1;
  p[n++] = 0; p[n++] = 0; p[n++] = 5; p[n++] = 0; p[n++] = 0;
  injectFrame(0x2B, p, n); run(50);
  uint8_t didx = crsfDynamicIdx();
  NOTE("fallback index %u before the option text; with '%s' the cap stops at index %u = 100 mW; Dynamic '%s' -> 'Dyn' = option %u (fallback %u)",
       idx0, crsfPowerOptions(), idx, crsfDynamicOptions(), didx, didx0);
  result(idx0 == CRSF_POWER_MAX_IDX_FALLBACK && idx == 3 && strcmp(crsfPowerOptions(), opts) == 0 && didx0 == 1 && didx == 1 && strcmp(crsfDynamicOptions(), dopts) == 0,
         "ELRS power: the 100 mW cap and Dynamic from the module's option text");
}

static void t_lidar() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); run(200);   // the red laser (and so the LiDAR) needs FULL
  lidarFakeSetCm(150);
  uint8_t pwrIdle = lidarFakeLastPower();
  laserSetForce(LaserForce::On); run(PLATES[4].moveMs + 900);           // the hand plate lifts first, then it fires and the LiDAR measures
  bool firing = laserOn() && pcaDigitalState(PCA_CH_LASER_RED);
  bool mm = lidarMm() == 1500, reg = lidarFakeLastReadReg() == 0x8f, woke = lidarFakeLastPower() == 0x00;
  laserSetForce(LaserForce::Off); run(1500 + PLATES[4].moveMs + CASCADE_HOLD_MS);   // off, then the hand plate closes
  bool off = !laserOn() && !pcaDigitalState(PCA_CH_LASER_RED), asleep = lidarFakeLastPower() == 0x05 && lidarMm() == 0;
  laserSetForce(LaserForce::Auto);
  NOTE("idle power reg 0x%02x; laser forced on %s -> lidar 150 cm read as 1500 mm %s from 0x%02x %s, woken %s; laser off %s -> asleep (0x05) and idle %s", pwrIdle, firing ? "y" : "N", mm ? "y" : "N", lidarFakeLastReadReg(), reg ? "y" : "N", woke ? "y" : "N", off ? "y" : "N", asleep ? "y" : "N");
  result(firing && mm && reg && woke && off && asleep, "LiDAR v3 map (0x8f high/low), laser gate, sleep");
}

static void t_ring_gesture() {
  modeForce(Mode::Show); powerFakeVolts(8.0f);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  thumb(800); fingers(800); run(600);
  palmOff(); uint32_t n0 = gestureCount();
  for (int k = 0; k < 3; k++) thumbPress(800, 90, 60, 260, false);   // three quick crunches, alike, ~400 ms apart
  run(800);
  bool fired = gestureCount() == n0 + 1, white = palmLevel() == PALM_DEFAULT_LEVEL && palmColourIndex() == 0; uint8_t lvlOn = palmLevel();
  // a slow curl is nothing
  uint32_t n1 = gestureCount();
  for (int k = 1; k <= 20; k++) { thumb(800 - 10 * k); run(50); } run(500); thumb(800); run(1500);
  bool slowNothing = gestureCount() == n1;
  // the closed hand (middle, ring, pinky) + a triple = off
  fingers(500); run(400);
  for (int k = 0; k < 3; k++) thumbPress(750, 90, 60, 260, false);
  run(800);
  bool off = palmLevel() == 0 && gestureCount() == n1 + 1;
  fingers(800); thumb(800); run(500);
  NOTE("open hand + triple: fired %s, ring on white %u/10 %s; slow curl: nothing %s; closed hand + triple: off %s", fired ? "y" : "N", lvlOn, white ? "y" : "N", slowNothing ? "y" : "N", off ? "y" : "N");
  result(fired && white && slowNothing && off, "thumb triple crunch -> ring");
}

static void t_finger_dip() {
  thumb(800); fingers(800); run(500);
  palmSet("c=white&lvl=8"); run(300);
  uint32_t d0 = gestureDipCount();
  for (int k = 1; k <= 3; k++) { flexFakeSetMv(1, 800 - 60 * k); run(20); } run(150);   // the index finger dips 180 mV ...
  for (int k = 2; k >= 0; k--) { flexFakeSetMv(1, 800 - 60 * k); run(20); } run(600);   // ... and comes back
  bool dimmer = gestureDipCount() == d0 + 1 && palmLevel() == 7; uint8_t lvlDim = palmLevel();
  for (int k = 1; k <= 3; k++) { flexFakeSetMv(4, 800 - 60 * k); run(20); } run(150);   // the pinky
  for (int k = 2; k >= 0; k--) { flexFakeSetMv(4, 800 - 60 * k); run(20); } run(600);
  bool brighter = gestureDipCount() == d0 + 2 && palmLevel() == 8; uint8_t lvlBright = palmLevel();
  fingers(500); run(400); fingers(800); run(600);                                       // the whole hand closing is not a dip
  bool handNothing = gestureDipCount() == d0 + 2;
  palmOff();
  NOTE("index dip -> %u/10 %s, pinky dip -> %u/10 %s, a whole-hand close is not a dip %s", lvlDim, dimmer ? "y" : "N", lvlBright, brighter ? "y" : "N", handNothing ? "y" : "N");
  result(dimmer && brighter && handNothing, "finger dips -> ring brightness");
}

static void t_arm_refusals() {
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  modeForce(Mode::Show); powerFakeVolts(8.0f); thumb(800); fingers(500);
  static const float Q_IDENTITY[4] = { 1, 0, 0, 0 };
  imuSetRelZero(0, Q_IDENTITY, nullptr); imuSetRelZero(1, Q_IDENTITY, nullptr);   // the page's step 1: the pose the sensors are injected in IS level and in line
  flyWorld(3000);
  String j = flySet("");
  bool modeRefused = j.indexOf("mode SHOW") >= 0;
  modeForce(Mode::Flight); powerFakeVolts(7.1f); flyWorld(3500); j = flySet("");
  bool battRefused = j.indexOf("arm floor") >= 0;
  powerFakeVolts(8.0f); flyWorld(3500, true, "ANGL!"); j = flySet("");
  bool droneRefused = j.indexOf("refuses to arm") >= 0;
  flyWorld(1000, true, "ANGL*", 40, 0); j = flySet("");
  bool notLevel = j.indexOf("not level") >= 0;
  flyWorld(1000);
  NOTE("mode SHOW %s, pack 7.1 V %s, drone '!' %s, drone 40 deg over %s - each named in the block text", modeRefused ? "y" : "N", battRefused ? "y" : "N", droneRefused ? "y" : "N", notLevel ? "y" : "N");
  result(modeRefused && battRefused && droneRefused && notLevel, "arming refused for mode, battery, drone");
}

static void t_arm_gesture_and_disarm() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); thumb(800);
  fingers(500); flyWorld(1500);                        // a fist, the drone linked, ready, level
  bool notYet = !flyArmed();
  fingers(800); flyWorld(300);                         // a quick open ...
  fingers(500); flyWorld(1500);                        // ... and the fist again, held
  bool armed = flyArmed() && crsfChannel(FLY_ARM_AUX) == CRSF_MAX && flyEnabled();
  uint8_t buf[26]; crsfBuildRcFrame(buf); uint32_t bit = 0; uint16_t ch4 = 0; const uint8_t* p = buf + 3;
  for (uint8_t i = 0; i < 16; i++) { uint16_t v = 0; for (uint8_t b = 0; b < 11; b++) { if (p[bit >> 3] & (1u << (bit & 7))) v |= (1u << b); bit++; } if (i == FLY_ARM_AUX) ch4 = v; }
  bool frameArmed = ch4 == CRSF_MAX;
  bool pucksRefused = !puckGroupOn(0, "test");        // no LED show while armed
  bool laserBlocked = true; { laserSetForce(LaserForce::On); flyWorld(200); laserBlocked = !laserOn(); laserSetForce(LaserForce::Auto); }
  // HandBench's disarm (ported 2026-10-07 night): the FINGER GUN - index up, middle / ring / pinky closed - then two thumb
  // clicks; a full fist (the descent) and its clicks never disarm
  thumbPress(800, 100, 120, 300, true);
  thumbPress(800, 100, 120, 300, true);
  flyWorld(500);
  bool fistKeepsFlying = flyArmed();
  flexFakeSetMv(1, 800); flyWorld(400);                // the index up: the finger gun
  thumbPress(800, 100, 120, 300, true);
  thumbPress(800, 100, 120, 300, true);
  flyWorld(500);
  bool disarmed = !flyArmed() && crsfChannel(FLY_ARM_AUX) == CRSF_MIN;
  fingers(800); flyWorld(300);
  NOTE("fist alone not armed %s; fist-open-fist -> ARMED %s (AUX1 %u in the frame %s); pucks refused while armed %s; laser forced on while armed stays off %s; a full fist + two clicks keeps flying %s; the finger gun + two clicks -> disarmed %s",
       notYet ? "y" : "N", armed ? "y" : "N", ch4, frameArmed ? "y" : "N", pucksRefused ? "y" : "N", laserBlocked ? "y" : "N", fistKeepsFlying ? "y" : "N", disarmed ? "y" : "N");
  result(notYet && armed && frameArmed && pucksRefused && laserBlocked && fistKeepsFlying && disarmed, "arm gesture, AUX1, show lock-out, the finger-gun disarm");
}

static void t_failsafes() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); thumb(800);
  fingers(500); flyWorld(1500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);
  bool armed = flyArmed();
  flyWorld(3000, false, "ANGL*");                      // the link statistics stop: lost after 2 s
  bool lostDisarm = !flyArmed();
  fingers(500); flyWorld(2500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);
  bool armed2 = flyArmed();
  modeForce(Mode::Show); flyWorld(200);
  bool modeDisarm = !flyArmed();
  modeForce(Mode::Flight); fingers(500); flyWorld(2500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);
  bool armed3 = flyArmed();
  goSafe("test");
  bool otaDisarm = !flyArmed() && crsfChannel(FLY_ARM_AUX) == CRSF_MIN && crsfChannel(2) == CRSF_MIN;
  NOTE("armed %s -> link lost = disarmed %s; armed %s -> switch to SHOW = disarmed %s; armed %s -> goSafe (OTA) = disarmed, throttle min %s", armed ? "y" : "N", lostDisarm ? "y" : "N", armed2 ? "y" : "N", modeDisarm ? "y" : "N", armed3 ? "y" : "N", otaDisarm ? "y" : "N");
  result(armed && lostDisarm && armed2 && modeDisarm && armed3 && otaDisarm, "flight fail-safes: link lost, mode switch, OTA");
}

// The palm LEDs' aluminium capped by the model (2026-10-20: one body per group, the cap 55 C): a repulsor burst from cold
// is ended by the heat cap - the 10 mm pair first (the least metal), then the 20 mm three, both before the backstop; a
// body over the start limit refuses the next burst; a burst started just under the limit is ended at the cap quickly.
static void t_plate_cap() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); pucksAllOff("test"); pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C); run(200);
  bool b1 = pucksShow("repulsor"); uint32_t s0 = millis(), end10 = 0, end20 = 0;
  while ((puckGroupIsOn(0) || puckGroupIsOn(1)) && millis() - s0 < PUCK_ON_MAX_MS_PALM + 500) {
    run(100);
    if (!end10 && !puckGroupIsOn(1)) end10 = millis() - s0;
    if (!end20 && !puckGroupIsOn(0)) end20 = millis() - s0;
  }
  bool order = end10 && end20 && end10 < end20 && end20 < PUCK_ON_MAX_MS_PALM;
  run(PUCK_OFF_MIN_MS_PALM + 200);                                        // past the groups' rest time
  pucksThermalSet(PLATE_START_MAX_C + 1, PLATE_AMBIENT_C, PLATE_AMBIENT_C);
  bool hotRefused = !pucksShow("repulsor") && strstr(pucksLastRefusal(), "aluminium") != nullptr;
  uint32_t cuts0 = pucksHeatCuts();
  pucksThermalSet(PLATE_START_MAX_C - 1, PLATE_AMBIENT_C, PLATE_AMBIENT_C);
  bool warmStart = puckGroupOn(0, "test"); s0 = millis(); while (puckGroupIsOn(0) && millis() - s0 < PUCK_ON_MAX_MS_PALM + 500) run(50);
  uint32_t lasted = millis() - s0;
  bool cut = pucksHeatCuts() > cuts0 && lasted < PUCK_ON_MAX_MS_PALM && pucksPlateC() < PLATE_CAP_C + 1;
  pucksAllOff("test"); pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C);
  NOTE("a repulsor burst from %.0f C %s: the 10 mm pair cut at %lu ms, the 20 mm three at %lu ms (backstop %lu) %s; at %.0f C the next is refused %s; group 1 from %.0f C is ended at the %.0f C cap after %lu ms %s",
       PLATE_AMBIENT_C, b1 ? "y" : "N", (unsigned long)end10, (unsigned long)end20, (unsigned long)PUCK_ON_MAX_MS_PALM, order ? "y" : "N",
       PLATE_START_MAX_C + 1, hotRefused ? "y" : "N", PLATE_START_MAX_C - 1, PLATE_CAP_C, (unsigned long)lasted, warmStart && cut ? "y" : "N");
  result(b1 && order && hotRefused && warmStart && cut, "palm LEDs' aluminium capped (thermal model, one body per group)");
}

// The accent LEDs and the strip's dim red pixels: on whenever the arm is on, SAFE included; off from LOW battery down.
static void t_idle_lights() {
  powerFakeVolts(8.0f); run(3000); modeForce(Mode::Safe); run(400);       // SAFE runs goSafe, which puts them back
  uint32_t rgb; uint8_t lvl, lit; int8_t first; forearmSummary(rgb, lvl, lit, first);
  uint32_t want = ((uint32_t)((FOREARM_IDLE_RGB >> 16 & 255) * FOREARM_IDLE_LEVEL / 255) << 16) | ((uint32_t)((FOREARM_IDLE_RGB >> 8 & 255) * FOREARM_IDLE_LEVEL / 255) << 8) | ((FOREARM_IDLE_RGB & 255) * FOREARM_IDLE_LEVEL / 255);
  bool safeOn = accentOn() && halFakeOutput(PIN_ACCENT) == HIGH && lit == FOREARM_IDLE_COUNT && first == FOREARM_IDLE_FIRST && rgb == want;
  bool glow = palmGlowing() && palmShownRgb() != 0 && palmLevel() == 0;    // the dim palm glow, the ring's own setting still off
  int s0acc = accentOn(); uint8_t s0lit = lit, s0lvl = lvl; int s0first = first; uint32_t s0rgb = rgb, s0ring = palmShownRgb();
  powerFakeVolts(6.9f); run(3000);                                         // LOW
  forearmSummary(rgb, lvl, lit, first);
  bool lowOff = !accentOn() && halFakeOutput(PIN_ACCENT) == LOW && lit == 0 && !palmGlowing();
  powerFakeVolts(8.0f); run(3000);                                         // back to OK (the hysteresis)
  forearmSummary(rgb, lvl, lit, first);
  bool back = accentOn() && lit == FOREARM_IDLE_COUNT && palmGlowing();
  modeForce(Mode::Show); run(400);
  bool showNoGlow = !palmGlowing() && palmShownRgb() == 0;                 // SHOW: the ring is the wearer's (off until a gesture)
  NOTE("mode SAFE: accent on and %u strip pixels at %06lX %s (seen: accent %d, %u lit from %d, %06lX, level %u), palm glow %06lX %s; pack LOW: all off %s; pack OK again: back %s (accent %d, %u lit); SHOW: no glow %s",
       FOREARM_IDLE_COUNT, (unsigned long)want, safeOn ? "y" : "N", s0acc, s0lit, s0first, (unsigned long)s0rgb, s0lvl, (unsigned long)s0ring, glow ? "y" : "N", lowOff ? "y" : "N", back ? "y" : "N", (int)accentOn(), lit, showNoGlow ? "y" : "N");
  result(safeOn && glow && lowOff && back && showNoGlow, "idle lights: accent LEDs, a few dim red strip pixels, the SAFE palm glow; off at low battery");
}

// The CRITICAL shutdown: a SILENT grace (Toby, 2026-10-08: no 5 s beep), then everything off, the puck told ($S flags
// bit 5), then the deep sleep - and a chirp every 5 s from it.
static void t_shutdown() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(3000);
  palmSet("c=white&lvl=5"); run(300); servosOpen(); run(200);              // things on and moving when the pack gives out
  powerFakeVolts(6.4f);
  uint32_t t0 = millis(); bool sounded = false;
  while (!shutdownActive() && millis() - t0 < 5000) { tick(5); shutdownPoll(); }   // (2026-10-20) on the way down LOW / IDLE chirp - the grace below is what stays silent
  bool armedIn = shutdownActive() && shutdownPhase() == 1;
  for (uint32_t t = 0; t < SHUTDOWN_GRACE_MS / 2; t += 5) { tick(5); shutdownPoll(); if (halFakeTone(PIN_BUZZER)) sounded = true; }
  bool notYet = !halFakeSlept();
  for (uint32_t t = 0; t < SHUTDOWN_GRACE_MS / 2 + 200; t += 5) { tick(5); shutdownPoll(); if (halFakeTone(PIN_BUZZER)) sounded = true; }
  bool silent = !sounded;
  bool phase2 = shutdownPhase() == 2;
  bool released = true; for (uint8_t c = 0; c < SERVO_COUNT; c++) if (servoChannelUs(c)) released = false;
  bool off = released && halFakeOutput(PIN_PCA_OE) == HIGH && servosAsleep() && !pucksBranchOn() && !laserRedOn() && !laserGreenOn() && !accentOn() && palmMilliampsNow() < PALM_COUNT * 1.5f && forearmMilliamps() < FOREARM_COUNT * 1.5f;
  run(S3_STATUS_MS * 2);
  glink::Parser p; const char* f[glink::MAX_FIELDS]; uint8_t nf; glink::Status st;
  bool told = parseLine(s3linkLastStatusLine(), p, f, nf) && glink::statusParse(f, nf, st) == glink::Result::Ok && (st.flags & 0x20);
  for (uint32_t t = 0; t < SHUTDOWN_TELL_MS + 100; t += 5) { tick(5); shutdownPoll(); }
  bool slept = halFakeSlept() && shutdownPhase() == 3 && halFakeSleepMs() == SHUTDOWN_CHIRP_EVERY_MS;
  bool radioOff = !Bench.wifiEnabled();                                     // (2026-10-08, WiFi on at boot) the radio off before the deep sleep
  uint32_t c0 = halFakeChirps(); bool woke = true;                         // two timer wakes, as setup()'s first line runs them
  for (int k = 0; k < 2; k++) { halFakeTimerWake(); if (!halWokeFromShutdown()) woke = false; else halShutdownChirpAndSleep(SHUTDOWN_CHIRP_HZ, SHUTDOWN_CHIRP_MS, SHUTDOWN_CHIRP_EVERY_MS); }
  bool chirps = woke && halFakeChirps() == c0 + 2 && halFakeSlept() && halFakeSleepMs() == SHUTDOWN_CHIRP_EVERY_MS;
  shutdownReset(); powerFakeVolts(8.0f); run(3000); goSafe("self-test: the shutdown test woke up");
  bool every5 = SHUTDOWN_CHIRP_EVERY_MS == 5000 && SHUTDOWN_CHIRP_MS <= 200;  // Toby's numbers: one short chirp every 5 s
  NOTE("CRITICAL -> shutdown armed %s, silent until asleep %s, not asleep during the grace %s; then everything off (servos released + PCA asleep, OE high, LED groups, lasers, accent, both chains dark) %s; the puck told ($S flags bit 5) %s; WiFi off before the sleep %s; deep sleep with a %lu s timer %s; two wakes: a %lu ms chirp each and asleep again %s",
       armedIn ? "y" : "N", silent ? "y" : "N", notYet ? "y" : "N", phase2 && off ? "y" : "N", told ? "y" : "N", radioOff ? "y" : "N", (unsigned long)(SHUTDOWN_CHIRP_EVERY_MS / 1000), slept ? "y" : "N", (unsigned long)SHUTDOWN_CHIRP_MS, chirps ? "y" : "N");
  result(armedIn && silent && notYet && phase2 && off && told && radioOff && slept && chirps && every5, "CRITICAL shutdown: silent grace, everything off, the puck blanked, WiFi off, deep sleep, one short chirp every 5 s");
}

// On-arm calibration (no computer needed): key K (flat) then key J (fist) give the gesture detector its numbers.
static void t_calibrate() {
  thumb(1900); fingers(1700); run(1200);                                   // a flat hand on 47 k dividers (settled)
  gestureCaptureStart(false); run(GESTURE_CAPTURE_MS + 100);
  thumb(1500); fingers(1200); run(1200);                                   // the fist
  gestureCaptureStart(true); run(GESTURE_CAPTURE_MS + 100);
  float cIdx = gestureFingerCurl(1);                                       // the fist itself: about 1
  fingers(1700); thumb(1900); run(1200);
  float cFlat = gestureFingerCurl(1);                                      // flat again: about 0
  bool armedOk = !gestureCapturing() && !isnan(cIdx) && cIdx > 0.9f && cFlat < 0.1f;
  NOTE("K (flat 1900 / 1700 mV) then J (fist 1500 / 1200 mV): index curl at the fist %.2f, flat again %.2f %s", cIdx, cFlat, armedOk ? "y" : "N");
  result(armedOk, "on-arm finger calibration (keys K / J, no computer needed)");
}



// Touch is view-only (Toby, 2026-10-07): the puck uses every touch for its own 3D view; on the arm each one is counted
// and logged and NOTHING else happens - no plate, no light, no LED group, no laser, no buzzer, no IMU zero.
static void t_touch_map() {
  modeForce(Mode::Show); run(100);
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(2500); }
  palmSet("c=white&lvl=5"); run(400);
  uint8_t col0 = palmColourIndex(); uint32_t touches0 = s3linkTouches(), plays0 = buzzerPlays(); bool zero0 = imuRelZeroed();
  const char* kinds[] = { "TAP", "DOUBLE", "LONG", "UP", "DOWN", "LEFT", "RIGHT", "PRESS", "RELEASE" };
  char body[48], line[glink::MAX_LINE]; uint8_t seq = 7;
  for (const char* k : kinds) { snprintf(body, sizeof body, "T,%u,%s,120,120", seq++, k); glink::frame(line, sizeof line, body); s3linkInject(line); run(300); }
  uint32_t got = s3linkTouches() - touches0;
  bool nothing = !servosBusy() && servosPlatesState() == PlatesState::Closed && palmLevel() == 5 && palmColourIndex() == col0 &&
                 !pucksBranchOn() && !laserRedOn() && !laserGreenOn() && buzzerPlays() == plays0 && imuRelZeroed() == zero0;
  palmOff();
  NOTE("9 touch kinds sent: %lu counted (want 9); plates closed, ring white 5/10, LED groups / lasers off, no buzzer, IMU zero untouched %s", (unsigned long)got, nothing ? "y" : "N");
  result(got == 9 && nothing, "touch is view-only: every touch counted, nothing on the arm");
}

// ---- the plates' mechanical order (Toby, 2026-10-07 night): plate 1 on its own; plate 2 needs the hand plate; plate 3 the
// hand plate and plate 2; open in that order, close in reverse; a plate closes only when nothing still needs it open.
static void allPlatesClosed() { if (servosPlatesState() != PlatesState::Closed) { thermalStop("test"); laserGreenSet(false, false); laserRedSet(false, false); servosClose(); run(3500); } }
static void settle() { uint32_t t0 = millis(); while (servosBusy() && millis() - t0 < 6000) run(10); run(50); }
static bool plateMove(uint8_t g, bool open) { bool ok = servosMovePlate(g, open); settle(); return ok && servosPlateState(g) == (open ? 1 : 0); }
// the first time each plate gets a pulse and the time it lands, through one cascade (the channels are released before it)
static void watchCascade(bool open, int32_t startAt[PLATE_GROUPS + 1], int32_t landAt[PLATE_GROUPS + 1]) {
  for (uint8_t g = 0; g <= PLATE_GROUPS; g++) startAt[g] = landAt[g] = -1;
  uint32_t t0 = millis();
  if (open) servosOpen(); else servosClose();
  while ((servosBusy() || millis() - t0 < 50) && millis() - t0 < 6000) {
    tick(5);
    int32_t t = (int32_t)(millis() - t0);
    for (uint8_t i = 0; i < PLATE_COUNT; i++) { uint8_t g = PLATE_GROUP_OF[i]; if (startAt[g] < 0 && servoChannelUs(PLATES[i].ch)) startAt[g] = t; }
    for (uint8_t g = 1; g <= PLATE_GROUPS; g++) if (landAt[g] < 0 && servosPlateState(g) == (open ? 1 : 0)) landAt[g] = t;
  }
}
static void t_plate_deps() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(100);
  allPlatesClosed();
  // one plate at a time
  bool r2 = !servosMovePlate(2, true) && strstr(servosLastRefusal(), "hand plate"), r3 = !servosMovePlate(3, true);
  bool o4 = plateMove(4, true);
  bool r3b = !servosMovePlate(3, true) && strstr(servosLastRefusal(), "plate 2");
  bool o2 = plateMove(2, true), o3 = plateMove(3, true);
  bool c4 = !servosMovePlate(4, false) && strstr(servosLastRefusal(), "is not closed"), c2 = !servosMovePlate(2, false);
  bool x3 = plateMove(3, false), x2 = plateMove(2, false), x4 = plateMove(4, false);
  bool p1 = plateMove(1, true) && plateMove(1, false);                    // plate 1 on its own, any time
  bool oneAtATime = r2 && r3 && o4 && r3b && o2 && o3 && c4 && c2 && x3 && x2 && x4 && p1;
  // the cascade (2026-10-31): plate 2 starts once the hand plate is PLATE_CLEAR_TRAVEL through its travel (1 = landed), plate 1
  // with plate 2 (PLATE_WITH), plate 3 once plate 2 has landed; closing in reverse, the hand plate passing that point only once plate 2 is home
  const bool lin4 = servosPlateLinear(4);   // (2026-10-20) the hand plate at constant speed: its time is its travel
  auto easeAt = [lin4](float travel) { if (lin4) return travel; float lo = 0, hi = 1; for (int i = 0; i < 24; i++) { float m = 0.5f * (lo + hi); if (m * m * (3.0f - 2.0f * m) < travel) lo = m; else hi = m; } return 0.5f * (lo + hi); };
  const int32_t handDur = (int32_t)servosPlateMs(4), tol = 25;
  const int32_t openClear = (int32_t)(easeAt(PLATE_CLEAR_TRAVEL[4]) * handDur), closeClear = (int32_t)(easeAt(1.0f - PLATE_CLEAR_TRAVEL[4]) * handDur);
  int32_t s[PLATE_GROUPS + 1], l[PLATE_GROUPS + 1];
  watchCascade(true, s, l);
  bool openOrder = s[4] >= 0 && s[2] >= s[4] + openClear - tol && s[2] <= l[4] + tol && abs(s[1] - s[2]) <= tol && s[3] >= l[2] && l[3] > 0 && servosPlatesState() == PlatesState::Open;
  int32_t os2 = s[2], os4 = s[4], ol4 = l[4], os3 = s[3], ol2 = l[2];
  settle();
  watchCascade(false, s, l);
  bool closeOrder = s[3] >= 0 && s[2] >= l[3] && abs(s[1] - s[2]) <= tol && s[4] <= l[2] + tol && s[4] + closeClear >= l[2] - tol && l[4] > 0 && servosPlatesState() == PlatesState::Closed;
  // (2026-10-21) the puck's plate buttons (servosMovePlateWith): plate 3 brings the hand plate and plate 2, plate 2 the
  // hand plate; closing the hand plate closes 3 and 2 first, closing plate 2 leaves the hand plate up; plate 1 untouched
  auto st = [](uint8_t g) { return servosPlateState(g); };
  bool w3 = servosMovePlateWith(3, true); settle();
  bool with3 = w3 && st(4) == 1 && st(2) == 1 && st(3) == 1 && st(1) == 0;
  bool w4 = servosMovePlateWith(4, false); settle();
  bool shut4 = w4 && st(3) == 0 && st(2) == 0 && st(4) == 0;
  bool w2 = servosMovePlateWith(2, true); settle();
  bool with2 = w2 && st(4) == 1 && st(2) == 1 && st(3) == 0;
  bool v2 = servosMovePlateWith(2, false); settle();
  bool shut2 = v2 && st(2) == 0 && st(4) == 1;
  plateMove(4, false);
  bool withOk = with3 && shut4 && with2 && shut2;
  NOTE("buttons: plate 3 opens the hand plate + 2 first %s; the hand plate closes 3 + 2 first %s; plate 2 opens the hand plate %s; plate 2 closes alone %s",
       with3 ? "y" : "N", shut4 ? "y" : "N", with2 ? "y" : "N", shut2 ? "y" : "N");
  closeOrder = closeOrder && withOk;
  NOTE("plate 2 refused with the hand plate closed %s, plate 3 %s; hand plate opens %s; plate 3 refused before plate 2 %s; plate 2 %s then 3 %s; hand plate refused while 2 / 3 open %s, plate 2 while 3 open %s; close 3 %s, 2 %s, hand %s; plate 1 alone %s; cascade open: the hand plate at %ld ms (lands %ld), plate 2 + 1 at %ld (its clear point = %ld ms in), plate 3 at %ld after plate 2 landed at %ld %s; close: 3 -> 2 + 1 -> the hand plate (its clear point %ld ms into its move) %s",
       r2 ? "y" : "N", r3 ? "y" : "N", o4 ? "y" : "N", r3b ? "y" : "N", o2 ? "y" : "N", o3 ? "y" : "N", c4 ? "y" : "N", c2 ? "y" : "N", x3 ? "y" : "N", x2 ? "y" : "N", x4 ? "y" : "N", p1 ? "y" : "N",
       (long)os4, (long)ol4, (long)os2, (long)openClear, (long)os3, (long)ol2, openOrder ? "y" : "N", (long)closeClear, closeOrder ? "y" : "N");
  result(oneAtATime && openOrder && closeOrder, "plate order: 2 needs the hand plate (landed), 3 needs both; plate 1 with plate 2; close in reverse");
}

// What holds a plate open: the thermal camera and the red laser hold the hand plate, the green laser plate 1, plate 3
// holds plate 2 - a cascade close leaves a held plate (and only it) up; the hold's own off then closes it.
static void t_plate_holds() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(100);
  allPlatesClosed();
  servosOpen(); settle();
  thermalOn("test"); run(100);
  servosClose(); settle();
  bool handHeld = servosPlateOpen(4) && servosPlateState(2) == 0 && servosPlateState(3) == 0 && servosPlateState(1) == 0 && thermalIsOn();
  thermalOff("test"); settle();
  bool handClosedAfter = servosPlateState(4) == 0;
  // plate 2 open: the camera's off must NOT close the hand plate
  plateMove(4, true); plateMove(2, true);
  thermalOn("test"); run(100); thermalOff("test"); settle();
  bool stillUp = servosPlateOpen(4) && servosPlateOpen(2);
  plateMove(2, false); plateMove(4, false);
  NOTE("cascade close with the camera on: plates 1 / 2 / 3 closed, the hand plate held up %s; the camera off -> it closes %s; with plate 2 open the camera's off leaves the hand plate up %s",
       handHeld ? "y" : "N", handClosedAfter ? "y" : "N", stillUp ? "y" : "N");
  result(handHeld && handClosedAfter && stillUp, "plate holds: the camera keeps the hand plate up, closes it only when nothing needs it");
}

// The lasers' sequences (FULL): red = the hand plate lifts, then it fires (the LiDAR with it); off = the laser off, then the
// hand plate closes (if nothing needs it). Green = plate 1 opens, then it fires; off = off, then plate 1 closes.
static void t_laser_plates() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); run(100);
  allPlatesClosed();
  bool accepted = laserRedSet(true); run(40);
  bool waitsForPlate = !laserRedOn() && servosMovingPlate() == LASER_RED_PLATE && !pcaDigitalState(PCA_CH_LASER_RED);
  settle(); run(60);
  bool firedAfter = laserRedOn() && servosPlateOpen(LASER_RED_PLATE) && pcaDigitalState(PCA_CH_LASER_RED);
  bool heldShut = !servosMovePlate(LASER_RED_PLATE, false);              // the hand plate never closes under the red laser
  laserRedSet(false); run(20);
  bool offFirst = !laserRedOn() && !pcaDigitalState(PCA_CH_LASER_RED);
  settle();
  bool redClosed = servosPlateState(LASER_RED_PLATE) == 0;
  bool gAccepted = laserGreenSet(true); run(40);
  bool gWaits = !laserGreenOn() && servosMovingPlate() == LASER_GREEN_PLATE;
  settle(); run(60);
  bool gFired = laserGreenOn() && servosPlateOpen(LASER_GREEN_PLATE) && pcaDigitalState(PCA_CH_LASER_GREEN);
  bool gHeld = !servosMovePlate(LASER_GREEN_PLATE, false);
  laserGreenSet(false); run(20); bool gOff = !laserGreenOn(); settle();
  bool greenClosed = servosPlateState(LASER_GREEN_PLATE) == 0;
  // a mode change stops a laser without closing its plate
  laserRedSet(true); settle(); run(60); bool on2 = laserRedOn();
  modeForce(Mode::Show); run(100); settle();
  bool modeStop = on2 && !laserRedOn() && servosPlateOpen(LASER_RED_PLATE);
  plateMove(LASER_RED_PLATE, false);
  NOTE("red: accepted %s, waits for the hand plate %s, fires once it is up %s, the plate cannot close under it %s, off first %s then the plate closes %s; green: accepted %s, waits for plate 1 %s, fires %s, held %s, off %s then plate 1 closes %s; FULL -> SHOW: off, its plate stays up %s",
       accepted ? "y" : "N", waitsForPlate ? "y" : "N", firedAfter ? "y" : "N", heldShut ? "y" : "N", offFirst ? "y" : "N", redClosed ? "y" : "N",
       gAccepted ? "y" : "N", gWaits ? "y" : "N", gFired ? "y" : "N", gHeld ? "y" : "N", gOff ? "y" : "N", greenClosed ? "y" : "N", modeStop ? "y" : "N");
  result(accepted && waitsForPlate && firedAfter && heldShut && offFirst && redClosed && gAccepted && gWaits && gFired && gHeld && gOff && greenClosed && modeStop,
         "laser sequences: its plate opens first, the laser fires, off then the plate closes");
}

// the $S flags and the last $E as the puck would read them
static int statusFlags() {
  glink::Parser p; const char* f[glink::MAX_FIELDS]; uint8_t nf = 0; glink::Status s;
  if (!parseLine(s3linkLastStatusLine(), p, f, nf) || glink::statusParse(f, nf, s) != glink::Result::Ok) return -1;
  return s.flags;
}
static bool lastEvent(glink::Event& e) {
  glink::Parser p; const char* f[glink::MAX_FIELDS]; uint8_t nf = 0;
  return parseLine(s3linkLastEventLine(), p, f, nf) && glink::eventParse(f, nf, e) == glink::Result::Ok;
}

// Flipping down mid-action (Toby, 2026-10-07 night): FULL -> SHOW = both lasers off at once, the camera and the plates
// carry on; -> SAFE = the camera off and a moving cascade FROZEN where it is - never driven shut.
static void t_mode_down() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); run(200);
  if (servosPlatesState() != PlatesState::Open) { servosOpen(); run(3500); }
  bool g = laserGreenSet(true); laserSetForce(LaserForce::On); run(300);
  bool bothOn = g && laserOn() && laserGreenOn() && pcaDigitalState(PCA_CH_LASER_RED) && pcaDigitalState(PCA_CH_LASER_GREEN);
  thermalOn("test"); run(300);                                             // the hand plate is already up: on at once
  bool camOn = thermalIsOn();
  modeForce(Mode::Show); run(20);
  bool lasersOff = !laserOn() && !laserGreenOn() && !pcaDigitalState(PCA_CH_LASER_RED) && !pcaDigitalState(PCA_CH_LASER_GREEN);
  run(300);
  bool carriedOn = thermalIsOn() && servosPlatesState() == PlatesState::Open && !servosBusy() && !laserOn();   // the forced-on laser does not come back in SHOW
  servosClose(); run(350);                                                 // a close under way ...
  bool moving = servosBusy();
  modeForce(Mode::Safe); run(20);                                          // ... and the switch to SAFE
  int16_t at[PLATE_COUNT]; for (uint8_t i = 0; i < PLATE_COUNT; i++) at[i] = servoPosPermille(i);
  run(2500);
  bool still = !servosBusy(), notShut = false;
  for (uint8_t i = 0; i < PLATE_COUNT; i++) { if (servoPosPermille(i) != at[i]) still = false; if (servoPosPermille(i) > 50) notShut = true; }
  bool released = halFakeOutput(PIN_PCA_OE) == HIGH && servosDrivenMask() == 0, camOff = !thermalIsOn();
  laserSetForce(LaserForce::Auto);
  NOTE("FULL: both lasers on %s, camera on %s; -> SHOW: lasers off at once %s, camera and plates carry on %s; a close moving %s -> SAFE: frozen where it was %s (not shut %s), servos released %s, camera off %s",
       bothOn ? "y" : "N", camOn ? "y" : "N", lasersOff ? "y" : "N", carriedOn ? "y" : "N", moving ? "y" : "N", still ? "y" : "N", notShut ? "y" : "N", released ? "y" : "N", camOff ? "y" : "N");
  result(bothOn && camOn && lasersOff && carriedOn && moving && still && notShut && released && camOff, "mode down: lasers off at once, the plates freeze (never closed), the camera off");
}

// The thermal camera: ON lifts the hand plate, then the $S THERMAL bit; OFF clears the bit, then the plate closes; in SAFE
// it is blocked (reported to the puck) and nothing moves.
static void t_thermal() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(200);
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(3500); }
  bool accepted = thermalOn("test"); run(60);
  bool lifting = !thermalIsOn() && servosBusy() && servosMovingPlate() == THERMAL_PLATE && strcmp(thermalStateName(), "lifting") == 0;
  run(PLATES[4].moveMs + CASCADE_HOLD_MS + 300);
  bool up = thermalIsOn() && servoPosPermille(4) >= 950 && servoPosPermille(5) >= 950 && servoPosPermille(0) <= 50;
  run(150);
  int fl = statusFlags(); bool bit = fl >= 0 && (fl & (1 << glink::FLAG_THERMAL));
  glink::Event e; bool evOn = lastEvent(e) && e.act == glink::ACT_THERMAL_ON && e.res == glink::RES_DONE;
  thermalOff("test"); run(20);
  bool bitOffFirst = !thermalIsOn();
  run(150); fl = statusFlags(); bool bitGone = fl >= 0 && !(fl & (1 << glink::FLAG_THERMAL));
  run(PLATES[4].moveMs + CASCADE_HOLD_MS + 300);
  bool closed = servoPosPermille(4) <= 50 && servoPosPermille(5) <= 50 && strcmp(thermalStateName(), "off") == 0;
  modeForce(Mode::Safe); run(100);
  bool refused = !thermalOn("test"); run(100);
  bool blocked = refused && lastEvent(e) && e.act == glink::ACT_THERMAL_ON && e.res == glink::RES_BLOCKED && e.mode == 0 && e.need == 1 && !servosBusy() && servoPosPermille(4) <= 50;
  modeForce(Mode::Show); run(100);
  NOTE("SHOW: accepted %s, the hand plate lifting first %s, up -> camera on %s, $S THERMAL bit %s, $E done %s; off: bit cleared first %s and gone from $S %s, then the plate closed %s; SAFE: blocked ($E BLOCKED, needs SHOW) and nothing moved %s",
       accepted ? "y" : "N", lifting ? "y" : "N", up ? "y" : "N", bit ? "y" : "N", evOn ? "y" : "N", bitOffFirst ? "y" : "N", bitGone ? "y" : "N", closed ? "y" : "N", blocked ? "y" : "N");
  result(accepted && lifting && up && bit && evOn && bitOffFirst && bitGone && closed && blocked, "thermal camera: the hand plate lifts, the THERMAL bit, off and closed, blocked in SAFE");
}

// SAFE: every gesture is still detected and reported to the puck ($E BLOCKED, which one and the mode it needs); nothing acts.
static void t_gesture_blocked() {
  modeForce(Mode::Safe); powerFakeVolts(8.0f); run(300);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  thumb(800); fingers(800); run(600);
  uint32_t n0 = gestureCount(), b0 = s3linkEventsBlocked();
  for (int k = 0; k < 3; k++) thumbPress(800, 90, 60, 260, false);        // the open-hand triple: ring on, if the mode let it
  run(800);
  bool seen = gestureCount() == n0 + 1, nothing = palmLevel() == 0 && palmGlowing();
  glink::Event e; bool told = lastEvent(e) && e.act == glink::ACT_RING_ON && e.res == glink::RES_BLOCKED && e.mode == 0 && e.need == 1 && s3linkEventsBlocked() == b0 + 1;
  modeForce(Mode::Show); run(300);
  for (int k = 0; k < 3; k++) thumbPress(800, 90, 60, 260, false);
  run(800);
  bool acted = palmLevel() > 0 && lastEvent(e) && e.act == glink::ACT_RING_ON && e.res == glink::RES_DONE && e.mode == 1;
  palmOff();
  NOTE("SAFE: the triple detected %s, the ring untouched (glow only) %s, the puck told RING_ON BLOCKED needs SHOW %s; SHOW: the same triple acts and is reported DONE %s",
       seen ? "y" : "N", nothing ? "y" : "N", told ? "y" : "N", acted ? "y" : "N");
  result(seen && nothing && told && acted, "SAFE: gestures detected and reported to the puck, nothing acts");
}

// Toby's gesture map (2026-10-21, gesturemap.cpp): the green laser (index + middle out, ring + pinky in: three thumb clicks
// on, two off, the fingers free while it is on), the ring unchanged, the plates (a thumb-in fist: the hand open quickly =
// open all, a quick open and back = close all), the wrist aligned for each (bent: refused, two buzzes), the thermal camera
// (thumb out, two quick finger pulses - on, again off) - with the drone ready it never arms, the drone's own single pulse
// still does, and a thumb-in pulse never does. One buzz = done, two = refused.
static float poseH[4] = { 1, 0, 0, 0 };
static void poseRun(uint32_t ms) { for (uint32_t t = 0; t < ms; t += 5) { if (t % 20 == 0) { imuFakeInject(0, 1, 0, 0, 0); imuFakeInject(1, poseH[0], poseH[1], poseH[2], poseH[3]); } if (t % 200 == 0) droneSays(true, "ANGL*", 3.8f, 0, 0); tick(5); } }
static void bendWrist(float deg) { float a = deg * DEG_TO_RAD / 2; poseH[0] = cosf(a); poseH[1] = 0; poseH[2] = sinf(a); poseH[3] = 0; }
static void clicks(int n) { for (int k = 0; k < n; k++) { for (int s = 1; s <= 3; s++) { thumb(800 - 90 * s / 3); poseRun(10); } poseRun(60); for (int s = 2; s >= 0; s--) { thumb(800 - 90 * s / 3); poseRun(10); } poseRun(260); } poseRun(800); }
static void twoFingerPose() { flexFakeSetMv(1, 800); flexFakeSetMv(2, 800); flexFakeSetMv(3, 500); flexFakeSetMv(4, 500); }
static void pulse() { fingers(800); poseRun(120); fingers(500); poseRun(100); }   // the four out and back, quick
static void t_gesture_map() {
  static const float Q_ID[4] = { 1, 0, 0, 0 };
  auto isLast = [](uint8_t act, uint8_t res) { glink::Event e; return lastEvent(e) && e.act == act && e.res == res; };
  modeForce(Mode::Flight); powerFakeVolts(8.0f); laserSetForce(LaserForce::Auto);
  imuSetRelZero(0, Q_ID, nullptr); imuSetRelZero(1, Q_ID, nullptr);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  allPlatesClosed(); thumb(800); fingers(800); palmOff(); bendWrist(0); poseRun(1500);
  // (1) the green laser
  twoFingerPose(); poseRun(500);
  uint32_t fb = hapticFeedbackCount();
  clicks(3);
  bool gOn = laserGreenWanted() && isLast(glink::ACT_LASER_GREEN, glink::RES_DONE) && hapticFeedbackCount() == fb + 1;
  poseRun(1500); bool gBeam = laserGreenOn();
  fingers(500); poseRun(600); fingers(800); poseRun(600); thumb(600); poseRun(300); thumb(800); poseRun(600);   // the fingers anywhere: it stays on
  bool gStays = laserGreenWanted() && laserGreenOn();
  twoFingerPose(); poseRun(800);
  clicks(2); poseRun(1500); settle();
  bool gOff = !laserGreenWanted() && !laserGreenOn() && isLast(glink::ACT_LASER_GREEN, glink::RES_DONE) && servosPlateState(LASER_GREEN_PLATE) == 0;
  // the ring, as on HandBench: the open hand + three clicks
  fingers(800); modeForce(Mode::Show); poseRun(800);
  clicks(3); bool ring = palmLevel() > 0; palmOff(); poseRun(800);
  // (2) the plates: a thumb-in fist held, then the whole hand open quickly = open all ...
  thumb(600); fingers(500); poseRun(500);
  fb = hapticFeedbackCount();
  thumb(800); fingers(800); poseRun(100);
  bool opening = isLast(glink::ACT_CASCADE, glink::RES_DONE) && hapticFeedbackCount() == fb + 1; poseRun(3000); settle();
  bool openAll = opening && servosPlatesState() == PlatesState::Open;
  // ... and the same fist, a quick open and back into it = close all
  thumb(600); fingers(500); poseRun(500);
  fingers(800); poseRun(200); fingers(500); poseRun(100);
  bool closing = isLast(glink::ACT_CASCADE, glink::RES_DONE); poseRun(3000); settle();
  bool closeAll = closing && servosPlatesState() == PlatesState::Closed;
  thumb(800); poseRun(400); fingers(800); poseRun(1200);                 // out of the fist with the thumb out first: nothing
  bool quietAfter = servosPlatesState() == PlatesState::Closed && !servosBusy();
  // (3) not aligned: the wrist bent 50 deg - the same open-all refused, two buzzes, nothing moves
  bendWrist(50); poseRun(1000);                                          // (past the laser's jolt window: the bend itself is a jolt)
  thumb(600); fingers(500); poseRun(500);
  fb = hapticFeedbackCount();
  thumb(800); fingers(800); poseRun(300);
  bool notAligned = isLast(glink::ACT_CASCADE, glink::RES_REFUSED) && hapticFeedbackCount() == fb + 1 && !servosBusy() && servosPlatesState() == PlatesState::Closed;
  bendWrist(0); poseRun(1200);
  // (4) the camera, FULL with the drone ready: thumb out, the fingers down, two quick pulses = on - and never armed
  modeForce(Mode::Flight); poseRun(300);
  fingers(500); poseRun(600);
  pulse(); fingers(800); poseRun(100); fingers(500); poseRun(1500);
  bool camOn = thermalIsOn() && !flyArmed();
  poseRun(400); pulse(); fingers(800); poseRun(100); fingers(500); poseRun(1500); settle();
  bool camOff = !thermalIsOn() && !flyArmed();
  // the drone's own arm still works: one pulse (thumb out), held ...
  poseRun(1500); fingers(800); poseRun(300); fingers(500); poseRun(1500);
  bool droneArms = flyArmed();
  flyDisarm("self-test"); fingers(800); poseRun(2500);
  // ... and a thumb-in pulse is the plates' (closed: nothing), never an arm
  thumb(600); fingers(500); poseRun(600); fingers(800); poseRun(300); fingers(500); poseRun(1500);
  bool thumbInNoArm = !flyArmed();
  thumb(800); poseRun(400); fingers(800); poseRun(2500); modeForce(Mode::Show); poseRun(100); allPlatesClosed();
  NOTE("green: two fingers + 3 clicks on %s (beam %s), the fingers moved - still on %s, + 2 clicks off and plate 1 shut %s; the ring as on HandBench %s; plates: thumb-in fist -> open quickly = open all %s, -> open and back = close all %s, then nothing %s; the wrist bent 50 deg: refused with two buzzes %s; camera (FULL, the drone ready): two pulses on %s, again off %s, never armed; the drone's single pulse arms %s, a thumb-in pulse does not %s",
       gOn ? "y" : "N", gBeam ? "y" : "N", gStays ? "y" : "N", gOff ? "y" : "N", ring ? "y" : "N", openAll ? "y" : "N", closeAll ? "y" : "N", quietAfter ? "y" : "N", notAligned ? "y" : "N",
       camOn ? "y" : "N", camOff ? "y" : "N", droneArms ? "y" : "N", thumbInNoArm ? "y" : "N");
  result(gOn && gBeam && gStays && gOff && ring && openAll && closeAll && quietAfter && notAligned && camOn && camOff && droneArms && thumbInNoArm,
         "gesture map (Toby's): green on / off, plates open / close, the camera, aligned, buzzes; the drone's arm untouched");
}

// Both lasers stand down while the drone flies and for LASER_AFTER_FLIGHT_MS after it (Toby, 2026-10-07 night: a wrist jerk
// during the finger-gun disarm must never fire the green laser); a laser gesture seen meanwhile is reported $E BLOCKED.
static float lzH[4] = { 1, 0, 0, 0 };                                    // the hand; the forearm stays at its zero
static void laserWorld(uint32_t ms) {
  for (uint32_t t = 0; t < ms; t += 5) {
    if (t % 20 == 0) { imuFakeInject(0, 1, 0, 0, 0); imuFakeInject(1, lzH[0], lzH[1], lzH[2], lzH[3]); }
    if (t % 200 == 0) droneSays(true, "ANGL*", 3.8f, 0, 0);
    tick(5);
  }
}
static void wristJolt(bool bent) { float a = bent ? 20 * DEG_TO_RAD / 2 : 0; lzH[0] = cosf(a); lzH[1] = 0; lzH[2] = sinf(a); lzH[3] = 0; }
// HandBench's laser gesture: open over a second (the next closing is fresh), middle / ring / pinky closed fast, the thumb
// crunched in, the index in (red) or up (green - the finger gun), then the wrist jolts 20 deg against the forearm
static void laserGesture(bool green) {
  fingers(800); thumb(800); wristJolt(false); laserWorld(1300);
  for (int s = 1; s <= 3; s++) { uint16_t mv = 800 - 100 * s; for (uint8_t i = 2; i <= 4; i++) flexFakeSetMv(i, mv); if (!green) flexFakeSetMv(1, mv); laserWorld(20); }
  thumb(600); laserWorld(150);
  wristJolt(true); laserWorld(250); wristJolt(false); laserWorld(50);
}
static String ljPick() {   // the laser detector's state, short (a failing note says where the gesture stopped)
  String s = laserJson(), o;
  for (const char* k : { "\"shut\":", "\"mc\":", "\"ff\":", "\"thumb\":", "\"jolt\":" }) { int a = s.indexOf(k); if (a >= 0) { int e = s.indexOf(',', a); o += s.substring(a + 1, e < 0 ? s.length() : e) + " "; } }
  return o;
}
static bool lastIs(uint8_t act, uint8_t res) { glink::Event e; return lastEvent(e) && e.act == act && e.res == res; }
static void t_laser_flight_hold() {
  modeForce(Mode::Flight); powerFakeVolts(8.0f); thumb(800); laserSetForce(LaserForce::Auto);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  allPlatesClosed();
  fingers(500); flyWorld(1500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);   // armed by the fist - open - fist
  bool armed = flyArmed();
  // armed: the red gesture is read and reported BLOCKED; nothing lifts, nothing fires; the routes / keys refuse both
  laserGesture(false);
  bool redArmed = flyArmed() && lastIs(glink::ACT_LASER_RED, glink::RES_BLOCKED) && !laserOn() && !laserRedWanted() && servosPlateState(LASER_RED_PLATE) == 0;
  bool apiArmed = !laserRedSet(true) && !laserGreenSet(true);
  // the finger-gun disarm, the wrist jerking as the second click lands: disarmed, and the green gesture that the jerk
  // completes is BLOCKED (before this rule it fired the moment the drone disarmed)
  fingers(800); thumb(800); wristJolt(false); laserWorld(1300);
  for (int s = 1; s <= 3; s++) { for (uint8_t i = 2; i <= 4; i++) flexFakeSetMv(i, 800 - 100 * s); laserWorld(20); }   // the finger gun, made fast
  laserWorld(350);                                                                                                  // held (the disarm wants the pose before click 1)
  String d1 = ljPick();
  for (int k = 1; k <= 3; k++) { thumb(800 - 100 * k / 3); laserWorld(10); } laserWorld(60);                      // click 1
  for (int k = 2; k >= 0; k--) { thumb(800 - 100 * k / 3); laserWorld(10); } laserWorld(150);
  for (int k = 1; k <= 3; k++) { thumb(800 - 200 * k / 3); laserWorld(10); }                                       // click 2, held in
  laserWorld(80);
  String d2 = ljPick();
  bool disarmed = !flyArmed();
  wristJolt(true); laserWorld(250);
  String dbg = "before click 1: " + d1 + "| after click 2: " + d2 + "| after the jerk: " + ljPick(); wristJolt(false);
  bool greenHeld = disarmed && !laserGreenWanted() && !laserGreenOn() && servosPlateState(LASER_GREEN_PLATE) == 0;   // (2026-10-21) no green from a finger gun + jolt any more (it is two fingers + three clicks): nothing fires
  bool apiHeld = lasersFlightHeld() && !laserRedSet(true) && !laserGreenSet(true);                                  // still inside the 1.5 s
  // after the hold (and past the arm gesture's 2 s pulse, so nothing re-arms): the same red gesture fires again
  thumb(800); fingers(800); laserWorld(2200);
  bool freeAgain = !lasersFlightHeld() && !outputsGreenBlockText()[0];
  laserGesture(false);
  bool redDone = lastIs(glink::ACT_LASER_RED, glink::RES_DONE) && laserOn();
  laserWorld(600); bool redFired = laserRedOn();
  fingers(800); thumb(800); laserWorld(600); settle();
  bool redOff = !laserOn() && !laserRedOn() && servosPlateState(LASER_RED_PLATE) == 0;
  laserWorld(2200);
  NOTE("armed %s: the red gesture BLOCKED, nothing lifted %s, keys / routes refused %s; finger-gun disarm %s with a wrist jerk: the green gesture BLOCKED, plate 1 shut %s, refused inside the 1.5 s %s; after it: free %s, the red gesture fires %s (beam %s), off and its plate shut %s%s%.300s",
       armed ? "y" : "N", redArmed ? "y" : "N", apiArmed ? "y" : "N", disarmed ? "y" : "N", greenHeld ? "y" : "N", apiHeld ? "y" : "N", freeAgain ? "y" : "N", redDone ? "y" : "N", redFired ? "y" : "N", redOff ? "y" : "N",
       greenHeld ? "" : " | ", greenHeld ? "" : dbg.c_str());
  result(armed && redArmed && apiArmed && disarmed && greenHeld && apiHeld && freeAgain && redDone && redFired && redOff,
         "lasers blocked in flight and 1.5 s after a disarm; the gesture reported $E BLOCKED");
}

// The wrist zero needs no touch and no computer (Toby, 2026-10-07: "the IMU zero is fixed in code"): with every saved zero
// forgotten, the fixed zero in config.h (IMU_ZERO_DEG + each sensor's heading at its first sample) stands in for the
// page's step 1, so the arming / lift-off heading solve has its frame from power-up - and the angles are the ones the
// fixed zero always gave (seg = d). Runs last: it forgets the saved zeros.
static void t_imu_fixed_zero() {
  imuZeroForget();
  imuFakeInject(0, 1, 0, 0, 0); imuFakeInject(1, 0.99619f, 0, 0, 0.08716f);    // the hand turned 10 deg about Z from the forearm
  run(300);
  auto wrap = [](float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; };
  float r, p, y, r0, p0, y0, r1, p1, y1; uint32_t age;
  bool rel = imuRelative(r, p, y, age), s0 = imuRelativeShifted(0, r0, p0, y0), s1 = imuRelativeShifted(10 * DEG_TO_RAD, r1, p1, y1);
  bool same = rel && s0 && fabsf(wrap(y - y0)) < 0.5f && fabsf(p - p0) < 0.5f && fabsf(wrap(r - r0)) < 0.5f;
  float k = s0 && s1 ? wrap(y1 - y0) / 10.0f : 0;
  bool solvable = fabsf(k) >= FLY_HEADING_SOLVE_MIN_GAIN, notMeasured = !imuRelZeroed();
  NOTE("saved zeros forgotten, no page, no key z: the heading-solve frame exists %s (side %+.1f deg, %.2f deg per deg of heading), angles as the fixed zero gives them %s, reported as not measured %s",
       s0 ? "y" : "N", y0, k, same ? "y" : "N", notMeasured ? "y" : "N");
  result(s0 && same && solvable && notMeasured, "IMU zero: the fixed one stands in for step 1 - no touch, no computer");
}

// ---- the audits of 2026-10-08 ---------------------------------------------------------------------------------------
// The task watchdog: a loop that stops must reset the board (GPIO 23 then floats, the 1 k pull-up takes OE high, MODE2
// OUTNE 00 turns every PCA output LOW). Proved for real once per flashed build: the first boot after a flash ends its
// self-test by hanging the loop on purpose; the next boot finds the reset reason TASK_WDT with the RTC flag set before
// the hang, and passes; later boots of the same build report it as proved.
RTC_NOINIT_ATTR static uint32_t wdtHangMagic;
static constexpr uint32_t WDT_HANG_MAGIC = 0x57D7E570;
static bool wdtPending = false;
static const char* rrName(esp_reset_reason_t r) {
  switch (r) { case ESP_RST_POWERON: return "POWERON"; case ESP_RST_SW: return "SW"; case ESP_RST_PANIC: return "PANIC"; case ESP_RST_INT_WDT: return "INT_WDT";
               case ESP_RST_TASK_WDT: return "TASK_WDT"; case ESP_RST_WDT: return "WDT"; case ESP_RST_DEEPSLEEP: return "DEEPSLEEP"; case ESP_RST_BROWNOUT: return "BROWNOUT";
               case ESP_RST_EXT: return "EXT"; default: return "other"; }
}
static void t_watchdog() {
  esp_reset_reason_t rr = esp_reset_reason();
  String md5 = ESP.getSketchMD5();
  Preferences p; p.begin("ironwdt", false); String proved = p.getString("proved", "");
  if (wdtHangMagic == WDT_HANG_MAGIC) {
    wdtHangMagic = 0;
    bool ok = rr == ESP_RST_TASK_WDT, oe = halFakeOutput(PIN_PCA_OE) == HIGH;
    if (ok) p.putString("proved", md5);
    p.end();
    NOTE("the previous boot hung its loop on purpose: this boot's reset reason %s (want TASK_WDT) %s; OE driven high by setup()'s first lines %s; MODE2 0x%02X (outputs LOW while OE is high) %s",
         rrName(rr), ok ? "y" : "N", oe ? "y" : "N", servosMode2(), servosMode2() == PCA_MODE2_VALUE ? "y" : "N");
    result(ok && oe && servosMode2() == PCA_MODE2_VALUE, "watchdog: a hung loop resets the board within 3 s, every PCA output off");
    return;
  }
  p.end();
  if (proved == md5) {
    NOTE("proved for this build (sketch %s) by its first boot's deliberate hang; MODE2 0x%02X", md5.substring(0, 8).c_str(), servosMode2());
    result(servosMode2() == PCA_MODE2_VALUE, "watchdog: a hung loop resets the board within 3 s, every PCA output off");
    return;
  }
  wdtPending = true;                               // not counted this boot: the run ends with the hang, the next boot reports it
}
static void watchdogHangIfPending() {
  if (!wdtPending) return;
  wdtPending = false;
  Bench.printf("selftest: the watchdog test - this build has not proved it yet: the loop hangs NOW on purpose; the board must reset within %lu ms and the next boot's self-test reports it\n", (unsigned long)WDT_TIMEOUT_MS);
  Serial.flush();
  wdtHangMagic = WDT_HANG_MAGIC;
  halWdtBegin(WDT_TIMEOUT_MS);
  uint32_t t0 = (millis)();
  while ((millis)() - t0 < 4 * WDT_TIMEOUT_MS) { }
  wdtHangMagic = 0;
  Bench.println("selftest: FAIL - the watchdog did not reset the board");
}

// The heat model at boot (2026-10-20): any reset but a real power-up takes it back from RTC memory (a reboot is not a
// cool-down), a real power-up starts it cold; the LED-driver bank is a body of its own with DRIVER_HEAT_W per lit driver.
static void t_boot_hot() {
  modeForce(Mode::Show); powerFakeVolts(8.0f); run(3000);                 // the pack read first (at boot it is "unknown" = no show)
  pucksThermalSet(PLATE_START_MAX_C + 2, PLATE_START_MAX_C + 3, PLATE_AMBIENT_C + 5); run(20);   // a hot model; the poll keeps it in RTC memory
  pucksThermalBoot(false);                                                 // a reset: an OTA push, a brownout, the watchdog
  float plate = pucksPlateC(), tab = pucksTabC(), bank = pucksBankC();
  bool hot = fabsf(plate - (PLATE_START_MAX_C + 2)) < 1 && fabsf(tab - (PLATE_START_MAX_C + 3)) < 1 && fabsf(bank - (PLATE_AMBIENT_C + 5)) < 1;
  bool refused = !puckGroupOn(2, "test");                                  // the finger stack carried over hot: no burst
  pucksThermalBoot(true);                                                  // a real power-up
  bool cold = pucksPlateC() < PLATE_AMBIENT_C + 0.5f && pucksPalm10C() < PLATE_AMBIENT_C + 0.5f && pucksTabC() < PLATE_AMBIENT_C + 0.5f && pucksBankC() < PLATE_AMBIENT_C + 0.5f;
  hot = hot && cold;
  pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_CAP_C);            // the plates cold, the bank hot
  bool bankRefused = !puckGroupOn(0, "test") && strstr(pucksLastRefusal(), "driver bank");
  pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C);
  bool on = puckGroupOn(0, "test"); run(10000); float bankAfter = pucksBankC(); pucksAllOff("test");
  bool heats = on && bankAfter > PLATE_AMBIENT_C + 1.0f;                     // three drivers x 1.5 W for 10 s into 35 J/K: ~ +1.2 K
  pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C);        // the rest of the run starts cold
  NOTE("over a reset: palm %.0f C, finger %.0f C, bank %.0f C carried over, a power-up cold %s; a finger burst refused while hot %s; the bank alone hot refuses a burst %s; group 1 for 10 s warms the bank to %.1f C %s",
       plate, tab, bank, hot ? "y" : "N", refused ? "y" : "N", bankRefused ? "y" : "N", bankAfter, heats ? "y" : "N");
  result(hot && refused && bankRefused && heats, "heat model: kept over a reset, cold after a power-up, the LED-driver bank's own heat");
}

// Flex faults: a reading no working sensor gives (open ~0 V, shorted 3.3 V, out of its calibrated range) or no samples
// (the ADS1115 silent) - the finger reads OPEN, never bent; the thumb makes no clicks; in flight one bad finger is left
// out, two disarm.
static void t_flex_fault() {
  modeForce(Mode::Show); powerFakeVolts(8.0f);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  thumb(800); fingers(800); run(400);
  flexFakeFastClear(false);
  uint32_t f0 = flexFaultsTotal();
  flexFakeSetMv(3, 500); flexFakeSetMv(2, 0); run(100);                    // the ring closed, the middle finger's wire broken, the pinky open
  bool brokenOpen = flexFault(2) && gestureFingerCurl(2) == 0.0f && !gestureFist();   // before: 0 mV = "closed" and two of three = a closed hand
  run(300); bool logged = flexFaultsTotal() == f0 + 1;
  flexFakeSetMv(1, 3300); run(300); bool shorted = flexFault(1) && strstr(flexFaultWhy(1), "shorted");
  flexFakeSetMv(2, 800); flexFakeSetMv(1, 800); flexFakeSetMv(3, 800); run(500); bool still = flexFault(2);
  run(700); bool cleared = !flexFault(2) && !flexFault(1);
  palmOff(); uint32_t g0 = gestureCount();                                  // a thumb wire flapping three times in rhythm
  for (int k = 0; k < 3; k++) { thumb(0); run(60); thumb(800); run(340); }
  run(1500);
  bool noClicks = gestureCount() == g0 && palmLevel() == 0;
  run(1200);
  flexFakeAdsGone(true); run(FLEX_STALE_MS + 50); bool stale = (flexFaultMask() & 0x1E) == 0x1E;
  flexFakeAdsGone(false); run(2500); bool back = flexFaultMask() == 0;     // re-probed within 1 s, then 1 s of good samples
  flexFakeFastClear(true);
  modeForce(Mode::Flight); fingers(500); flyWorld(1500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);
  bool armed = flyArmed();
  flexFakeSetMv(4, 0); flyWorld(500); bool oneOk = flyArmed();              // the pinky broken: the other three fly it
  flexFakeSetMv(3, 0); flyWorld(FLY_FLEX_LOST_MS + 200); bool twoDisarm = !flyArmed();
  fingers(800); flyWorld(300);
  NOTE("a broken middle finger (0 mV) reads open and the hand is not closed %s, logged %s; a shorted index (3.3 V) %s; back: still faulted after 0.5 s %s, clear after 1 s %s; a flapping thumb wire x3: no clicks, no triple %s; the ADS1115 silent: all four faulted %s, back %s; flying: armed %s, one finger broken still flying %s, two broken -> disarmed %s",
       brokenOpen ? "y" : "N", logged ? "y" : "N", shorted ? "y" : "N", still ? "y" : "N", cleared ? "y" : "N", noClicks ? "y" : "N", stale ? "y" : "N", back ? "y" : "N", armed ? "y" : "N", oneOk ? "y" : "N", twoDisarm ? "y" : "N");
  result(brokenOpen && logged && shorted && still && cleared && noClicks && stale && back && armed && oneOk && twoDisarm, "flex faults: never read as bent, no clicks, the throttle on the others, two lost disarm");
}

// The ADS1115's rate and the smoothing: the next conversion starts as one is read, and the fingers smooth by time
// (FLEX_SMOOTH_TAU_MS = HandBench's 0.25 per 10 ms), so the feel does not depend on the rate.
static void t_flex_rate() {
  modeForce(Mode::Show); fingers(800); run(500);
  uint32_t n0 = flexAdsReads(); run(1000); uint32_t n = flexAdsReads() - n0;
  bool fast = n >= 180;                                                     // one conversion per 5 ms poll (the old code: ~100)
  flexFakeSetMv(2, 500); run(35); float a = flexMv(2); run(65); float b = flexMv(2);
  bool tau = a > 560 && a < 700 && b < 545;                                 // ~63 % of a 300 mV step after one time constant, ~95 % after 100 ms
  fingers(800); run(300);
  NOTE("%lu conversions in 1 s of the fake clock (one per poll) %s; a 300 mV step: %.0f mV after 35 ms, %.0f after 100 ms %s; on the real ADS1115 (860 SPS, 400 kHz) each finger comes every ~6-8 ms (125-165 Hz) against HandBench's 10 ms, smoothed %.1f ms both",
       (unsigned long)n, fast ? "y" : "N", a, b, tau ? "y" : "N", FLEX_SMOOTH_TAU_MS);
  result(fast && tau, "flex: the next ADS1115 conversion at once, smoothing by time (HandBench's 35 ms)");
}

// LAND NOW: armed and the glove pack under BATT_LAND_WARN_V for 2 s - the vibrator every 5 s, NO sound (Toby, 2026-10-08);
// arming is refused under the 7.5 V floor (the ELRS module is a 7-13 V part).
static void t_land_warn() {
  modeForce(Mode::Flight); thumb(800);
  gestureSet("tf=800&tc=600&fl=800,800,800,800&fx=500,500,500,500");
  powerFakeVolts(7.4f); flyWorld(3500);
  bool floorRefuses = !powerArmOk() && powerState() == PowerState::ArmFloor;
  powerFakeVolts(8.0f); flyWorld(3500);
  fingers(500); flyWorld(1500); fingers(800); flyWorld(300); fingers(500); flyWorld(1500);
  bool armed = flyArmed();
  bool vibAt8 = false; for (int t = 0; t < 3000; t += 5) { flyWorld(5); if (vibOn()) vibAt8 = true; }
  powerFakeVolts(7.25f);
  bool tone = false, vib = false; for (int t = 0; t < 6000; t += 5) { flyWorld(5); if (halFakeTone(PIN_BUZZER)) tone = true; if (vibOn()) vib = true; }
  bool stillFlying = flyArmed();
  flyDisarm("test"); powerFakeVolts(8.0f); fingers(800); flyWorld(3500);
  NOTE("7.4 V: arming refused (floor 7.5) %s; armed at 8.0 V %s, no warning there %s; 7.25 V armed: LAND NOW - the vibrator %s, no sound %s, still flying (a warning, not a disarm) %s",
       floorRefuses ? "y" : "N", armed ? "y" : "N", !vibAt8 ? "y" : "N", vib ? "y" : "N", !tone ? "y" : "N", stillFlying ? "y" : "N");
  result(floorRefuses && armed && !vibAt8 && vib && !tone && stillFlying, "battery: arm floor 7.5 V, LAND NOW (the vibrator, no sound) under 7.3 V while armed");
}


// What is set on the arm on the day with the /cal page, no recompile (2026-10-13): each servo's closed / open end, the
// PCA9685 channel map, the battery's meter factor - saved in NVS, used by every move, and each undone by its own "forget".
static uint16_t nvsUShort(const char* k) { Preferences p; uint16_t v = 0; if (p.begin("ironman", true)) { v = p.getUShort(k, 0); p.end(); } return v; }
// The puck's control panel ($C, 2026-10-20): a button is a gesture's action under the gesture's rules - SAFE blocks it,
// SHOW does the lights, a laser needs FULL, turning off is never blocked, "all off" clears the lights; one $E each.
static void t_panel() {
  auto press = [](uint8_t act, uint16_t a, uint16_t b) { static uint16_t sq = 0; glink::Command c; c.seq = ++sq; c.act = act; c.a = a; c.b = b; panelCommand(c); run(30); };
  powerFakeVolts(8.0f); pucksAllOff("test"); pucksThermalSet(PLATE_AMBIENT_C, PLATE_AMBIENT_C, PLATE_AMBIENT_C);
  modeForce(Mode::Safe); run(100);
  press(glink::ACT_FINGERS, 1, 0); bool safeBlocked = lastIs(glink::ACT_FINGERS, glink::RES_BLOCKED) && !puckGroupIsOn(2);
  modeForce(Mode::Show); run(100);
  press(glink::ACT_FINGERS, 1, 0); bool showOn = lastIs(glink::ACT_FINGERS, glink::RES_DONE) && puckGroupIsOn(2);
  press(glink::ACT_LASER_RED, 1, 0); bool laserBlocked = lastIs(glink::ACT_LASER_RED, glink::RES_BLOCKED) && !laserRedWanted();
  modeForce(Mode::Safe); run(100);                                         // SAFE turns the group off itself (goSafe) ...
  press(glink::ACT_FINGERS, 0, 0); bool offAnyMode = lastIs(glink::ACT_FINGERS, glink::RES_DONE) && !puckGroupIsOn(2);   // ... and an off is never blocked
  modeForce(Mode::Show); run(100);
  press(glink::ACT_BURST, 1, 1); bool g1 = puckGroupIsOn(0) && !puckGroupIsOn(1);
  press(glink::ACT_RING_OFF, 0, 0); press(glink::ACT_RING_ON, 3, 0);      // (2026-10-20) a = palette index + 1: red (2)
  bool ringRed = lastIs(glink::ACT_RING_ON, glink::RES_DONE) && palmColourIndex() == 2 && palmLevel() > 0;
  press(glink::ACT_RING_ON, 6, 0); bool ringGreen = palmColourIndex() == 5 && palmLevel() > 0;   // already on: the colour changes, the level stays
  press(glink::ACT_RING_ON, 99, 0); bool ringBad = lastIs(glink::ACT_RING_ON, glink::RES_REFUSED) && palmColourIndex() == 5;
  bool ring = ringRed && ringGreen && ringBad;
  // (2026-10-20) the LiDAR on its own: FULL only; the hand plate lifts, then it measures; off = it stops, the plate closes
  press(glink::ACT_LIDAR, 1, 0); bool lidarShow = lastIs(glink::ACT_LIDAR, glink::RES_BLOCKED) && !lidarAloneWanted();
  modeForce(Mode::Flight); run(100);
  press(glink::ACT_LIDAR, 1, 0); bool lidarAsked = lastIs(glink::ACT_LIDAR, glink::RES_DONE) && lidarAloneWanted();
  run(servosPlateMs(LASER_RED_PLATE) + laserSettleMs() + CASCADE_HOLD_MS + 400);
  bool lidarMeasuring = lidarAloneOn() && servosPlateOpen(LASER_RED_PLATE);
  press(glink::ACT_LIDAR, 0, 0); run(servosPlateMs(LASER_RED_PLATE) + CASCADE_HOLD_MS + 400);
  bool lidarOff = lastIs(glink::ACT_LIDAR, glink::RES_DONE) && !lidarAloneOn() && servosPlateState(LASER_RED_PLATE) == 0;
  modeForce(Mode::Show); run(100);
  bool lidar = lidarShow && lidarAsked && lidarMeasuring && lidarOff;
  // (2026-10-21) the puck's update: the RC frames stop, and come back after the maximum quiet when no heartbeat returns
  bool streamWas = crsfStreaming(); press(glink::ACT_PUCK_UPDATE, 0, 0);
  bool updStopped = lastIs(glink::ACT_PUCK_UPDATE, glink::RES_DONE) && !crsfStreaming();
  for (uint32_t t = 0; t < PUCK_UPDATE_QUIET_MAX_MS + 1000; t += 1000) { halAdvance(1000); panelPoll(); }
  bool updBack = crsfStreaming() == streamWas && !panelPuckUpdating();
  bool puckUpd = updStopped && updBack;
  press(glink::ACT_ALL_OFF, 0, 0); bool allOff = lastIs(glink::ACT_ALL_OFF, glink::RES_DONE) && !puckGroupIsOn(0) && !puckGroupIsOn(1) && !puckGroupIsOn(2);
  press(glink::ACT_ARM, 0, 0); bool noArm = lastIs(glink::ACT_ARM, glink::RES_REFUSED) && !flyArmed();
  pucksAllOff("test");
  NOTE("SAFE blocks a button %s; SHOW: finger LEDs on %s, the red laser BLOCKED (needs FULL) %s; an off in SAFE done %s; palm group 1 alone %s; the ring in a palette colour (red, then green, a bad one refused) %s; the LiDAR: SHOW blocked, FULL -> the hand plate up and measuring, off -> closed %s; a puck update: the RC frames off, back after 5 min %s; all off %s; arming from a button refused %s",
       safeBlocked ? "y" : "N", showOn ? "y" : "N", laserBlocked ? "y" : "N", offAnyMode ? "y" : "N", g1 ? "y" : "N", ring ? "y" : "N", lidar ? "y" : "N", puckUpd ? "y" : "N", allOff ? "y" : "N", noArm ? "y" : "N");
  result(safeBlocked && showOn && laserBlocked && offAnyMode && g1 && ring && lidar && puckUpd && allOff && noArm, "the puck's control panel: buttons under the gestures' rules");
}

// The puck's CALIBRATE screen ($C ACT_CAL, 2026-10-21): steps 1 / 2 save one pose each at once (the other kept), a finger
// whose swing is too small refuses the step and changes nothing, a step while one runs and a step past 5 are refused - one
// $E each. The IMU steps (3-5) need live sensors and are proved on the arm.
bool gestureCalMv(uint8_t flexIdx, uint16_t& flat, uint16_t& closed);   // gesture.cpp (declared in flex.cpp too)
static void t_calib_steps() {
  auto step = [](uint8_t s) { static uint16_t sq = 100; glink::Command c; c.seq = ++sq; c.act = glink::ACT_CAL; c.a = s; panelCommand(c); };
  modeForce(Mode::Safe); powerFakeVolts(8.0f); thumb(1900); fingers(1750); run(1200);   // any mode: SAFE as well
  step(1); step(2); run(30); bool busyRefused = lastIs(glink::ACT_CAL, glink::RES_REFUSED);   // a second press while step 1 runs
  run(CAL_SETTLE_MS + CAL_AVG_MS + 300);
  uint16_t fl = 0, fx = 0; gestureCalMv(1, fl, fx);
  bool flatSaved = lastIs(glink::ACT_CAL, glink::RES_DONE) && fl > 1700 && fl < 1800;
  thumb(1500); fingers(1250); run(1200);
  step(2); run(CAL_SETTLE_MS + CAL_AVG_MS + 300); gestureCalMv(1, fl, fx);
  bool fistSaved = lastIs(glink::ACT_CAL, glink::RES_DONE) && fx > 1200 && fx < 1300 && fl > 1700;
  thumb(1100); fingers(1750); run(1200); step(3); run(CAL_SETTLE_MS + CAL_AVG_MS + 300);   // step 3: the thumb closed on its own
  bool thumbAlone = lastIs(glink::ACT_CAL, glink::RES_DONE); thumb(1900); run(1200);
  fingers(1740); run(1200);                                                 // a "fist" 10 mV under flat: the swing is too small
  step(2); run(CAL_SETTLE_MS + CAL_AVG_MS + 300); uint16_t fl2 = 0, fx2 = 0; gestureCalMv(1, fl2, fx2);
  bool smallRefused = lastIs(glink::ACT_CAL, glink::RES_REFUSED) && fx2 == fx;
  step(9); run(30); bool badRefused = lastIs(glink::ACT_CAL, glink::RES_REFUSED);
  // (2026-10-21) a replaced sensor: the thumb keeps the old one's numbers (800 / 600) and the new one reads 1900 flat -
  // faulted by the old range, yet steps 1 and 3 take it (Toby's new thumb was refused by every step)
  fingers(1750); gestureSet("tf=800&tc=600"); thumb(1900); run(1500);
  bool oldRangeFault = flexFault(0);
  step(1); run(CAL_SETTLE_MS + CAL_AVG_MS + 300);
  uint16_t ntf = 0, ntc = 0; gestureCalMv(0, ntf, ntc);
  bool newFlat = lastIs(glink::ACT_CAL, glink::RES_DONE) && ntf > 1850 && ntf < 1950;
  run(1500); thumb(1100); run(1200); step(3); run(CAL_SETTLE_MS + CAL_AVG_MS + 300);
  bool newAlone = lastIs(glink::ACT_CAL, glink::RES_DONE);
  bool replaced = oldRangeFault && newFlat && newAlone;
  fingers(1750); thumb(1900); run(1200); modeForce(Mode::Show); run(100);
  NOTE("in SAFE: step 1 (flat) saved %s (index %u mV), a press while it ran refused %s; step 2 (closed) saved %s (index %u mV, flat kept); the thumb alone (step 3) saved %s; a closed 10 mV under flat refused, nothing changed %s; step 9 refused %s; a replaced thumb (old numbers, faulted %s) takes step 1 %s (%u mV) and step 3 %s",
       flatSaved ? "y" : "N", fl, busyRefused ? "y" : "N", fistSaved ? "y" : "N", fx, thumbAlone ? "y" : "N", smallRefused ? "y" : "N", badRefused ? "y" : "N",
       oldRangeFault ? "y" : "N", newFlat ? "y" : "N", ntf, newAlone ? "y" : "N");
  result(flatSaved && busyRefused && fistSaved && thumbAlone && smallRefused && badRefused && replaced, "the puck's CALIBRATE steps: one pose saved at a time, a too-small swing refused, a replaced sensor taken, one $E each");
}

static void t_live_setup() {
  goSafe("self-test: setup"); modeForce(Mode::Show); powerFakeVolts(8.0f); run(1500);   // no hold left over from the tests before
  if (servosPlatesState() != PlatesState::Closed) { servosClose(); run(2500); }
  servoCalRelease(); run(100);
  // (1) a servo's end: nothing saved = config.h's; a held pulse saved wins, a plate move lands on it, Forget brings config.h back
  uint16_t c0 = servoEndUs(0, false), o0 = servoEndUs(0, true);
  uint16_t want = o0 > 1500 ? o0 - 40 : o0 + 40;                         // 40 us in from the open end: inside the envelope
  bool none = !servoCalSaved(0, false) && !servoCalSaved(0, true) && c0 != o0;
  bool saveRefused = servoCalSave(0, true)[0] != 0;                      // nothing held: nothing to save
  bool held = servoCalMove(0, want)[0] == 0 && servoCalHeld() == 0 && servoChannelUs(PLATES[0].ch) == want;
  bool saved = servoCalSave(0, true)[0] == 0 && servoEndUs(0, true) == want && servoEndUs(0, false) == c0 && nvsUShort("so0") == want;
  run(SERVO_CAL_HOLD_MS + 200);
  bool letGo = servoCalHeld() < 0 && servoChannelUs(PLATES[0].ch) == 0;  // a held servo is released after 15 s without a move
  bool plateUsed = servosMovePlate(1, true); run(PLATES[0].moveMs + CASCADE_HOLD_MS + 200);
  plateUsed = plateUsed && servoLastUs(0) == want;                       // the plate opened to the saved end
  servosMovePlate(1, false); run(20);
  bool busyRefused = servoCalMove(1, 1500)[0] != 0;                      // no hand moves while a plate is moving
  run(PLATES[0].moveMs + CASCADE_HOLD_MS + 200);
  servoCalClear(0);
  bool forgot = servoEndUs(0, true) == o0 && !servoCalSaved(0, true) && nvsUShort("so0") == 0;
  // (2) the channel map: a swap each, refused while anything is driven, one-to-one, saved, reset
  bool ident0 = pcaMapIsIdentity();
  servoCalMove(2, 1500);
  bool drivenRefused = pcaMapSet("6:13")[0] != 0 && pcaMapIsIdentity();
  servoCalRelease();
  bool swap = pcaMapSet("6:13")[0] == 0 && pcaPhysical(6) == 13 && pcaPhysical(13) == 6 && pcaPhysical(5) == 5;
  bool chain = pcaMapSet("9=6")[0] == 0 && pcaPhysical(6) == 13 && pcaPhysical(9) == 6 && pcaPhysical(13) == 9;
  bool oneToOne = true; { bool seen[16] = {}; for (uint8_t c = 0; c < 16; c++) { uint8_t p = pcaPhysical(c); if (p > 15 || seen[p]) oneToOne = false; else seen[p] = true; } }
  uint8_t m[16] = {}; { Preferences p; if (p.begin("ironman", true)) { p.getBytes("pcam", m, 16); p.end(); } }
  bool kept = m[6] == 13 && m[9] == 6 && m[13] == 9;
  bool badRefused = pcaMapSet("6:16")[0] != 0 && pcaMapSet("x")[0] != 0 && pcaMapSet("")[0] != 0 && pcaPhysical(9) == 6;
  bool reset = pcaMapSet("reset")[0] == 0 && pcaMapIsIdentity();
  // (2b) the plates switched off: no plate moves, every plate counts as open (the lasers / camera act at once); back on
  servosSetPlatesOff(true);
  bool pOff = false; { Preferences p; if (p.begin("ironman", true)) { pOff = p.getBool("nopl", false); p.end(); } }
  bool offWorks = servosPlatesOff() && pOff && !servosMovePlate(1, true) && servosPlateOpen(1) && servosPlateOpen(4) && !servosBusy();
  servosOpen(); run(50);
  offWorks = offWorks && !servosBusy() && servoChannelUs(PLATES[0].ch) == 0;
  servosSetPlatesOff(false);
  bool onAgain = !servosPlatesOff() && !servosPlateOpen(4);
  // (3) the battery against a meter: the factor scales the reading, out-of-range numbers refused, 0 forgets it
  powerFakeVolts(8.0f); run(1500);
  bool meterBad = powerCalMeter(12.0f)[0] != 0 && powerCalMeter(6.0f)[0] != 0 && powerCalFactor() == 1.0f;
  bool meterSet = powerCalMeter(7.8f)[0] == 0 && fabsf(powerCalFactor() - 0.975f) < 0.002f && fabsf(powerVolts() - 7.8f) < 0.02f;
  run(1500);
  float kSet = powerCalFactor(), vSet = powerVolts();
  bool meterHolds = fabsf(powerVolts() - 7.8f) < 0.02f;
  bool meterForgot = powerCalMeter(0)[0] == 0 && powerCalFactor() == 1.0f && fabsf(powerVolts() - 8.0f) < 0.02f;
  NOTE("servo 0 %s: ends %u/%u, none saved %s, save unheld refused %s; held %u %s, saved OPEN %s, let go %u s %s, "
       "plate opens to it %s, refused mid-move %s, Forget %s; PCA map: identity %s, "
       "refused while driven %s, 6:13 %s, 9=6 %s, 1:1 %s, NVS %s, bad input refused %s, reset %s; plates off %s, on again %s; "
       "battery: 12 / 6 V refused %s, meter 7.8 at 8.0 -> k %.4f reads %.2f %s, holds %s, 0 forgets %s",
       PLATES[0].name, c0, o0, none ? "y" : "N", saveRefused ? "y" : "N", want, held ? "y" : "N", saved ? "y" : "N", (unsigned)(SERVO_CAL_HOLD_MS / 1000),
       letGo ? "y" : "N", plateUsed ? "y" : "N", busyRefused ? "y" : "N", forgot ? "y" : "N", ident0 ? "y" : "N", drivenRefused ? "y" : "N",
       swap ? "y" : "N", chain ? "y" : "N", oneToOne ? "y" : "N", kept ? "y" : "N", badRefused ? "y" : "N", reset ? "y" : "N", offWorks ? "y" : "N", onAgain ? "y" : "N",
       meterBad ? "y" : "N", kSet, vSet, meterSet ? "y" : "N", meterHolds ? "y" : "N", meterForgot ? "y" : "N");
  result(none && saveRefused && held && saved && letGo && plateUsed && busyRefused && forgot && ident0 && drivenRefused && swap && chain && oneToOne &&
         kept && badRefused && reset && offWorks && onAgain && meterBad && meterSet && meterHolds && meterForgot,
         "set on the day: servo ends, the PCA channel map, the battery's meter factor (saved, used, forgotten)");
  servoCalRelease(); servosParkAll();
}

void selftestRun() {
  report = ""; passed = failed = 0;
  Bench.println("=============== SELF-TEST (HW_FAKE) ===============");
  s3linkTestMode(true);                            // the link's frames stay off the wire while the clock is stepped by hand (t_link_nonblocking takes it back for its real-time part)
  t_watchdog();
  t_boot_state();
  t_boot_hot();
  t_tables();
  t_led_cap();
  t_servo_envelope();
  t_cascade();
  t_cascade_abort();
  t_battery_rules();
  t_mode_debounce();
  t_mode_powerup();
  t_safe_mode_refuses();
  t_pucks();
  t_plate_cap();
  t_servo_stall();
  t_outputs();
  t_mode_down();
  t_thermal();
  t_plate_deps();
  t_plate_holds();
  t_laser_plates();
  t_idle_lights();
  t_link_protocol();
  t_s3link();
  t_touch_map();
  t_plate_alone();
  t_link_live();
  t_link_nonblocking();
  t_buzzer();
  t_shutdown();
  t_crsf_frame();
  t_telemetry_parse();
  t_power_lock();
  t_lidar();
  t_ring_gesture();
  t_finger_dip();
  t_flex_fault();
  t_flex_rate();
  t_gesture_blocked();
  t_gesture_map();
  t_arm_refusals();
  t_arm_gesture_and_disarm();
  t_laser_flight_hold();
  t_land_warn();
  t_failsafes();
  t_calibrate();
  t_imu_fixed_zero();
  t_panel();
  t_calib_steps();
  t_live_setup();
  goSafe("self-test done");
  shutdownReset();
  s3linkTestMode(false);
  modeFollowSwitch();
  { Preferences p; if (p.begin("ironman", false)) { p.clear(); p.end(); } }   // the test's calibrations do not outlive it
  Bench.print(report);
  Bench.println(selftestSummary());
  Bench.println("===================================================");
  watchdogHangIfPending();                         // the first boot of a new build: the deliberate hang (the next boot reports it)
}
String selftestReport()  { return report + selftestSummary() + "\n"; }
String selftestSummary() { char b[64]; snprintf(b, sizeof b, "selftest: %u passed, %u failed", passed, failed); return String(b); }
bool   selftestPassed()  { return failed == 0 && passed > 0; }
#endif
