#include "config.h"
#include "fly.h"
#include "crsf.h"
#include "gesture.h"
#include "imu.h"
#include "flex.h"
#include "palm.h"
#include "config.h"
#include "modesw.h"
#include "power.h"
#include "s3link.h"
#include "outputs.h"
#include "gesturemap.h"
#include <GauntletLink.h>
#include <GauntletBenchIO.h>
#include <Preferences.h>
#include <math.h>
#include <stdarg.h>

static uint8_t thrTopPct = FLY_THR_TOP_PCT_DEFAULT, stickMaxPct = FLY_STICK_MAX_PCT_DEFAULT, yawMaxPct = FLY_YAW_MAX_PCT_DEFAULT, thrSensPct = FLY_THR_SENS_PCT_DEFAULT, thrFloorPct = FLY_THR_FLOOR_PCT_DEFAULT;   // live tuning, NVS "fthr" "fstk" "fyaw" "fsen" "fflo"
static uint8_t hoverPct = FLY_THR_HOVER_GUESS_PCT;      // where the drone hovers, % of full at the reference voltage: learned from every flight's mean throttle (NVS "fhv2"); the climb curve bends there
static float   fistAnchor = NAN;                        // the fist's curl when it armed (a tighter fist may still raise it until lift-off; after that it stays put under the hand). NAN = no fist seen yet: throttle stays at minimum
static bool    anchorLocked = false;                    // lift-off has happened: the reference no longer moves (a mid-flight squeeze used to move hover under his fingers - "sometimes quick, sometimes slow", "an atom too closed and it drops")
// The wrist's own range, per axis and direction: full stick at these angles (roll -/+, pitch -/+, yaw -/+), degrees.
// Defaults until the page's step 7 captures them (Toby: "I can go right a lot, left only slightly - map it to my arm").
static float    rngNeg[3] = { FLY_RANGE_DEFAULT_NEG[0], FLY_RANGE_DEFAULT_NEG[1], FLY_RANGE_DEFAULT_NEG[2] }, rngPos[3] = { FLY_RANGE_DEFAULT_POS[0], FLY_RANGE_DEFAULT_POS[1], FLY_RANGE_DEFAULT_POS[2] };   // the defaults until step 7 (NVS "frng")
static bool     rngSet = false, rngOn = false;
static uint32_t rngUntil = 0;                           // a range capture runs until then
static float    rngMin[3], rngMax[3];                   // its running extremes ...
static float    rngStartSum[3]; static uint16_t rngStartN[3];   // ... and the pose it started from (the mean of the first FLY_RANGE_START_MS): the range is how far the wrist MOVED each way, not where it happened to sit (a glove resting upside down read "tilt 173" as an extreme, 2026-10-01)
static const char* AXIS[3] = { "tilt", "bend", "turn" };
static void tuningLoad() {
  Preferences p; if (!p.begin("ironman", true)) return;
  thrTopPct = p.getUChar("fthr", FLY_THR_TOP_PCT_DEFAULT); stickMaxPct = p.getUChar("fstk", FLY_STICK_MAX_PCT_DEFAULT); yawMaxPct = p.getUChar("fyaw", FLY_YAW_MAX_PCT_DEFAULT); thrSensPct = p.getUChar("fsen", FLY_THR_SENS_PCT_DEFAULT); thrFloorPct = p.getUChar("fflo", FLY_THR_FLOOR_PCT_DEFAULT);
  hoverPct = p.getUChar("fhv2", FLY_THR_HOVER_GUESS_PCT);
  float r[6]; if (p.getBytes("frng", r, sizeof r) == sizeof r) { for (int k = 0; k < 3; k++) { rngNeg[k] = r[2 * k]; rngPos[k] = r[2 * k + 1]; } rngSet = true; }
  p.end();
}
static void tuningSave() {
  Preferences p; if (!p.begin("ironman", false)) return;
  p.putUChar("fthr", thrTopPct); p.putUChar("fstk", stickMaxPct); p.putUChar("fyaw", yawMaxPct); p.putUChar("fsen", thrSensPct); p.putUChar("fflo", thrFloorPct); p.putUChar("fhv2", hoverPct);
  if (rngSet) { float r[6]; for (int k = 0; k < 3; k++) { r[2 * k] = rngNeg[k]; r[2 * k + 1] = rngPos[k]; } p.putBytes("frng", r, sizeof r); } else p.remove("frng");
  p.end();
}
static bool craftLow = false; static uint32_t craftLowSince = 0;   // the drone's own battery under FLY_CRAFT_LOW_V: it comes down by itself
static bool craftWarn = false; static uint32_t craftWarnSince = 0, warnDipT0 = 0, lowSince = 0;   // ... under FLY_CRAFT_WARN_V first: the throttle dips twice every few seconds and the ring blinks slowly - "so I notice it"
static bool warnDip = false;                                        // this cycle's dip, applied to the sent throttle after the filter so it is felt
static float vFilt = 0, vComp = 1.0f;                                // the drone's voltage, filtered, and the throttle factor FLY_VREF / vFilt it gives (1 without telemetry)

static bool     armed = false;
static bool     bench = false;                         // a bench arm (flyBenchArm): the page's ARM, timed
static uint32_t benchUntil = 0;
static uint32_t armedAt = 0, lastPollMs = 0;
static uint32_t closedSince = 0, closedEnd = 0, openSince = 0, thumbSince = 0, linkLostSince = 0;
static float    thr = 0, roll = 0, pitch = 0, yaw = 0;        // the sticks as sent: throttle 0..1 of full, roll/pitch -1..1 (after the filters)
static float    curlMean = NAN, curlThumb = NAN;       // inputs this cycle
static uint32_t flexLostSince = 0;                     // (Ironman) since when two or more of the four fingers are faulted
static float    wFlex = NAN, wTwist = NAN, wSide = NAN;
static float    yawZero = 0;                           // the side angle at the moment of arming: yaw is measured from there (the heading-based side angle drifts; a -22 % yaw with a straight hand showed on the first flight)
static float    trimRoll = 0, trimPitch = 0;           // the hand's roll and the wrist's flex at the moment of arming (capped at FLY_TRIM_MAX_DEG): that pose is neutral
static float    hRoll = NAN, hPitch = NAN;              // the hand against its page step-1 pose (level palm down): "genuinely horizontal"
static float    rawRoll = NAN, rollIn = NAN, pitchIn = NAN, yawIn = NAN, lastRollIn = 0;   // the stick inputs this cycle, degrees: raw roll (hand vs gravity, or wrist twist), and the three after the trims
static bool     handFresh = false, handLevel = false, inLine = false;
static uint32_t openTook = 0;                          // (second cut) how long the last closed -> open took, ms
static uint32_t pulseStart = 0, pulseLen = 0, wasOpenMs = 0;   // the squeeze pulse: when the hand left open, how long it took to be open again
static float    pulseFrom = 0, fistTop = 0;            // the fist the pulse left from (its tightest), for a quick flick's depth
static bool     pulseThumbIn = false;                  // (2026-10-21) the thumb was tucked in as the pulse began: that is the plates' close-all (gesturemap.cpp), not an arm
static float    pulsePeak = 0;                         // the deepest mean curl inside the pulse
static bool     pending = false;                       // a pulse completed: judge it once the hand has stayed open FLY_ARM_OPEN_MS
static uint32_t waitReadySince = 0;                    // the gesture was good but the drone said '!' - waiting up to FLY_ARM_WAIT_READY_MS for '*' with the fist held
static uint32_t nearSince = 0, nearOpenMs = 0, weakLogMs = 0; static float nearPeak = 0;
// the hover number (2026-10-03, "the only thing you need is the hover number ... do a test where I hover it as best I can and you
// average it"): every flight's throttle is averaged without its take-off and landing; a HOVER TEST takes that as the hover
// number outright, an ordinary flight blends it in (FLY_HOVER_BLEND)
static bool     hoverTest = false; static uint8_t hoverTestDone = 0;
static float    hovSum = 0, hovTime = 0; static constexpr uint8_t HOV_RING = 120; static float hovRing[HOV_RING], hovRingDt[HOV_RING]; static uint8_t hovIdx = 0;   // the last ~2.4 s, taken off again at disarm
static uint32_t accSentMs = 0;                         // the page's "level the drone": when the calibration command went out (the feed says whether the drone acknowledged)   // a "fist" that never reached FLY_CLOSED_CURL, opened and closed again: the gesture as a shifted glove reads it - recorded, so "it just won't arm" has a reason (2026-10-03)
static uint32_t lastDisarm = 0;                        // the thumb disarm-triple count when it armed: a later one disarms (or stops a flip-over)
static bool     lifted = false;                        // armed and the fingers have lifted once (FLY_THR_LIFT_X): the floor applies. Before that the motors idle - no buzzing on the ground after a gesture arm
static uint32_t cutSince = 0, cuts = 0;                // a deliberate cut: a tight fist held FLY_THR_CUT_MS
// flight stats, printed at disarm: time lifted, mean commanded throttle (at the reference voltage), the wrist's extremes used
static float    stPitchSum = 0, stRollSum = 0, stStickT = 0;   // this flight's mean pitch / roll stick, for the flight log ("it was really hard to make it come back")
static float    stThrSum = 0, stTime = 0, stMin[3], stMax[3]; static uint8_t stMeanPct = 0;
// the drone's own readiness, for the log: when it starts and stops refusing ('!' in its mode string)
static bool     craftWasBlocked = false; static uint32_t craftBlockedSince = 0;
// every arm attempt, with what was measured, so "why won't it arm" is answered on the page, not guessed
struct Try { uint32_t ms; uint16_t fist, took; int8_t hr, hp, fl, sd, tw; uint8_t curl; uint8_t old; char result[72]; };   // fist = the pulse's length ms, took = its peak curl %; old = from before the last power-up
static Try  tries[4]; static uint8_t nTries = 0;       // newest first
// The records live in NVS as well, because Toby power-cycles the glove between sessions and the one time it mattered
// (eight refused arms after a battery change, 2026-10-01) every record was gone before anyone looked.
static void triesSave() { Preferences p; if (!p.begin("ironman", false)) return; p.putBytes("ftry", tries, sizeof tries); p.putUChar("ftryn", nTries); p.end(); }
static void triesLoad() {
  Preferences p; if (!p.begin("ironman", true)) return;
  if (p.getBytes("ftry", tries, sizeof tries) == sizeof tries) { nTries = p.getUChar("ftryn", 0); if (nTries > 4) nTries = 4; for (uint8_t k = 0; k < nTries; k++) { tries[k].old = 1; tries[k].result[sizeof tries[k].result - 1] = 0; } }
  p.end();
  if (nTries) Bench.printf("fly: %u arm tr%s kept from before this power-up - newest: fist-open-fist %u ms, out to %u %%, hand roll %d pitch %d, wrist flex %d side %d twist %d -> %s\n", nTries, nTries == 1 ? "y" : "ies", (unsigned)tries[0].fist, (unsigned)tries[0].took, tries[0].hr, tries[0].hp, tries[0].fl, tries[0].sd, tries[0].tw, tries[0].result);
}
static uint32_t fistShortLogMs = 0;
static void recordTry(uint32_t pulseMs, float peak, const char* result) {
  for (int k = 3; k > 0; k--) tries[k] = tries[k - 1];
  Try& t = tries[0];
  t.ms = millis(); t.fist = pulseMs > 65535 ? 65535 : pulseMs; t.took = (uint16_t)lroundf(fmaxf(0, fminf(100, peak * 100)));
  t.hr = isnan(hRoll) ? 0 : (int8_t)lroundf(fmaxf(-99, fminf(99, hRoll))); t.hp = isnan(hPitch) ? 0 : (int8_t)lroundf(fmaxf(-99, fminf(99, hPitch)));
  t.fl = isnan(wFlex) ? 0 : (int8_t)lroundf(fmaxf(-99, fminf(99, wFlex))); t.sd = isnan(wSide) ? 0 : (int8_t)lroundf(fmaxf(-99, fminf(99, wSide))); t.tw = isnan(wTwist) ? 0 : (int8_t)lroundf(fmaxf(-99, fminf(99, wTwist)));
  t.curl = isnan(curlMean) ? 255 : (uint8_t)lroundf(curlMean * 100);
  snprintf(t.result, sizeof t.result, "%s", result);
  t.old = 0;
  if (nTries < 4) nTries++;
  triesSave();
  Bench.printf("fly: arm try - fist-open-fist %u ms, out to %u %% curl, hand roll %d pitch %d, wrist flex %d side %d twist %d, fingers now %u %% -> %s\n",
               (unsigned)t.fist, (unsigned)t.took, t.hr, t.hp, t.fl, t.sd, t.tw, t.curl == 255 ? 0 : t.curl, result);
}
static uint32_t arms = 0, disarms = 0;
static char     why[96] = "boot";                      // the last arm / disarm reason
static uint32_t whyMs = 0;
static char     block[200] = "";                       // why it cannot arm right now (for the pages); "" = it could
static bool     blockWaitable = false;                 // the current block is the drone not being ready yet ('!', telemetry not fresh): a good gesture waits for it
static uint32_t lastBlockLogMs = 0;

static float expo(float x, float e) { return (1 - e) * x + e * x * x * x; }                     // x in -1..1, same sign out
static uint16_t stickCh(float s)   { return (uint16_t)lroundf(CRSF_MID + s * (CRSF_MAX - CRSF_MID)); }   // -1..1 -> 173..1811
static uint16_t thrCh(float t)     { return (uint16_t)lroundf(CRSF_MIN + t * (CRSF_MAX - CRSF_MIN)); }   // 0..1 -> 172..1811
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }
// ---- the flight log (2026-10-03): "it fell out of the sky, what happened?" had no answer - the 4 KB console ring had rolled
// over in a minute of gesture chatter and the glove had been power-cycled. The last FLY_EVENTS events (arm, lift-off, disarm
// and why, battery stages, flip-overs, a cut) live in NVS with the throttle, the drone's voltage and attitude at that moment.
// GET :8080/fly?ev=1; the page shows them under the drone row; the boot log prints the newest.
static constexpr uint8_t FLY_EVENTS = 10;
struct FlyEv { uint32_t ms; uint16_t boot; uint8_t thr, v10; int8_t ar, ap; char what[46]; };
static FlyEv    evs[FLY_EVENTS]; static uint8_t nEvs = 0; static uint16_t bootNo = 0;
static void evSave() { Preferences p; if (!p.begin("ironman", false)) return; p.putBytes("fev", evs, sizeof evs); p.putUChar("fevn", nEvs); p.end(); }
static void evLoad() {
  Preferences p; if (!p.begin("ironman", false)) return;
  bootNo = (uint16_t)(p.getUShort("fboot", 0) + 1); p.putUShort("fboot", bootNo);
  if (p.getBytes("fev", evs, sizeof evs) == sizeof evs) { nEvs = p.getUChar("fevn", 0); if (nEvs > FLY_EVENTS) nEvs = FLY_EVENTS; for (uint8_t k = 0; k < nEvs; k++) evs[k].what[sizeof evs[k].what - 1] = 0; }
  p.end();
}
static void flyEvent(const char* fmt, ...) {
  for (int k = FLY_EVENTS - 1; k > 0; k--) evs[k] = evs[k - 1];
  FlyEv& e = evs[0];
  e.ms = millis(); e.boot = bootNo; e.thr = (uint8_t)lroundf(clampf(thr, 0, 1) * 100);
  float v = crsfCraftVolts(); e.v10 = (v > 0.5f && v < 25) ? (uint8_t)lroundf(v * 10) : 0;
  float r = 0, pp = 0, yy = 0; uint32_t a = 0;
  if (crsfAttitude(r, pp, yy, a) && a <= 2000) { e.ar = (int8_t)lroundf(clampf(r, -127, 127)); e.ap = (int8_t)lroundf(clampf(pp, -127, 127)); } else { e.ar = 0; e.ap = 0; }
  va_list va; va_start(va, fmt); vsnprintf(e.what, sizeof e.what, fmt, va); va_end(va);
  if (nEvs < FLY_EVENTS) nEvs++;
  evSave();
}
void flyNote(const char* fmt, ...) { char t[46]; va_list va; va_start(va, fmt); vsnprintf(t, sizeof t, fmt, va); va_end(va); flyEvent("%s", t); }
static float wrap180(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
// The forearm-hand heading, solved when the drone arms and again at lift-off (2026-10-03 evening - Toby: "it was actually
// the software that was wobbling it, make sure that doesn't happen again"). The two BNO085s share "up" but not a heading;
// the drift healer keeps them aligned while disarmed, and when it guessed wrong (the glove off on the desk, a relaxed wrist
// twisted, the arm hanging down) raising or lowering the arm leaked into pitch and yaw in flight - the wobble. At arming and
// at lift-off the hand is held out in line with the forearm, so the wrist's side angle is zero by definition: turn the
// hand's heading until it is. Flight then never depends on what happened while disarmed. Skipped (the old heading stays)
// when the forearm is so steep or rolled that the side angle hardly depends on the heading.
static float solveWristHeading() {
  float r0, p0, y0, r1, p1, y1;
  if (!imuRelativeShifted(0, r0, p0, y0) || !imuRelativeShifted(10 * DEG_TO_RAD, r1, p1, y1)) return NAN;
  float k = wrap180(y1 - y0) / 10.0f;                      // side degrees per degree of heading: about +-1 with the forearm level
  if (fabsf(k) < FLY_HEADING_SOLVE_MIN_GAIN) return NAN;
  float e = -wrap180(y0) / k;
  for (int it = 0; it < 3; it++) { float rr, pp, yy; if (!imuRelativeShifted(e * DEG_TO_RAD, rr, pp, yy)) return NAN; if (fabsf(wrap180(yy)) < 0.5f) break; e -= wrap180(yy) / k; }
  { float rr, pp, yy; if (!imuRelativeShifted(e * DEG_TO_RAD, rr, pp, yy) || fabsf(wrap180(yy)) > 5) return NAN; }   // did not converge: keep the old heading
  e = wrap180(e);
  imuShiftHandYaw(e);
  float tw, fl, sd; uint32_t age;
  if (imuRelative(tw, fl, sd, age)) { wTwist = tw; wFlex = fl; wSide = sd; }   // this cycle's wrist with the new heading, for the trims that follow
  return e;
}
// wrist angle -> stick: dead band, then linear (FLY_STICK_EXPO 0) up to maxOut at that direction's own full angle
static float stickFrom(float deg, int8_t sign, float fullNeg, float fullPos, float maxOut, float dead) {
  if (isnan(deg)) return 0;
  float a = fabsf(deg);
  if (a <= dead) return 0;
  float full = deg < 0 ? fullNeg : fullPos; if (full <= dead + 1) full = dead + 1;
  float u = (a - dead) / (full - dead); if (u > 1) u = 1;
  float s = expo(u, FLY_STICK_EXPO) * maxOut;
  return (deg < 0 ? -s : s) * sign;
}

// ---- the wrist's range (the page's step 7): the extremes the wrist reaches in FLY_RANGE_CAPTURE_MS, full stick at a fraction of each
static float rangeFull(float extreme) {   // an extreme reached -> the angle for full stick; too small = that way was not shown, keep the default (NAN)
  if (isnan(extreme) || extreme < FLY_RANGE_NEED_DEG) return NAN;
  return clampf(extreme * FLY_RANGE_USE, FLY_RANGE_MIN_DEG, FLY_RANGE_MAX_DEG);
}
static void rangeFinish() {
  rngOn = false;
  char line[240]; int n = 0, kept = 0;
  for (int k = 0; k < 3; k++) {
    bool any = rngMin[k] <= rngMax[k];
    float start = rngStartN[k] ? rngStartSum[k] / rngStartN[k] : (any ? (rngMin[k] + rngMax[k]) / 2 : 0);   // where the wrist sat when the capture began
    float dn = any ? rangeFull(start - rngMin[k]) : NAN, up = any ? rangeFull(rngMax[k] - start) : NAN;        // how far it went each way from there
    float def = k == 2 ? FLY_YAW_FULL_DEG : FLY_STICK_FULL_DEG;
    rngNeg[k] = isnan(dn) ? def : dn; rngPos[k] = isnan(up) ? def : up;
    if (!isnan(dn)) kept++; if (!isnan(up)) kept++;
    if (any) n += snprintf(line + n, sizeof line - n, "%s%s %.0f/+%.0f from %.0f -> full at -%.0f/+%.0f", k ? ", " : "", AXIS[k], rngMin[k] - start, rngMax[k] - start, start, rngNeg[k], rngPos[k]);
    else     n += snprintf(line + n, sizeof line - n, "%s%s no data", k ? ", " : "", AXIS[k]);
    if (n >= (int)sizeof line - 1) break;
  }
  rngSet = kept > 0; tuningSave();
  Bench.printf("fly: wrist range captured - %s deg (%u of 6 directions; a direction under %.0f deg keeps the default)\n", line, kept, FLY_RANGE_NEED_DEG);
}
static String rangeStart() {
  if (armed) return "fly: wrist range - not while armed";
  if (!handFresh) return "fly: wrist range - hand or forearm sensor quiet";
  rngOn = true; rngUntil = millis() + FLY_RANGE_CAPTURE_MS; for (int k = 0; k < 3; k++) { rngMin[k] = 1e9f; rngMax[k] = -1e9f; rngStartSum[k] = 0; rngStartN[k] = 0; }
  Bench.printf("fly: wrist range - %u s: tilt, bend and turn the hand as far as you comfortably go, every direction\n", (unsigned)(FLY_RANGE_CAPTURE_MS / 1000));
  return String("fly: wrist range - move the wrist to every extreme for ") + (FLY_RANGE_CAPTURE_MS / 1000) + " s";
}
static String rangeForget() {
  rngSet = false; rngOn = false; for (int k = 0; k < 3; k++) { rngNeg[k] = FLY_RANGE_DEFAULT_NEG[k]; rngPos[k] = FLY_RANGE_DEFAULT_POS[k]; } tuningSave();
  Bench.println("fly: wrist range forgotten - the defaults again");
  return String("fly: wrist range forgotten - the defaults: full stick at tilt -") + (int)FLY_RANGE_DEFAULT_NEG[0] + "/+" + (int)FLY_RANGE_DEFAULT_POS[0] + ", bend -" + (int)FLY_RANGE_DEFAULT_NEG[1] + "/+" + (int)FLY_RANGE_DEFAULT_POS[1] + ", turn -" + (int)FLY_RANGE_DEFAULT_NEG[2] + "/+" + (int)FLY_RANGE_DEFAULT_POS[2] + " deg";
}

// Why the drone may not be armed right now - nullptr when it may. Every reason is in words the
// pages show, because the answer to "why won't it arm" must not need the console. forFlip = the
// flip-over after a crash: the drone lying over is the point, so its level check is skipped.
static const char* armBlock(bool withHand = true, bool forFlip = false) {   // withHand false = the page's ARM: the drone's checks only
  static char b[200];
  blockWaitable = false;
  if (!modeAllowsArming()) { snprintf(b, sizeof b, "mode %s - the switch must be on FULL", modeNowName()); return b; }
  if (!powerArmOk()) { snprintf(b, sizeof b, "%s", powerBlockText()); return b; }
  if (!crsfStreaming()) return "RC frames are off (key C)";
  if (!crsfModuleHeard()) return "module silent - powered? CRSF wire on GPIO 23? in WiFi mode (power-cycle it)?";
  if (!crsfLinkUp() || crsfLQ() < FLY_MIN_LQ) { snprintf(b, sizeof b, "no link to the drone (LQ %u) - drone on? bound?", crsfLQ()); return b; }
  if (FLY_NEED_TELEMETRY) {
    blockWaitable = true;                                                  // from here on the drone may simply need a moment
    if (!crsfFcSeen() || crsfFcModeAgeMs() > FLY_TELEM_FRESH_MS) return "no telemetry from the drone (Betaflight: Receiver tab, telemetry on)";
    if (crsfCraftBlocked()) { char fl[150]; crsfArmingFlagText(fl, sizeof fl); snprintf(b, sizeof b, "the drone refuses to arm ('%s'%s%s)", crsfFcMode(), fl[0] ? ": " : " - asking it why", fl); return b; }   // Betaflight's own flags, over MSP, when it has answered
    if (crsfCraftArmed()) return "the drone says it is already armed";
    float r, p, y; uint32_t age;
    if (!crsfAttitude(r, p, y, age) || age > FLY_TELEM_FRESH_MS) return "no attitude from the drone yet";
    blockWaitable = false;
    if (!forFlip && (fabsf(r) > FLY_LEVEL_DEG || fabsf(p) > FLY_LEVEL_DEG)) { snprintf(b, sizeof b, "the drone is not level (roll %.0f, pitch %.0f deg)", r, p); return b; }
  }
  if (!withHand) return nullptr;
  if (!handFresh) return "hand or forearm sensor quiet";
  if (isnan(curlMean)) return "fingers not calibrated (steps 4 and 5 on the page)";
  if (!handLevel) { snprintf(b, sizeof b, "hand not level (roll %.0f, pitch %.0f deg - step 1 is level)", hRoll, hPitch); return b; }
  if (!inLine) { snprintf(b, sizeof b, "wrist not in line with the forearm (flex %.0f, side %.0f, twist %.0f deg)", wFlex, wSide, wTwist); return b; }
  if (curlMean < FLY_CLOSED_CURL) { snprintf(b, sizeof b, "make a fist (fingers at %u %%, a fist is %u %% or more) - level, wrist straight - then open and close the fingers quickly", (unsigned)(curlMean * 100), (unsigned)(FLY_CLOSED_CURL * 100 + 0.5f)); return b; }
  return nullptr;
}

static void note(const char* w) { snprintf(why, sizeof why, "%s", w); whyMs = millis(); }

// ---- flip-over after a crash: Betaflight's "flip over after crash" (turtle mode), AUX3 on this whoop. The drone
// lies over, he does the arm gesture: flip mode on, ARM with the throttle low (the motors take their reversed
// direction), then short pushes of the stick - about the axis it lies over on, one way, the other, the other axis -
// until its own attitude says upright; then disarm at once and flip mode off. He arms again to fly.
static uint8_t  flipPhase = 0;                         // 0 idle, 1 mode on, 2 armed and waiting, 3 pushing
static uint32_t flipT0 = 0, flipPhaseT = 0, flips = 0, flipsOk = 0;
static uint8_t  flipPush = 0;                          // pushes done
static uint8_t  flipTried = 1;                         // directions tried on the current axis (1 or 2)
static float    flipBefore = NAN;                      // how far over it lay before the current push (max of |roll|, |pitch|): a push that gains FLY_FLIP_GAIN_DEG is repeated
static int8_t   flipAxis = 0, flipDir = +1;            // 0 roll / 1 pitch, sign of the push: + = stick right / forward = Betaflight spins the motors on that side, which lifts that side when inverted
static bool     flipArmOut = false;
static float    flipRollCmd = 0, flipPitchCmd = 0;
static bool craftAttitude(float& r, float& p) { float y; uint32_t age; if (!crsfAttitude(r, p, y, age) || age > FLY_TELEM_FRESH_MS) return false; return true; }
static bool craftInverted() { float r, p; return craftAttitude(r, p) && (fabsf(r) > FLY_FLIP_INVERTED_DEG || fabsf(p) > FLY_FLIP_INVERTED_DEG); }
static bool craftUpright()  { float r, p; return craftAttitude(r, p) && fabsf(r) <= FLY_FLIP_UPRIGHT_DEG && fabsf(p) <= FLY_FLIP_UPRIGHT_DEG; }
static void flipEnd(const char* how, bool ok) {
  if (!flipPhase) return;
  flipPhase = 0; flipArmOut = false; flipRollCmd = flipPitchCmd = 0;
  crsfSetSticks(CRSF_MID, CRSF_MID, CRSF_MIN, CRSF_MID); crsfSetAux(FLY_ARM_AUX, false); crsfSetAux(FLY_FLIP_AUX, false);
  if (ok) flipsOk++;
  note(how);
  float r = 0, p = 0; craftAttitude(r, p);
  flyEvent("flip-over %s: %s", ok ? "worked" : "ended", how);
  Bench.printf("fly: flip-over %s - %s after %lu ms (the drone reads roll %.0f pitch %.0f). %s\n", ok ? "DONE" : "ended", how, (unsigned long)(millis() - flipT0), r, p, ok ? "arm again to fly" : "pick it up and set it down the right way up");
}
static void flipStart(const char* how) {
  float r = 0, p = 0; craftAttitude(r, p);
  flipPhase = 1; flipT0 = flipPhaseT = millis(); flipPush = 0; flips++; flipArmOut = false; flipRollCmd = flipPitchCmd = 0; flipTried = 1; flipBefore = fmaxf(fabsf(r), fabsf(p));
  flipAxis = fabsf(r) >= fabsf(p) ? 0 : 1; flipDir = (flipAxis == 0 ? r : p) >= 0 ? +1 : -1;   // first about the axis it lies furthest over on, toward the sign it reports; then the other way, the other axis, the other way
  lastDisarm = gestureDisarmCount();
  crsfSetSticks(CRSF_MID, CRSF_MID, CRSF_MIN, CRSF_MID); crsfSetAux(FLY_ARM_AUX, false); crsfSetAux(FLY_FLIP_AUX, true);   // the mode first: Betaflight reads the switch when it arms
  note("flipping it back over");
  flyEvent("flip-over started (lying at roll %.0f pitch %.0f)", r, p);
  Bench.printf("fly: FLIP-OVER (%s): the drone lies at roll %.0f pitch %.0f. flip mode on (AUX%u), arming with the throttle low, then pushes of %.0f %% stick for %u ms - %s %s first. STAND CLEAR OF THE PROPS. pointing + two thumb presses or DISARM stop it\n",
               how, r, p, (unsigned)(FLY_FLIP_AUX - 3), FLY_FLIP_PUSH * 100, (unsigned)FLY_FLIP_PUSH_MS, flipAxis ? "pitch" : "roll", flipDir > 0 ? "+" : "-");
}
static void flipPoll(uint32_t now) {
  const char* d = nullptr;
  if (gestureDisarmCount() != lastDisarm) { lastDisarm = gestureDisarmCount(); d = "two thumb presses - flip-over stopped"; }
  else if (!crsfLinkUp()) d = "link to the drone lost - flip-over stopped";
  else if (crsfFcModeAgeMs() > FLY_TELEM_LOST_MS) d = "the drone's telemetry stopped - flip-over stopped";
  else if (now - flipT0 > FLY_FLIP_MAX_MS) d = "it took too long";
  if (d) { flipEnd(d, false); return; }
  if (flipPhase >= 2 && craftUpright()) { flipEnd("it is the right way up", true); return; }
  if (flipPhase == 1) { if (now - flipPhaseT >= FLY_FLIP_MODE_LEAD_MS) { flipPhase = 2; flipPhaseT = now; flipArmOut = true; Bench.println("fly: flip-over: armed in flip mode, motors reversed, pushing in a moment"); } }
  else if (flipPhase == 2) { if (now - flipPhaseT >= FLY_FLIP_ARM_LEAD_MS) { flipPhase = 3; flipPhaseT = now; float r = 0, p = 0; if (craftAttitude(r, p)) flipBefore = fmaxf(fabsf(r), fabsf(p)); } }
  else {
    uint32_t t = now - flipPhaseT;
    float cmd = t < FLY_FLIP_PUSH_MS ? FLY_FLIP_PUSH * flipDir : 0;   // a full-stick jab, then centred for a pause long enough for a fresh attitude frame
    flipRollCmd = flipAxis == 0 ? cmd : 0; flipPitchCmd = flipAxis == 1 ? cmd : 0;
    if (t >= FLY_FLIP_PUSH_MS + FLY_FLIP_PAUSE_MS) {                 // the pause is over: what did that push do?
      flipPush++; flipPhaseT = now;
      float r = 0, p = 0; bool have = craftAttitude(r, p); float over = have ? fmaxf(fabsf(r), fabsf(p)) : NAN;
      bool gained = have && !isnan(flipBefore) && over <= flipBefore - FLY_FLIP_GAIN_DEG;
      if (flipPush >= FLY_FLIP_MAX_PUSHES) { char m[48]; snprintf(m, sizeof m, "%u pushes and it is still over", (unsigned)flipPush); flipEnd(m, false); return; }
      const char* next;
      if (gained) next = "it moved - the same way again";                                                   // keep lifting the side that is coming up
      else if (flipTried < 2) { flipDir = -flipDir; flipTried = 2; next = "no gain - the other way"; }
      else { flipAxis = 1 - flipAxis; flipTried = 1; flipDir = (flipAxis == 0 ? r : p) >= 0 ? +1 : -1; next = "no gain either way - the other axis"; }
      flipBefore = over;
      Bench.printf("fly: flip-over push %u done (it reads roll %.0f pitch %.0f, %s): push %u = %s %s\n", (unsigned)flipPush, r, p, next, (unsigned)flipPush + 1, flipAxis ? "pitch" : "roll", flipDir > 0 ? "+" : "-");
    }
  }
  crsfSetSticks(stickCh(flipRollCmd), stickCh(flipPitchCmd), CRSF_MIN, CRSF_MID);
  crsfSetAux(FLY_ARM_AUX, flipArmOut); crsfSetAux(FLY_FLIP_AUX, true);
}

static void arm(const char* how) {
  armed = true; armedAt = millis(); arms++;
  thr = roll = pitch = yaw = 0; lifted = false; anchorLocked = false; cutSince = 0;
  float hs = FLY_SOLVE_HEADING && handFresh ? solveWristHeading() : NAN;   // the wrist's side angle is zero now: the forearm-hand heading follows from it
  yawZero = isnan(wSide) ? 0 : wSide;
  trimRoll  = FLY_TRIM_AT_ARM && !isnan(rawRoll) ? clampf(rawRoll, -FLY_TRIM_MAX_DEG, FLY_TRIM_MAX_DEG) : 0;   // the pose it armed in is neutral (as yaw already was)
  trimPitch = FLY_TRIM_AT_ARM && !isnan(wFlex)   ? clampf(wFlex,   -FLY_TRIM_MAX_DEG, FLY_TRIM_MAX_DEG) : 0;
  fistAnchor = (!isnan(curlMean) && curlMean >= 0.35f) ? curlMean : NAN;   // the fist that armed is the reference; the page's ARM with an open hand has none yet - the throttle waits at minimum for the first fist
  pending = false; pulseStart = 0; wasOpenMs = 0; waitReadySince = 0;
  lastDisarm = gestureDisarmCount();                   // (this and the two above sat inside a trailing comment from the fifth round until 2026-10-01: a thumb double or ring triple made while DISARMED disarmed the next arm at once)
  stThrSum = 0; stTime = 0; for (int k = 0; k < 3; k++) { stMin[k] = 1e9f; stMax[k] = -1e9f; }
  stPitchSum = stRollSum = stStickT = 0; gestureFlightClicksReset();
  hovSum = 0; hovTime = 0; hovIdx = 0; for (uint8_t k = 0; k < HOV_RING; k++) hovRingDt[k] = 0;
  crsfSetAux(FLY_FLIP_AUX, false); crsfSetAux(FLY_ARM_AUX, true);
  note(how);
  { char hb[20] = ""; if (!isnan(hs)) snprintf(hb, sizeof hb, ", heading %+.0f", hs); flyEvent("ARMED (%s%s)", bench ? "page" : "gesture", hb); }
  Bench.printf("fly: *** ARMED *** (%s). fist %s %% of travel; trim roll %+.0f pitch %+.0f yaw %+.0f deg. motors idle until the fingers lift; then a fist = %u %% (gentle descent), hover about %u %%, full lift = %u %%; tilt the hand = roll, bend the wrist = pitch, turn it = yaw; POINT (INDEX UP, OTHER THREE CLOSED) + TWO THUMB PRESSES = disarm\n",
               how, isnan(fistAnchor) ? "not seen yet" : String((int)lroundf(fistAnchor * 100)).c_str(), trimRoll, trimPitch, yawZero, (unsigned)thrFloorPct, (unsigned)hoverPct, (unsigned)thrTopPct);
}

void flyDisarm(const char* w) {
  bool was = armed;
  if (flipPhase) { flipEnd(w, false); return; }
  if (was) {                                           // the flight's numbers first, so the disarm stays the newest line of the flight log
    char cb[48]; if (gestureFlightClicks(cb, sizeof cb)) flyNote("%s", cb);
    if (stStickT >= 2.0f) flyNote("sticks avg pitch %+d roll %+d %%, bend %+.0f/%+.0f", (int)lroundf(stPitchSum / stStickT * 100), (int)lroundf(stRollSum / stStickT * 100), stMin[1] <= stMax[1] ? stMin[1] : 0.0f, stMin[1] <= stMax[1] ? stMax[1] : 0.0f);
    flyEvent("disarm: %s (%.0f s live)", w, stTime);   // before the throttle is zeroed: the event keeps what it was
  }   // before the throttle is zeroed: the event keeps what it was
  armed = false; bench = false;
  thr = roll = pitch = yaw = 0;
  crsfSetSticks(CRSF_MID, CRSF_MID, CRSF_MIN, CRSF_MID);
  crsfSetAux(FLY_ARM_AUX, false); crsfSetAux(FLY_FLIP_AUX, false);
  thumbSince = 0; closedSince = 0; closedEnd = 0; openSince = 0; pulseStart = 0; pending = false; wasOpenMs = 0; waitReadySince = 0;
  lifted = false; anchorLocked = false; cutSince = 0;
  if (was) {
    disarms++; note(w);
    if (stTime >= 2.0f) {                              // what the flight used, for the tuning: hover is about the mean throttle of a flight that starts and ends on the ground
      stMeanPct = (uint8_t)lroundf(stThrSum / stTime * 100);
      { float cut = 0; uint8_t k = hovIdx;                                  // take the landing off the hover average: the last FLY_HOVER_SKIP_TAIL_S
        for (uint8_t n = 0; n < HOV_RING && cut < FLY_HOVER_SKIP_TAIL_S; n++) { k = (uint8_t)((k + HOV_RING - 1) % HOV_RING); if (hovRingDt[k] <= 0) break; hovSum -= hovRing[k]; hovTime -= hovRingDt[k]; cut += hovRingDt[k]; hovRingDt[k] = 0; } }
      uint8_t hm = hovTime > 1.0f ? (uint8_t)lroundf(hovSum / hovTime * 100) : 0;   // the middle of the flight
      char hs[120] = "";
      if (hoverTest && hm) { hoverTest = false; hoverTestDone = hm; if (hm != hoverPct) { hoverPct = hm; tuningSave(); } snprintf(hs, sizeof hs, " - HOVER TEST: hover = %u %% (saved; fingers in the middle of their lift give exactly that now)", hm); }
      else if (hoverTest) { hoverTest = false; snprintf(hs, sizeof hs, " - hover test: flight too short to measure, still armed for the next one"); hoverTest = true; }
      else if (stTime >= FLY_HOVER_LEARN_MIN_S && hm >= 10 && hm <= thrTopPct) { uint8_t nh = (uint8_t)lroundf(hoverPct * (1 - FLY_HOVER_BLEND) + hm * FLY_HOVER_BLEND); snprintf(hs, sizeof hs, " - hover estimate %u -> %u %%", (unsigned)hoverPct, (unsigned)nh); if (nh != hoverPct) { hoverPct = nh; tuningSave(); } }
      Bench.printf("fly: DISARMED (%s). flight: %.0f s with the throttle live, mean %u %%, middle %u %% at the %.1f V reference%s; wrist used tilt %.0f..%+.0f, bend %.0f..%+.0f, turn %.0f..%+.0f deg from the arming pose\n",
                   w, stTime, (unsigned)stMeanPct, (unsigned)hm, FLY_VREF, hs,
                   stMin[0] <= stMax[0] ? stMin[0] : 0.0f, stMin[0] <= stMax[0] ? stMax[0] : 0.0f, stMin[1] <= stMax[1] ? stMin[1] : 0.0f, stMin[1] <= stMax[1] ? stMax[1] : 0.0f, stMin[2] <= stMax[2] ? stMin[2] : 0.0f, stMin[2] <= stMax[2] ? stMax[2] : 0.0f);
    } else Bench.printf("fly: DISARMED (%s)\n", w);
  }
  trimRoll = trimPitch = 0;
}

void flyEnable(bool on, const char* w) {       // no separate flight mode any more (Toby 2026-09-30 night: "make fly on and arm test the same thing, fly off and disarm the same"): armed = the hand is the sticks and the ring / laser gestures are off
  if (!on) { flyDisarm(w); return; }
  Bench.println("fly: there is no flight mode switch - ARM (the page / key A) arms through the drone's checks, the squeeze gesture arms, DISARM / a thumb crunch disarm");
}
// The page's "arm test": the drone's checks but not the gesture's, then the glove flies it like a gesture
// arm would (fingers = throttle, wrist = roll / pitch) when the fingers are calibrated and the hand
// sensors are fresh - with props off that is how the mapping gets proven; sticks held still otherwise.
// With the drone lying over it starts the flip-over instead.
String flyBenchArm(uint32_t seconds) {
  if (armed) return "fly: already armed";
  if (flipPhase) return "fly: flipping it over right now";
  if (seconds < 3) seconds = FLY_BENCH_ARM_S; if (seconds > 120) seconds = 120;
  if (craftInverted()) {
    const char* b = armBlock(false, true);
    if (b) { String m = String("fly: flip-over refused - ") + b; Bench.println(m); return m; }
    flipStart("the page's ARM");
    return "fly: the drone lies over - FLIPPING IT BACK OVER (flip mode + reversed motors, a few short pushes). Stand clear. Arm again to fly once it is upright";
  }
  const char* b = armBlock(false);
  if (b) { String m = String("fly: arm test refused - ") + b; Bench.println(m); return m; }
  bool handDriven = handFresh && !isnan(curlMean);
  bench = true; benchUntil = millis() + seconds * 1000;
  arm(handDriven ? "arm test - the glove is flying it: fingers = throttle, hand = roll / pitch / yaw" : "arm test - hand not calibrated or not fresh, sticks held still");   // the hand may be open: the motors idle until the first fist, then until the fingers lift
  Bench.printf("fly: arm test - disarms by itself in %lu s, or pointing + two thumb presses / D / the page's DISARM. PROPS OFF. the drone's mode string should drop its '*'\n", (unsigned long)seconds);
  return String("fly: ARMED for ") + seconds + " s - " + (handDriven ? "make a fist once, then lift the fingers = up, fist = gentle descent; tilt = roll, bend = pitch, turn = yaw; pointing + two thumb presses disarm" : "sticks held still (hand not calibrated / not fresh)") + ". PROPS OFF";
}
bool flyEnabled() { return armed || flipPhase != 0; }
bool flyRangeCapturing() { return rngOn; }   // step 7 running: the laser's heading healer stands down (it would erase the turns being measured)
float flyFingerCurl() { return curlMean; }   // (2026-10-21) the four fingers as one, this cycle - gesturemap.cpp's fist / pulse gestures
bool flyArmed()   { return armed; }

void flyBegin() {
  crsfSafe();
  tuningLoad();
  triesLoad();
  evLoad();
  if (nEvs) {
    Bench.printf("fly: flight log (this is power-up %u), newest first:\n", (unsigned)bootNo);
    for (uint8_t k = 0; k < nEvs && k < 6; k++) Bench.printf("  power-up %u, %lu s in: %s (thr %u %%, %.1f V, drone roll %d pitch %d)\n", (unsigned)evs[k].boot, (unsigned long)(evs[k].ms / 1000), evs[k].what, (unsigned)evs[k].thr, evs[k].v10 / 10.0f, evs[k].ar, evs[k].ap);
  }
  Bench.printf("fly: disarmed at boot. throttle = fingers (fist in flight = %u %% floor, hover about %u %%, full lift = %u %%, lift span from sensitivity %u %%), roll = the hand's tilt, pitch = wrist bend, yaw = hand turned (full stick at tilt -%.0f/+%.0f, bend -%.0f/+%.0f, turn -%.0f/+%.0f deg%s), arm = fist - quick open - fist, disarm = pointing + two thumb presses; the same gesture with the drone lying over flips it back (AUX%u)\n",
               (unsigned)thrFloorPct, (unsigned)hoverPct, (unsigned)thrTopPct, (unsigned)thrSensPct, rngNeg[0], rngPos[0], rngNeg[1], rngPos[1], rngNeg[2], rngPos[2], rngSet ? ", from step 7" : ", defaults", (unsigned)(FLY_FLIP_AUX - 3));
}

void flyPoll() {
  uint32_t now = millis();
  if (now - lastPollMs < CRSF_PERIOD_MS) return;
  float dt = lastPollMs ? (now - lastPollMs) / 1000.0f : 0.02f; if (dt > 0.1f) dt = 0.1f;
  lastPollMs = now;
  { float v = crsfCraftVolts();
    if (v > 2.5f && crsfLinkUp()) { if (vFilt < 2.5f) vFilt = v; else vFilt += (v - vFilt) * (dt / FLY_VOLT_TAU_S > 1 ? 1 : dt / FLY_VOLT_TAU_S); vComp = FLY_VREF / vFilt; if (vComp < FLY_VCOMP_MIN) vComp = FLY_VCOMP_MIN; if (vComp > FLY_VCOMP_MAX) vComp = FLY_VCOMP_MAX; }
    else { vFilt = 0; vComp = 1.0f; }
    bool linked = crsfLinkUp();
    bool warnNow = v > 0.5f && v <= FLY_CRAFT_WARN_V && linked, lowNow = v > 0.5f && v <= FLY_CRAFT_LOW_V && linked;
    if (warnNow) { if (!craftWarnSince) craftWarnSince = now; if (!craftWarn && now - craftWarnSince >= FLY_CRAFT_LOW_MS) { craftWarn = true; warnDipT0 = now; palmAlertRate(PALM_WARN_BLINK_MS); palmAlert(true); flyEvent("battery getting low %.2f V", v); Bench.printf("fly: drone battery GETTING LOW %.2f V - ring blinking red slowly, the throttle dips twice every %lu s in flight; land soon\n", v, (unsigned long)(FLY_WARN_EVERY_MS / 1000)); } }
    else { craftWarnSince = 0; if (craftWarn && !craftLow && (v > FLY_CRAFT_WARN_V + 0.1f || !linked)) { craftWarn = false; palmAlert(false); } }
    if (lowNow) { if (!craftLowSince) craftLowSince = now; if (!craftLow && now - craftLowSince >= FLY_CRAFT_LOW_MS) { craftLow = true; lowSince = now; palmAlertRate(300); palmAlert(true); flyEvent("battery LOW %.2f V - descending", v); Bench.printf("fly: DRONE BATTERY LOW %.2f V - bringing it down: the throttle eases to the slow-descent throttle over %.0f s and stays there; steer it to the floor, then two thumb presses; ring blinking red fast\n", v, FLY_LOW_DESCEND_S); } }
    else { craftLowSince = 0; if (craftLow && (v > FLY_CRAFT_LOW_V + 0.1f || !linked)) { craftLow = false; if (craftWarn && linked) palmAlertRate(PALM_WARN_BLINK_MS); else { craftWarn = false; palmAlert(false); } } } }
  if (crsfWatchdogTripped()) { Bench.println("fly: the CRSF task took the sticks down itself - loop() had stopped feeding it"); if (armed || flipPhase) flyDisarm("loop stalled"); }
  { bool bl = crsfLinkUp() && crsfFcSeen() && crsfFcModeAgeMs() <= FLY_TELEM_FRESH_MS && crsfCraftBlocked();   // the drone's own readiness, logged on every change: the first arms after a disarm or a power-up met '!' for a few seconds
    if (bl != craftWasBlocked) {
      if (bl) { craftBlockedSince = now; Bench.printf("fly: the drone says it will not arm ('%s') - %s\n", crsfFcMode(), armed ? "while we are armed" : whyMs ? String(String((now - whyMs) / 1000) + " s after '" + why + "'").c_str() : "at boot"); }
      else Bench.printf("fly: the drone says ready ('%s') again after %lu ms\n", crsfFcMode(), (unsigned long)(now - craftBlockedSince));
      craftWasBlocked = bl;
    } }

  // ---- inputs: the four fingers, the thumb, the wrist, the hand
  float cs[4]; int n = 0, bad = 0;
  for (uint8_t i = 1; i <= 4; i++) { if (flexFault(i)) { bad++; continue; } float c = gestureFingerCurl(i); if (!isnan(c)) cs[n++] = c; }   // (Ironman, 2026-10-08) a FAULTED finger is left out
  if (n == 3 && bad == 1) {                              // one finger faulted (broken, shorted): the median of the other three
    for (int a = 1; a < 3; a++) for (int b = a; b > 0 && cs[b] < cs[b - 1]; b--) { float t = cs[b]; cs[b] = cs[b - 1]; cs[b - 1] = t; }
    curlMean = cs[1];
  } else if (n == 4) {                                          // the mean of the MIDDLE TWO of the four (2026-10-07): when all four move together it is the mean, and one finger reading wrong - his middle finger reads closed when he straightens it - drops out instead of pulling the throttle a quarter of the way
    for (int a = 1; a < 4; a++) for (int b = a; b > 0 && cs[b] < cs[b - 1]; b--) { float t = cs[b]; cs[b] = cs[b - 1]; cs[b - 1] = t; }
    curlMean = (cs[1] + cs[2]) / 2;
  } else curlMean = NAN;                                 // all four or nothing: a missing finger would bias the throttle (two faulted: nothing - disarmed below)
  if (bad >= 2) { if (!flexLostSince) flexLostSince = now; } else flexLostSince = 0;
  curlThumb = gestureFingerCurl(0);
  float tw, fl, sd; uint32_t age;
  bool rel = imuRelative(tw, fl, sd, age);               // the wrist: twist, flexion (+ fingers down), side (+ left)
  handFresh = rel && age <= FLY_HAND_STALE_MS;
  wTwist = handFresh ? tw : NAN; wFlex = handFresh ? fl : NAN; wSide = handFresh ? sd : NAN;
  float hr, hp, hy; uint32_t hage;
  bool hz = imuSegAngles(1, hr, hp, hy, hage) && hage <= FLY_HAND_STALE_MS;   // against the page's step-1 pose (the console's zero pose is a different mount, 2026-09-22)
  hRoll = hz ? hr : NAN; hPitch = hz ? hp : NAN;
  handLevel = hz && fabsf(hr) <= FLY_ARM_LEVEL_DEG && fabsf(hp) <= FLY_ARM_LEVEL_DEG;
  inLine = handFresh && fabsf(fl) <= FLY_ARM_FLEX_DEG && fabsf(sd) <= FLY_ARM_SIDE_DEG && fabsf(tw) <= FLY_ARM_TWIST_DEG;
  // the stick inputs: roll = the hand's own tilt against gravity (FLY_ROLL_ABSOLUTE - tilting the whole arm counts; a palm-down wrist can hardly roll further one way on its own), pitch = wrist flexion, yaw = the hand turned on the forearm
  rawRoll = FLY_ROLL_ABSOLUTE ? hRoll : wTwist;
  if (FLY_ROLL_ABSOLUTE && !isnan(hPitch) && fabsf(hPitch) > FLY_ROLL_HOLD_PITCH_DEG) rawRoll = NAN;   // the hand near vertical: roll is ill-defined there, hold the last
  if (!isnan(rawRoll)) { rollIn = rawRoll - trimRoll; lastRollIn = rollIn; } else rollIn = armed && handFresh ? lastRollIn : NAN;
  pitchIn = isnan(wFlex) ? NAN : wFlex - trimPitch;
  yawIn   = isnan(wSide) ? NAN : wSide - yawZero;
  if (rngOn) {                                           // step 7 running: the extremes
    float v[3] = { rawRoll, wFlex, wSide };
    bool starting = (int32_t)(rngUntil - now) > (int32_t)(FLY_RANGE_CAPTURE_MS - FLY_RANGE_START_MS);   // the first FLY_RANGE_START_MS: where the wrist sits before it moves
    for (int k = 0; k < 3; k++) if (!isnan(v[k])) { if (v[k] < rngMin[k]) rngMin[k] = v[k]; if (v[k] > rngMax[k]) rngMax[k] = v[k]; if (starting) { rngStartSum[k] += v[k]; rngStartN[k]++; } }
    if ((int32_t)(now - rngUntil) >= 0) rangeFinish();
  }
  if (flipPhase) { flipPoll(now); return; }              // the flip-over owns the sticks and the AUX channels until it ends

  // ---- arming and disarming
  if (armed) {
    const char* d = nullptr;
    if (bench && (int32_t)(now - benchUntil) >= 0) d = "arm test time is up";
    else if (!handFresh && !bench) d = "hand or forearm sensor quiet";
    else if (!bench && flexLostSince && now - flexLostSince > FLY_FLEX_LOST_MS) d = "finger sensors lost (two or more faulted - the ADS1115 silent?)";   // (Ironman, 2026-10-08)
    else if (gestureDisarmCount() != lastDisarm) { lastDisarm = gestureDisarmCount(); d = "pointing + two thumb presses"; s3linkAct(glink::ACT_DISARM, glink::RES_DONE, 2); hapticFeedback(1); }   // pointing + two thumb presses (gesture.cpp counts the second the moment it reaches the size)
    if (!d) { if (!crsfLinkUp()) { if (!linkLostSince) linkLostSince = now; if (now - linkLostSince > 1000) d = "link to the drone lost"; } else linkLostSince = 0; }
    if (!d && FLY_NEED_TELEMETRY && crsfFcModeAgeMs() > FLY_TELEM_LOST_MS) d = "the drone's telemetry stopped";
    if (!d && !modeAllowsArming()) d = "the mode switch left FULL";
    if (!d && powerIdleForced()) d = powerBlockText();
    { float ar, ap, ay; uint32_t aage; if (!d && crsfAttitude(ar, ap, ay, aage) && aage <= FLY_TELEM_FRESH_MS && (fabsf(ar) > FLY_FLIP_DEG || fabsf(ap) > FLY_FLIP_DEG)) d = "the drone flipped over"; }   // its own attitude past FLY_FLIP_DEG: off at once
    if (d) flyDisarm(d);
  } else {                                               // disarmed: watch for the arm gesture - from a fist, a quick open-and-close of the four fingers
    linkLostSince = 0;
    bool fistNow = !isnan(curlMean) && curlMean >= FLY_CLOSED_CURL;
    if (pulseStart == 0) {                               // waiting: the hand must be in a fist first ...
      if (fistNow) { if (!wasOpenMs || now - wasOpenMs > 150) fistTop = curlMean; else if (curlMean > fistTop) fistTop = curlMean; wasOpenMs = now; }   // (when it was last in the fist, and how tight it got)
      else if (wasOpenMs && now - wasOpenMs <= 150 && !isnan(curlMean)) { pulseStart = now; pulsePeak = curlMean; pulseFrom = fistTop; pulseThumbIn = gmapThumbIn(); }   // ... and has just left it: a pulse may be starting
    } else {
      if (!isnan(curlMean) && curlMean < pulsePeak) pulsePeak = curlMean;   // the lowest curl = how far the fingers came out
      if (fistNow) { pulseLen = now - pulseStart; openSince = now; pulseStart = 0; wasOpenMs = now; pending = true; waitReadySince = 0; }   // back in the fist: the pulse is complete, judge it after the hold
      else if (now - pulseStart > FLY_ARM_PULSE_MS) {    // never closed again in time: the hand just opened, not an arm gesture
        char r[72]; snprintf(r, sizeof r, "refused: the fist opened but did not close again inside %u ms", (unsigned)FLY_ARM_PULSE_MS);   // recorded, so "nothing happens" has numbers
        if (pulsePeak <= 0.60f) recordTry(now - pulseStart, pulsePeak, r);
        pulseStart = 0; wasOpenMs = 0;
      }
    }
    // the near miss: fingers that only got part way to a fist, came out and closed part way again - nothing used to be recorded for that
    if (!isnan(curlMean) && pulseStart == 0 && !pending) {
      if (curlMean >= FLY_CLOSED_CURL) { nearSince = 0; nearOpenMs = 0; }                                             // a real fist: the normal path has it
      else if (!nearSince) { if (curlMean >= FLY_WEAK_FIST_CURL) { nearSince = now; nearPeak = curlMean; nearOpenMs = 0; } }
      else {
        if (!nearOpenMs) { if (curlMean > nearPeak) nearPeak = curlMean; if (curlMean <= nearPeak - FLY_WEAK_FIST_DROP) nearOpenMs = now; if (now - nearSince > 4000) nearSince = 0; }   // closing further, or came out; a long part-fist is just a relaxed hand
        else if (now - nearOpenMs > FLY_ARM_PULSE_MS) { nearSince = 0; nearOpenMs = 0; }                           // stayed out: an open hand
        else if (curlMean >= nearPeak - FLY_WEAK_FIST_BACK) {                                                      // back to where it was: that was an attempt
          if (now - weakLogMs > 3000) { weakLogMs = now; char r[72]; snprintf(r, sizeof r, "refused: the fist only got to %u %% (a fist is %u %%) - squeeze more, or redo step 5", (unsigned)lroundf(nearPeak * 100), (unsigned)(FLY_CLOSED_CURL * 100 + 0.5f)); recordTry(now - nearOpenMs, nearPeak, r); }
          nearSince = 0; nearOpenMs = 0;
        }
      }
    }
    bool over = craftInverted();                         // the drone lies over: the same gesture flips it back instead of arming
    const char* b = armBlock(true, over);
    bool waitable = blockWaitable;
    if (!waitReadySince) snprintf(block, sizeof block, "%s", b ? b : over ? "the drone lies over - the arm gesture will flip it back (stand clear)" : "");
    if (pending && !fistNow) {                           // opened again before the hold was up, or let go while waiting: that was not the end of it
      if (waitReadySince) { char r[72]; snprintf(r, sizeof r, "refused: let go while waiting for the drone (%s)", b ? "still not ready" : "it had just got ready"); recordTry(pulseLen, pulsePeak, r); }
      pending = false; waitReadySince = 0;
    }
    if (pending && (pulseThumbIn || (gmapCameraDoubleMs() && now - gmapCameraDoubleMs() < 1000))) {   // (2026-10-21) Toby's map: the drone's pulse has the thumb OUT and comes alone
      Bench.printf("fly: that pulse was %s - not an arm\n", pulseThumbIn ? "made with the thumb in (the plates' close-all)" : "the second of two quick ones (the thermal camera)");
      pending = false; waitReadySince = 0;
    }
    if (pending && now - openSince >= FLY_ARM_OPEN_MS && gmapTwoFingers()) {   // ... and ends in a real fist - judged once the hold is up: closing, the fingers pass through that shape for a moment
      Bench.println("fly: the hand ended in the two-finger pose (the green laser's) - not an arm");
      pending = false; waitReadySince = 0;
    }
    if (pending && now - openSince >= FLY_ARM_OPEN_MS) {
      static char weak[80];
      if (pulsePeak > FLY_PULSE_OPEN_CURL && pulseFrom - pulsePeak < FLY_PULSE_DROP) { snprintf(weak, sizeof weak, "open the fingers more - %u to %u %% curl (need %u less, or %u)", (unsigned)(pulseFrom * 100), (unsigned)(pulsePeak * 100), (unsigned)(FLY_PULSE_DROP * 100 + 0.5f), (unsigned)(FLY_PULSE_OPEN_CURL * 100 + 0.5f)); b = weak; waitable = false; }   // out to FLY_PULSE_OPEN_CURL, OR a quick flick FLY_PULSE_DROP down from the fist (2026-10-07: "I have to do it fairly slow now" - a quick one only came out to 35 %)
      if (!b) {
        pending = false;
        char r[72]; snprintf(r, sizeof r, "%s%s", over ? "FLIP-OVER" : "ARMED", waitReadySince ? String(String(" after waiting ") + (now - waitReadySince) + " ms for the drone").c_str() : "");
        recordTry(pulseLen, pulsePeak, r);
        if (over) flipStart("the arm gesture with the drone lying over"); else arm("a fist, then a quick open-and-close, level and in line");
        s3linkAct(over ? glink::ACT_FLIP : glink::ACT_ARM, glink::RES_DONE, 2); hapticFeedback(1);   // (2026-10-21) one buzz: armed
        waitReadySince = 0;
      } else if (waitable && (!waitReadySince || now - waitReadySince < FLY_ARM_WAIT_READY_MS)) {   // a good gesture, the drone just not ready: hold the fist, it arms the moment the drone says so
        if (!waitReadySince) { waitReadySince = now; Bench.printf("fly: gesture good, but %s - waiting up to %u ms with the fist held\n", b, (unsigned)FLY_ARM_WAIT_READY_MS); }
        snprintf(block, sizeof block, "hold the fist - waiting for the drone (%s)", b);
      } else {
        pending = false;
        char r[72]; snprintf(r, sizeof r, "refused: %s%s", b, waitReadySince ? " (waited 3 s)" : ""); recordTry(pulseLen, pulsePeak, r); snprintf(block, sizeof block, "%s", b);
        s3linkAct(over ? glink::ACT_FLIP : glink::ACT_ARM, modeAllowsArming() ? glink::RES_REFUSED : glink::RES_BLOCKED, 2); hapticFeedback(2);   // the gesture was seen; below FULL the mode blocked it - two buzzes
        waitReadySince = 0;
      }
    }
  }
  // ---- the throttle: the fist is the reference. Motors idle until the fingers have lifted once; from then on a fist is the
  // FLOOR (a gentle descent, never a drop - Toby: "there is never a time I want to drop the throttle that quickly"), hover
  // where the flights have shown it, the range top at a full lift - gentle just above hover, the full climb only at the
  // top (FLY_THR_CLIMB_GAMMA: "never go up super fast"). A tight fist held FLY_THR_CUT_MS is a deliberate cut (landing).
  float tTarget = 0;
  float top = thrTopPct / 100.0f, floorT = thrFloorPct / 100.0f; if (floorT > top * FLY_THR_FLOOR_MAX_OF_TOP) floorT = top * FLY_THR_FLOOR_MAX_OF_TOP;
  float hover = clampf(hoverPct / 100.0f, 0.10f, top - 0.005f);
  if (armed && !isnan(curlMean)) {
    if (isnan(fistAnchor)) { if (curlMean >= 0.35f) { fistAnchor = curlMean; Bench.printf("fly: fist seen at %u %% of finger travel - that is the throttle's reference\n", (unsigned)(curlMean * 100)); } }   // no fist seen yet (the page's ARM with an open hand)
    else if (!anchorLocked && curlMean > fistAnchor) fistAnchor = curlMean;                                        // before lift-off a tighter fist may still raise the reference; after it the map stays put under the hand
    if (!isnan(fistAnchor)) {
      float span = 0.04f + 0.60f * (1.0f - thrSensPct / 100.0f);                                                  // the lift that covers the whole range: sensitivity 0 = 64 % of the fingers' travel, 100 = 4 %
      float lift = (fistAnchor - FLY_THR_FIST_DEAD) - curlMean;                                                    // how far the fingers have come up from the fist (a slightly looser fist is still the fist)
      float x = lift / span; if (x < 0) x = 0; if (x > 1) x = 1;
      if (!lifted) {
        if (x >= FLY_THR_LIFT_X) {
          lifted = true; anchorLocked = true; cutSince = 0;
          float hs = FLY_SOLVE_HEADING && handFresh ? solveWristHeading() : NAN;   // again in the flying pose, before the trims are taken
          if (FLY_TRIM_AT_LIFT && handFresh) {                                                                      // the neutral sticks = the pose the hand is in NOW, lifting off - his flying pose, not the arming fist (a fist drops the wrist 16 deg, the open flying hand came up 45: a steady pitch command all flight, 2026-10-03)
            if (!isnan(rawRoll)) trimRoll = clampf(rawRoll, -FLY_TRIM_MAX_DEG, FLY_TRIM_MAX_DEG);
            if (!isnan(wFlex))   trimPitch = clampf(wFlex, -FLY_TRIM_MAX_DEG, FLY_TRIM_MAX_DEG);
            if (!isnan(wSide))   yawZero = wSide;
          }
          Bench.printf("fly: throttle live - the fingers lifted. fist (%u %% of travel) = %u %%, the middle of a %u %% lift = hover %u %%, a full lift = %u %%; the reference is locked now; neutral = this pose (trim roll %+.0f pitch %+.0f yaw %+.0f deg)\n", (unsigned)lroundf(fistAnchor * 100), (unsigned)lroundf(fminf(floorT, hover * FLY_FIST_FACTOR) * 100), (unsigned)lroundf(span * 100), (unsigned)lroundf(hover * 100), (unsigned)thrTopPct, trimRoll, trimPitch, yawZero);
          { char hb[20] = ""; if (!isnan(hs)) snprintf(hb, sizeof hb, ", heading %+.0f", hs); flyEvent("lift-off (hover %u %%%s)", (unsigned)lroundf(hover * 100), hb); }
        }
      } else {
        bool tight = FLY_THR_CUT_ENABLED && curlMean >= fistAnchor + FLY_THR_CUT_OVER && curlMean >= FLY_THR_CUT_MIN;   // (off since 2026-10-03: the same squeeze as "come down" - it dropped the drone from height)                      // a deliberate squeeze well past the arming fist ...
        if (tight) { if (!cutSince) cutSince = now; if (now - cutSince >= FLY_THR_CUT_MS) { lifted = false; cutSince = 0; cuts++; flyEvent("throttle CUT (a tight fist held)"); Bench.println("fly: throttle CUT - a tight fist held. motors idle; lift the fingers to fly again, pointing + two thumb presses to disarm"); } }
        else cutSince = 0;
      }
      if (lifted) {
        // "anyone can fly it" (2026-10-03): the MIDDLE of the lift is hover - a band FLY_HOVER_BAND each side of it gives
        // exactly the hover number, so finger wander there does nothing; above the band a slow climb up to the range top at
        // a full lift, below it a slow descent down to the fist throttle (FLY_FIST_FACTOR x hover, never above the floor)
        float fistT = fminf(floorT, hover * FLY_FIST_FACTOR), lo = 0.5f - FLY_HOVER_BAND, hi = 0.5f + FLY_HOVER_BAND;
        if (x >= lo && x <= hi) tTarget = hover;
        else if (x > hi) tTarget = hover + (top - hover) * ((x - hi) / (1 - hi));
        else tTarget = hover - (hover - fistT) * ((lo - x) / lo);
        if (craftLow) {                                                                                             // LOW: down it comes whatever the fingers say - to the fist throttle over FLY_LOW_DESCEND_S, then held there until he disarms on the floor; the fingers can only ask for less
          float t = (now - lowSince) / 1000.0f, cap;
          if (t < FLY_LOW_DESCEND_S) cap = hover - (hover - fistT) * (t / FLY_LOW_DESCEND_S);
          else cap = fistT;                                                                                       // ... and stays there: no timed cut to zero (2026-10-03 - "it must have landed by now" is a guess that can drop it from height)
          if (tTarget > cap) tTarget = cap;
        }
        warnDip = false;
        if (craftWarn && !craftLow) {                                                                               // GETTING LOW: two quick dips every FLY_WARN_EVERY_MS, "bup bup", applied after the filter below so they are felt
          uint32_t ph = (now - warnDipT0) % FLY_WARN_EVERY_MS;
          warnDip = ph < FLY_WARN_DIP_MS || (ph >= FLY_WARN_DIP_MS + FLY_WARN_DIP_GAP_MS && ph < 2 * FLY_WARN_DIP_MS + FLY_WARN_DIP_GAP_MS);
        }
        stThrSum += tTarget * dt; stTime += dt;
        if (stTime > FLY_HOVER_SKIP_HEAD_S) { hovSum += tTarget * dt; hovTime += dt; hovRing[hovIdx] = tTarget * dt; hovRingDt[hovIdx] = dt; hovIdx = (uint8_t)((hovIdx + 1) % HOV_RING); }   // for the hover number: after the take-off (the landing comes off at disarm)
        tTarget *= vComp; if (tTarget > 0.95f) tTarget = 0.95f;                                                    // the same thrust on a fresh or a tired cell (FLY_VREF / voltage)
      }
    }
  }
  bool hand = armed && handFresh && !isnan(curlMean);    // the glove flies it, test arm or gesture arm; without a fresh calibrated hand the sticks hold still
  float k = dt / FLY_FILTER_S; if (k > 1) k = 1;        // the sticks follow the hand through one short filter
  float kt = dt / FLY_THR_FILTER_S; if (kt > 1) kt = 1; // the throttle through a longer one ("an average, still real time") ...
  float sm = stickMaxPct / 100.0f, ym = yawMaxPct / 100.0f;
  { float d = (tTarget - thr) * kt; float step = (d > 0 ? FLY_THR_SLEW_UP : FLY_THR_SLEW_DOWN) * dt; if (d > step) d = step; if (d < -step) d = -step; thr = hand ? thr + d : 0; }   // ... and never faster than the slew rates: up slower than down ("never go up super fast")
  roll  = hand ? roll  + (stickFrom(rollIn,  FLY_ROLL_SIGN,  rngNeg[0], rngPos[0], sm, FLY_STICK_DEAD_DEG) - roll)  * k : 0;
  pitch = hand ? pitch + (stickFrom(pitchIn, FLY_PITCH_SIGN, rngNeg[1], rngPos[1], sm, FLY_STICK_DEAD_DEG) - pitch) * k : 0;
  yaw   = hand ? yaw   + (stickFrom(yawIn,   FLY_YAW_SIGN,   rngNeg[2], rngPos[2], ym, FLY_YAW_DEAD_DEG)   - yaw)   * k : 0;
  if (armed && lifted && hand) { stPitchSum += pitch * dt; stRollSum += roll * dt; stStickT += dt; }
  if (armed && lifted) { float v[3] = { rollIn, pitchIn, yawIn }; for (int i = 0; i < 3; i++) if (!isnan(v[i])) { if (v[i] < stMin[i]) stMin[i] = v[i]; if (v[i] > stMax[i]) stMax[i] = v[i]; } }
  float thrOut = thr; if (warnDip && lifted) { thrOut -= FLY_WARN_DIP; if (thrOut < 0) thrOut = 0; }
  crsfSetSticks(stickCh(roll), stickCh(pitch), thrCh(thrOut), stickCh(yaw));
  crsfSetAux(FLY_ARM_AUX, armed);
}

static String queryValue(const String& q, const char* name) {
  String key = String(name) + "=";
  int p = 0;
  while (p <= (int)q.length()) {
    int e = q.indexOf('&', p); if (e < 0) e = q.length();
    String part = q.substring(p, e);
    if (part.startsWith(key)) return part.substring(key.length());
    p = e + 1;
  }
  return String();
}

static String jsonStr(const char* s) {        // no quotes or backslashes in our strings, but keep it honest
  String o; for (const char* p = s; *p; p++) { if (*p == '"' || *p == '\\') o += '\\'; o += *p; } return o;
}

static String flyEventsJson() {             // GET :8080/fly?ev=1 - newest first: [power-ups ago, seconds ago (this power-up) or seconds in (an earlier one), throttle %, volts x10, drone roll, pitch, what]
  String s = "{\"boot\":" + String(bootNo) + ",\"ev\":[";
  for (uint8_t k = 0; k < nEvs; k++) {
    const FlyEv& e = evs[k];
    bool cur = e.boot == bootNo;
    char b[96]; snprintf(b, sizeof b, "%s[%u,%lu,%u,%u,%d,%d,\"", k ? "," : "", (unsigned)(uint16_t)(bootNo - e.boot), cur ? (unsigned long)((millis() - e.ms) / 1000) : (unsigned long)(e.ms / 1000), (unsigned)e.thr, (unsigned)e.v10, e.ar, e.ap);
    s += b; s += jsonStr(e.what); s += "\"]";
  }
  return s + "]}";
}

String flyJson() {
  float ar = 0, ap = 0, ay = 0; uint32_t aage = 0; bool att = crsfAttitude(ar, ap, ay, aage);
  char tempTxt[16]; { uint32_t tage; float tc = crsfTempC(&tage, nullptr); if (isnan(tc) || tage > 10000) strlcpy(tempTxt, "null", sizeof tempTxt); else snprintf(tempTxt, sizeof tempTxt, "%.1f", tc); }
  char coreTxt[12]; { uint32_t cage; float cc = crsfCoreTempC(&cage); if (isnan(cc) || cage > 15000) strlcpy(coreTxt, "null", sizeof coreTxt); else snprintf(coreTxt, sizeof coreTxt, "%.0f", cc); }
  uint8_t ver[3]; crsfModuleVersion(ver);
  char b[1400];
  snprintf(b, sizeof b, "{\"en\":%d,\"armed\":%d,\"hold\":%d,\"thr\":%.2f,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"curl\":%.2f,\"thumb\":%.2f,\"wf\":%.0f,\"wt\":%.0f,\"hand\":%d,"
                        "\"ch\":[%u,%u,%u,%u,%u],\"frames\":%u,\"stream\":%d,\"heard\":%d,\"age\":%u,\"lq\":%u,\"rssi\":%d,\"rf\":%u,\"dlq\":%u,"
                        "\"mode\":\"%s\",\"fcage\":%u,\"volts\":%.1f,\"pct\":%u,\"att\":[%.0f,%.0f],\"attage\":%u,\"arms\":%u,\"ago\":%u,\"bind\":%u,\"bench\":%u,\"thrmax\":%u,\"thrsens\":%u,\"anchor\":%d,\"vcomp\":%.2f,\"stickmax\":%u,\"yawmax\":%u,\"vlow\":%d,\"hr\":%.0f,\"hp\":%.0f,\"lvl\":%d,\"inl\":%d,\"took\":%u,"
                        "\"floor\":%u,\"hover\":%u,\"lift\":%d,\"cuts\":%u,\"trim\":[%.0f,%.0f,%.0f],\"in\":[%.0f,%.0f,%.0f],\"rng\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],\"rngset\":%d,\"rngt\":%u,\"fmean\":%u,\"flip\":%u,\"flips\":%u,\"flipok\":%u,\"over\":%d,\"wait\":%u,\"htest\":%u,\"hdone\":%u,\"acccal\":%u,\"accms\":%u,\"vwarn\":%d,\"lowt\":%u,\"amps\":%.1f,\"atty\":%.0f,\"temp\":%s,\"core\":%s,\"cycle\":%u,\"load\":%u,\"modname\":\"%s\",\"elrs\":\"%u.%u.%u\",\"pwr\":%u,",
           (armed || flipPhase) ? 1 : 0, armed ? 1 : 0, (armed && isnan(fistAnchor)) ? 1 : 0, thr, roll, pitch, yaw, isnan(curlMean) ? -1.0f : curlMean, isnan(curlThumb) ? -1.0f : curlThumb,
           isnan(wFlex) ? 0.0f : wFlex, isnan(wTwist) ? 0.0f : wTwist, handFresh ? 1 : 0,
           crsfChannel(0), crsfChannel(1), crsfChannel(2), crsfChannel(3), crsfChannel(FLY_ARM_AUX), (unsigned)crsfFramesSent(), crsfStreaming() ? 1 : 0,
           crsfModuleHeard() ? 1 : 0, (unsigned)crsfModuleAgeMs(), crsfLQ(), crsfRssi(), crsfRfMode(), crsfDownLQ(),
           crsfFcMode(), crsfFcSeen() ? (unsigned)crsfFcModeAgeMs() : 0, crsfCraftVolts(), crsfCraftPct(), att ? ar : 0.0f, att ? ap : 0.0f, att ? (unsigned)aage : 0,
           (unsigned)arms, whyMs ? (unsigned)(millis() - whyMs) : 0, (unsigned)crsfBindBursts(), (armed && bench) ? (unsigned)((benchUntil - millis() + 999) / 1000) : 0, (unsigned)thrTopPct, (unsigned)thrSensPct, isnan(fistAnchor) ? -1 : (int)lroundf(fistAnchor * 100), vComp, (unsigned)stickMaxPct, (unsigned)yawMaxPct, craftLow ? 1 : 0,
           isnan(hRoll) ? 0.0f : hRoll, isnan(hPitch) ? 0.0f : hPitch, handLevel ? 1 : 0, inLine ? 1 : 0, (unsigned)openTook,
           (unsigned)thrFloorPct, (unsigned)hoverPct, (armed && lifted) ? 1 : 0, (unsigned)cuts, trimRoll, trimPitch, yawZero, isnan(rollIn) ? 0.0f : rollIn, isnan(pitchIn) ? 0.0f : pitchIn, isnan(yawIn) ? 0.0f : yawIn,
           rngNeg[0], rngPos[0], rngNeg[1], rngPos[1], rngNeg[2], rngPos[2], rngSet ? 1 : 0, rngOn ? (unsigned)((int32_t)(rngUntil - millis()) > 0 ? rngUntil - millis() : 1) : 0, (unsigned)stMeanPct,
           (unsigned)(flipPhase ? (flipPhase < 3 ? flipPhase : 3 + flipPush) : 0), (unsigned)flips, (unsigned)flipsOk, craftInverted() ? 1 : 0, waitReadySince ? (unsigned)(millis() - waitReadySince) : 0,
           hoverTest ? 1u : 0u, (unsigned)hoverTestDone, accSentMs ? ((int32_t)(crsfAccCalAckMs() - accSentMs) >= 0 ? 2u : 1u) : 0u, accSentMs ? (unsigned)(millis() - accSentMs) : 0u,
           craftWarn ? 1 : 0, craftLow ? (unsigned)((millis() - lowSince) / 1000) : 0u, crsfCraftAmps(), att ? ay : 0.0f, tempTxt, coreTxt, (unsigned)crsfFcCycleUs(), (unsigned)crsfFcLoadPct(), crsfModuleName(), ver[0], ver[1], ver[2], (unsigned)crsfPowerLevel());
  String s = String(b);
  if (rngOn) {                                           // the running extremes while step 7 runs, from the starting pose, for the page to show
    float e[6];
    for (int k = 0; k < 3; k++) { bool any = rngMin[k] <= rngMax[k]; float st = rngStartN[k] ? rngStartSum[k] / rngStartN[k] : (any ? (rngMin[k] + rngMax[k]) / 2 : 0); e[2 * k] = any ? rngMin[k] - st : 0; e[2 * k + 1] = any ? rngMax[k] - st : 0; }
    char x[96]; snprintf(x, sizeof x, "\"rngx\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],", e[0], e[1], e[2], e[3], e[4], e[5]);
    s += x;
  }
  s += "\"why\":\"" + jsonStr(why) + "\",\"block\":\"" + jsonStr(block) + "\",\"tries\":[";
  for (uint8_t k = 0; k < nTries; k++) {                 // newest first: [ms ago, fist ms, open ms, hand roll, hand pitch, flex, side, twist, fingers %, result]
    const Try& t = tries[k];
    char tb[96]; snprintf(tb, sizeof tb, "%s[%u,%u,%u,%d,%d,%d,%d,%d,%d,\"", k ? "," : "", t.old ? 0u : (unsigned)(millis() - t.ms), (unsigned)t.fist, (unsigned)t.took, t.hr, t.hp, t.fl, t.sd, t.tw, t.curl == 255 ? -1 : (int)t.curl);
    s += tb; s += jsonStr(t.result); s += t.old ? "\",1]" : "\",0]";   // the last element: 1 = from before the last power-up
  }
  return s + "]}";
}

String flySet(const String& q) {
  if (queryValue(q, "raw") == "1") return crsfRawJson();
  if (queryValue(q, "ev") == "1") return flyEventsJson();
  String msg;
  String en = queryValue(q, "enable");
  if (en.length()) { flyEnable(en == "1", "the page"); msg = en == "1" ? "fly: no flight mode switch any more - ARM arms, DISARM disarms" : (armed ? "fly: DISARMED (the page)" : "fly: disarmed"); }
  if (queryValue(q, "disarm") == "1") { if (armed || flipPhase) { flyDisarm("the page"); msg = "fly: DISARMED (the page)"; } else msg = "fly: already disarmed"; }
  auto tune = [&](const char* key, uint8_t& v, uint8_t lo, uint8_t hi, const char* what) { String x = queryValue(q, key); if (!x.length()) return; int n = x.toInt(); if (n < lo) n = lo; if (n > hi) n = hi; v = (uint8_t)n; tuningSave(); msg = String("fly: ") + what + " = " + n + " % (saved)"; Bench.println(msg); };
  tune("thrmax", thrTopPct, 10, 100, "throttle range (0 to this)"); tune("stickmax", stickMaxPct, 10, 100, "roll / pitch at full wrist"); tune("yawmax", yawMaxPct, 10, 100, "yaw at full turn"); tune("thrsens", thrSensPct, 0, 100, "throttle sensitivity");
  tune("thrfloor", thrFloorPct, 10, 60, "throttle floor (a fist in flight)"); tune("hover", hoverPct, 10, 80, "hover estimate (the climb curve bends there)");
  if (queryValue(q, "msp") == "1") { crsfMspAsk(); msg = "crsf: asked the drone for its arming flags (MSP_STATUS over CRSF) - key e / the log shows the answer"; }
  if (queryValue(q, "acccal") == "1") {                 // the page's "level the drone": Betaflight's accelerometer calibration over the link
    if (armed || flipPhase) msg = "fly: not while armed";
    else if (!crsfLinkUp()) msg = "fly: no link to the drone";
    else { crsfAccCalibrate(); accSentMs = millis(); msg = "fly: told the drone to calibrate its accelerometer - keep it still and level on the floor for 3 s; its level line here should then read 0/0 on that floor"; }
  }
  String ht = queryValue(q, "hovertest");
  if (ht.length()) { hoverTest = ht == "1"; msg = hoverTest ? "fly: HOVER TEST armed - fly and hold a hover as well as you can for a while, then land and disarm: the middle of that flight becomes the hover number" : "fly: hover test off"; Bench.println(msg); }
  String rg = queryValue(q, "range");
  if (rg.length()) msg = rg == "1" ? rangeStart() : rangeForget();
  if (queryValue(q, "flip") == "1") {                    // a bench way to run the flip-over without the gesture (the drone must lie over and be linked)
    if (armed || flipPhase) msg = "fly: busy (armed or flipping)";
    else if (!craftInverted()) msg = "fly: the drone is not lying over (or no attitude) - nothing to flip";
    else { const char* b = armBlock(false, true); if (b) msg = String("fly: flip-over refused - ") + b; else { flipStart("the page / curl"); msg = "fly: FLIPPING IT BACK OVER - stand clear"; } }
  }
  String am = queryValue(q, "arm");
  if (am.length()) msg = flyBenchArm(am == "1" ? FLY_BENCH_ARM_S : (uint32_t)am.toInt());
  String bd = queryValue(q, "bind");
  if (bd.length()) { crsfBindLoop(bd == "1"); msg = bd == "1" ? "crsf: binding - a Bind command every 4 s for 90 s, stops when the drone links. whoop in bind mode (3 power cycles)?" : "crsf: bind loop stopped"; }
  if (queryValue(q, "quiet") == "1")  { crsfQuiet(); msg = "crsf: module power -> lowest"; }
  if (queryValue(q, "power") == "up") { crsfPowerUp(); msg = "crsf: module power one notch up"; }
  String fr = queryValue(q, "frames");
  if (fr.length()) { crsfStream(fr == "1"); msg = fr == "1" ? "crsf: RC frames on" : "crsf: RC frames OFF"; }
  String j = flyJson();
  j.remove(j.length() - 1);
  return j + ",\"msg\":\"" + jsonStr(msg.c_str()) + "\"}";
}

String flyStatusLine() {
  char b[460];
  const char* bl = armBlock(true, craftInverted());
  snprintf(b, sizeof b, "fly: %s, %s%s; throttle %u %% (%s; fingers %.2f, fist ref %s, floor %u %%, hover ~%u %%, top %u %%), roll %+.2f pitch %+.2f yaw %+.2f (in: tilt %.0f bend %.0f turn %.0f deg%s; full at -%.0f/+%.0f, -%.0f/+%.0f, -%.0f/+%.0f%s); %u arms, %u cuts, %u flip-overs (%u worked); last: %s %lu s ago; can arm now: %s",
           flipPhase ? "FLIPPING THE DRONE BACK OVER" : armed ? "the hand is the sticks (ring + laser gestures off)" : "watching for the fist - open - fist gesture", armed ? "ARMED " : "disarmed", armed ? String(String((millis() - armedAt) / 1000) + " s").c_str() : "",
           (unsigned)(thr * 100 + 0.5f), !armed ? "idle" : lifted ? "live" : "idle until the fingers lift", isnan(curlMean) ? -1.0f : curlMean, isnan(fistAnchor) ? "none" : String((int)lroundf(fistAnchor * 100)).c_str(), (unsigned)thrFloorPct, (unsigned)hoverPct, (unsigned)thrTopPct,
           roll, pitch, yaw, isnan(rollIn) ? 0.0f : rollIn, isnan(pitchIn) ? 0.0f : pitchIn, isnan(yawIn) ? 0.0f : yawIn, handFresh ? "" : ", hand NOT fresh",
           rngNeg[0], rngPos[0], rngNeg[1], rngPos[1], rngNeg[2], rngPos[2], rngSet ? " from step 7" : " default",
           (unsigned)arms, (unsigned)cuts, (unsigned)flips, (unsigned)flipsOk, why, (unsigned long)((millis() - whyMs) / 1000), bl ? bl : craftInverted() ? "the drone lies over - the gesture flips it" : "YES");
  return String(b);
}

