#include "config.h"
#include "laser.h"
#include "config.h"
#include "imu.h"
#include "gesture.h"
#include "flex.h"
#include "fly.h"
#include "outputs.h"
#include "modesw.h"
#include "s3link.h"
#include "gesturemap.h"
#include <GauntletLink.h>
#include <GauntletBenchIO.h>
#include <math.h>
#include <string.h>
static float      jq[16][4]; static uint32_t jqT[16]; static uint8_t jqI = 0; static uint32_t jqMs = 0;   // the wrist as a quaternion every 40 ms: the jolt is the largest rotation between any two of the last LASER_JOLT_WINDOW_MS
static float      fingersCurl = NAN;      // the MEDIAN of middle / ring / pinky curl in the page's calibration (resistance), NAN without it - one finger reading wrong cannot spoil it (2026-10-05)
static float      joltVal = 0; static uint32_t joltAt = 0;   // the jolt now, recomputed every 40 ms
static float      tCurl = NAN, iCurl = NAN;  // the thumb's and the index's curl now
static bool       shut = false;           // middle / ring / pinky closed: their median past LASER_FIST_CURL, until it drops under LASER_SHUT_LEAVE_CURL. The gesture's timing is judged on this alone; the thumb and the index only have to be in at the moment it fires, so one of them flickering at its line cannot reset an attempt
static uint32_t   shutSince = 0, notShutSince = 0;   // closed since / not closed since (0 = not / it is)
static bool       shutFresh = true;       // this closing came after LASER_FRESH_OPEN_MS with the hand not closed (the drone's fist-open-fist re-closes inside a second)
// one record per laser ATTEMPT (2026-10-05, "I did it once and I tried maybe 50 times"): every closing of the three from an open hand, judged until it fires or LASER_AFTER_FIST_MS passes
static bool       lAtt = false, lAttFresh = false, lAttThumb = false, lAttIndex = false;
static uint32_t   lAttShut = 0, lAttClose = 0;
static float      lAttJolt = 0, lAttTc = 0, lAttIc = 0;
static bool       ltHave = false; static uint32_t ltMs = 0; static uint16_t ltClose = 0; static uint8_t ltWhy = 0, ltJolt = 0, ltTc = 0, ltIc = 0;
static uint16_t   lCnt[8];                // how every attempt since power-up ended, by reason
static uint32_t   judgedShut  = 0;        // the closing whose gesture was last judged (fired, refused or blocked by the mode): once per closing
static const char* const L_WHY[8] = { "FIRED", "not fresh (closed again within a second)", "fingers closed too slowly", "let go too soon", "thumb not crunched in", "index not closed enough", "no wrist jolt", "thumb, index and jolt not together" };
static float      lastJolt = 0;           // the jolt the last judgement saw, degrees (feed "jolt")

static bool       on          = false;
static LaserForce force       = LaserForce::Auto;
static uint32_t   onSince     = 0, shots = 0, bootMs = 0;
static uint32_t   lastInline  = 0;        // when the wrist was last flat / in line
static bool       readyFromInline = false; // the wrist has been in line since the last shot: a flick may fire
static uint32_t   pendingFlick = 0;       // the wrist got to LASER_FIRE_DEG quickly at this time; the fist may follow within LASER_FIST_GRACE_MS
static uint32_t   openSince   = 0;        // the fingers have read "let go" since this (0 = not)
static float      flexRing[16]; static uint32_t flexRingT[16]; static uint8_t flexRingI = 0; static uint32_t flexRingMs = 0;   // the wrist flex over the last ~800 ms, a sample every 50 ms, for the jolt
static uint32_t   lastLetGoMs = 0;        // the last time the fingers read clearly open: a fist that formed within LASER_CLOSE_FAST_MS of it closed FAST     // the current fist was made after LASER_FRESH_OPEN_MS without a fist: the drone's fist-open-fist re-closes inside a second and must not fire the laser
static float      wP = NAN, wY = NAN, wR = NAN;   // the hand relative to the forearm: flexion (+ = toward the palm), side, twist - degrees
static float      openMv      = NAN;      // middle/ring/pinky above their half-curl thresholds, averaged, mV
static bool       fresh       = false;    // both IMUs reported within LASER_STALE_MS
static bool       fist        = false;    // the laser's fist: middle, ring, pinky closed AND the thumb crunched in AND the index at least partly closed
static bool       thumbIn     = false, indexIn = false;
static bool       zeroedNow   = false;    // both sensors have a MEASURED zero pose (key z or the sim page); else the fixed one in config.h
static char       why[56]     = "boot";
static uint32_t   whyMs       = 0;
static uint32_t   lastHealMs  = 0; static float healed = 0;   // yaw drift taken out since boot, degrees
static uint32_t   healFastUntil = 0;      // until then the fine healer converges quickly (just after boot, a snap or the page's step 1); after it only at drift speed, so a hand held turned is not straightened under him

static bool closeOnOff = true;              // a laser off closes the hand plate after it (outputs.cpp: only if nothing else needs it) - not when a mode change or goSafe stops it
static bool drive(bool v) { return laserRedSet(v, closeOnOff); }   // the red laser is PCA channel PCA_CH_LASER_RED through outputs.cpp: the hand plate lifts first (FULL only)
static const char* forceName() { return force == LaserForce::Auto ? "auto" : force == LaserForce::On ? "forced on" : "forced off"; }

static void set(bool v, const char* reason) {
  if (v == on) return;
  on = v; strlcpy(why, reason, sizeof why); whyMs = millis();
  if (on) { onSince = millis(); shots++; }
  if (!drive(on)) { on = false; return; }            // refused by outputs.cpp (mode, battery): stays off, the refusal is in the log
  Bench.printf("laser: %s - %s (wrist flex %+.0f side %+.0f twist %+.0f deg, hand %s %+.0f mV, %s)\n", on ? "ON" : "off", reason,
               isnan(wP) ? 0.0f : wP, isnan(wY) ? 0.0f : wY, isnan(wR) ? 0.0f : wR, fist ? "closed" : "open", isnan(openMv) ? 0.0f : openMv, forceName());
}

void laserBegin() { bootMs = millis(); healFastUntil = bootMs + LASER_HEAL_FAST_MS; }

static bool inLineNow() { return !isnan(wP) && fabsf(wP) <= LASER_INLINE_DEG && fabsf(wY) <= LASER_SIDE_DEG && fabsf(wR) <= LASER_TWIST_DEG; }

// The wrist = the hand relative to the forearm, from the two sensors' zeroed orientations (imuRelative). Both zero
// poses are taken together with the hand in line (the sim page's step 1 -> /laser?zero=1 -> imuZeroAsync), which
// makes the wrist read 0/0/0 there and removes the heading mismatch between the two game rotation vectors; from
// then on flexion is flexion whatever the arm does. The two headings drift apart slowly (no magnetometer), so
// while the hand is flat, open and the laser off - and the forearm near level, where a heading nudge is a
// side-angle nudge - the hand's yaw origin is eased toward zero side angle.
static bool healPose() {   // the arm out level and the hand flat palm down, both by gravity (no heading involved, so a heading error cannot fake it): the only pose where a heading shift is purely a side angle and a relaxed hand is in line
  float r, p, y; uint32_t a;
  if (!imuSegAngles(0, r, p, y, a) || fabsf(r) > LASER_HEAL_POSE_DEG || fabsf(p) > LASER_HEAL_POSE_DEG) return false;
  if (!imuSegAngles(1, r, p, y, a) || fabsf(r) > LASER_HEAL_POSE_DEG || fabsf(p) > LASER_HEAL_POSE_DEG) return false;
  return true;
}
static void measure(uint32_t now) {
  float r, p, y; uint32_t age = 0;
  zeroedNow = imuRelZeroed();                                              // key z (or the page) has measured both sensors together
  bool ok = imuIsZeroed() && imuRelative(r, p, y, age);
  fresh = ok && age <= LASER_STALE_MS;
  if (ok) { wP = p; wY = y; wR = r; } else { wP = wY = wR = NAN; }
  if (ok && now - flexRingMs >= 50) { flexRingMs = now; flexRing[flexRingI] = p; flexRingT[flexRingI] = now; flexRingI = (flexRingI + 1) % 16; }
  if (ok && now - jqMs >= 40) { float q[4]; uint32_t qa = 0; if (imuRelativeQuat(q, qa) && qa <= LASER_STALE_MS) { memcpy(jq[jqI], q, sizeof q); jqT[jqI] = now; jqI = (jqI + 1) % 16; jqMs = now; } }
  tCurl = gestureFingerCurl(0); iCurl = gestureFingerCurl(1);   // the thumb's curl is against its fist-limited closed value; an uncalibrated finger does not block
  thumbIn = isnan(tCurl) || tCurl >= LASER_THUMB_CURL; indexIn = isnan(iCurl) || iCurl >= LASER_INDEX_CURL;
  { float mc = gestureFingerCurl(2), rc = gestureFingerCurl(3), pc = gestureFingerCurl(4); fingersCurl = (isnan(mc) || isnan(rc) || isnan(pc)) ? NAN : fmaxf(fminf(mc, rc), fminf(fmaxf(mc, rc), pc)); }   // the median of the three
  shut = isnan(fingersCurl) ? gestureFist() : fingersCurl >= (shut ? LASER_SHUT_LEAVE_CURL : LASER_FIST_CURL);
  if (!shut) { shutSince = 0; if (!notShutSince) notShutSince = now; }
  else { if (!shutSince) { shutSince = now; shutFresh = notShutSince == 0 || now - notShutSince >= LASER_FRESH_OPEN_MS; } notShutSince = 0; }
  fist = shut && thumbIn && indexIn;      // the laser's fist (Toby 2026-10-01: the drone's arming fist keeps the thumb out, the laser's crunches it in; the index "can't crunch that much, but there is a noticeable difference")
  openMv = gestureHandOpenMv();
  bool letGo = isnan(fingersCurl) ? (!isnan(openMv) && openMv >= (float)LASER_RELEASE_MV) : fingersCurl < LASER_OPEN_CURL;   // let go past half the bend (2026-10-05: the mV rule needed the fingers nearly straight - "I had to lift my fingers up all the way until it stopped")
  if (!letGo) openSince = 0; else { if (!openSince) openSince = now; lastLetGoMs = now; }
  // the drift healer, any arm pose: with the hand open and relaxed the wrist is about straight, so the heading offset between
  // the two sensors is whatever shift of the hand's heading makes the whole forearm-to-hand angle smallest - probe a degree
  // either way and step downhill, slowly. Not while the drone is armed (yaw is measured from the side angle then).
  if (fresh && !on && !shut && force == LaserForce::Auto && !flyArmed() && !flyRangeCapturing() && now - lastHealMs >= 100 && healPose()) {   // (not during the page's step 7 either: it would erase the very turns being measured - "it snapped back to the middle", 2026-10-03)
    float a0 = imuRelAngleDeg(0);
    if (!isnan(a0) && a0 > LASER_HEAL_MAX_DEG && notShutSince && now - notShutSince >= LASER_SNAP_OPEN_MS) {   // a big heading error (a power-up: the two headings restart anywhere) with the hand open and relaxed two seconds: snap to the shift that makes the wrist straightest, then heal finely
      lastHealMs = now;
      float best = 0, bestA = a0;
      for (int d = -180; d < 180; d += 5) { float a = imuRelAngleDeg(d * DEG_TO_RAD); if (!isnan(a) && a < bestA) { bestA = a; best = d; } }
      for (float d = best - 5; d <= best + 5; d += 1) { float a = imuRelAngleDeg(d * DEG_TO_RAD); if (!isnan(a) && a < bestA) { bestA = a; best = d; } }
      if (bestA < a0 - 5 && bestA <= LASER_SNAP_GOOD_DEG) {   /* only a shift that really explains the error (2026-10-03: +-25 deg snaps improving 55 -> 49 alternated every few seconds) */ imuNudgeHandYaw(-best); healed += best; healFastUntil = now + LASER_HEAL_FAST_MS; flyNote("wrist heading re-zeroed by %.0f (left %.0f)", best, bestA); Bench.printf("laser: wrist re-zeroed by %.0f deg - the hand's heading was %.0f deg off the forearm's (open relaxed hand, a power-up or a long drift); now %.0f deg\n", best, a0, bestA); }
    }
    else if (!isnan(a0) && a0 > 1.0f && a0 <= LASER_HEAL_MAX_DEG) {
      lastHealMs = now;
      float ap = imuRelAngleDeg(1.0f * DEG_TO_RAD), am = imuRelAngleDeg(-1.0f * DEG_TO_RAD);
      float dir = ap < a0 - 0.05f ? 1.0f : am < a0 - 0.05f ? -1.0f : 0.0f;
      if (dir != 0) {                                                       // nudge(deg) takes deg OUT of yawFix, so -step adds it
        bool fast = (int32_t)(healFastUntil - now) > 0;
        float step = dir * (a0 < 10 ? a0 : 10) * (fast ? LASER_HEAL_GAIN : LASER_HEAL_GAIN_SLOW);
        float cap = fast ? 10.0f : LASER_HEAL_SLOW_MAX_DEG; if (step > cap) step = cap; if (step < -cap) step = -cap;   // drift is a few degrees a minute: outside the fast window nothing faster than that gets taken out
        imuNudgeHandYaw(-step); healed += step;
      }
    }
  }
}

static float joltNow(uint32_t now) {      // the largest rotation of the hand relative to the forearm between any two samples of the last LASER_JOLT_WINDOW_MS, any direction (the smallest |dot|) - over a fraction of a second the heading error between the two sensors cancels out
  float minDot = 1;
  for (uint8_t a = 0; a < 16; a++) {
    if (!jqT[a] || now - jqT[a] > LASER_JOLT_WINDOW_MS) continue;
    for (uint8_t b = a + 1; b < 16; b++) {
      if (!jqT[b] || now - jqT[b] > LASER_JOLT_WINDOW_MS) continue;
      float d = fabsf(jq[a][0] * jq[b][0] + jq[a][1] * jq[b][1] + jq[a][2] * jq[b][2] + jq[a][3] * jq[b][3]);
      if (d < minDot) minDot = d;
    }
  }
  return 2 * acosf(minDot > 1 ? 1 : minDot) * RAD_TO_DEG;
}
static void lTry(uint32_t now, uint8_t w) {   // an attempt is over: keep it for the feed and the page, count it, say it in the log
  lAtt = false; ltHave = true; ltMs = now; ltWhy = w; if (lCnt[w] < 65535) lCnt[w]++;
  ltClose = lAttClose > 65535 ? 65535 : (uint16_t)lAttClose; ltJolt = (uint8_t)fminf(255, lAttJolt);
  ltTc = (uint8_t)lroundf(fminf(1, fmaxf(0, lAttTc)) * 100); ltIc = (uint8_t)lroundf(fminf(1, fmaxf(0, lAttIc)) * 100);
  Bench.printf("laser: try - closed in %u ms (%u or less), wrist jolt %u deg (%.0f), thumb %u %% (%.0f), index %u %% (%.0f) -> %s\n", (unsigned)ltClose, (unsigned)LASER_CLOSE_FAST_MS,
               (unsigned)ltJolt, LASER_JOLT_DEG, (unsigned)ltTc, LASER_THUMB_CURL * 100, (unsigned)ltIc, LASER_INDEX_CURL * 100, L_WHY[w]);
}
void laserPoll() {
  uint32_t now = millis();
  measure(now);

  bool judged = false;
  bool held = lasersFlightHeld();                                      // the drone flies, or stopped under LASER_AFTER_FLIGHT_MS ago (Ironman, 2026-10-07 night)
  if (force == LaserForce::On)       { if (!on && modeAllowsFull() && !held) set(true, "forced on"); }   // below FULL or in flight a forced laser waits (no refusal every loop)
  else if (force == LaserForce::Off) { if (on)  set(false, "forced off"); }
  else if (now - bootMs < LASER_BOOT_HOLD_MS) { if (on) set(false, "boot hold"); }
  else if (!fresh) { if (on) set(false, flyEnabled() ? "flight mode - the fist is the throttle now" : "an IMU went quiet - fail safe"); readyFromInline = false; pendingFlick = 0; }
  else {
    // In flight and just after it the gesture is still READ - a shot it would have fired is reported $E BLOCKED - but nothing
    // fires (HandBench switches the gesture off in flight; here the puck is told, and the hold runs on for 1.5 s after a
    // disarm so a wrist jerk during the finger-gun disarm cannot fire the green laser).
    if (held) { if (on) set(false, flyEnabled() ? "flight mode - the fist is the throttle now" : "the drone was just disarmed"); readyFromInline = false; pendingFlick = 0; }
    if (!on) {
      // Toby's laser gesture: middle, ring and pinky close FAST from an open hand (their median from under LASER_OPEN_CURL to
      // past LASER_FIST_CURL within LASER_CLOSE_FAST_MS - a FRESH closing, after LASER_FRESH_OPEN_MS without one), and while
      // they stay closed (LASER_FIST_HOLD_MS to LASER_AFTER_FIST_MS after) the thumb is crunched in, the index is in and the
      // wrist has jolted: LASER_JOLT_DEG of rotation of the hand against the forearm within the last LASER_JOLT_WINDOW_MS, any
      // direction (over a fraction of a second the heading error between the two sensors cancels out). No "in line" rule.
      judged = true;
      if (now - joltAt >= 40) { joltAt = now; joltVal = joltNow(now); }
      bool inWin = shut && shutSince && now - shutSince <= LASER_AFTER_FIST_MS;
      uint32_t closeMs = (shutSince && lastLetGoMs && shutSince >= lastLetGoMs) ? shutSince - lastLetGoMs : 0xFFFFFFFFu;
      if (lAtt && (!inWin || shutSince != lAttShut))     // the attempt is over without a shot: what was missing, in the gesture's own order
        lTry(now, !lAttFresh ? 1 : lAttClose > LASER_CLOSE_FAST_MS ? 2 : now - lAttShut < LASER_FIST_HOLD_MS + 20 ? 3 : !lAttThumb ? 4 : !lAttIndex ? 5 : lAttJolt < LASER_JOLT_DEG ? 6 : 7);
      if (inWin && !held && !lAtt && shutSince != lAttShut && closeMs <= LASER_TRY_FROM_OPEN_MS) {   // the three closed from an open hand: an attempt (not recorded in flight - the throttle fist would fill the log)
        lAtt = true; lAttShut = shutSince; lAttClose = closeMs; lAttFresh = shutFresh; lAttThumb = lAttIndex = false; lAttJolt = 0; lAttTc = 0; lAttIc = 0;
      }
      if (lAtt && inWin) {
        if (joltVal > lAttJolt) lAttJolt = joltVal;
        if (!isnan(tCurl) && tCurl > lAttTc) lAttTc = tCurl;
        if (!isnan(iCurl) && iCurl > lAttIc) lAttIc = iCurl;
        if (thumbIn) lAttThumb = true;
        if (indexIn) lAttIndex = true;
      }
      if (inWin && shutFresh && closeMs <= LASER_CLOSE_FAST_MS && now - shutSince >= LASER_FIST_HOLD_MS && thumbIn && indexIn && !flexAnyFault()) {   // (2026-10-08) never on a faulted finger
        lastJolt = joltVal;
        if (joltVal >= LASER_JOLT_DEG && judgedShut != shutSince) {   // judged once per closing, in every mode
          judgedShut = shutSince;
          char aw[110];
          if (!modeAllowsFull() || held) { s3linkAct(glink::ACT_LASER_RED, glink::RES_BLOCKED, 2); gmapBuzz(false); lAtt = false; }   // the gesture was seen; the mode (the lasers need FULL) or the flight said no - s3linkAct logs which
          else if (!gmapAligned(aw, sizeof aw)) { Bench.printf("laser: the red gesture seen, but %s - not fired\n", aw); s3linkAct(glink::ACT_LASER_RED, glink::RES_REFUSED, 2); gmapBuzz(false); lAtt = false; }   // (2026-10-21, Toby: the hand aligned for every gesture)
          else { set(true, "fingers closed fast + wrist jolt"); s3linkAct(glink::ACT_LASER_RED, on ? glink::RES_DONE : glink::RES_REFUSED, 2); gmapBuzz(on); if (lAtt) lTry(now, 0); }
        }
      }
      // (the green laser is the two-finger pose + three thumb clicks since 2026-10-21 - gesturemap.cpp; the finger gun + jolt is gone)

    } else {
      if (openSince && now - openSince >= LASER_OPEN_MS) { set(false, "fingers let go"); gmapBuzz(true); }   // the ONLY way off while the hand keeps its shape: move anywhere, it stays
    }
  }
  if (!judged) lAtt = false;                                            // flying, forced, an IMU quiet: no attempt carries over
  if (on && millis() - onSince >= LASER_MAX_ON_MS) {                     // whatever switched it on. millis(), not `now`: set() stamps onSince with a fresh millis() later in this poll, and `now - onSince` then wrapped to ~4e9 whenever the clock ticked in between - the guard switched a laser off the moment it came on (found 2026-10-07 night; HandBench has the same line)
    set(false, "on 5 minutes - forgot-it-on guard");
    if (force == LaserForce::On) force = LaserForce::Auto;
    readyFromInline = false; pendingFlick = 0;
  }
}

bool       laserOn()    { return on; }
LaserForce laserForce() { return force; }
float      laserWristDeg() { return wP; }

void laserSetForce(LaserForce f) {
  force = f; readyFromInline = false; pendingFlick = 0;
  Bench.printf("laser: %s%s\n", forceName(), f == LaserForce::Auto ? " - the wrist gesture decides (start with the wrist flat)" : "");
}
void laserKeyCycle() { laserSetForce(force == LaserForce::Auto ? LaserForce::On : force == LaserForce::On ? LaserForce::Off : LaserForce::Auto); }
void laserOff(const char* reason) { force = LaserForce::Off; closeOnOff = false; set(false, reason); closeOnOff = true; }   // off and FORCED off (the OTA, the shutdown): the gesture stays dead until key p
void laserStop(const char* reason) { if (force == LaserForce::On) force = LaserForce::Auto; closeOnOff = false; set(false, reason); closeOnOff = true; readyFromInline = false; pendingFlick = 0; }   // off now, the gesture still works later (a mode change, goSafe)

static void fmt1(char* b, size_t n, float v) { if (isnan(v)) strlcpy(b, "null", n); else snprintf(b, n, "%.1f", v); }
static void fmt0(char* b, size_t n, float v) { if (isnan(v)) strlcpy(b, "null", n); else snprintf(b, n, "%.0f", v); }

String laserJson() {
  char p[16], y[16], r[16], o[16], mc[16];
  fmt1(p, sizeof p, wP); fmt1(y, sizeof y, wY); fmt1(r, sizeof r, wR); fmt0(o, sizeof o, openMv); fmt0(mc, sizeof mc, isnan(fingersCurl) ? NAN : fingersCurl * 100);
  char b[420];
  snprintf(b, sizeof b, "{\"on\":%d,\"force\":%u,\"w\":%s,\"wy\":%s,\"wr\":%s,\"open\":%s,\"fist\":%d,\"ff\":%d,\"thumb\":%d,\"index\":%d,\"yf\":%.1f,\"inl\":%d,\"fresh\":%d,\"zeroed\":%d,\"n\":%u,\"why\":\"%s\",\"ago\":%u,\"fire\":%.0f,\"rel\":%u,\"healed\":%.1f,\"jolt\":%.0f,\"shut\":%d,\"mc\":%s,\"lcnt\":[%u,%u,%u,%u,%u,%u,%u,%u]",
           on ? 1 : 0, (unsigned)force, p, y, r, o, fist ? 1 : 0, (shut && shutFresh) ? 1 : 0, thumbIn ? 1 : 0, indexIn ? 1 : 0, imuHandYawFixDeg(), inLineNow() ? 1 : 0, fresh ? 1 : 0, zeroedNow ? 1 : 0, (unsigned)shots, why, (unsigned)(millis() - whyMs),
           LASER_FIRE_DEG, (unsigned)LASER_RELEASE_MV, healed, lastJolt, shut ? 1 : 0, mc,
           lCnt[0], lCnt[1], lCnt[2], lCnt[3], lCnt[4], lCnt[5], lCnt[6], lCnt[7]);
  String s(b);
  if (ltHave) { char t[64]; snprintf(t, sizeof t, ",\"ltry\":[%u,%u,%u,%u,%u,%u]", (unsigned)(millis() - ltMs), (unsigned)ltClose, (unsigned)ltJolt, (unsigned)ltTc, (unsigned)ltIc, (unsigned)ltWhy); s += t; }
  return s + "}";
}

static String argOf(const String& q, const char* name) {   // "qa=1,0,0,0&fa=..." -> the value of name, "" if absent
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
static bool floats(const String& s, float* out, int n) {   // "a,b,c" -> n floats
  int p = 0;
  for (int i = 0; i < n; i++) {
    if (p > (int)s.length()) return false;
    int e = s.indexOf(',', p); if (e < 0) e = s.length();
    if (e == p) return false;
    out[i] = s.substring(p, e).toFloat();
    p = e + 1;
  }
  return true;
}

String laserSet(const String& q) {
  String msg;
  if (q.indexOf("zero=1") >= 0) {
    // The page's zero: its step-1 quaternions (the 2 s means the model itself uses) and, once steps 2/3 are done, each
    // sensor's forward axis. Without them (a bare zero=1) the board averages its own window, key-z style.
    float qa[4], qh[4], fa[3], fh[3];
    bool hasQa = floats(argOf(q, "qa"), qa, 4), hasQh = floats(argOf(q, "qh"), qh, 4), hasFa = floats(argOf(q, "fa"), fa, 3), hasFh = floats(argOf(q, "fh"), fh, 3);
    if (hasQa || hasQh) {
      if (hasQa || hasFa) imuSetRelZero(0, hasQa ? qa : nullptr, hasFa ? fa : nullptr);
      if (hasQh || hasFh) imuSetRelZero(1, hasQh ? qh : nullptr, hasFh ? fh : nullptr);
      msg = String("laser: wrist zero from the page (") + (hasQa ? "arm" : "") + (hasQa && hasQh ? " + " : "") + (hasQh ? "hand" : "") + (hasFa || hasFh ? ", with forward axes" : ", forward = X until steps 2/3") + ")";
    } else { imuZeroAsync(LASER_ZERO_MS); msg = "laser: zeroing both sensors over the next " + String(LASER_ZERO_MS) + " ms - this pose is the flat wrist"; }
    healed = 0; readyFromInline = false; pendingFlick = 0; healFastUntil = millis() + LASER_HEAL_FAST_MS;
  }
  int i = q.indexOf("force=");
  if (i >= 0) {
    String v = q.substring(i + 6); int e = v.indexOf('&'); if (e >= 0) v = v.substring(0, e);
    if (v == "on")        { laserSetForce(LaserForce::On);   msg = "laser forced ON (5 min guard)"; }
    else if (v == "off")  { laserSetForce(LaserForce::Off);  msg = "laser forced off"; }
    else if (v == "auto") { laserSetForce(LaserForce::Auto); msg = "laser on the wrist gesture: wrist flat, then a fist with a quick small flick; let the fingers go to stop"; }
    else msg = "laser: force=on|off|auto";
  }
  String s = laserJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

String laserStatusLine() {
  String w = isnan(wP) ? String("wrist n/a (needs both IMUs)")
                       : String("wrist flex ") + String(wP, 0) + " side " + String(wY, 0) + " twist " + String(wR, 0) + " deg" +
                         (zeroedNow ? "" : " - the fixed zero (config IMU_ZERO_DEG, heading at power-up; key z re-measures it at the bench)");
  String h = isnan(fingersCurl) ? String("hand ? (the page has not sent the finger numbers)")
           : String("fingers ") + String(fingersCurl * 100, 0) + " % (the middle of middle / ring / pinky) - " +
             (fist ? "closed, thumb in, index in" : shut ? (String("closed but ") + (!thumbIn ? "the thumb is out" : "the index is out")) : fingersCurl < LASER_OPEN_CURL ? "open" : "loose");
  String t = String(lCnt[0]) + " fired";
  for (uint8_t k = 1; k < 8; k++) if (lCnt[k]) t += String(", ") + lCnt[k] + " " + L_WHY[k];
  return String("laser: ") + (on ? "ON" : "off") + " (" + forceName() + ", last: " + why + ") - " + w + ", " + h +
         "; fires when middle / ring / pinky close within " + String(LASER_CLOSE_FAST_MS) + " ms from an open hand (not within " + String(LASER_FRESH_OPEN_MS) +
         " ms of the last closing), held " + String(LASER_FIST_HOLD_MS) + " ms, the thumb in (" + String(LASER_THUMB_CURL * 100, 0) + " %), the index in (" + String(LASER_INDEX_CURL * 100, 0) +
         " %) and a wrist jolt of " + String(LASER_JOLT_DEG, 0) + " deg; off when the fingers let go under " + String(LASER_OPEN_CURL * 100, 0) + " % for " + String(LASER_OPEN_MS) +
         " ms; attempts since power-up: " + t + "; yaw drift healed " + String(healed, 1) + " deg; PCA ch " + String(PCA_CH_LASER_RED) + "; " + String(shots) + " shot" + (shots == 1 ? "" : "s") + " so far";
}

