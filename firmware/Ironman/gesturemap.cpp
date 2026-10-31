#include "gesturemap.h"
#include "config.h"
#include "imu.h"
#include "flex.h"
#include "gesture.h"
#include "modesw.h"
#include "palm.h"
#include "servos.h"
#include "outputs.h"
#include "thermal.h"
#include "fly.h"
#include "laser.h"
#include "s3link.h"
#include <GauntletBenchIO.h>
#include <GauntletLink.h>
#include <math.h>

// ======== TOBY'S MAP (2026-10-21) - every gesture below needs the wrist ALIGNED (in line with the forearm) =============
//   GREEN LASER on   index + middle out, ring + pinky in, three thumb clicks        (FULL; plate 1 opens first)
//               off  the same two fingers out, two thumb clicks - nothing else turns it off (move the fingers freely)
//   RED LASER + LIDAR  thumb in, then the fingers in fast, a small wrist jolt - HandBench's gesture (laser.cpp)   (FULL)
//   PLATES open all  a fist, thumb in, held, no jolt - then the whole hand open quickly   (every plate closed; SHOW)
//          close all the same fist - a quick open and back into the fist                  (any plate open; SHOW)
//   THERMAL CAMERA   thumb out, the four fingers down: up-down, up-down quickly (two pulses) - on / off   (SHOW)
//   DRONE arm        thumb out, the four fingers down: one pulse out and back, held - HandBench's (fly.cpp)    (FULL)
//         disarm     pointing (index out, the rest in) + two thumb clicks - HandBench's (gesture.cpp)
//   THE RING         three thumb clicks, hand open = on / white <-> red, closed or pointing = off - HandBench's, unchanged
// The answer in the hand: one buzz = done, two = refused (the mode, the battery, not aligned, busy). In flight the fingers
// are the throttle: nothing here acts. The lines: config.h GMAP_*; the four fingers as one = the drone's own measure.
// =====================================================================================================================

// ---- the hand ----
static constexpr uint8_t TH_N = GMAP_THUMB_LOOK_MS / 20;
static float    thumbRing[TH_N];
static uint8_t  thIdx = 0;
static uint32_t thAt = 0;
enum class Th : uint8_t { Unknown, In, Out };
static Th thumbState() {                        // its highest curl over the last GMAP_THUMB_LOOK_MS
  float mx = NAN;
  for (uint8_t k = 0; k < TH_N; k++) if (!isnan(thumbRing[k]) && (isnan(mx) || thumbRing[k] > mx)) mx = thumbRing[k];
  if (isnan(mx)) return Th::Unknown;
  return mx >= GMAP_THUMB_IN_CURL ? Th::In : mx <= GMAP_THUMB_OUT_CURL ? Th::Out : Th::Unknown;
}
bool gmapThumbIn() { return thumbState() == Th::In; }
static const char* thName(Th t) { return t == Th::In ? "in" : t == Th::Out ? "out" : "half"; }
static float curlOf(uint8_t i) { return flexFault(i) ? NAN : gestureFingerCurl(i); }
static bool twoFingers() {                      // the green laser's pose: index + middle out, ring + pinky in
  float i = curlOf(1), m = curlOf(2), r = curlOf(3), p = curlOf(4);
  return !isnan(i) && !isnan(m) && !isnan(r) && !isnan(p) && i <= GMAP_FINGER_OUT_CURL && m <= GMAP_FINGER_OUT_CURL && r >= GMAP_FINGER_IN_CURL && p >= GMAP_FINGER_IN_CURL;
}
static bool fullFist() {                        // a REAL fist for the plates and the camera: the index in and three of the four in (the drone's measure also counts a finger gun)
  float c[4]; uint8_t in = 0;
  for (uint8_t i = 0; i < 4; i++) { c[i] = curlOf(i + 1); if (!isnan(c[i]) && c[i] >= GMAP_FINGER_IN_CURL) in++; }
  return !isnan(c[0]) && c[0] >= GMAP_FINGER_IN_CURL && in >= 3;
}
static bool pointing() {                        // the disarm's finger gun (index up, the median of middle / ring / pinky closed)
  float i = curlOf(1), m = curlOf(2), r = curlOf(3), p = curlOf(4);
  if (isnan(i) || isnan(m) || isnan(r) || isnan(p)) return false;
  float med = fmaxf(fminf(m, r), fminf(fmaxf(m, r), p));
  return i <= DISARM_INDEX_UP_CURL && med >= DISARM_FIST_CURL;
}
bool gmapAligned(char* why, size_t n) {         // the wrist in line with the forearm - the drone's arming check
  float tw, fl, sd; uint32_t age;
  if (!imuRelative(tw, fl, sd, age) || age > FLY_HAND_STALE_MS) { snprintf(why, n, "no wrist reading (a sensor is quiet)"); return false; }
  if (fabsf(fl) <= GMAP_ALIGN_FLEX_DEG && fabsf(sd) <= GMAP_ALIGN_SIDE_DEG && fabsf(tw) <= GMAP_ALIGN_TWIST_DEG) return true;
  snprintf(why, n, "the wrist not in line (bend %.0f, side %.0f, twist %.0f deg - %.0f / %.0f / %.0f allowed)", fl, sd, tw, GMAP_ALIGN_FLEX_DEG, GMAP_ALIGN_SIDE_DEG, GMAP_ALIGN_TWIST_DEG);
  return false;
}
bool gmapTwoFingers() { return twoFingers(); }
void gmapBuzz(bool ok) { hapticFeedback(ok ? 1 : 2); }
static void answer(uint8_t act, uint8_t res, uint8_t need) { s3linkAct(act, res, need); gmapBuzz(res == glink::RES_DONE); }   // s3linkAct logs it; the buzz says it in the hand
static bool mayAct(uint8_t act, uint8_t need, const char* what) {   // the mode, then the wrist: false = refused, answered, buzzed twice
  if (need >= 2 ? !modeAllowsFull() : !modeAllowsShow()) { Bench.printf("gesture: %s - the mode switch says no\n", what); answer(act, glink::RES_BLOCKED, need); return false; }
  char why[110];
  if (!gmapAligned(why, sizeof why)) { Bench.printf("gesture: %s seen, but %s - nothing done\n", what, why); answer(act, glink::RES_REFUSED, need); return false; }
  return true;
}

// ---- the actions ----
static void greenOn() {
  const uint8_t act = glink::ACT_LASER_GREEN;
  if (laserGreenWanted()) return;                                     // on already
  if (!mayAct(act, 2, "two fingers out + three thumb clicks (the green laser)")) return;
  if (lasersFlightHeld()) { answer(act, glink::RES_BLOCKED, 2); return; }   // the drone was just disarmed (LASER_AFTER_FLIGHT_MS)
  bool ok = laserGreenSet(true);
  answer(act, ok ? glink::RES_DONE : glink::RES_REFUSED, 2);
  if (ok) Bench.println("gesture: two fingers out + three thumb clicks -> green laser ON (plate 1 opens first); off = the same two fingers + two clicks");
}
static void plates(bool open) {
  const uint8_t act = glink::ACT_CASCADE;
  if (!mayAct(act, 1, open ? "fist (thumb in) - hand open quickly (open all the plates)" : "fist (thumb in) - quick open and back (close all the plates)")) return;
  if (servosBusy()) { Bench.println("gesture: the plates - a move is running, nothing started"); answer(act, glink::RES_REFUSED, 1); return; }
  if (open) servosOpen(); else servosClose();
  answer(act, servosBusy() ? glink::RES_DONE : glink::RES_REFUSED, 1);   // a sequence started = done; nothing to move or refused = the log says why
  Bench.printf("gesture: fist (thumb in) - %s -> plates %s\n", open ? "hand open quickly" : "quick open and back", open ? "OPEN ALL" : "CLOSE ALL");
}
static void camera() {
  const bool on = strcmp(thermalStateName(), "off") != 0;
  const uint8_t act = on ? glink::ACT_THERMAL_OFF : glink::ACT_THERMAL_ON;
  if (!on && !mayAct(act, 1, "thumb out, two finger pulses (the thermal camera)")) return;   // off is never refused
  bool ok = on ? thermalOff("the gesture") : thermalOn("the gesture");   // the camera reports itself to the puck
  gmapBuzz(ok);
  Bench.printf("gesture: thumb out, two finger pulses -> thermal camera %s\n", on ? "off" : (ok ? "ON" : "refused"));
}

// ---- thumb clicks (gesture.cpp) ----
void gmapTriple(bool closedHand) {
  if (twoFingers()) { greenOn(); return; }                            // before the ring: ring + pinky in read as HandBench's closed hand
  const uint8_t a = (!pointing() && !closedHand) ? glink::ACT_RING_ON : glink::ACT_RING_OFF;   // the ring, exactly as on HandBench
  if (!modeAllowsShow()) { s3linkAct(a, glink::RES_BLOCKED, 1); return; }   // SAFE: seen and reported, nothing changes
  palmGesture(a == glink::ACT_RING_OFF);
  s3linkAct(a, glink::RES_DONE, 1);
}
bool gmapThumbDouble() {
  if (flyEnabled() || !laserGreenWanted() || !twoFingers()) return false;
  laserGreenSet(false);                                               // its plate closes after it, if nothing else needs it
  answer(glink::ACT_LASER_GREEN, glink::RES_DONE, 2);
  Bench.println("gesture: two fingers out + two thumb clicks -> green laser off");
  return true;
}

// ---- the four fingers as one: a fist, leaving it, coming back (every loop, after flyPoll) ----
enum class Fp : uint8_t { Idle, Fist, Out };
static Fp       fp = Fp::Idle;
static uint32_t fistSince = 0, leftAt = 0;
static float    fistTop = 0, outLow = 1;
static Th       leftThumb = Th::Unknown;
static bool     leftHeld = false, used = false, gotOut = false, leftFull = false;
static uint32_t fullSeenMs = 0;                 // the last time the hand was a REAL fist (fullFist) - leaving a fist counts as real if it was within GMAP_FULL_RECENT_MS
static uint32_t pulse1Back = 0;                 // the camera: when its first pulse (thumb out) came back
static uint32_t lastDoubleMs = 0;
uint32_t gmapCameraDoubleMs() { return lastDoubleMs; }

void gmapPoll() {
  uint32_t now = millis();
  if (now - thAt >= 20) { thAt = now; thumbRing[thIdx] = curlOf(0); thIdx = (thIdx + 1) % TH_N; }
  const float four = flyFingerCurl();
  if (flyEnabled() || isnan(four)) { fp = Fp::Idle; pulse1Back = 0; return; }   // in flight the fingers are the throttle
  const bool fist = four >= FLY_CLOSED_CURL;
  if (fullFist()) fullSeenMs = now;               // (judged on the recent past: by the moment the fingers leave the fist they are already half open)
  if (fp == Fp::Idle) { if (fist) { fp = Fp::Fist; fistSince = now; fistTop = four; } return; }
  if (fp == Fp::Fist) {
    if (fist) { if (four > fistTop) fistTop = four; return; }
    fp = Fp::Out; leftAt = now; outLow = four; gotOut = false;        // the fingers just left the fist
    leftThumb = thumbState(); leftHeld = now - fistSince >= GMAP_FIST_HOLD_MS; leftFull = fullSeenMs && now - fullSeenMs <= GMAP_FULL_RECENT_MS;   // (2026-10-21 self-test: a finger gun with the thumb held in, then the hand opened = "open all" - a disarm's end)
    used = laserOn() || laserRedWanted();                             // a laser fist: letting go turns the laser off, it opens nothing
    if (pulse1Back && now - pulse1Back > GMAP_DOUBLE_GAP_MS) pulse1Back = 0;   // too late to be the camera's second pulse
  }
  if (four < outLow) outLow = four;
  if (outLow <= FLY_PULSE_OPEN_CURL || fistTop - outLow >= FLY_PULSE_DROP) gotOut = true;
  // OPEN ALL: the fist had the thumb in and was held, the whole hand opened quickly, every plate is closed
  if (!used && leftFull && leftThumb == Th::In && leftHeld && now - leftAt <= GMAP_QUICK_MS && four <= GMAP_OPEN_CURL) {
    float t = curlOf(0);
    if (!isnan(t) && t <= GMAP_THUMB_OUT_CURL && servosPlatesState() == PlatesState::Closed) { used = true; plates(true); }
  }
  if (fist) {                                                         // back in the fist
    if (!used && gotOut && now - leftAt <= GMAP_PULSE_MS) {
      if (leftThumb == Th::In && leftHeld && leftFull) { if (servosPlatesState() != PlatesState::Closed) plates(false); }   // CLOSE ALL
      else if (leftThumb == Th::Out) {                                // the camera: the second of two quick pulses (only the first needs the real fist -
        if (pulse1Back && leftAt - pulse1Back <= GMAP_DOUBLE_GAP_MS) { pulse1Back = 0; lastDoubleMs = now; camera(); }   // between two quick flicks the fingers may not close all the way)
        else if (leftFull) pulse1Back = now;
      }
    }
    fp = Fp::Fist; fistSince = now; fistTop = four;
  } else if (now - leftAt > GMAP_PULSE_MS) { fp = Fp::Idle; pulse1Back = 0; }   // stayed out: an open hand
}

const char* gmapPoseText() {
  static char b[160];
  char why[110] = ""; bool al = gmapAligned(why, sizeof why);
  float f = flyFingerCurl();
  snprintf(b, sizeof b, "fingers %s (%.0f %%), thumb %s, %s%s; %s", isnan(f) ? "?" : f >= FLY_CLOSED_CURL ? "in a fist" : f <= GMAP_OPEN_CURL ? "open" : "half",
           isnan(f) ? 0.0f : f * 100, thName(thumbState()), twoFingers() ? "two fingers out" : pointing() ? "pointing" : "", (twoFingers() || pointing()) ? ", " : "", al ? "the wrist aligned" : why);
  return b;
}
