#include "config.h"
#include "gesture.h"
#include "config.h"
#include "flex.h"
#include "palm.h"
#include "fly.h"
#include "modesw.h"
#include "s3link.h"
#include "gesturemap.h"
#include <GauntletLink.h>
#include <Preferences.h>
#include <GauntletBenchIO.h>
#include <math.h>

static uint16_t tFlat = 0, tFist = 0;       // thumb mV: flat, and closed as far as a fist allows. 0 = not armed
static uint16_t tAlone = 0;                  // (2026-10-21) thumb mV closed on its own (fingers out, curled all the way): its full curl for the model; 0 = unknown (the fist value stands in). NVS "gta"
static void thumbSwingWarn(const char* from) {   // a thumb that "moved" 35 mV between flat and closed (2026-10-03) put every thumb rule at the noise floor: the drone disarmed by itself in flight
  if (tFlat && tFist && tFlat > tFist && tFlat - tFist < GESTURE_THUMB_SWING_WARN_MV)
    Bench.printf("gesture: WARNING - the thumb calibration %s says it moves only %u mV between flat (%u) and closed (%u); it used to be ~190. Redo steps 4, 5 and 6 with the thumb really crunched. Meanwhile a disarm press must be %u mV\n", from, tFlat - tFist, tFlat, tFist, (unsigned)DISARM_AMP_MIN_MV);
}
static uint16_t fFlat[4]   = { 0, 0, 0, 0 };  // index..pinky mV flat and closed, from the page (0 = unknown) - the dips and the closed line come from these
static uint16_t fClosed[4] = { 0, 0, 0, 0 };
static uint16_t fistLo[4]  = { 0, 0, 0, 0 };  // index..pinky mV: below = that finger is closed; 0 = unknown. = flat - GESTURE_FIST_FRAC * (flat - closed), or an old page's own number (f=)
static constexpr uint8_t HIST = 32;         // 320 ms of thumb samples at FLEX_SAMPLE_MS
static uint16_t hist[HIST];
static uint8_t  hIdx         = 0;
// the flick in progress: opened by a fast fall, confirmed by a release, or discarded
static bool     inFlick      = false;
static uint16_t flickMin     = 0;
static uint16_t flickStartMv = 0;           // where the fall started, for the excursion
static uint32_t flickStart   = 0;
static uint32_t flickLastLow = 0;           // when the fall last made a new low
// the last three confirmed flicks (sliding), judged whenever a new one is confirmed
static uint32_t flickT[3]    = { 0, 0, 0 };
static uint16_t flickAmp[3]  = { 0, 0, 0 };  // each flick's own excursion
static uint16_t flickDepth[3] = { 0, 0, 0 }; // and how far below the flat thumb it went
static uint32_t refusedNoteMs = 0;                  // the last refused disarm put into the flight log (one per 2 s)
static uint8_t  nFlicks      = 0;
static uint32_t afterFlightUntil = 0;           // no triple until this: the clicks just before were the disarm's (GESTURE_AFTER_FLIGHT_MS)
static uint32_t fired        = 0;
static uint32_t lastFireMs   = 0;
static uint32_t dropped      = 0;           // confirmed flicks that arrived inside the cooldown after a hit
static uint32_t wobbles      = 0;           // fast falls that never reached the deliberate size
static uint32_t heldOnes     = 0;           // fast falls that were never released in time (held, or stair-stepped)
static uint16_t wobbleMax    = 0;           // the largest too-small fall seen since boot - where the jitter really sits
static uint16_t recentMax    = 0;           // and in the current 5 s summary window
static uint8_t  recentN      = 0;
static uint32_t summaryMs    = 0;
static constexpr uint16_t TRACE = 256;      // 2.56 s of the thumb at 10 ms, for GET /gesture?trace=1
static uint16_t traceRaw[TRACE], traceMed[TRACE];
static uint16_t traceIdx     = 0;
static uint32_t lastPollMs   = 0;
static uint16_t rawPrev[4]   = { 0, 0, 0, 0 };   // the four samples before this one: a median of five kills ADC spikes up to two samples long, costs 20 ms
static uint32_t flicksSeen   = 0;           // every confirmed flick, hit or not - the telemetry page ticks the thumb trace with them
static uint32_t doubles      = 0;           // thumb doubles (two flicks in rhythm and alike), counted when the second one ends
static uint32_t quickDoubles = 0;           // a pair counted the moment the SECOND flick OPENS (no size rule) - the stats only
static uint32_t disarmPresses = 0;          // the drone's DISARM: POINTING (index up, middle / ring / pinky closed) since before a thumb press, and a second thumb press 150-800 ms after it - counted the moment it starts, only while the drone is armed or flipping (2026-10-05; two big presses with the fingers still before - refused in flight: "fingers moved 56 %")
static uint16_t fcClicks = 0, fcPairs = 0, fcNoPose1 = 0, fcLost2 = 0;   // this flight's thumb clicks for the flight log at disarm (the console log is gone after every power-up)
static uint32_t disarmFiredMs = 0;          // when that last fired: the ring's own triple, judged on the same crunches, is then left alone
static char     verdict[128] = "";          // what judge() last decided, in words, for the telemetry page
static uint32_t verdictMs    = 0;
static Preferences prefs;

// ---- finger dips (2026-09-29): pinky = brighter, index = dimmer ----
// One finger falls and comes back while the other three and the thumb stay still. Judged on the
// smoothed readings (a dip is slower than a thumb flick; the 35 ms of smoothing costs nothing and
// kills ADC spikes) against a shared history of all five fingers.
static constexpr uint8_t FH = 32;           // 320 ms of all five, smoothed
static uint16_t fh[5][FH];
static uint8_t  fhIdx = 0;
static bool     fhFilled = false;
struct Dip {
  uint8_t     finger;                       // flex index: 1 index, 4 pinky
  uint8_t     guards[4];                    // the fingers that must stay still meanwhile (flex indexes)
  const char* name;
  bool        open;
  uint32_t    t0;                           // when the fall started
  uint16_t    startMv, minMv;
  uint16_t    gLo[4], gHi[4];               // each guard's range from just before the fall to now
  uint32_t    lastMs;
  uint32_t    count, ignored;
};
static Dip dips[2] = { { 1, { 0, 2, 3, 4 }, "index", false, 0, 0, 0, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, 0, 0, 0 },
                       { 4, { 0, 1, 2, 3 }, "pinky", false, 0, 0, 0, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, 0, 0, 0 } };

static uint16_t median5(const uint16_t* v) {     // insertion sort of five, middle one
  uint16_t s[5]; for (int i = 0; i < 5; i++) s[i] = v[i];
  for (int i = 1; i < 5; i++) { uint16_t x = s[i]; int j = i - 1; while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; j--; } s[j + 1] = x; }
  return s[2];
}

static bool     armed()    { return tFlat > tFist + 30; }
static uint16_t swing()    { return tFlat - tFist; }
static uint16_t floorTo(float v, uint16_t lo) { uint16_t u = (uint16_t)(v + 0.5f); return u < lo ? lo : u; }
static uint16_t dropSize() { return floorTo(swing() * GESTURE_FLICK_FRAC, GESTURE_FLICK_MIN_MV); }   // the fast fall that opens a flick
static uint16_t ampMin()   { return floorTo(swing() * GESTURE_AMP_MIN_FRAC, GESTURE_AMP_MIN_MV); }   // the excursion that makes it deliberate
static const char* crunchName() { return GESTURE_FLICKS == 2 ? "double" : GESTURE_FLICKS == 3 ? "triple" : GESTURE_FLICKS == 4 ? "quadruple" : "multi"; }
static uint16_t swingOf(uint8_t flexIdx) {       // flat - closed in mV for any finger (0 thumb .. 4 pinky), 0 = unknown
  if (flexIdx == 0) return armed() ? swing() : 0;
  uint8_t i = flexIdx - 1;
  return (fFlat[i] && fClosed[i] && fFlat[i] > fClosed[i]) ? fFlat[i] - fClosed[i] : 0;
}
static uint16_t closedLine(uint16_t flat, uint16_t closed) { return flat - (uint16_t)((flat - closed) * GESTURE_FIST_FRAC + 0.5f); }

static void load() {
  if (!prefs.begin("ironman", true)) return;
  tFlat = prefs.getUShort("gtf", 0); tFist = prefs.getUShort("gtc", 0); tAlone = prefs.getUShort("gta", 0);
  thumbSwingWarn("from NVS");
  for (int i = 0; i < 4; i++) {
    char k[5];
    snprintf(k, sizeof k, "gf%d", i); fistLo[i]  = prefs.getUShort(k, 0);
    snprintf(k, sizeof k, "gl%d", i); fFlat[i]   = prefs.getUShort(k, 0);
    snprintf(k, sizeof k, "gx%d", i); fClosed[i] = prefs.getUShort(k, 0);
  }
  prefs.end();
}
static void save() {
  if (!prefs.begin("ironman", false)) return;
  prefs.putUShort("gtf", tFlat); prefs.putUShort("gtc", tFist); prefs.putUShort("gta", tAlone);
  for (int i = 0; i < 4; i++) {
    char k[5];
    snprintf(k, sizeof k, "gf%d", i); prefs.putUShort(k, fistLo[i]);
    snprintf(k, sizeof k, "gl%d", i); prefs.putUShort(k, fFlat[i]);
    snprintf(k, sizeof k, "gx%d", i); prefs.putUShort(k, fClosed[i]);
  }
  prefs.end();
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
static void parse4(const String& s, uint16_t* out) {   // "712,690,0,655" -> four mV, 0 where absent or silly
  for (int i = 0; i < 4; i++) out[i] = 0;
  int p = 0;
  for (int i = 0; i < 4 && p <= (int)s.length(); i++) {
    int e = s.indexOf(',', p); if (e < 0) e = s.length();
    int v = s.substring(p, e).toInt();
    out[i] = (v >= 50 && v <= 3300) ? (uint16_t)v : 0;
    p = e + 1;
  }
}

// The "closed hand" that turns the ring off: middle, ring and pinky below their thresholds. The index
// finger is ignored either way - Toby keeps it out, "it's hard to do 3 thumb switches with all fingers
// down" (2026-09-27; before that, three of the four fingers). All three thresholds must be known.
static bool fistNow() {                     // the closed hand: at least TWO of middle, ring and pinky below their closed lines (each of the three until 2026-10-07 - his middle-finger sensor reading open made a closed hand read open: "I have to click my thumb about 10 times")
  uint8_t closed = 0;
  for (int i = 1; i < 4; i++) { if (!fistLo[i]) return false; if (!flexFault(i + 1) && flexMv(i + 1) < fistLo[i]) closed++; }   // fistLo[0] is the index finger: not consulted; a FAULTED finger is never closed (2026-10-08)
  return closed >= 2;
}
static String fistWords() {
  String s;
  for (int i = 1; i < 4; i++) s += String(i > 1 ? " " : "") + FLEX_NAMES[i + 1][0] + (flexFault(i + 1) ? "-FAULT" : fistLo[i] ? (flexMv(i + 1) < fistLo[i] ? "-closed" : "-open") : "-?");
  return s + " (index ignored)";
}
static String numbersLine() {                // what the detector runs on, for the log and the status line
  String s = "thumb flat " + String(tFlat) + " / closed-in-a-fist " + String(tFist) + " mV (a flick opens with a fall of " + String(dropSize()) + " mV inside " + String(GESTURE_FLICK_MS) + " ms); ";
  s += "closed below " + String(fistLo[1]) + " / " + String(fistLo[2]) + " / " + String(fistLo[3]) + " mV (middle / ring / pinky";
  if (fFlat[1] || fFlat[2] || fFlat[3]) s += String(" = ") + String((unsigned)(GESTURE_FIST_FRAC * 100 + 0.5f)) + " % of the way from flat to closed";
  s += ", index ignored, 0 = unknown)";
  uint16_t sp = swingOf(4), si = swingOf(1);
  s += sp && si ? "; dips: pinky " + String(floorTo(sp * DIP_FRAC, DIP_MIN_MV)) + " mV down and back = brighter, index " + String(floorTo(si * DIP_FRAC, DIP_MIN_MV)) + " mV = dimmer"
                : "; dips off (no flat/closed for the pinky and index yet - the page sends them)";
  return s;
}

void gestureBegin() {
  load();
  for (uint8_t i = 0; i < HIST; i++) hist[i] = 0;
  if (armed()) Bench.println("gesture: armed - " + numbersLine() + ". from the page's calibration, saved");
  else         Bench.printf("gesture: thumb %s-crunch not armed yet - calibrate the fingers on the sim page and it sends the numbers\n", crunchName());
}

static void fire(uint32_t g1, uint32_t g2, bool fake) {
  bool fist = fistNow();
  fired++; lastFireMs = millis(); nFlicks = 0;
  Bench.printf("gesture: thumb %s-crunch #%u (%sgaps %u / %u ms, sizes %u / %u / %u mV), other fingers %s [%s]\n",
               crunchName(), (unsigned)fired, fake ? "pretend, " : "", (unsigned)g1, (unsigned)g2, flickAmp[0], flickAmp[1], flickAmp[2],
               fist ? "closed (middle, ring, pinky)" : "open", fistWords().c_str());
  snprintf(verdict, sizeof verdict, "#%u FIRED%s: gaps %u / %u ms, sizes %u / %u / %u mV, hand %s", (unsigned)fired, fake ? " (pretend)" : "",
           (unsigned)g1, (unsigned)g2, flickAmp[0], flickAmp[1], flickAmp[2], fist ? "closed" : "open");
  verdictMs = millis();
  if (flyEnabled()) { Bench.println("gesture: flight mode is on - the ring is not touched (a thumb crunch disarms the drone instead)"); return; }
  gmapTriple(fist);                                  // the gesture map (gesturemap.cpp): in the ring context exactly the HandBench ring; arm raised / palm up = the placeholders
}

// The last three flicks make the gesture only when every gap is short (<= GESTURE_WINDOW_MS) and the two
// gaps roughly match (longer <= GESTURE_EVEN_RATIO_X10/10 x shorter) - "in unison" - AND the three are
// alike in size (largest <= GESTURE_AMP_RATIO_X10/10 x smallest) by either measure: their own excursions
// ("down, pause, down, pause, down" - equal steps) or their depths below the flat thumb ("down, up a
// little, down, up a little, down" - equal depths). One big crunch and two little ones from rest fail
// both. The history slides, so a good three is caught whenever it happens; a failed three is logged.
static bool alike3(const uint16_t* v) {
  uint16_t hi = v[0], lo = v[0];
  for (int i = 1; i < 3; i++) { if (v[i] > hi) hi = v[i]; if (v[i] < lo) lo = v[i]; }
  return (uint32_t)hi * 10 <= (uint32_t)lo * GESTURE_AMP_RATIO_X10;
}
static void judge() {
  if (flyEnabled() || (int32_t)(millis() - afterFlightUntil) < 0) return;   // (the second: Ironman, 2026-10-07 night - just after a flight the clicks still in the history were the disarm's; with the gesture map a third one would have completed "pointing + triple" = the thermal camera) in flight the thumb is the disarm's alone (2026-10-07: a "triple" fired here while flying - nothing for the ring, but it wiped the click history and blocked the thumb for 600 ms, and his disarm clicks spammed into it: "about 50 times before it turns off")
  if (nFlicks < 3) return;
  if (disarmFiredMs && millis() - disarmFiredMs < 1500) { snprintf(verdict, sizeof verdict, "two big presses = the drone's disarm - the ring is left alone"); verdictMs = millis(); return; }
  uint32_t g1 = flickT[1] - flickT[0], g2 = flickT[2] - flickT[1];
  uint32_t hiG = g1 > g2 ? g1 : g2, loG = g1 > g2 ? g2 : g1;
  bool fast  = hiG <= GESTURE_WINDOW_MS && loG >= GESTURE_MIN_GAP_MS;
  bool even  = hiG * 10 <= loG * GESTURE_EVEN_RATIO_X10;
  bool alike = alike3(flickAmp) || alike3(flickDepth);
  if (fast && even && alike) fire(g1, g2, false);
  else if (hiG <= 1500) {                                             // say why only when the three were close enough to have been an attempt
    const char* why = !fast ? "not quick enough" : !even ? "not in one rhythm" : "not alike in size";
    Bench.printf("gesture: three flicks but %s (gaps %u / %u ms, sizes %u / %u / %u mV) - ignored\n", why, (unsigned)g1, (unsigned)g2, flickAmp[0], flickAmp[1], flickAmp[2]);
    snprintf(verdict, sizeof verdict, "three flicks but %s: gaps %u / %u ms, sizes %u / %u / %u mV", why, (unsigned)g1, (unsigned)g2, flickAmp[0], flickAmp[1], flickAmp[2]);
    verdictMs = millis();
  }
}

// ---- the disarm's pose (2026-10-05): POINTING - the index up, middle, ring and pinky closed. Toby: "my pinky, ring and middle
//      close and my thumb twice ... turns it off instantly", then: "I don't want it to disarm when I'm descending - my pointer
//      finger down and all my fingers down and I accidentally double click ... only when my pointer finger is up and the other
//      three are down". The MEDIAN of middle, ring and pinky past DISARM_FIST_CURL of its travel in the page's curl
//      (resistance, like the throttle) - the middle value of the three, so one finger reading wrong cannot spoil it (his
//      middle-finger sensor read 0, 9 and 21 % in three of four refused attempts the same night) - and the index at most
//      DISARM_INDEX_UP_CURL; the pose holds until the median drops under DISARM_FIST_OPEN_CURL or the index curls past
//      DISARM_INDEX_DOWN_CURL. A full fist (the descent) is never the pose, and without the page's numbers for all four
//      fingers there is no pose at all (the arming needs them anyway). The pose must be held DISARM_CLOSED_MS when the first
//      click begins (so the thumb dragged along by the hand closing is not a click) and seen within DISARM_POSE_GRACE_MS when
//      the second begins - between the two it may flicker (until the same night it had to hold without a single 10 ms gap).
static bool     handClosedLatched = false;
static uint32_t handClosedSince = 0;          // the pose held continuously since this; 0 = not
static uint32_t poseSeenMs = 0;               // the last 10 ms sample with the pose held
static bool     flickPoseNow = false;          // the flick in progress began with the pose held DISARM_CLOSED_MS
static bool     flickPose[3] = { false, false, false };   // the same for the last three confirmed flicks, oldest first (like flickT)
static float med3(float a, float b, float c) { return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c)); }
static float threeCurl(float* m, float* r, float* p) { *m = gestureFingerCurl(2); *r = gestureFingerCurl(3); *p = gestureFingerCurl(4); return (isnan(*m) || isnan(*r) || isnan(*p)) ? NAN : med3(*m, *r, *p); }   // the MEDIAN of middle / ring / pinky
static bool handClosedState() {
  float m, r, p; float md = threeCurl(&m, &r, &p), ix = gestureFingerCurl(1);
  if (isnan(md) || isnan(ix)) return handClosedLatched = false;
  if (!handClosedLatched) handClosedLatched = ix <= DISARM_INDEX_UP_CURL && md >= DISARM_FIST_CURL;
  else handClosedLatched = !(ix > DISARM_INDEX_DOWN_CURL || md < DISARM_FIST_OPEN_CURL);
  return handClosedLatched;
}
static void endFlick(uint16_t mv) {
  inFlick = false;
  for (uint8_t i = 0; i < HIST; i++) hist[i] = mv;   // the next flick must be a fresh fall from here, not the tail of this one
}

static void thumbPoll(uint32_t now) {
  if (handClosedState()) { if (!handClosedSince) handClosedSince = now; poseSeenMs = now; } else handClosedSince = 0;   // the disarm's pointing pose, every 10 ms
  if (flexFault(0)) { if (inFlick) inFlick = false; nFlicks = 0; rawPrev[0] = 0; return; }   // (2026-10-08) a faulted thumb (open, shorted, out of range) makes no clicks: a flapping wire must never be a crunch
  uint16_t rawNow = flexRawMv(0);                      // the unsmoothed sample: the display smoothing (35 ms) hides the pause between two quick crunches
  if (!rawPrev[0]) for (int i = 0; i < 4; i++) rawPrev[i] = rawNow;
  uint16_t five[5] = { rawPrev[0], rawPrev[1], rawPrev[2], rawPrev[3], rawNow };
  uint16_t mv = median5(five);                         // ... but a spike of a sample or two is not a movement
  for (int i = 0; i < 3; i++) rawPrev[i] = rawPrev[i + 1];
  rawPrev[3] = rawNow;
  traceRaw[traceIdx] = rawNow; traceMed[traceIdx] = mv; traceIdx = (traceIdx + 1) % TRACE;
  if (recentN && now - summaryMs >= 5000) {              // what was ignored lately, and how big it was - the jitter's own measure
    Bench.printf("gesture: %u fall%s under the %u mV size in the last 5 s (largest %u mV)\n", recentN, recentN == 1 ? "" : "s", ampMin(), recentMax);
    recentN = 0; recentMax = 0; summaryMs = now;
  }
  uint8_t back = GESTURE_FLICK_MS / FLEX_SAMPLE_MS; if (back > HIST - 1) back = HIST - 1;
  uint16_t mx = mv;                                    // the highest reading in the last GESTURE_FLICK_MS
  for (uint8_t k = 1; k <= back; k++) { uint16_t v = hist[(uint8_t)(hIdx + HIST - k) % HIST]; if (v > mx) mx = v; }
  if (!inFlick) {
    if (mx >= mv + dropSize()) {                                             // a fast fall: a flick may be starting
      inFlick = true; flickStartMv = mx; flickMin = mv; flickStart = now; flickLastLow = now;
      flickPoseNow = handClosedSince && now - handClosedSince >= DISARM_CLOSED_MS;   // pointing when this click began?
      if (nFlicks >= 1 && !(fired && now - lastFireMs < GESTURE_COOLDOWN_MS)) {
        uint32_t g2 = now - flickT[2];
        if (g2 >= GESTURE_MIN_GAP_MS && g2 <= GESTURE_WINDOW_MS) quickDoubles++;   // (stats)
        if (flyEnabled() && g2 >= GESTURE_MIN_GAP_MS && g2 <= DISARM_WINDOW_MS) {   // a second thumb press while flying: pointing when the first began and still at this one = the drone's DISARM - counted the moment this one starts
          bool posed = flickPose[2] && poseSeenMs && now - poseSeenMs <= DISARM_POSE_GRACE_MS;   // pointing when the first click began, and still (or a moment ago) now
          fcPairs++; if (!posed) { if (!flickPose[2]) fcNoPose1++; else fcLost2++; }
          if (posed) { disarmPresses++; disarmFiredMs = now; }
          else {
            float m, r, p; float md = threeCurl(&m, &r, &p), ix = gestureFingerCurl(1);
            if (!isnan(md) && !isnan(ix) && md >= 0.4f && ix <= 0.6f && now - refusedNoteMs > 2000) {   // a pointing hand not quite made: that was an attempt - say what was missing (a full fist, index down too, is the descent: its clicks are ignored without a word)
              refusedNoteMs = now;
              const char* at = !flickPose[2] ? "not pointing at click 1" : "pointing lost at click 2";
              flyNote("%s: i%u m%u r%u p%u", at, (unsigned)lroundf(ix * 100), (unsigned)lroundf(m * 100), (unsigned)lroundf(r * 100), (unsigned)lroundf(p * 100));
              Bench.printf("gesture: two thumb clicks, but %s: index %u %% (%u or less), middle / ring / pinky %u / %u / %u %% (the middle value of the three %u or more) - not a disarm\n", at,
                           (unsigned)lroundf(ix * 100), (unsigned)lroundf(DISARM_INDEX_UP_CURL * 100), (unsigned)lroundf(m * 100), (unsigned)lroundf(r * 100), (unsigned)lroundf(p * 100), (unsigned)lroundf(DISARM_FIST_CURL * 100));
            }
          }
        }
      }
    }
  } else {
    if (mv + 3 < flickMin) { flickMin = mv; flickLastLow = now; } else if (mv < flickMin) flickMin = mv;
    uint16_t exc = flickStartMv - flickMin;
    uint16_t rel = (uint16_t)(exc * GESTURE_RELEASE_FRAC); if (rel < GESTURE_RELEASE_MIN_MV) rel = GESTURE_RELEASE_MIN_MV;
    bool released = mv >= flickMin + rel;                                    // came back up a little ...
    bool paused   = now - flickLastLow >= GESTURE_PLATEAU_MS;                // ... or stopped falling: either way the crunch is over and it counts
    if (released || paused) {
      if (exc < ampMin()) {                                                   // under the opening size after all (jitter)
        wobbles++; if (exc > wobbleMax) wobbleMax = exc;
        if (!recentN) summaryMs = now; recentN++; if (exc > recentMax) recentMax = exc;
      }
      else if (fired && now - lastFireMs < GESTURE_COOLDOWN_MS) dropped++;   // the tail of the burst that just fired
      else {
        uint16_t depth = exc; if (tFlat > flickMin && tFlat - flickMin > depth) depth = tFlat - flickMin;   // how deep it went below the flat thumb
        flickT[0] = flickT[1]; flickT[1] = flickT[2]; flickT[2] = flickStart;
        flickPose[0] = flickPose[1]; flickPose[1] = flickPose[2]; flickPose[2] = flickPoseNow;
        if (flyEnabled()) fcClicks++;
        flickAmp[0] = flickAmp[1]; flickAmp[1] = flickAmp[2]; flickAmp[2] = exc;
        flickDepth[0] = flickDepth[1]; flickDepth[1] = flickDepth[2]; flickDepth[2] = depth;
        if (nFlicks < 3) nFlicks++;
        flicksSeen++;
        bool usedUp = false;
        if (nFlicks >= 2) {                                                   // a thumb DOUBLE (the drone's disarm): the last two flicks in rhythm and alike in size
          uint32_t g = flickT[2] - flickT[1];
          uint16_t hi = flickAmp[1] > flickAmp[2] ? flickAmp[1] : flickAmp[2], lo = flickAmp[1] > flickAmp[2] ? flickAmp[2] : flickAmp[1];
          if (g >= GESTURE_MIN_GAP_MS && g <= GESTURE_WINDOW_MS && (uint32_t)hi * 10 <= (uint32_t)lo * GESTURE_AMP_RATIO_X10) { doubles++; usedUp = gmapThumbDouble(); }   // (2026-10-21) two fingers out + a double = the green laser off
        }
        if (usedUp) nFlicks = 0; else judge();                                // a used double never becomes the first two of a triple
      }
      endFlick(mv);
    } else if (now - flickStart > GESTURE_FLICK_MAX_MS) {                    // still falling after all this time: a slow curl, not a flick
      heldOnes++;
      endFlick(mv);
    }
  }
  hist[hIdx] = mv; hIdx = (hIdx + 1) % HIST;
}

// ---- dips ----
static uint16_t stillLimit(uint8_t flexIdx) {     // how much a guard finger may move during a dip
  uint16_t s = swingOf(flexIdx);
  if (!s) return DIP_STILL_MIN_MV * 2;            // not calibrated: an absolute allowance
  return floorTo(s * DIP_STILL_FRAC, DIP_STILL_MIN_MV);
}
static uint16_t fhBack(uint8_t f, uint8_t k) { return fh[f][(uint8_t)(fhIdx + FH - k) % FH]; }   // finger f, k samples ago (k >= 1)

static void dipDone(Dip& d, uint16_t exc, uint32_t tookMs, uint32_t now) {
  for (int g = 0; g < 4; g++) {
    uint16_t moved = d.gHi[g] - d.gLo[g], lim = stillLimit(d.guards[g]);
    if (flexFault(d.guards[g])) continue;               // a faulted guard is left out (its reading means nothing)
    if (moved > lim) {
      d.ignored++;
      if (!flyEnabled()) Bench.printf("gesture: %s dip ignored - the %s moved %u mV with it (limit %u; a dip is one finger on its own)\n", d.name, FLEX_NAMES[d.guards[g]], moved, lim);
      return;
    }
  }
  if (inFlick || (fired && now - lastFireMs < GESTURE_COOLDOWN_MS)) { d.ignored++; if (!flyEnabled()) Bench.printf("gesture: %s dip ignored - the thumb is busy\n", d.name); return; }
  if (flyEnabled()) { d.ignored++; return; }                     // in flight mode the fingers are the throttle, not a brightness knob
  if (d.lastMs && now - d.lastMs < DIP_GAP_MS) return;
  d.lastMs = now; d.count++;
  if (!modeAllowsShow()) { s3linkAct(d.finger == 4 ? glink::ACT_RING_UP : glink::ACT_RING_DOWN, glink::RES_BLOCKED, 1); return; }   // SAFE: seen and reported, nothing changes
  String m = palmDip(d.finger == 4); s3linkAct(d.finger == 4 ? glink::ACT_RING_UP : glink::ACT_RING_DOWN, glink::RES_DONE, 1);
  Bench.printf("gesture: %s dip #%u (%u mV down and back in %u ms) -> %s\n", d.name, (unsigned)d.count, exc, (unsigned)tookMs, m.c_str());
}
static void dipPoll(Dip& d, uint32_t now) {
  uint16_t sw = swingOf(d.finger);
  uint16_t mv = flexMv(d.finger);
  if (!sw || !fhFilled || flexFault(d.finger)) { d.open = false; return; }   // a faulted finger never dips (2026-10-08)
  uint16_t need = floorTo(sw * DIP_FRAC, DIP_MIN_MV);
  uint8_t back = DIP_FALL_MS / FLEX_SAMPLE_MS; if (back > FH - 1) back = FH - 1;
  if (!d.open) {
    uint16_t mx = mv;                                    // the highest reading in the last DIP_FALL_MS
    for (uint8_t k = 1; k <= back; k++) { uint16_t v = fhBack(d.finger, k); if (v > mx) mx = v; }
    if (mx >= mv + need) {                               // a fall of a quarter of the swing inside DIP_FALL_MS: a dip may be starting
      d.open = true; d.t0 = now; d.startMv = mx; d.minMv = mv;
      for (int g = 0; g < 4; g++) {                      // the guards' range over the same window, so a fall they shared shows
        uint16_t v = flexMv(d.guards[g]); d.gLo[g] = d.gHi[g] = v;
        for (uint8_t k = 1; k <= back; k++) { uint16_t u = fhBack(d.guards[g], k); if (u < d.gLo[g]) d.gLo[g] = u; if (u > d.gHi[g]) d.gHi[g] = u; }
      }
    }
    return;
  }
  if (mv < d.minMv) d.minMv = mv;
  for (int g = 0; g < 4; g++) { uint16_t v = flexMv(d.guards[g]); if (v < d.gLo[g]) d.gLo[g] = v; if (v > d.gHi[g]) d.gHi[g] = v; }
  uint16_t exc = d.startMv - d.minMv;
  uint16_t up = (uint16_t)(exc * DIP_BACK_FRAC + 0.5f);
  if (mv >= d.minMv + up && exc >= need) {                // came back up: the dip is complete
    d.open = false;
    dipDone(d, exc, now - d.t0, now);
    for (uint8_t k = 0; k < FH; k++) fh[d.finger][k] = mv;   // the next one must be a fresh fall from here
  } else if (now - d.t0 > DIP_MAX_MS) {                   // still down: a curl or a hold, not a dip
    d.open = false;
    for (uint8_t k = 0; k < FH; k++) fh[d.finger][k] = mv;
  }
}

// ---- on-arm calibration (2026-10-07; with no computer connected): keys K (hand flat) and J (fist) average every finger
// for GESTURE_CAPTURE_MS and hand the result to gestureSet() exactly as the sim page's steps 4 and 5 do over WiFi ----
static int8_t   capMode = -1;                  // -1 idle, 0 capturing flat, 1 capturing the fist
static uint32_t capT0 = 0, capN = 0;
static float    capSum[5];
// (2026-10-21) the puck's CALIBRATE steps 1 / 2: one pose, saved at once, the other pose kept - so one step can be redone
// alone. index..pinky: a faulted sensor, or a swing against the other pose under GESTURE_FIST_MIN_MV, refuses the step and
// changes nothing. The thumb never refuses it (Toby: there may be no thumb sensor): its number goes in only when it is a
// reading (over FLEX_FAULT_LOW_MV) and fits the other pose.
const char* gestureCalPose(bool fist, const uint16_t mv[5]) {
  static char why[110]; why[0] = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (flexBroken(i + 1)) { snprintf(why, sizeof why, "the %s finger's sensor reads nothing (%u mV: open, shorted or silent)", FLEX_NAMES[i + 1], mv[i + 1]); return why; }   // (2026-10-21) not the old range: a new sensor reads outside it
    uint16_t flat = fist ? fFlat[i] : mv[i + 1], closed = fist ? mv[i + 1] : fClosed[i];
    if (flat && closed && !(flat > closed + GESTURE_FIST_MIN_MV)) { snprintf(why, sizeof why, "the %s finger moved only %d mV between flat and closed (needs %u)", FLEX_NAMES[i + 1], (int)flat - (int)closed, (unsigned)GESTURE_FIST_MIN_MV); return why; }
  }
  for (uint8_t i = 0; i < 4; i++) {
    (fist ? fClosed : fFlat)[i] = mv[i + 1];
    fistLo[i] = (fFlat[i] && fClosed[i]) ? closedLine(fFlat[i], fClosed[i]) : 0;
  }
  if (mv[0] > FLEX_FAULT_LOW_MV && !flexBroken(0)) {
    uint16_t tf = fist ? tFlat : mv[0], tc = fist ? mv[0] : tFist;
    if (!tf || !tc || (tc >= 50 && tf >= tc + 30)) { if (fist) tFist = mv[0]; else tFlat = mv[0]; }
  }
  save();
  inFlick = false; nFlicks = 0;
  for (int k = 0; k < 2; k++) dips[k].open = false;
  Bench.printf("calibrate: %s saved - index %u, middle %u, ring %u, pinky %u mV (thumb %u, %s)\n", fist ? "FIST" : "FLAT", mv[1], mv[2], mv[3], mv[4], mv[0],
               (fist ? tFist : tFlat) == mv[0] ? "kept" : "not used");
  return "";
}

// (2026-10-21) the puck's CALIBRATE step 3: the thumb closed on its own - its full closed position (the model's thumb; the
// gestures keep the fist-limited one). Refused without a thumb sensor, or with a swing from flat under 30 mV.
const char* gestureCalThumbAlone(uint16_t mv) {
  static char why[100]; why[0] = 0;
  if (flexBroken(0) || mv <= FLEX_FAULT_LOW_MV) { snprintf(why, sizeof why, "no thumb sensor reading (%u mV)", mv); return why; }
  if (tFlat && !(tFlat > mv + 30)) { snprintf(why, sizeof why, "the thumb moved only %d mV from flat (needs 30)", (int)tFlat - (int)mv); return why; }
  tAlone = mv; save();
  Bench.printf("calibrate: THUMB ALONE saved - %u mV (flat %u, inside a fist %u)\n", mv, tFlat, tFist);
  return "";
}
// The thumb's curl for the model: against its closed-alone value when there is one, else the fist-limited one
float gestureThumbCurlFull() {
  if (!tAlone || !tFlat || tAlone >= tFlat) return gestureFingerCurl(0);
  if (flexFault(0)) return 0.0f;
  auto R = [](float v) { if (v < 50) v = 50; if (v > 3250) v = 3250; return FLEX_DIVIDER_OHM * (3300.0f / v - 1.0f); };
  float r = R(flexMv(0)), r0 = R(tFlat), r1 = R(tAlone);
  if (r1 <= r0) return NAN;
  float c = (r - r0) / (r1 - r0);
  return c < 0 ? 0 : (c > 1 ? 1 : c);
}

void gestureCaptureStart(bool fist) {
  capMode = fist ? 1 : 0; capT0 = millis(); capN = 0; for (int i = 0; i < 5; i++) capSum[i] = 0;
  Bench.printf("calibrate: hold the hand %s and still for %lu s ...\n", fist ? "in a FIST (thumb crunched in as far as the fist lets it)" : "FLAT (fingers and thumb straight)", (unsigned long)(GESTURE_CAPTURE_MS / 1000));
}
bool gestureCapturing() { return capMode >= 0; }
static void capturePoll(uint32_t now) {
  if (capMode < 0) return;
  for (uint8_t i = 0; i < 5; i++) capSum[i] += flexMv(i);
  capN++;
  if (now - capT0 < GESTURE_CAPTURE_MS) return;
  uint16_t m[5]; for (uint8_t i = 0; i < 5; i++) m[i] = (uint16_t)(capSum[i] / (capN ? capN : 1) + 0.5f);
  uint16_t tf = capMode == 0 ? m[0] : tFlat, tc = capMode == 1 ? m[0] : tFist;
  uint16_t fl[4], fx[4];
  for (uint8_t i = 0; i < 4; i++) { fl[i] = capMode == 0 ? m[i + 1] : fFlat[i]; fx[i] = capMode == 1 ? m[i + 1] : fClosed[i]; }
  Bench.printf("calibrate: %s = thumb %u, index %u, middle %u, ring %u, pinky %u mV (%lu samples)%s\n", capMode ? "FIST" : "FLAT", m[0], m[1], m[2], m[3], m[4], (unsigned long)capN,
               (capMode == 0 && !fClosed[0]) || (capMode == 1 && !fFlat[0]) ? " - now the other pose" : "");
  capMode = -1;
  char q[160];
  snprintf(q, sizeof q, "tf=%u&tc=%u&fl=%u,%u,%u,%u&fx=%u,%u,%u,%u", tf, tc, fl[0], fl[1], fl[2], fl[3], fx[0], fx[1], fx[2], fx[3]);
  if (tf && tc) gestureSet(String(q));                     // both thumb numbers known: the same path as the page (checks, NVS, the closed lines)
  else { tFlat = tf; tFist = tc; for (uint8_t i = 0; i < 4; i++) { fFlat[i] = fl[i]; fClosed[i] = fx[i]; } }   // half done: kept, not saved until the other pose
}

void gesturePoll() {
  uint32_t now = millis();
  if (now - lastPollMs < FLEX_SAMPLE_MS) return;      // the flex sampler runs at this rate; no point going faster
  lastPollMs = now;
  capturePoll(now);
  { static bool flying = false; bool f = flyEnabled(); if (f) flying = true; else if (flying) { flying = false; nFlicks = 0; afterFlightUntil = now + GESTURE_AFTER_FLIGHT_MS; } }   // a flight ended: its clicks never count toward a triple
  if (armed()) thumbPoll(now);
  for (int k = 0; k < 2; k++) dipPoll(dips[k], now);  // judged against the history BEFORE this sample goes in
  for (uint8_t f = 0; f < 5; f++) fh[f][fhIdx] = flexMv(f);
  fhIdx = (fhIdx + 1) % FH; if (fhIdx == 0) fhFilled = true;
}

void gestureFake() { fire(0, 0, true); }     // key n: run the crunch action with the fingers as they are now

bool gestureCalMv(uint8_t flexIdx, uint16_t& flat, uint16_t& closed) {   // flex.cpp's range check: the calibrated flat / closed millivolts
  if (flexIdx == 0) { flat = tFlat; closed = tFist; return armed() && flat && closed; }
  if (flexIdx > 4) return false;
  flat = fFlat[flexIdx - 1]; closed = fClosed[flexIdx - 1];
  return flat && closed;
}

float gestureFingerCurl(uint8_t flexIdx) {   // the model's curve: a straight line in resistance between flat and closed (the divider is convex), 0..1
  uint16_t flat, closed;
  if (flexFault(flexIdx) && (flexIdx == 0 ? armed() : (flexIdx <= 4 && fFlat[flexIdx - 1] && fClosed[flexIdx - 1]))) return 0.0f;   // (2026-10-08) FAULTED = open, never bent
  if (flexIdx == 0) { if (!armed()) return NAN; flat = tFlat; closed = tFist; }
  else { if (flexIdx > 4) return NAN; uint8_t i = flexIdx - 1; if (!fFlat[i] || !fClosed[i]) return NAN; flat = fFlat[i]; closed = fClosed[i]; }
  auto R = [](float mv) { if (mv < 50) mv = 50; if (mv > 3250) mv = 3250; return FLEX_DIVIDER_OHM * (3300.0f / mv - 1.0f); };   // 3V3 -> sensor -> pin -> 47k -> GND
  float rf = R(flat), rc = R(closed);
  if (fabsf(rc - rf) < 1.0f) return NAN;
  float c = (R(flexMv(flexIdx)) - rf) / (rc - rf);
  return c < 0 ? 0 : c > 1 ? 1 : c;
}
uint32_t gestureCount()    { return fired; }
uint32_t gestureDipCount() { return dips[0].count + dips[1].count; }
uint32_t gestureDipUpCount()   { return dips[1].count; }   // pinky dips: one level brighter
uint32_t gestureDipDownCount() { return dips[0].count; }   // index dips: one level dimmer
uint32_t gestureDoubleCount() { return quickDoubles; }
uint32_t gestureDisarmCount() { return disarmPresses; }
void     gestureFlightClicksReset() { fcClicks = fcPairs = fcNoPose1 = fcLost2 = 0; }
bool     gestureFlightClicks(char* b, size_t n) {   // "thumb clicks 9, pairs 4: nopoint 3 lost 0": clicks = thumb clicks seen while flying, pairs = second clicks in time, nopoint = not pointing at the first, lost = pointing gone at the second
  if (!fcClicks) return false;
  snprintf(b, n, "thumb clicks %u, pairs %u: nopoint %u lost %u", fcClicks, fcPairs, fcNoPose1, fcLost2);
  return true;
}   // fly.cpp disarms (or stops a flip-over) on any change
bool     gestureFist()     { return fistNow(); }
float    gestureHandOpenMv() {
  float sum = 0; int n = 0;
  int known = 0;
  for (int i = 1; i < 4; i++) if (fistLo[i]) { known++; if (flexFault(i + 1)) continue; sum += (float)flexMv(i + 1) - (float)fistLo[i]; n++; }   // a faulted finger is left out
  return known == 3 && n >= 2 ? sum / n : NAN;
}

// The detector as JSON: the numbers it runs on, the rule constants by their config.h values, the
// flick in progress, the last three confirmed flicks (ms ago / excursion / depth, oldest first),
// the last verdict in words, the dips and the counters. /gesture and the telemetry page's /tele share it.
static String stateJson(const String& msg) {
  uint32_t now = millis();
  char b[900];
  int n = snprintf(b, sizeof b, "{\"tf\":%u,\"tc\":%u,\"drop\":%u,\"ampmin\":%u,\"f\":[%u,%u,%u,%u],\"fl\":[%u,%u,%u,%u],\"fx\":[%u,%u,%u,%u],\"armed\":%d,\"n\":%u,\"dropped\":%u,\"wobbles\":%u,\"held\":%u,\"mv\":%u,\"raw\":%u,\"flick\":%d,\"fist\":%d,\"flicks\":%u,\"maxgap\":%u,\"mingap\":%u,\"even\":%u,\"ampratio\":%u,\"cooldown\":%u,\"flickms\":%u,\"plateau\":%u,\"maxms\":%u,\"relpct\":%u,\"relmin\":%u",
               tFlat, tFist, dropSize(), ampMin(), fistLo[0], fistLo[1], fistLo[2], fistLo[3], fFlat[0], fFlat[1], fFlat[2], fFlat[3], fClosed[0], fClosed[1], fClosed[2], fClosed[3],
               armed() ? 1 : 0, (unsigned)fired, (unsigned)dropped, (unsigned)wobbles, (unsigned)heldOnes,
               flexMv(0), flexRawMv(0), inFlick ? 1 : 0, fistNow() ? 1 : 0, (unsigned)GESTURE_FLICKS, (unsigned)GESTURE_WINDOW_MS, (unsigned)GESTURE_MIN_GAP_MS, (unsigned)GESTURE_EVEN_RATIO_X10, (unsigned)GESTURE_AMP_RATIO_X10,
               (unsigned)GESTURE_COOLDOWN_MS, (unsigned)GESTURE_FLICK_MS, (unsigned)GESTURE_PLATEAU_MS, (unsigned)GESTURE_FLICK_MAX_MS, (unsigned)(GESTURE_RELEASE_FRAC * 100 + 0.5f), (unsigned)GESTURE_RELEASE_MIN_MV);
  n += snprintf(b + n, sizeof b - n, ",\"open\":[%u,%u,%u],\"seen\":%u,\"last\":[",                               // the flick in progress: from mV, lowest mV, ms so far (0 0 0 = none)
                inFlick ? flickStartMv : 0, inFlick ? flickMin : 0, inFlick ? (unsigned)(now - flickStart) : 0, (unsigned)flicksSeen);
  for (uint8_t i = 3 - nFlicks; i < 3; i++)
    n += snprintf(b + n, sizeof b - n, "%s[%u,%u,%u]", i > 3 - nFlicks ? "," : "", (unsigned)(now - flickT[i]), flickAmp[i], flickDepth[i]);
  n += snprintf(b + n, sizeof b - n, "],\"fingers\":[%u,%u,%u,%u],\"dipup\":%u,\"dipdown\":%u,\"dipign\":%u,\"dip\":%d,\"v\":\"%s\",\"vago\":%u,\"msg\":\"",   // dip: the finger dipping right now (1 index, 4 pinky), 0 none
                flexMv(1), flexMv(2), flexMv(3), flexMv(4), (unsigned)dips[1].count, (unsigned)dips[0].count, (unsigned)(dips[0].ignored + dips[1].ignored),
                dips[1].open ? 4 : dips[0].open ? 1 : 0, verdict, verdictMs ? (unsigned)(now - verdictMs) : 0);
  return String(b) + msg + "\"}";
}
String gestureJson() { return stateJson(""); }

String gestureSet(const String& q) {
  if (queryValue(q, "trace") == "1") {                 // the last TRACE samples, oldest first: [[raw, filtered], ...]
    String s; s.reserve(TRACE * 12 + 64);
    s += "{\"ms\":" + String(FLEX_SAMPLE_MS) + ",\"n\":" + String(TRACE) + ",\"drop\":" + String(dropSize()) + ",\"ampmin\":" + String(ampMin()) + ",\"trace\":[";
    for (uint16_t k = 0; k < TRACE; k++) { uint16_t i = (traceIdx + k) % TRACE; s += String(k ? ",[" : "[") + traceRaw[i] + "," + traceMed[i] + "]"; }
    s += "]}";
    return s;
  }
  String fake = queryValue(q, "fake");                 // a bench check of the actions, fingers as they are: crunch (= key n), up (a pinky dip), down (an index dip)
  if (fake.length()) {
    if (fake == "crunch") { gestureFake(); return stateJson(String("gesture: pretend crunch - ") + verdict); }
    if (fake == "up" || fake == "down") { String m = palmDip(fake == "up"); Bench.println("gesture: pretend " + String(fake == "up" ? "pinky" : "index") + " dip -> " + m); return stateJson(m); }
    return stateJson("gesture: fake= takes crunch, up or down");
  }
  String sTf = queryValue(q, "tf"), sTc = queryValue(q, "tc"), sF = queryValue(q, "f"), sFl = queryValue(q, "fl"), sFx = queryValue(q, "fx");
  if (!sTf.length() && !sTc.length() && !sF.length() && !sFl.length() && !sFx.length()) {
    if (queryValue(q, "lo").length()) return stateJson("gesture: that page is the old one (lo/hi) - reload it, the board now takes tf/tc/fl/fx");
    return stateJson("");
  }
  String thumbNo;
  if (sTf.length() || sTc.length()) {
    int nTf = sTf.toInt(), nTc = sTc.toInt();
    if (nTc < 50 || nTf > 3300 || nTf < nTc + 30) {   // (2026-10-20) the thumb's numbers alone are refused - the fingers' still go in (a dead thumb had blocked K / J for all five)
      thumbNo = "gesture: thumb numbers refused (flat " + String(nTf) + ", closed " + String(nTc) + " mV) - need 50 <= closed, closed + 30 <= flat <= 3300; the thumb keeps its old numbers";
      Bench.println(thumbNo);
      if (!sFl.length() && !sFx.length() && !sF.length()) return stateJson(thumbNo);
    } else {
      tFlat = (uint16_t)nTf; tFist = (uint16_t)nTc;
      thumbSwingWarn("just sent by the page");
    }
  }
  if (sFl.length() || sFx.length()) {              // "fl=850,840,860,790&fx=430,470,450,400": index..pinky flat and closed mV, 0 = unknown - the closed line and the dips come from these
    uint16_t fl[4], fx[4]; parse4(sFl, fl); parse4(sFx, fx);
    for (int i = 0; i < 4; i++) {
      if (fl[i] && fx[i] && fl[i] > fx[i] + GESTURE_FIST_MIN_MV) { fFlat[i] = fl[i]; fClosed[i] = fx[i]; fistLo[i] = closedLine(fl[i], fx[i]); }
      else { fFlat[i] = fClosed[i] = 0; fistLo[i] = 0; }
    }
  } else if (sF.length()) {                        // an older page: the closed line itself ("f=712,690,0,655"), no dips
    parse4(sF, fistLo);
    for (int i = 0; i < 4; i++) fFlat[i] = fClosed[i] = 0;
  }
  save();
  inFlick = false; nFlicks = 0;
  for (int k = 0; k < 2; k++) dips[k].open = false;
  String m = "gesture: armed - " + numbersLine() + ". saved. hand open + " + String(GESTURE_FLICKS) + " crunches = off -> on -> white <-> red; middle+ring+pinky closed + crunches = off";
  Bench.println(m);
  if (thumbNo.length()) m += " (" + thumbNo.substring(9) + ")";   // the fingers went in, the thumb's numbers did not
  return stateJson(m);
}

String gestureStatusLine() {
  if (!armed()) return "gesture: not armed - the sim page sends the thumb numbers once the fingers are calibrated";
  return "gesture: armed - thumb " + String(flexMv(0)) + " mV now; " + numbersLine() + "; a flick ends when the thumb comes back up a little or pauses " + String(GESTURE_PLATEAU_MS) + " ms; " + String(GESTURE_FLICKS) +
         " with gaps " + String(GESTURE_MIN_GAP_MS) + "-" + String(GESTURE_WINDOW_MS) + " ms alike within " + String(GESTURE_EVEN_RATIO_X10 / 10) + "." + String(GESTURE_EVEN_RATIO_X10 % 10) + "x and sizes alike within " +
         String(GESTURE_AMP_RATIO_X10 / 10) + "." + String(GESTURE_AMP_RATIO_X10 % 10) + "x fire, " + String(GESTURE_COOLDOWN_MS) +
         " ms cooldown after; fingers " + fistWords() + (fistNow() ? " = closed hand, crunches turn it off" : "") + "; " + String(fired) + " " + crunchName() + "-crunch" + (fired == 1 ? "" : "es") + ", " +
         String(dips[1].count) + " pinky dip" + (dips[1].count == 1 ? "" : "s") + " (brighter), " + String(dips[0].count) + " index dip" + (dips[0].count == 1 ? "" : "s") + " (dimmer), " + String(dips[0].ignored + dips[1].ignored) + " dips ignored (other fingers moved); " +
         String(dropped) + " flicks dropped after hits, " + String(wobbles) + " too-small falls (largest " + String(wobbleMax) + " mV) and " + String(heldOnes) + " never-released ones ignored";
}

