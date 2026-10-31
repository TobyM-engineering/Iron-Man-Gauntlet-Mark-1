// arm_link.h - the main arm's GauntletLink frames (GauntletLink.h, a VERBATIM copy of ironman-gauntlet's
// lib/GauntletLink/src/GauntletLink.h, protocol v2 as of arm commit 5228cb8, blob e3344a3) -> what the puck shows.
//
//   $G  20 Hz, the live state: both IMU segments (wrist = conj(forearm) * hand), the five finger curls, each servo's
//       position, the palm ring as it shows, the strip, the LED groups, accent, lasers, LiDAR, mode, the last
//       event, the battery. This drives the model in LIVE.
//   $S  10 Hz, the status: mode, battery, plates state, ring index, LED drivers, lasers, LiDAR. Without $G (an arm
//       that sends only $S) it drives the older ARM view: the plate cascade replayed from the state, a fist from the
//       red laser.
//   $E  one per gesture / action the arm saw: DONE, BLOCKED by the mode switch, or REFUSED - the HUD's top word.
//   A line of a type the puck does not know is ignored. A frame from another protocol version is never read field by
//   field: it is counted, and the HUD says
//   PROTOCOL MISMATCH with both versions until a good frame arrives.
//
// Two byte streams, one parser each: the arm's UART (GPIO16, header pin 8) and the USB serial, where a typed
// "$G,...*CCCC" or "$S,...*CCCC" line goes through the very same parser. A USB line that does not start with '$' is
// a test command (test_command below). Hardware-free: app.cpp feeds it bytes, the PC tools feed it lines.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "GauntletLink.h"
#include "kin.h"
#include "render.h"
#include "lights_data.h"
#include "panel.h"

namespace app {

constexpr uint32_t ARM_LINK_LOST_MS = 1000;   // no valid $S for longer than this -> "LINK --" (ARM view)
// $G link-lost behaviour (a proposal, 2026-10-07 - Toby decides):
//   up to LIVE_STALE_MS without a $G: nothing changes (one or two frames late is normal jitter)
//   LIVE_STALE_MS .. LIVE_LOST_MS: the model FREEZES on the last frame, the HUD's link line says LINK -- (accent)
//   after LIVE_LOST_MS: "ARM LINK LOST", the wrist and fingers glide to the neutral pose over ~1 s, the lights the arm
//   reported go dark (they are unknown now), the plates stay where they were (they are physical and did not move
//   by themselves). Never the DEMO animation once an arm was seen this power-up - fake motion on stage would lie.
//   The next good $G: "ARM LINKED", everything glides to the arm's state (no snap).
constexpr uint32_t LIVE_STALE_MS = 300;
constexpr uint32_t LIVE_LOST_MS = 3000;
constexpr uint32_t MISMATCH_SHOW_MS = 3000;   // a refused version within this long and no good frame since -> PROTOCOL MISMATCH
constexpr float LIVE_TAU_S = 0.040f;          // the model eases toward each $G frame with this time constant (20 Hz -> 15 fps)
constexpr float LOST_TAU_S = 0.30f;           // ... and toward the neutral pose after the link is lost

// palm ring palette, index = the $S ringi field (glink::PALETTE_NAMES order, the arm's palm.cpp)
static const uint8_t kRingRGB[9][3] = {{255, 255, 255}, {150, 205, 255}, {255, 0, 0}, {255, 70, 0}, {255, 170, 0},
                                       {0, 255, 0}, {0, 210, 255}, {0, 30, 255}, {255, 0, 170}};
static const char* const kRingName[9] = {"WHITE", "REPULSOR", "RED", "ORANGE", "AMBER", "GREEN", "CYAN", "BLUE", "MAGENTA"};
constexpr float RING_CAP = 77.0f;             // the arm's 30 % cap: a $G "ring" channel of 77 = level 10

// ---------------------------------------------------------------- plate cascade replayed from $S (no $G)
// The puck's model has one body per plate; Ironman has six servos. Times in ms from the cascade's start, each move
// eased with smoothstep like the firmware. Plate 3 has two servos (A, B) that move at different times: the model's
// plate 3 follows the mean of the two. Model four-bars: 0 plate_1, 1 plate_3, 2 plate_2, 3 hand_plate.
struct Move { uint16_t t0, t1; };
//                                    plate_1      plate_3 B    plate_3 A    plate_2      hand
static const Move kOpenMoves[5]  = {{0, 380},   {340, 660},  {600, 900},  {340, 640},  {0, 320}};
static const Move kCloseMoves[5] = {{520, 900}, {0, 320},    {260, 560},  {0, 300},    {520, 840}};
constexpr uint16_t CASCADE_MS = 900;

inline float ease01(float x) { x = x < 0 ? 0 : (x > 1 ? 1 : x); return x * x * (3 - 2 * x); }
inline float move_frac(const Move& m, float t) { return ease01((t - m.t0) / (float)(m.t1 - m.t0)); }

struct Cascade {
  float frac[model::NFOURBAR] = {0, 0, 0, 0};   // 0 closed .. 1 open, per model four-bar
  float from[model::NFOURBAR] = {0, 0, 0, 0};
  int8_t dir = 0;                               // +1 opening, -1 closing, 0 still
  float t = 0;                                  // ms since the cascade started
  int8_t p1dir = 0; float p1t = 0, p1from = 0;  // plate 1 on its own (round 4)
  static constexpr float P1_MS = 380;
  void start(int8_t d) { memcpy(from, frac, sizeof frac); dir = d; t = 0; p1dir = 0; }
  void snap(float v) { for (int j = 0; j < model::NFOURBAR; j++) frac[j] = v; dir = 0; p1dir = 0; }
  void solo(int8_t d) { if (dir || p1dir == d) return; p1from = frac[0]; p1dir = d; p1t = 0; }
  void step(float dtMs) {
    if (!dir && p1dir) {
      p1t += dtMs;
      frac[0] = p1from + ((p1dir > 0 ? 1.0f : 0.0f) - p1from) * ease01(p1t / P1_MS);
      if (p1t >= P1_MS) p1dir = 0;
    }
    if (!dir) return;
    t += dtMs;
    const Move* m = dir > 0 ? kOpenMoves : kCloseMoves;
    float k[model::NFOURBAR] = {move_frac(m[0], t), 0.5f * (move_frac(m[1], t) + move_frac(m[2], t)), move_frac(m[3], t), move_frac(m[4], t)};
    float target = dir > 0 ? 1.0f : 0.0f;
    for (int j = 0; j < model::NFOURBAR; j++) frac[j] = from[j] + (target - from[j]) * k[j];
    if (t >= CASCADE_MS) dir = 0;
  }
};

// ---------------------------------------------------------------- what the lights show this frame
struct Lights {
  uint8_t ring[3] = {0, 0, 0}; float ringK = 0;   // palm ring colour (full scale) and level 0..1 (0 = off)
  int8_t ringIdx = -1;                            // the palette colour that is SET (HUD name), -1 = unknown
  uint8_t pucks = 0;                              // the LED drivers' groups on: bit k = group k + 1 ($G groups, $S pucks)
  bool red = false, green = false;
  uint16_t lidar = 0;                             // mm, 0 = none
  bool strip = false; uint8_t stripRGB[3] = {0, 0, 0}; float stripK = 0;   // forearm strip
  float stripFrom = 0, stripTo = 1;               // the lit part of the strip, 0 = the elbow end .. 1
  bool accent = false;                            // the three red accent LEDs
};

// the LED groups light the LEDs at their real openings (lights_data.h kLed, from the CAD): group 2 = the two square
// LEDs at the heel of the palm, group 3 = the index and middle fingertips. Group 1 (the three 20 mm palm LEDs) is not in
// the CAD - nothing is drawn for it until Toby says where they sit.

// test overrides for the USB test commands: -1 = not set
struct TestOverrides {
  int8_t fist = -1;                               // ARM view ($S only): 1 = make a fist, 0 = open hand
  int8_t stripColour = -1; int8_t stripLevel = 0; // forearm strip: palette index + level 0..10
  int8_t accent = -1;
  int8_t plate1 = -1;                             // ARM view: plate 1 alone, 1 = open and wait, 0 = closed, -1 = follow
  int8_t mlxsim = -1;
  int8_t lines = -1;   // "lines N": how the edge lines are drawn (view.h line_mode)
  int8_t creases = -1; // "creases N": where the inner creases are left out (view.h crease_level)
  int8_t pinid = -1;                              // 1 = reboot into the header pin-identification mode (app.cpp)
  int8_t hud = -1;                                // "hud N": the HUD direction 1..3, 4 = none (puck_core.h)
  int8_t thermal = -1;                            // "thermal 1|0|auto": force the thermal page on / off, or follow the arm ($S flags bit 6)
  int8_t uartOff = -1;                            // "uart 0": ignore the arm's UART (bench tests with the scripted arm alone); "uart 1" back
  int8_t update = -1;                             // "update": restart into the updater (update mode), as a power-up hold does
  int8_t page = -1;                               // "page 0|1": the main page / the control panel (panel.h)
  int8_t press = -1;                              // "press N|NAME": a control-panel button, as a tap would (panel.h kPanelBtn)
  int8_t sweep = -1;                              // "sweep": the orbit fps sweep (app.cpp)                             // the made-up thermal-camera load on core 0 (app.cpp show_bus.h): 1 on, 0 off
};

// the scripted arm (USB "garm 1"): $G frames built here at 20 Hz and fed through the USB parser, so LIVE can be
// measured with no arm attached; the arm's own fake build (demo.cpp) is the real thing on the UART
struct FakeArm { bool on = false; uint32_t t0 = 0, next = 0; uint16_t seq = 0, evSeq = 0; uint8_t load = 1; float freeze = -1; };

struct ArmLink {
  glink::Parser uart, usb;
  // $S
  glink::Status st;
  bool have = false;            // a valid $S since boot
  uint32_t lastMs = 0;          // when the last valid $S arrived
  uint32_t frames = 0;          // valid $S frames (both streams)
  uint8_t lastPlates = 0;
  Cascade cas;
  bool p1Solo = false;
  // $G
  glink::Live live;
  bool haveLive = false;        // a valid $G since boot
  uint32_t liveMs = 0;          // when the last one arrived (puck ms)
  uint32_t liveFrames = 0, liveOld = 0;   // valid $G; and ones older than the last (by the arm's clock), skipped
  uint32_t liveGapMax = 0;      // the longest gap between two $G this second (ms), for the debug view
  uint32_t liveGapWork = 0;
  bool gnSeen = false; uint16_t lastGn = 0;
  uint8_t pendingGev = 0;       // an event the arm reported, for app.cpp's toast (0 = none)
  // $E
  glink::Event ev;              // the newest gesture / action event
  bool evSeen = false;          // an $E since boot: from then on the HUD's words come from $E, not $G's gev
  bool evNew = false;           // a new one, for puck_core.h
  glink::Event evq[4]; uint8_t evqN = 0;   // every $E since the last frame (the control panel's two-line presses get two at once)
  uint32_t evFrames = 0, evLost = 0;   // $E lines; and gaps in their seq (lines lost)
  // other versions
  uint32_t badVersion = 0, badFields = 0;  // frames refused by their version / their fields
  long theirVersion = 0;        // the version a refused frame carried (1 = a v1 line)
  uint32_t mismatchMs = 0;      // when the last refused version arrived
  // health
  TestOverrides test;
  bool debug = false;           // the link-health view (long press, or "debug")
  uint32_t rateT0 = 0, good0 = 0, bad0 = 0, live0 = 0;
  float goodRate = 0, badRate = 0, liveRate = 0;   // valid / dropped frames per second, $G per second
  FakeArm fake;
  char line[300]; int lineN = 0;   // the USB line being typed
  char reply[400] = "";            // what the last test command did (app.cpp prints it)

  bool up(uint32_t ms) const { return have && ms - lastMs <= ARM_LINK_LOST_MS; }
  bool liveFresh(uint32_t ms) const { return haveLive && ms - liveMs <= LIVE_STALE_MS; }
  bool liveUp(uint32_t ms) const { return haveLive && ms - liveMs <= LIVE_LOST_MS; }
  bool anyUp(uint32_t ms) const { return up(ms) || liveUp(ms); }
  uint32_t good() const { return uart.good + usb.good; }
  uint32_t bad() const { return uart.dropped() + usb.dropped() + badFields; }
  // a refused version recently and no good frame of ours since
  bool mismatch(uint32_t ms) const {
    if (!mismatchMs || ms - mismatchMs > MISMATCH_SHOW_MS) return false;
    uint32_t lastGood = lastMs > liveMs ? lastMs : liveMs;
    return !(have || haveLive) || (int32_t)(mismatchMs - lastGood) > 0;
  }
};

inline void note_mismatch(ArmLink& L, long their, uint32_t ms) { L.badVersion++; L.theirVersion = their; L.mismatchMs = ms ? ms : 1; }

// A complete good frame (by its CRC) came out of `p`.
inline void arm_frame(ArmLink& L, glink::Parser& p, uint32_t ms) {
  const char* f[glink::MAX_FIELDS];
  char type = p.type();
  uint8_t n = p.fields(f, glink::MAX_FIELDS);
  if (type == 'G') {
    glink::Live g;
    glink::Result r = glink::liveParse(f, n, g);
    if (r == glink::Result::BadVersion) { long v = 0; glink::toLong(f[1], 0, 255, v); note_mismatch(L, v, ms); return; }
    if (r != glink::Result::Ok) { L.badFields++; return; }
    // an older frame by the arm's clock (a reordered or replayed line): skipped; a big jump back = the arm rebooted
    if (L.haveLive && g.t < L.live.t && L.live.t - g.t < 10000) { L.liveOld++; return; }
    if (L.haveLive) { uint32_t gap = ms - L.liveMs; if (gap > L.liveGapWork) L.liveGapWork = gap; }
    if (L.gnSeen && g.gn != L.lastGn && g.gev) L.pendingGev = g.gev;
    L.lastGn = g.gn; L.gnSeen = true;
    L.live = g; L.haveLive = true; L.liveMs = ms; L.liveFrames++;
    return;
  }
  if (type == 'S') {
    glink::Status s;
    glink::Result r = glink::statusParse(f, n, s);
    if (r == glink::Result::BadVersion) { long v = 1; if (n == glink::S_FIELDS) glink::toLong(f[17], 0, 255, v); note_mismatch(L, v, ms); return; }
    if (r != glink::Result::Ok) { L.badFields++; return; }
    L.st = s; L.have = true; L.lastMs = ms; L.frames++;
    // the cascade (ARM view): start it on the frame that says opening / closing; a settled state the puck did not
    // see move (boot, or frames lost) snaps; unknown / mixed holds where the model is
    int8_t want = s.plates == 3 || s.plates == 1 ? 1 : (s.plates == 4 || s.plates == 2 ? -1 : 0);
    if (s.plates != L.lastPlates) {
      if (s.plates == 3) L.cas.start(+1);
      else if (s.plates == 4) L.cas.start(-1);
      else if ((s.plates == 1 || s.plates == 2) && L.cas.dir != want) L.cas.snap(s.plates == 1 ? 1.0f : 0.0f);
      L.lastPlates = s.plates;
    }
    return;
  }
  if (type == 'E') {
    glink::Event ev;
    glink::Result r = glink::eventParse(f, n, ev);
    if (r == glink::Result::BadVersion) { long v = 0; glink::toLong(f[n - 1], 0, 255, v); note_mismatch(L, v, ms); return; }
    if (r != glink::Result::Ok) { L.badFields++; return; }
    if (L.evSeen && ev.seq == L.ev.seq) return;   // the same line again
    if (L.evSeen && (uint16_t)(ev.seq - L.ev.seq) > 1) L.evLost += (uint16_t)(ev.seq - L.ev.seq) - 1;
    L.ev = ev; L.evSeen = true; L.evNew = true; L.evFrames++;
    if (L.evqN < 4) L.evq[L.evqN++] = ev;
    return;
  }
  // $H / $T / $L / $A are the puck's own direction, and any other type is not ours: ignored
}

// one byte through a parser; a whole protocol-v1 line (two-digit CRC8: the parser counts it, never returns it) is
// a version mismatch too, noted the moment it ends
inline void arm_feed(ArmLink& L, glink::Parser& p, char ch, uint32_t ms) {
  uint32_t v1 = p.v1;
  if (p.feed(ch)) arm_frame(L, p, ms);
  else if (p.v1 != v1) note_mismatch(L, 1, ms);
}
inline void arm_uart_byte(ArmLink& L, char ch, uint32_t ms) { arm_feed(L, L.uart, ch, ms); }

inline void feed_usb_line(ArmLink& L, const char* fr, uint32_t ms) {
  for (const char* c = fr; *c; c++) arm_feed(L, L.usb, *c, ms);
}

// "s plates=3 ring=6,8 pucks=7 laser=1 lidar=1200 mv=7800 mode=1 bstate=0": edit the last status, frame it with its
// CRC and send it through the USB parser like a typed line
inline void arm_inject(ArmLink& L, const char* args, uint32_t ms) {
  glink::Status s = L.st;
  s.seq = (uint16_t)((s.seq + 1) % 10000);
  char tmp[200]; strncpy(tmp, args, sizeof tmp - 1); tmp[sizeof tmp - 1] = 0;
  for (char* tok = strtok(tmp, " "); tok; tok = strtok(nullptr, " ")) {
    char* eq = strchr(tok, '=');
    if (!eq) continue;
    *eq = 0; const char* v = eq + 1;
    if (!strcmp(tok, "mode")) s.mode = atoi(v);
    else if (!strcmp(tok, "mv")) s.mv = atoi(v);
    else if (!strcmp(tok, "bstate")) s.bstate = atoi(v);
    else if (!strcmp(tok, "plates")) s.plates = atoi(v);
    else if (!strcmp(tok, "ring")) { s.ringi = atoi(v); const char* c = strchr(v, ','); s.ringlvl = c ? atoi(c + 1) : 10; }
    else if (!strcmp(tok, "ringlvl")) s.ringlvl = atoi(v);
    else if (!strcmp(tok, "pucks")) s.pucks = (uint8_t)strtoul(v, nullptr, 0);
    else if (!strcmp(tok, "laser")) s.laser = atoi(v);
    else if (!strcmp(tok, "lidar")) s.lidar = atoi(v);
    else if (!strcmp(tok, "flags")) s.flags = atoi(v);
  }
  char body[180], fr[200];
  if (!glink::statusBody(body, sizeof body, s) || !glink::frame(fr, sizeof fr, body)) { snprintf(L.reply, sizeof L.reply, "s: frame too long"); return; }
  feed_usb_line(L, fr, ms);
  snprintf(L.reply, sizeof L.reply, "fed %.*s", (int)(strlen(fr) - 1), fr);
}

inline uint32_t rgb24(const uint8_t c[3], float k) {
  return ((uint32_t)(c[0] * k + 0.5f) << 16) | ((uint32_t)(c[1] * k + 0.5f) << 8) | (uint32_t)(c[2] * k + 0.5f);
}
inline void quat_axis(float deg, float x, float y, float z, float q[4]) {
  float h = deg * 0.0087266463f, s = sinf(h);
  q[0] = cosf(h); q[1] = x * s; q[2] = y * s; q[3] = z * s;
}
inline void quat_mul(const float a[4], const float b[4], float r[4]) {
  r[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
  r[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
  r[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
  r[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

// The scripted arm (round 10), a 40 s loop that runs every action the puck's views follow, in the arm's own order:
//    6.0  the hand plate lifts, 6.5-10.0 the red laser (and the LiDAR), 10.2 the plate closes          -> hand view
//   11.5-14.5 a fist, 12-14 all three LED groups (a palm burst)
//   14.0  plate 1 opens, 14.5-18.0 the green laser, 18.2 plate 1 closes                              -> whole arm
//   20.0  the cascade opens, 22-25 both lasers and every LED group, 26.0 the cascade closes           -> whole arm
//   28.0  the hand plate lifts, 28.5-33.0 the thermal camera ($S flags bit 6), 33.2 the plate closes  -> thermal page
//   34-35 the switch on SAFE: a red laser gesture BLOCKED at 34.5; 35.5 an arm REFUSED; 36-40 the drone armed, a green
//         laser gesture at 38.0 BLOCKED because it is flying (mode = need = FULL)
// with the forearm turning slowly, the wrist moving and the fingers curling all the time. $E lines at those moments,
// $S at 10 Hz. "garm freeze 23" = every plate open, both lasers, every light; "garm freeze 13" = a fist, the LED groups.
constexpr float FAKE_LOOP_S = 40.0f;
struct FakeEv { float t; uint8_t act, res; };
static const FakeEv kFakeEvents[] = {{6.5f, glink::ACT_LASER_RED, 0}, {12.0f, glink::ACT_BURST, 0}, {14.5f, glink::ACT_LASER_GREEN, 0},
                                     {20.0f, glink::ACT_CASCADE, 0}, {26.0f, glink::ACT_CASCADE, 0}, {28.0f, glink::ACT_THERMAL_ON, 0},
                                     {33.0f, glink::ACT_THERMAL_OFF, 0}, {34.5f, glink::ACT_LASER_RED, 1}, {35.5f, glink::ACT_ARM, 2},
                                     {36.0f, glink::ACT_ARM, 0}, {38.0f, glink::ACT_LASER_GREEN, 1}, {39.9f, glink::ACT_DISARM, 0}};
inline float fake_mode(float c) { return c >= 34.0f && c < 35.0f ? 0 : 2; }
inline void fake_live(float t, uint8_t load, glink::Live& g) {
  float c = fmodf(t, FAKE_LOOP_S);
  float fa[4], hr[4], a[4], b[4], hq[4];
  quat_axis(25.0f * sinf(0.35f * t), 0, 0, 1, a);                       // the forearm: a slow turn about up ...
  quat_axis(15.0f * sinf(0.23f * t + 1.0f), 0, 1, 0, b);               // ... and a nod
  quat_mul(a, b, fa);
  quat_axis(35.0f * sinf(1.1f * t), 0, 1, 0, a);                        // the wrist: flexion (about Y, left) ...
  quat_axis(20.0f * sinf(0.7f * t + 0.5f), 0, 0, 1, b);                // ... side to side (about Z, up)
  float w[4]; quat_mul(a, b, w);
  quat_axis(30.0f * sinf(0.5f * t + 2.0f), 1, 0, 0, a);                // ... and twist (about X, forward)
  quat_mul(w, a, hr);
  quat_mul(fa, hr, hq);                                                 // hand = forearm * wrist
  for (int i = 0; i < 4; i++) { g.fq[i] = glink::q10k(fa[i]); g.hq[i] = glink::q10k(hq[i]); }
  g.imu = 7;
  bool fist = c > 11.5f && c < 14.5f;
  for (int f = 0; f < 5; f++) {
    float k = fist ? 0.92f : 0.45f - 0.40f * cosf(2.2f * t - 0.6f * f);
    g.curl[f] = (int16_t)(k * 1000);
    g.fmv[f] = (uint16_t)(1200 - 500 * k);
  }
  auto ramp = [&](float t0) { float x = (c - t0) / 0.4f; return x < 0 ? 0.0f : (x > 1 ? 1.0f : x * x * (3 - 2 * x)); };
  auto win = [&](float a0, float a1) { return c >= a0 && c < a1; };
  auto span = [&](float o, float s) { return ramp(o) * (1 - ramp(s)); };
  float pos[6] = {0, 0, 0, 0, 0, 0};
  // the arm's cascade order (2026-10-07 night): plate 1 and the hand plate, plate 2 once the hand plate has landed, plate 3's
  // pair (one mirrored step) once plate 2 has landed; closing in reverse
  const float cOpen[6] = {20.0f, 20.35f, 20.7f, 20.7f, 20.0f, 20.0f}, cShut[6] = {26.7f, 26.35f, 26.0f, 26.0f, 26.7f, 26.7f};
  for (int i = 0; i < 6; i++) pos[i] = span(cOpen[i], cShut[i]);
  pos[0] = fmaxf(pos[0], span(14.0f, 18.2f));                          // plate 1 alone, for the green laser
  float hp = fmaxf(span(6.0f, 10.2f), span(28.0f, 33.2f));               // the hand plate, for the red laser and the camera
  pos[4] = fmaxf(pos[4], hp); pos[5] = fmaxf(pos[5], hp);
  g.drv = 0;
  bool anyOpen = false, allOpen = true, moving = false;
  for (int i = 0; i < glink::SERVOS; i++) {
    g.servo[i] = (int16_t)(pos[i] * 1000);
    bool mv = pos[i] > 0.001f && pos[i] < 0.999f;
    if (mv) { g.drv |= 1 << i; moving = true; }
    anyOpen |= pos[i] > 0.5f; allOpen &= pos[i] > 0.5f;
  }
  g.plates = moving ? (win(20, 21.2f) ? 3 : (win(26, 27) ? 4 : 5)) : (allOpen ? 1 : (anyOpen ? 5 : 2));
  g.ringset = rgb24(kRingRGB[1], 1); g.ringlvl = 10; g.ringmode = 0; g.ring = rgb24(kRingRGB[1], RING_CAP / 255.0f);
  g.groups = load && (win(12, 14) || win(22, 25)) ? 7 : 0; g.accent = 0;
  bool cascade = win(20, 21.2f) || win(26, 27.2f);
  // the strip as the arm drives it: its idle look (FOREARM_IDLE_*: 3 pixels, level 40, red) and the whole strip during a cascade
  g.strip = 0xFF0000; g.striplvl = load && cascade ? 60 : 40; g.striplit = load && cascade ? 100 : 3; g.strippos = 0;

  g.lasers = (win(6.5f, 10) || win(22, 25) ? 1 : 0) | (win(14.5f, 18) || win(22, 25) ? 2 : 0);
  if (!load) g.lasers &= 1;
  g.lidar = g.lasers & 1 ? (uint16_t)(1460 + 300 * sinf(t)) : 0;
  g.mode = (uint8_t)fake_mode(c); g.hand = fist ? 2 : 1;
  g.mv = 7800; g.bstate = 0; g.ma = moving ? 900 : 40;
  g.outs = (cascade ? 4 : 0) | (win(36, 40) ? 16 : 0);
}
inline void fake_status(float t, glink::Status& s) {
  float c = fmodf(t, FAKE_LOOP_S);
  glink::Live g; fake_live(t, 1, g);
  s.mode = g.mode; s.mv = g.mv; s.bstate = 0; s.plates = g.plates; s.ringi = 1; s.ringlvl = 10;
  s.armed = c >= 36 && c < 40; s.lq = 100; s.cv100 = 395; strcpy(s.fcmode, s.armed ? "ANGL" : "ANGL*");
  s.lidar = g.lidar; s.laser = g.lasers; s.pucks = g.groups; s.flags = (c >= 28.5f && c < 33.0f) ? (uint8_t)(1 << glink::FLAG_THERMAL) : 0;
  s.page = 0;
}

// every $G that is due (the puck renders at ~15 fps, the arm sends at 20 Hz: some frames bring two, like the UART),
// $S every other one, and the $E lines whose moment passed
inline void fake_tick(ArmLink& L, uint32_t ms) {
  if (!L.fake.on) return;
  if ((int32_t)(ms - L.fake.next) > 200) L.fake.next = ms;   // stalled: no burst of catch-up frames
  while ((int32_t)(ms - L.fake.next) >= 0) {
    uint32_t at = L.fake.next;
    L.fake.next += glink::G_PERIOD_MS;
    float t = L.fake.freeze >= 0 ? L.fake.freeze : (at - L.fake.t0) / 1000.0f;
    glink::Live g;
    fake_live(t, L.fake.load, g);
    g.seq = L.fake.seq++; g.t = at;
    char body[glink::MAX_LINE], fr[glink::MAX_LINE];
    if (glink::liveBody(body, sizeof body, g) && glink::frame(fr, sizeof fr, body)) feed_usb_line(L, fr, ms);
    if (g.seq & 1) {
      glink::Status st; fake_status(t, st); st.seq = (uint16_t)((g.seq / 2) % 10000);
      if (glink::statusBody(body, sizeof body, st) && glink::frame(fr, sizeof fr, body)) feed_usb_line(L, fr, ms);
    }
    if (L.fake.freeze < 0) {
      float c1 = fmodf(t, FAKE_LOOP_S), c0 = c1 - glink::G_PERIOD_MS / 1000.0f;
      for (const FakeEv& e : kFakeEvents)
        if (e.t > c0 && e.t <= c1) {
          glink::Event ev; ev.seq = L.fake.evSeq++; ev.act = e.act; ev.res = e.res; ev.mode = (uint8_t)fake_mode(e.t); ev.need = e.act == glink::ACT_LASER_RED || e.act == glink::ACT_LASER_GREEN || e.act == glink::ACT_ARM ? 2 : 1;
          if (glink::eventBody(body, sizeof body, ev) && glink::frame(fr, sizeof fr, body)) feed_usb_line(L, fr, ms);
        }
    }
  }
}

inline void test_command(ArmLink& L, char* cmd, uint32_t ms) {
  while (*cmd == ' ') cmd++;
  char* arg = strchr(cmd, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; } else arg = cmd + strlen(cmd);
  int a = atoi(arg);
  if (!strcmp(cmd, "s")) { arm_inject(L, arg, ms); return; }
  if (!strcmp(cmd, "garm") && !strncmp(arg, "freeze", 6)) {
    L.fake.freeze = arg[6] ? (float)atof(arg + 7) : -1;
    snprintf(L.reply, sizeof L.reply, "scripted arm %s", L.fake.freeze >= 0 ? "frozen at that moment of its loop" : "running");
    return;
  }
  if (!strcmp(cmd, "garm")) {
    L.fake.on = a != 0 || !strcmp(arg, "light");
    L.fake.load = strcmp(arg, "light") ? 1 : 0;
    L.fake.t0 = ms; L.fake.next = ms;
    snprintf(L.reply, sizeof L.reply, "scripted arm %s (%s): $G 20 Hz, $S 10 Hz, $E through the USB parser - a 40 s loop: red laser 6, green 14, cascade 20, thermal 28, SAFE 34, armed 36",
             L.fake.on ? "ON" : "off", L.fake.load ? "every light on" : "ring only");
    return;
  }
  if (!strcmp(cmd, "fist")) { L.test.fist = !strcmp(arg, "auto") ? -1 : (int8_t)(a != 0); snprintf(L.reply, sizeof L.reply, "fist %s (ARM view, $S only)", L.test.fist < 0 ? "auto (red laser on = fist)" : (L.test.fist ? "on" : "off")); return; }
  if (!strcmp(cmd, "strip")) {
    if (!*arg || !strcmp(arg, "off")) { L.test.stripColour = -1; snprintf(L.reply, sizeof L.reply, "forearm strip test off"); return; }
    L.test.stripColour = (int8_t)(a < 0 ? 0 : (a > 8 ? 8 : a));
    const char* c = strchr(arg, ' '); L.test.stripLevel = (int8_t)(c ? atoi(c + 1) : 10);
    snprintf(L.reply, sizeof L.reply, "forearm strip %s %d/10 (test override)", kRingName[L.test.stripColour], L.test.stripLevel);
    return;
  }
  if (!strcmp(cmd, "plate1")) {
    if (!L.have) arm_inject(L, "mode=1 mv=7800 bstate=0 plates=2", ms);
    L.test.plate1 = !strcmp(arg, "auto") ? -1 : (int8_t)(a != 0);
    snprintf(L.reply, sizeof L.reply, "plate 1 %s (ARM view, $S only)", L.test.plate1 < 0 ? "follows the cascade" : (L.test.plate1 ? "opens alone and stays open" : "closes"));
    return;
  }
  if (!strcmp(cmd, "creases")) { L.test.creases = (int8_t)(a < 0 ? 0 : (a > 5 ? 5 : a)); snprintf(L.reply, sizeof L.reply, "creases level %d (0 everywhere, 1 not on the fingers, 2 + hand plate, 3 + hand, 4 + plates, 5 outlines only)", (int)L.test.creases); return; }
  if (!strcmp(cmd, "lines")) { L.test.lines = (int8_t)(a < 0 ? 0 : (a > 5 ? 5 : a)); snprintf(L.reply, sizeof L.reply, "edge lines mode %d (0 glow on all, 1 glow on outlines, 2 no glow, 3 outlines only, 4 none)", (int)L.test.lines); return; }
  if (!strcmp(cmd, "mlxsim")) { L.test.mlxsim = (int8_t)(a != 0); snprintf(L.reply, sizeof L.reply, "thermal-camera load simulation %s (core 0; only while no MLX90640 answers)", a ? "ON" : "off"); return; }
  if (!strcmp(cmd, "hud")) { L.test.hud = (int8_t)(a < 1 || a > 4 ? 4 : a); snprintf(L.reply, sizeof L.reply, "HUD direction %d (1 BEZEL, 2 RETICLE, 3 ORBIT, 4 none)", L.test.hud); return; }
  if (!strcmp(cmd, "thermal")) {
    L.test.thermal = (int8_t)(!strcmp(arg, "auto") ? -1 : (a != 0));
    snprintf(L.reply, sizeof L.reply, "thermal page %s", L.test.thermal < 0 ? "follows the arm ($S flags bit 6)" : (L.test.thermal ? "forced on" : "forced off"));
    return;
  }
  if (!strcmp(cmd, "e")) {   // "e ACT RES": an $E line through the parser (act: glink::ACT_*, res 0 DONE 1 BLOCKED 2 REFUSED)
    glink::Event ev; ev.seq = (uint16_t)(L.evSeen ? L.ev.seq + 1 : 1); ev.act = (uint8_t)a;
    const char* r = strchr(arg, ' '); ev.res = (uint8_t)(r ? atoi(r + 1) : 0); ev.mode = L.liveUp(ms) ? L.live.mode : L.st.mode; ev.need = 1;
    char body[64], fr[80];
    if (glink::eventBody(body, sizeof body, ev) && glink::frame(fr, sizeof fr, body)) { feed_usb_line(L, fr, ms); snprintf(L.reply, sizeof L.reply, "fed %.*s (%s)", (int)(strlen(fr) - 1), fr, glink::actName(ev.act)); }
    return;
  }
  if (!strcmp(cmd, "uart")) { L.test.uartOff = (int8_t)(a == 0); snprintf(L.reply, sizeof L.reply, "arm UART %s", a == 0 ? "IGNORED (bench test) - uart 1 to listen again" : "listened to"); return; }
  if (!strcmp(cmd, "update")) { L.test.update = 1; snprintf(L.reply, sizeof L.reply, "update mode: restarting into the updater (WiFi, over-the-air update; a tap goes back)"); return; }
  if (!strcmp(cmd, "page") || !strcmp(cmd, "panel")) { L.test.page = (int8_t)(!strcmp(cmd, "panel") ? (*arg ? a != 0 : 1) : (a != 0)); snprintf(L.reply, sizeof L.reply, "page: %s", L.test.page ? "the control panel" : "the main page"); return; }
  if (!strcmp(cmd, "press")) {
    int b = *arg >= '0' && *arg <= '9' ? a : -1;
    for (int k = 0; b < 0 && k < PB_COUNT; k++) if (!strcasecmp(arg, kPanelBtn[k].label)) b = k;
    if (b < 0 || b >= PB_COUNT) { int n = snprintf(L.reply, sizeof L.reply, "press N|NAME:"); for (int k = 0; k < PB_COUNT && n < (int)sizeof L.reply - 16; k++) n += snprintf(L.reply + n, sizeof L.reply - n, " %d %s,", k, kPanelBtn[k].label); return; }
    L.test.press = (int8_t)b; snprintf(L.reply, sizeof L.reply, "press %s (as a tap on the control panel)", kPanelBtn[b].label); return;
  }
  if (!strcmp(cmd, "sweep")) { L.test.sweep = 1; snprintf(L.reply, sizeof L.reply, "orbit fps sweep: 62 directions at each zoom, ~1.3 s each"); return; }
  if (!strcmp(cmd, "pinid")) { L.test.pinid = 1; snprintf(L.reply, sizeof L.reply, "rebooting into the header pin-identification mode (meter each wire; unplug or \"exit\" to leave)"); return; }
  if (!strcmp(cmd, "debug")) { L.debug = !L.debug; snprintf(L.reply, sizeof L.reply, "link debug view %s", L.debug ? "on" : "off"); return; }
  if (!strcmp(cmd, "accent")) { L.test.accent = (int8_t)(a != 0); snprintf(L.reply, sizeof L.reply, "accent LEDs %s (test override)", a ? "on" : "off"); return; }
  if (!strcmp(cmd, "demo")) { arm_inject(L, "mode=1 mv=7800 bstate=0 plates=1 ring=1,10 pucks=127 laser=3 lidar=1500", ms); return; }
  snprintf(L.reply, sizeof L.reply,
           "cmds: $G/$S/$E lines | garm 1|light|0|freeze T | hud 1-4 | thermal 1|0|auto | e ACT RES | page 0|1 | press N|NAME | sweep | update | uart 0|1 | mlxsim 1|0 | pinid | s plates= ring=I,L pucks= laser= lidar= mv= mode= bstate= | plate1 1|0|auto | fist 1|0|auto | strip I L|off | accent 1|0 | debug | demo");
}

// a USB byte: '$' lines go to the parser, other lines are test commands
inline void arm_usb_byte(ArmLink& L, char ch, uint32_t ms) {
  const bool eol = ch == '\n' || ch == '\r';
  if (L.lineN == 0 && eol) return;
  if ((L.lineN == 0 && ch == '$') || (L.lineN > 0 && L.line[0] == '$')) {
    if (L.lineN < (int)sizeof L.line - 1) L.line[L.lineN++] = ch;
    arm_feed(L, L.usb, ch, ms);
    if (eol) L.lineN = 0;
    return;
  }
  if (eol) { L.line[L.lineN] = 0; test_command(L, L.line, ms); L.lineN = 0; return; }
  if (L.lineN < (int)sizeof L.line - 1) L.line[L.lineN++] = ch;
}

// The puck's heartbeat ($H, 1 Hz, to the arm): page, show-bus devices present, the LCD's fps, free heap, our version.
inline size_t arm_heartbeat(char* out, size_t cap, uint16_t seq, uint32_t ms, uint8_t page, bool mlx, bool oled, float fps, uint32_t heap) {
  glink::Heartbeat h; h.seq = seq; h.uptimeS = ms / 1000; h.page = page; h.mlx = mlx; h.oled = oled;
  h.fps = (uint8_t)(fps < 0 ? 0 : (fps > 255 ? 255 : fps + 0.5f)); h.heap = heap;
  char body[96];
  return glink::heartbeatBody(body, sizeof body, h) ? glink::frame(out, cap, body) : 0;
}

// ---------------------------------------------------------------- the frames -> the lights
// battery 0..1 from the glove pack: a straight line from the arm's glink::BATT_EMPTY_MV (0 %) to glink::BATT_FULL_MV
// (100 %), clamped (2026-10-21, Toby: "make 8.4 actually 100 %" - his charger stops at 4.2 V a cell; it was 6.6-8.7 V)
inline float battery_frac(uint16_t mv) {
  float f = ((float)mv - glink::BATT_EMPTY_MV) / (float)(glink::BATT_FULL_MV - glink::BATT_EMPTY_MV);
  return f < 0 ? 0 : (f > 1 ? 1 : f);
}

// $S (ARM view): the palm ring follows its colour and level; the glove's battery warnings blink it as palm.cpp does
inline void lights_from_status(const glink::Status& s, uint32_t ms, Lights& out) {
  out = Lights();
  if (s.bstate == 4) {
    if ((ms / 300) & 1) { memcpy(out.ring, kRingRGB[2], 3); out.ringK = 0.6f; out.ringIdx = 2; }
  } else if (s.bstate == 2 || s.bstate == 3) {
    if ((ms / 1000) & 1) { memcpy(out.ring, kRingRGB[0], 3); out.ringK = 0.5f; out.ringIdx = 0; }
  } else if (s.ringlvl > 0) {
    out.ringIdx = s.ringi < 9 ? s.ringi : 0;
    memcpy(out.ring, kRingRGB[out.ringIdx], 3);
    out.ringK = (s.ringlvl > 10 ? 10 : s.ringlvl) / 10.0f;
  }
  out.pucks = s.pucks & 7;
  out.red = s.laser & 1;
  out.green = s.laser & 2;
  out.lidar = s.lidar;
}

// $G (LIVE): the ring as it shows right now (the arm's own blink and hold included), the set colour for its name
inline void lights_from_live(const glink::Live& g, Lights& out) {
  out = Lights();
  uint8_t r = g.ring >> 16 & 255, gg = g.ring >> 8 & 255, b = g.ring & 255;
  int mx = r > gg ? (r > b ? r : b) : (gg > b ? gg : b);
  if (mx > 0) {
    out.ring[0] = (uint8_t)(r * 255 / mx); out.ring[1] = (uint8_t)(gg * 255 / mx); out.ring[2] = (uint8_t)(b * 255 / mx);
    out.ringK = mx / RING_CAP > 1 ? 1.0f : mx / RING_CAP;
  }
  int best = -1, bd = 1 << 30;   // the set colour -> the nearest palette entry (the HUD's name)
  for (int i = 0; i < 9; i++) {
    int dr = (int)(g.ringset >> 16 & 255) - kRingRGB[i][0], dg = (int)(g.ringset >> 8 & 255) - kRingRGB[i][1], db = (int)(g.ringset & 255) - kRingRGB[i][2];
    int d = dr * dr + dg * dg + db * db;
    if (d < bd) { bd = d; best = i; }
  }
  out.ringIdx = (int8_t)best;
  out.pucks = g.groups & 7;
  out.red = g.lasers & 1; out.green = g.lasers & 2;
  out.lidar = g.lidar;
  // the strip exactly as the arm reports it ($G 35-38: colour, brightness, pixels lit, first pixel) - its idle look (a
  // few dim red pixels) included; until 2026-10-10 it showed only while a cascade ran and popped up with every plate
  // move (Toby). The accent LEDs never (round 9: at rest they read as random dots on the model).
  if (g.striplit && g.striplvl && g.strip && g.strippos >= 0) {
    out.strip = true;
    out.stripRGB[0] = g.strip >> 16 & 255; out.stripRGB[1] = g.strip >> 8 & 255; out.stripRGB[2] = g.strip & 255;
    out.stripK = g.striplvl / RING_CAP > 1 ? 1.0f : g.striplvl / RING_CAP;
    out.stripFrom = g.strippos / 100.0f; out.stripTo = (g.strippos + g.striplit) / 100.0f;
  }
  out.accent = false;
}

inline void lights_apply_tests(const TestOverrides& t, Lights& out) {
  if (t.stripColour >= 0 && t.stripLevel > 0) {
    out.strip = true; memcpy(out.stripRGB, kRingRGB[t.stripColour], 3); out.stripK = t.stripLevel / 10.0f; out.stripFrom = 0; out.stripTo = 1;
  }
  if (t.accent == 1) out.accent = true;
}

// ---------------------------------------------------------------- $G -> the model
// The arm's segment frame (X forward along the arm, Y to the left, Z up; the step-1 pose = identity) -> the model's:
// X = a (elbow -> fingers), Y = -l (the CAD's lateral axis points to the pinky; left with the palm down is the
// thumb side of this right hand), Z = d (the back of the hand). S's columns are those three, in model coordinates.
inline void live_wrist_matrix(const float qWrist[4], float R[9]) {
  float Rq[9];
  kin::quat_to_mat(qWrist, Rq);
  float S[9] = {model::kFrameA[0], -model::kFrameL[0], model::kFrameD[0],
                model::kFrameA[1], -model::kFrameL[1], model::kFrameD[1],
                model::kFrameA[2], -model::kFrameL[2], model::kFrameD[2]};
  float St[9], T[9];
  kin::mat3_transpose(S, St);
  kin::mat3_mul(S, Rq, T);
  kin::mat3_mul(T, St, R);
}

// wrist = conj(forearm) * hand, unit; identity when either sensor is not live
inline void live_wrist_quat(const glink::Live& g, float q[4]) {
  q[0] = 1; q[1] = q[2] = q[3] = 0;
  if ((g.imu & 3) != 3) return;
  float f[4] = {g.fq[0] / 1e4f, -g.fq[1] / 1e4f, -g.fq[2] / 1e4f, -g.fq[3] / 1e4f}, h[4] = {g.hq[0] / 1e4f, g.hq[1] / 1e4f, g.hq[2] / 1e4f, g.hq[3] / 1e4f};
  float r[4]; quat_mul(f, h, r);
  float n = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
  if (n < 0.5f) return;
  for (int i = 0; i < 4; i++) q[i] = r[i] / n;
}

// eases q toward target (both unit) by k (0..1), the short way round
inline void quat_ease(float q[4], const float target[4], float k) {
  float d = q[0] * target[0] + q[1] * target[1] + q[2] * target[2] + q[3] * target[3];
  float s = d < 0 ? -1.0f : 1.0f;
  for (int i = 0; i < 4; i++) q[i] += (s * target[i] - q[i]) * k;
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-6f) { q[0] = 1; q[1] = q[2] = q[3] = 0; return; }
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// the six servos -> the model's four plates (model four-bars: 0 plate_1, 1 plate_3, 2 plate_2, 3 hand_plate). Each
// servo drives its own plate; plate 3 and the hand plate are one body each on the model, so they follow the mean of
// their two servos (A and B carry one plate). A servo never driven since the arm booted (-1) takes the plates state.
inline void live_plate_targets(const glink::Live& g, const float now[model::NFOURBAR], float out[model::NFOURBAR]) {
  auto pos = [&](int i, float fallback) -> float {
    if (g.servo[i] >= 0) return g.servo[i] / 1000.0f;
    return g.plates == 1 ? 1.0f : (g.plates == 2 ? 0.0f : fallback);
  };
  auto pair = [&](int a, int b, float fallback) -> float {
    bool ka = g.servo[a] >= 0, kb = g.servo[b] >= 0;
    if (ka && kb) return 0.5f * (g.servo[a] + g.servo[b]) / 1000.0f;
    return ka ? pos(a, fallback) : pos(b, fallback);
  };
  out[0] = pos(0, now[0]);          // plate 1
  out[1] = pair(2, 3, now[1]);      // plate 3 (A, B)
  out[2] = pos(1, now[2]);          // plate 2
  out[3] = pair(4, 5, now[3]);      // the hand plate (A, B)
}

// ---------------------------------------------------------------- light effects in the scene
inline kin::M34 body_view(const rnd::Camera& cam, const kin::M34 world[], int body) { return kin::mul(cam.view, world[body]); }

// The model's lights are the palm repulsor ring and the two lasers, nothing else (Toby, 2026-10-10): the LED drivers'
// palm / finger LEDs, the forearm strip and the accent LEDs are not drawn (their state is still read from the arm).
inline void build_fx(rnd::Fx& fx, const rnd::Camera& cam, const kin::M34 world[], const Lights& li) {
  fx.clear();
  static const uint8_t kRed[3] = {255, 40, 48}, kGreen[3] = {60, 255, 90};
  const kin::M34 Mh = body_view(cam, world, model::BODY_WRIST);
  const kin::M34 Ma = body_view(cam, world, model::BODY_ARM_ROOT);
  if (li.ringK > 0.01f) {
    for (int i = 0; i < lights::NRING; i++) rnd::fx_glow(fx, cam, Mh, lights::kRing[i].p, 3.2f, li.ring, 0.55f + 0.45f * li.ringK);
    rnd::fx_glow(fx, cam, Mh, lights::kRingCentre.p, 26.0f, li.ring, 0.45f * li.ringK);
    rnd::fx_glow(fx, cam, Mh, lights::kRingCentre.p, 75.0f, li.ring, 0.30f * li.ringK, true);
  }
  if (li.red) {   // the red laser from the knuckles along the fingers: it ends where the LiDAR says, else 3 m
    const float* F = lights::kHandForward;
    float len = li.lidar ? li.lidar : 3000.0f;
    const float* o = lights::kRedLaser.p;
    float e[3] = {o[0] + F[0] * len, o[1] + F[1] * len, o[2] + F[2] * len};
    rnd::fx_beam(fx, cam, Mh, o, e, 2, kRed);
    rnd::fx_glow(fx, cam, Mh, o, 5.0f, kRed, 1.0f);   // where it leaves the hand; no glow where it lands (Toby, round 10)
  }
  if (li.green) {   // the green laser from plate 1's edge nearest the wrist, straight ahead past the fist
    static const uint8_t kGreenHalo[3] = {20, 110, 40}, kGreenCore[3] = {150, 255, 170};
    const float* o = lights::kGreenLaser.p; const float* G = lights::kGreenDir;
    float e[3] = {o[0] + G[0] * 3000.0f, o[1] + G[1] * 3000.0f, o[2] + G[2] * 3000.0f};
    rnd::fx_beam(fx, cam, Ma, o, e, 7, kGreenHalo);
    rnd::fx_beam(fx, cam, Ma, o, e, 3, kGreen);
    rnd::fx_beam(fx, cam, Ma, o, e, 0, kGreenCore);
    rnd::fx_glow(fx, cam, Ma, o, 16.0f, kGreen, 1.2f);
    rnd::fx_glow(fx, cam, Ma, o, 5.0f, kGreenCore, 1.6f);
    for (int k = 0; k < 2; k++) {
      const float* Q = lights::kGreenCross[k];
      float a[3] = {o[0] - Q[0] * 14, o[1] - Q[1] * 14, o[2] - Q[2] * 14}, b[3] = {o[0] + Q[0] * 14, o[1] + Q[1] * 14, o[2] + Q[2] * 14};
      rnd::fx_beam(fx, cam, Ma, a, b, 1, kGreen);
    }
  }
}

}  // namespace app
