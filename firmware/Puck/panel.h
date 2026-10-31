// panel.h - the control panel, the puck's second page (2026-10-20, Toby: "the one I have right now is perfect ... I
// scroll to the right on the screen and it brings me to a separate screen, with the same battery and the same one, two
// or three safety guards ... buttons that could shoot laser, open plates, shoot hand lights, palm LED ... a control
// panel with essentially everything").
//   Getting there: on the 3D view (or the thermal page) a quick swipe toward the middle that starts out on the left or right
//   side (PANEL_EDGE_R from the centre); a drag starting nearer the middle, or slower, or up / down, still orbits. Back: any sideways swipe on the panel (except on the
//   ring's colour row, where a swipe steps the colour).
//   The page: the main screen's battery arc and mode symbol (BEZEL, drawn by hud.h), a list of buttons that scrolls with
//   an up / down drag (fading out at the window's top and bottom), page dots at the bottom, a scroll mark on the rim.
//   Round 2 (Toby, the same day): no words at the top ("not sure what they mean"); the mode shows what can be pressed -
//   every button the switch allows is outlined RED, the rest grey (SAFE: only the off buttons and DISARM; SHOW: + the
//   plates, lights, ring, camera, vibrator; FULL: everything); BLUE = on right now, or just pressed and waiting for the
//   arm; a press the arm did not do blinks red. ALL LIGHTS (both palm groups + the fingers), the ring's colours as a row
//   to step through (white, red, orange, yellow, green, blue, purple, round again - either way), THERMAL jumps to the
//   camera's page and a tap there turns it off (back to the panel).
//   A tap = one press = one (ALL LIGHTS: two) $C line(s) to the arm (GauntletLink.h); the arm answers each with an $E.
//   The red outline is the puck's picture of the arm's rules (panel.cpp on the arm: SHOW for the plates, lights, ring,
//   camera and vibrator, FULL for the lasers, turning off never blocked) - the arm still decides every press.
#pragma once
#include <stdint.h>
#include <string.h>
#include "hud.h"
#include "GauntletLink.h"

namespace app {

enum Page : uint8_t { PAGE_MAIN = 0, PAGE_PANEL = 1 };

// Round 3 (2026-10-21, Toby on the arm: "it took like 30 tries to get into the screen ... I can't scroll, I can't click
// anything"): the CST816S reports far less movement than the finger makes (its logged drags: 22-41 px) and little of the
// outer rim, so every distance here is the CHIP's: short, and measured from where it first saw the finger.
constexpr int PANEL_EDGE_R = 68;           // main page: a swipe into the panel starts at least this far from the centre ...
constexpr int PANEL_SWIPE_PX = 24;         // ... on the left / right side, and goes this far toward the middle ...
constexpr uint32_t PANEL_SWIPE_MS = 700;   // ... within this (slower, or the other way, or up / down: it orbits)
constexpr int PANEL_SLOP_PX = 12;          // the panel: moved this far = a drag (scroll, swipe), not a tap
constexpr int PANEL_BACK_PX = 28;          // the panel: a sideways swipe this long goes back
constexpr int PANEL_COLOUR_SWIPE_PX = 22;  // the colour row: a sideways swipe this long steps the colour
constexpr float PANEL_SCROLL_GAIN = 1.6f;  // the list moves this much faster than the chip's reported finger
constexpr int PANEL_TAP_MOVE_PX = 18;      // a quick touch that moved less than this is still a tap
constexpr uint32_t PANEL_TAP_QUICK_MS = 400;
constexpr uint32_t PANEL_TAP_MAX_MS = 700; // a press: down and up within this
constexpr uint32_t PANEL_ANSWER_MS = 1500; // no $E for the press by then: it blinks red (not done)
constexpr uint32_t PANEL_BLINK_MS = 1500;  // how long a press the arm did not do blinks
constexpr uint32_t PANEL_FLASH_MS = 600;
constexpr uint32_t PANEL_BATTERY_MS = 2500; // the battery stopped it ($E RES_BATTERY): the button says BATTERY this long   // a one-shot button (VIBRATE, DIM, ...) stays blue this long after it was done

// Round 5 (Toby, 2026-10-21): RING ON beside RING OFF, no ALL OFF ("I don't know what all off means"), LEDS ON / OFF
// (the three LED-driver groups: both palm groups and the fingers), a LIDAR button of its own, the distance at the top.
// Round 6 (Toby, 2026-10-21): CALIBRATE at the bottom - a screen of six steps (fingers out, fingers in, thumb in, arm
// straight out, arm down, arm up), each captured and saved on the arm so it is done once. The thumb has its own step (as
// on HandBench: inside a fist it only gets halfway); with no thumb sensor yet that step says so and a tap skips it.
// $C ACT_CAL (the arm's, 19) a = the step 1..6; the arm answers with one $E for ACT_CAL when the step is captured and
// saved (DONE) or could not be (REFUSED).
constexpr uint8_t ACT_CAL = glink::ACT_CAL;   // 19 (GauntletLink.h, the arm's, 2026-10-21)
static_assert(ACT_CAL == 19, "ACT_CAL moved: the arm's GauntletLink.h numbers only ever grow at the end");
constexpr int CAL_STEPS = 6;
constexpr int CAL_THUMB_STEP = 3;
// the arm steps (4 out, 5 down, 6 up) show the live arm, small, from the side, in the space between the words (Toby: "so
// I could see arm up, arm out ... the wrist aligned"): the whole-arm framing scaled to CAL_ARM_SCALE, centred at CAL_ARM_CY
constexpr int CAL_ARM_FIRST = 4;
constexpr float CAL_ARM_SCALE = 0.30f;
constexpr float CAL_ARM_CY = 134.0f;
constexpr uint32_t CAL_CAPTURE_MS = 3000;   // how long the arm is expected to take (the bar under "HOLD STILL")
constexpr uint32_t CAL_TIMEOUT_MS = 9000;   // no answer by then: "NO ANSWER"
constexpr uint32_t CAL_SAVED_MS = 900;      // "SAVED" shows this long, then the next step
struct CalStep { const char* title; const char* how; };
static const CalStep kCalStep[CAL_STEPS] = {
    {"FINGERS OUT", "Hand flat, fingers straight out"}, {"FINGERS IN", "Fingers curled all the way in"},
    {"THUMB IN", "Fingers out, thumb curled all in"},   {"ARM OUT", "Arm straight out, palm down"},
    {"ARM DOWN", "Arm hanging straight down"},          {"ARM UP", "Arm straight up"}};
// the thumb's sensor is there: the arm's $G reads it inside the flex range (0 mV = no sensor; the arm's own fault limits
// are 300 / 3100 mV). Unknown (no $G) counts as there - the arm then decides.
inline bool cal_thumb_present(bool live, uint16_t mv) { return !live || (mv >= 300 && mv <= 3100); }
enum PBtn : uint8_t { PB_OPEN, PB_CLOSE, PB_P1, PB_P2, PB_P3, PB_HAND, PB_THERMAL, PB_RED, PB_GREEN, PB_LIDAR, PB_VIBRATE,
                      PB_PALM20, PB_PALM10, PB_FINGERS, PB_LEDSON, PB_LEDSOFF, PB_RPREV, PB_RCOLOUR, PB_RNEXT, PB_RINGON,
                      PB_RINGOFF, PB_DIM, PB_BRIGHT, PB_DISARM, PB_CAL, PB_COUNT };
// need: the mode the arm wants for it (0 any, 1 SHOW, 2 FULL); oneShot: a press, not a state (blue for a moment)
struct PanelBtnDef { const char* label; uint8_t act; uint16_t a, b; uint8_t need; bool oneShot; };
// 2 = toggle (the arm decides on or off from what it is doing); THERMAL sends ON or OFF by the camera's state; the
// ring's colour buttons send RING_ON with a = the arm's palette index + 1; ALL LIGHTS sends BURST (both palm groups) and
// FINGERS, both on unless all three are on
static const PanelBtnDef kPanelBtn[PB_COUNT] = {
    {"OPEN ALL", glink::ACT_CASCADE, 1, 0, 1, false},    {"CLOSE ALL", glink::ACT_CASCADE, 0, 0, 1, false},
    {"PLATE 1", glink::ACT_PLATE, 1, 2, 1, false},       {"PLATE 2", glink::ACT_PLATE, 2, 2, 1, false},
    {"PLATE 3", glink::ACT_PLATE, 3, 2, 1, false},       {"HAND PLATE", glink::ACT_PLATE, 4, 2, 1, false},
    {"THERMAL", glink::ACT_THERMAL_ON, 0, 0, 1, false},
    {"RED LASER", glink::ACT_LASER_RED, 2, 0, 2, false}, {"GREEN LASER", glink::ACT_LASER_GREEN, 2, 0, 2, false},
    {"LIDAR", glink::ACT_LIDAR, 2, 0, 2, false},                {"VIBRATE", glink::ACT_VIBRATE, 0, 0, 1, true},
    {"PALM 20", glink::ACT_BURST, 1, 2, 1, false},       {"PALM 10", glink::ACT_BURST, 2, 2, 1, false},
    {"FINGERS", glink::ACT_FINGERS, 2, 0, 1, false},
    {"LEDS ON", glink::ACT_BURST, 0, 1, 1, false},   {"LEDS OFF", glink::ACT_BURST, 0, 0, 0, true},
    {"<", glink::ACT_RING_ON, 0, 0, 1, true},            {"", glink::ACT_RING_ON, 0, 0, 1, false},   // the colour's name
    {">", glink::ACT_RING_ON, 0, 0, 1, true},
    {"RING ON", glink::ACT_RING_ON, 0, 0, 1, false},     {"RING OFF", glink::ACT_RING_OFF, 0, 0, 0, true},
    {"DIM", glink::ACT_RING_DOWN, 0, 0, 1, true},        {"BRIGHT", glink::ACT_RING_UP, 0, 0, 1, true},
    {"DISARM", glink::ACT_DISARM, 0, 0, 0, true},      {"CALIBRATE", ACT_CAL, 0, 0, 0, false},   // opens the calibration screen
};
// the ring's colours in Toby's order, round and round: the arm's palette index (GauntletLink PALETTE_NAMES: white,
// repulsor, red, orange, amber, green, cyan, blue, magenta) and what the swatch shows
constexpr int RING_PICKS = 7;
struct RingPick { const char* name; uint8_t armIdx; uint8_t rgb[3]; };
static const RingPick kRingPick[RING_PICKS] = {
    {"WHITE", 0, {255, 255, 255}}, {"RED", 2, {255, 30, 30}},   {"ORANGE", 3, {255, 120, 0}}, {"YELLOW", 4, {255, 215, 0}},
    {"GREEN", 5, {40, 255, 60}},   {"BLUE", 7, {50, 100, 255}}, {"PURPLE", 8, {175, 50, 255}}};
inline int ring_pick_of(uint8_t armIdx) { for (int i = 0; i < RING_PICKS; i++) if (kRingPick[i].armIdx == armIdx) return i; return -1; }

constexpr int PANEL_ROWS = 12;
constexpr int PANEL_COLOUR_ROW = 7;
static const int8_t kPanelRows[PANEL_ROWS][3] = {
    {PB_OPEN, PB_CLOSE, -1}, {PB_P1, PB_P2, PB_P3}, {PB_HAND, PB_THERMAL, -1}, {PB_RED, PB_GREEN, -1}, {PB_LIDAR, PB_VIBRATE, -1},
    {PB_PALM20, PB_PALM10, PB_FINGERS}, {PB_LEDSON, PB_LEDSOFF, -1}, {PB_RPREV, PB_RCOLOUR, PB_RNEXT}, {PB_RINGON, PB_RINGOFF, -1},
    {PB_DIM, PB_BRIGHT, -1}, {PB_DISARM, -1, -1}, {PB_CAL, -1, -1}};
// how long ago t was, never negative: a touch's time can be a few ms AFTER the frame time it is compared with (round 5:
// "ms - t" wrapped to ~4e9 and THERMAL's wait for the camera, the colour row's hold and a press's answer were dropped)
inline uint32_t panel_age(uint32_t now, uint32_t t) { return (int32_t)(now - t) > 0 ? now - t : 0; }

// the list: 160 px wide between the mode symbol (left) and the battery arc (right), rows 38 px apart, seen through a
// window (y 30 .. 212, faded over its top and bottom PANEL_FADE_PX) and the glass (radius PANEL_CLIP_R); the battery's
// number (BEZEL, x 176.., y 178..193) is left clear
constexpr int PANEL_X0 = 40, PANEL_X1 = 200, PANEL_GAP = 5, PANEL_BH = 32, PANEL_PITCH = 38, PANEL_TOP = 60;   // 60 (36 until round 6: the top row sat too high to press easily)
constexpr int PANEL_WY0 = 30, PANEL_WY1 = 212, PANEL_FADE_PX = 16;
constexpr float PANEL_CLIP_R = 99.0f;
constexpr float PANEL_SCROLL_MAX = PANEL_TOP + PANEL_ROWS * PANEL_PITCH - (PANEL_PITCH - PANEL_BH) - (PANEL_WY1 - 6);
inline bool panel_clip(int x, int y) {
  if (y < PANEL_WY0 || y > PANEL_WY1) return false;
  float dx = x + 0.5f - 120, dy = y + 0.5f - 120;
  if (dx * dx + dy * dy > PANEL_CLIP_R * PANEL_CLIP_R) return false;
  return !(x >= 175 && y >= 177 && y <= 194);
}
inline int panel_fade(int y) {   // 0..256: the window's top and bottom fade out, so a row half out of view is not a button
  int d = y - PANEL_WY0 < PANEL_WY1 - y ? y - PANEL_WY0 : PANEL_WY1 - y;
  return d >= PANEL_FADE_PX ? 256 : (d <= 0 ? 0 : d * 256 / PANEL_FADE_PX);
}
// the button's box on the screen at this scroll (x1, y1 inclusive)
inline void panel_rect(int b, float scroll, int& x0, int& y0, int& x1, int& y1) {
  for (int r = 0; r < PANEL_ROWS; r++) {
    int n = 0, k = -1;
    for (int i = 0; i < 3; i++) { if (kPanelRows[r][i] >= 0) n++; if (kPanelRows[r][i] == b) k = i; }
    if (k < 0) continue;
    if (r == PANEL_COLOUR_ROW) {   // narrow arrows either side of a wide swatch: 34 + 82 + 34
      static const int kx[3] = {PANEL_X0, PANEL_X0 + 39, PANEL_X0 + 126}, kw[3] = {34, 82, 34};
      x0 = kx[k]; x1 = x0 + kw[k] - 1;
    } else {
      int w = n == 1 ? 110 : (PANEL_X1 - PANEL_X0 - (n - 1) * PANEL_GAP) / n;
      x0 = n == 1 ? 120 - w / 2 : PANEL_X0 + k * (w + PANEL_GAP); x1 = x0 + w - 1;
    }
    y0 = PANEL_TOP + r * PANEL_PITCH - (int)(scroll + 0.5f); y1 = y0 + PANEL_BH - 1;
    return;
  }
  x0 = y0 = 0; x1 = y1 = -1;
}
inline int panel_hit(int x, int y, float scroll) {
  if (!panel_clip(x, y) || panel_fade(y) < 128) return -1;
  for (int b = 0; b < PB_COUNT; b++) {
    int x0, y0, x1, y1; panel_rect(b, scroll, x0, y0, x1, y1);
    if (x >= x0 - 2 && x <= x1 + 2 && y >= y0 - 2 && y <= y1 + 2) return b;   // the gaps' edges count for the nearer button
  }
  return -1;
}
inline bool panel_on_colour_row(int y, float scroll) { int x0, y0, x1, y1; panel_rect(PB_RCOLOUR, scroll, x0, y0, x1, y1); return y >= y0 - 2 && y <= y1 + 2; }

struct PanelPress { int8_t btn = -1; uint8_t act = 0, act2 = 0; uint16_t a = 0, seq = 0; uint32_t ms = 0; int8_t res = -1; uint8_t need = 0; bool drone = false, noArm = false, late = false; uint32_t resMs = 0; };
struct PanelState {
  uint8_t page = PAGE_MAIN;
  float scroll = 0, vel = 0, scroll0 = 0;
  int8_t down = -1;          // the button under the finger (drawn pressed)
  int8_t start = -1;         // the button where this touch began (pressed by a tap, however it wobbled)
  int startZone = 0;         // this touch began on the arrow above the list (-1) or below it (+1)
  bool pressed = false;      // this touch has pressed something (the chip's click must not press it again)
  uint32_t lastUp = 0;       // when the last touch ended
  float scrollTo = -1;       // the arrows: the list glides here
  uint8_t gesture = 0;       // this touch: 0 undecided, 1 scrolling, 2 swiping back, 4 swiping the colour row
  uint32_t lt = 0;
  uint16_t seq = 0;          // the next $C's seq (app.cpp starts it at a random number: the arm drops a repeat of its last seq)
  PanelPress last;
  int8_t pick = 0; uint32_t pickMs = 0;   // the ring colour shown in the colour row (kRingPick), and when it was last stepped here
  int16_t armRingi = -1;     // the arm's ring colour ($S ringi) last seen: the row follows the arm only when it CHANGES
  int8_t lidarOn = 0;        // the LiDAR as the last answered LIDAR press left it (no state bit from the arm)
  uint8_t cal = 0;           // the calibration screen: 0 not up, 1..CAL_STEPS the step, CAL_STEPS + 1 finished
  uint8_t calSt = 0;         // 0 waiting for a tap, 1 captured by the arm (waiting for its answer), 2 saved, 3 refused, 4 no answer, 5 no arm
  uint32_t calMs = 0;        // when the step was sent / answered
  uint8_t calSaved = 0;      // bit (step - 1): saved this time round
  uint32_t thermalWait = 0;  // THERMAL pressed here: show the camera's page once the arm says it is on
  bool thermalReturn = false;   // ... and come back to the panel when it goes off
  uint32_t presses = 0, answered = 0, unanswered = 0, opens = 0;
  char tx[100] = ""; uint8_t txLen = 0;  // the $C line(s) for app.cpp to send
  char log[160] = "";                    // what the panel last did, for app.cpp to print (then cleared)
};

// what each button shows: state 0 off, 1 on, 2 moving / changing; avail = the mode switch allows it (red outline)
struct PanelView {
  uint8_t state[PB_COUNT] = {};
  bool avail[PB_COUNT] = {};
  int8_t pick = 0;
  float scroll = 0; int8_t down = -1; PanelPress last; uint32_t t = 0;
  bool touching = false; int tx = 0, ty = 0;   // where the puck sees the finger (a ring under it)
  uint16_t lidar = 0;        // mm, 0 = no reading: shown at the top
  uint8_t cal = 0, calSt = 0, calSaved = 0; uint32_t calMs = 0;   // the calibration screen (PanelState)
  int16_t curl[5] = {-1, -1, -1, -1, -1}; uint16_t fmv[5] = {};      // the fingers as the arm sees them ($G), for its finger steps
  bool thumb = true;         // a thumb sensor (cal_thumb_present)
};
// (round 4: no arrows - Toby: "it should just be scrollable"; the zone is kept at 0 so the tap code stays one path)
inline int panel_arrow_zone(int, int) { return 0; }

// the answer in words, for the log (the screen shows it by colour only)
inline const char* panel_word(const PanelPress& p, uint32_t t) {
  if (p.btn < 0) return "-";
  if (p.noArm) return "no arm on the link, nothing sent";
  if (p.res < 0) return panel_age(t, p.ms) < PANEL_ANSWER_MS ? "sent, waiting" : "no answer";
  if (p.res == glink::RES_DONE) return "done";
  if (p.res == glink::RES_BLOCKED) return p.drone ? "blocked: the drone is flying" : (p.need >= 2 ? "blocked: needs FULL" : "blocked: needs SHOW");
  if (p.res == glink::RES_BATTERY) return "refused: the battery is too low for it";
  return "refused";
}
// the press did not happen (refused, blocked, no answer, no arm): its button blinks red for PANEL_BLINK_MS
inline bool panel_failed(const PanelPress& p, uint32_t t) {
  if (p.btn < 0) return false;
  if (p.noArm) return panel_age(t, p.ms) < PANEL_BLINK_MS;
  if (p.res < 0) return panel_age(t, p.ms) >= PANEL_ANSWER_MS && panel_age(t, p.ms) < PANEL_ANSWER_MS + PANEL_BLINK_MS;
  return p.res != glink::RES_DONE && panel_age(t, p.resMs) < (p.res == glink::RES_BATTERY ? PANEL_BATTERY_MS : PANEL_BLINK_MS);
}

// the calibration screen: the step dots, the step's name and what to do, the four fingers live on the finger steps, then
// TAP TO CAPTURE / HOLD STILL (a bar) / SAVED / why not; swipe left = skip the step, right = back to the buttons
inline void draw_cal(uint16_t* fb, int y0, int rows, const hud::Look& L, const PanelView& v) {
  hud::Strip s{fb, nullptr, y0, rows};
  static const uint8_t kW[3] = {255, 255, 255}, kGrey[3] = {150, 150, 160}, kGood[3] = {90, 255, 130}, kBlue[3] = {40, 120, 255};
  const bool done = v.cal > CAL_STEPS;
  for (int i = 0; i < CAL_STEPS; i++) {   // the step dots
    float x = 120 + (i - (CAL_STEPS - 1) / 2.0f) * 14.0f;
    const uint8_t* c = (v.calSaved >> i & 1) ? kGood : (i + 1 == v.cal ? kW : kGrey);
    hud::sector(s, x, 34.5f, 0, i + 1 == v.cal ? 3.6f : 2.6f, 0, 359.9f, c, (v.calSaved >> i & 1) || i + 1 == v.cal ? 255 : 90);
  }
  if (done) {
    hud::centred2(s, 92, "CALIBRATED", kGood, 255);
    hud::centred(s, 118, "Saved on the arm", kW, 230);
    hud::centred(s, 132, "(the drone uses it too)", kGrey, 200);
    hud::centred(s, 180, "TAP TO GO BACK", L.accent, 230);
    return;
  }
  const CalStep& st = kCalStep[v.cal - 1];
  char b[24]; snprintf(b, sizeof b, "STEP %u OF %d", (unsigned)v.cal, CAL_STEPS);
  hud::centred(s, 46, b, kGrey, 220);
  hud::centred2(s, 60, st.title, kW, 255);
  hud::centred(s, 84, st.how, L.accent, 240);
  if (v.cal <= CAL_THUMB_STEP) {   // the five fingers live: how far each is curled, its millivolts under it (-- = no sensor)
    static const char* kF[5] = {"T", "I", "M", "R", "P"};
    for (int f = 0; f <= 4; f++) {
      int cx = 120 + (f - 2) * 28, x0 = cx - 7, x1 = cx + 7, top = 96, bot = 148;
      float k = v.curl[f] >= 0 ? v.curl[f] / 1000.0f : (v.fmv[f] ? v.fmv[f] / 3300.0f : 0);
      k = k < 0 ? 0 : (k > 1 ? 1 : k);
      int fillTop = bot - (int)((bot - top) * k + 0.5f);
      for (int y = top; y <= bot; y++) for (int x = x0; x <= x1; x++) {
        bool edge = x == x0 || x == x1 || y == top || y == bot;
        if (edge) hud::plot(s, x, y, kGrey, 120); else if (y >= fillTop) hud::plot(s, x, y, kBlue, 255);
      }
      hud::text(s, cx - 2, 152, kF[f], kW, 230);
      char mv[8]; if (v.fmv[f]) snprintf(mv, sizeof mv, "%u", (unsigned)v.fmv[f]); else snprintf(mv, sizeof mv, "--");
      hud::text(s, cx - hud::text_w(mv) / 2, 162, mv, kGrey, 200);
    }
  }
  const uint32_t age = panel_age(v.t, v.calMs);
  if (v.cal == CAL_THUMB_STEP && !v.thumb && v.calSt != 1) {   // no thumb sensor yet: nothing to capture
    hud::centred(s, 174, "NO THUMB SENSOR", kGrey, 255); hud::centred(s, 186, "TAP TO SKIP", L.accent, 220);
    hud::centred(s, 200, "< BUTTONS    NEXT >", kGrey, 190);   // the next step lies to the right, the buttons to the left (Toby)
    return;
  }
  switch (v.calSt) {
    case 0: hud::centred(s, 178, "TAP TO CAPTURE", L.accent, hud::blink(v.t, 600) ? 255 : 150); break;
    case 1: {
      hud::centred(s, 176, "HOLD STILL", kW, 255);
      float f = age >= CAL_CAPTURE_MS ? 1 : age / (float)CAL_CAPTURE_MS;
      for (int y = 187; y < 191; y++) for (int x = 80; x < 160; x++) hud::plot(s, x, y, x - 80 < f * 80 ? kBlue : kGrey, x - 80 < f * 80 ? 255 : 70);
      break;
    }
    case 2: hud::centred(s, 178, "SAVED", kGood, 255); break;
    case 3: hud::centred(s, 174, "DIDN'T TAKE", L.hud, 255); hud::centred(s, 186, "TAP TO TRY AGAIN", L.accent, 220); break;
    case 4: hud::centred(s, 174, "NO ANSWER", L.hud, 255); hud::centred(s, 186, "TAP TO TRY AGAIN", L.accent, 220); break;
    default: hud::centred(s, 174, "NO ARM", L.hud, 255); hud::centred(s, 186, "TAP TO TRY AGAIN", L.accent, 220); break;
  }
  hud::centred(s, 200, "< BUTTONS    NEXT >", kGrey, 190);   // the next step lies to the right, the buttons to the left (Toby)
}

// one strip of the panel page: black, the buttons, the page dots, the scroll mark; the battery and the mode symbol are
// hud::panel_overlay's (app.cpp draws them after this, as on the main page)
inline void draw_panel(uint16_t* fb, int y0, int rows, const hud::Look& L, const PanelView& v) {
  memset(fb, 0, (size_t)rows * rnd::W * 2);
  if (v.cal) { draw_cal(fb, y0, rows, L, v); hud::page_dots(fb, y0, rows, L, 1); return; }
  hud::Strip s{fb, nullptr, y0, rows};
  const int ya = y0, yb = y0 + rows - 1;
  auto put = [&](int x, int y, int r, int g, int bl) {   // solid, clipped to the list's window and the glass, faded at its ends
    if (y < ya || y > yb || x < 0 || x >= rnd::W || !panel_clip(x, y)) return;
    int f = panel_fade(y);
    r = r * f >> 8; g = g * f >> 8; bl = bl * f >> 8;
    fb[(y - y0) * rnd::W + x] = rnd::pack565_swapped(r > 255 ? 255 : r, g > 255 ? 255 : g, bl > 255 ? 255 : bl);
  };
  static const uint8_t kBlue[3] = {40, 120, 255}, kRed[3] = {255, 40, 48}, kGreyO[3] = {70, 70, 78};
  const PanelPress& p = v.last;
  const float pulse = 0.5f + 0.5f * sinf(v.t * 0.0105f);
  const bool failBlink = panel_failed(p, v.t) && hud::blink(v.t, 200);
  for (int b = 0; b < PB_COUNT; b++) {
    int x0, by0, x1, by1; panel_rect(b, v.scroll, x0, by0, x1, by1);
    if (by1 < ya || by0 > yb || by1 < PANEL_WY0 || by0 > PANEL_WY1) continue;
    const bool mine = p.btn == b, av = v.avail[b];
    const bool waiting = mine && !p.noArm && p.res < 0 && panel_age(v.t, p.ms) < PANEL_ANSWER_MS;
    const bool flash = mine && kPanelBtn[b].oneShot && p.res == glink::RES_DONE && panel_age(v.t, p.resMs) < PANEL_FLASH_MS;
    uint8_t st = v.state[b];
    // the fill: blue = on (pulsing while it moves), red for the drone armed (DISARM), dark otherwise; the swatch its colour
    int fr = 18, fg = 18, fbl = 24;
    const uint8_t* oc = av ? kRed : kGreyO;   // the outline: red = the switch allows it, grey = not in this mode
    if (b == PB_RCOLOUR) {
      const uint8_t* c = kRingPick[v.pick].rgb; float k = st ? 0.85f : 0.45f;
      fr = (int)(c[0] * k); fg = (int)(c[1] * k); fbl = (int)(c[2] * k);
      if (st) { oc = kBlue; }
    } else if (b == PB_DISARM && st) { fr = 170; fg = 20; fbl = 26; oc = kRed; }
    else if (st == 1 || flash) { fr = 20; fg = 70; fbl = 190; oc = kBlue; }
    else if (st == 2) { fr = (int)(15 + 25 * pulse); fg = (int)(40 + 50 * pulse); fbl = (int)(100 + 110 * pulse); oc = kBlue; }
    if (waiting) { oc = kBlue; fr = fr / 2 + 10; fg = fg / 2 + 40; fbl = fbl / 2 + (int)(60 + 60 * pulse); }   // pressed, the arm not answered yet
    if (mine && failBlink) { fr = 150; fg = 20; fbl = 26; oc = kRed; }   // the arm did not do it
    if (v.down == b) { fr += 60; fg += 60; fbl += 70; }
    const int rad = 7;
    for (int y = by0 < ya ? ya : by0; y <= (by1 > yb ? yb : by1); y++)
      for (int x = x0; x <= x1; x++) {
        int dx = x < x0 + rad ? x0 + rad - x : (x > x1 - rad ? x - (x1 - rad) : 0);
        int dy = y < by0 + rad ? by0 + rad - y : (y > by1 - rad ? y - (by1 - rad) : 0);
        int d2 = dx * dx + dy * dy;
        if (d2 > rad * rad) continue;                                        // the rounded corners
        bool edge = d2 > (rad - 2) * (rad - 2) || x <= x0 + 1 || x >= x1 - 1 || y <= by0 + 1 || y >= by1 - 1;   // a 2 px outline
        if (edge) put(x, y, oc[0], oc[1], oc[2]);
        else put(x, y, fr, fg, fbl);
      }
    // the label: white where the switch allows it, grey where it does not; the swatch's name dark on a light colour
    const char* lab = b == PB_RCOLOUR ? kRingPick[v.pick].name : kPanelBtn[b].label;
    if (mine && p.res == glink::RES_BATTERY && panel_failed(p, v.t)) lab = "BATTERY";   // it was the battery that stopped it
    int lr = av ? 255 : 120, lg = av ? 255 : 120, lb = av ? 255 : 130;
    if (b == PB_RCOLOUR) { const uint8_t* c = kRingPick[v.pick].rgb; if (c[0] + c[1] + c[2] > 450) lr = lg = lb = av ? 10 : 70; }
    const int sc = (b == PB_RPREV || b == PB_RNEXT) ? 2 : 1;   // the arrows twice the size
    int tw = sc * hud::text_w(lab) + (sc - 1), tx = (x0 + x1 + 1) / 2 - tw / 2, ty = (by0 + by1 + 1) / 2 - (7 * sc) / 2;
    if (ty + 7 * sc >= ya && ty <= yb)
      for (const char* q = lab; *q; q++, tx += 6 * sc) {
        const unsigned char* g = kHudFont + 5 * (unsigned char)*q;
        for (int cx = 0; cx < 5; cx++)
          for (int cy = 0; cy < 7; cy++)
            if (g[cx] >> cy & 1)
              for (int a = 0; a < sc; a++) for (int e = 0; e < sc; e++) put(tx + cx * sc + e, ty + cy * sc + a, lr, lg, lb);
      }
  }
  // the finger, where the puck sees it: a thin ring (so a tap that lands elsewhere shows itself)
  if (v.touching) { static const uint8_t kF[3] = {255, 255, 255}; hud::sector(s, v.tx + 0.5f, v.ty + 0.5f, 9.0f, 11.0f, 0, 359.9f, kF, 170); }
  static const uint8_t kGrey[3] = {150, 150, 160};
  hud::page_dots(fb, y0, rows, L, 1);   // the page dots: this one lit
  // the LiDAR's distance at the top while it measures (the LIDAR button, or the red laser), in feet like the main page
  if (v.lidar) { char b[16]; hud::lidar_feet(v.lidar, b, sizeof b); hud::centred2(s, 10, b, L.accent, 255); }
  else if (v.state[PB_LIDAR]) hud::centred(s, 14, "NO RETURN", kGrey, 200);
  // where the list is: a short arc on the lower left rim
  float f = PANEL_SCROLL_MAX > 0 ? hud::clampd(v.scroll / PANEL_SCROLL_MAX, 0, 1) : 0;
  hud::arc_solid(s, 106, 109, 205, 250, kGrey, 45);
  float a1 = 250 - f * 33, a0 = a1 - 12;
  hud::arc_solid(s, 105, 110, a0, a1, L.accent, 220);
}

}  // namespace app
