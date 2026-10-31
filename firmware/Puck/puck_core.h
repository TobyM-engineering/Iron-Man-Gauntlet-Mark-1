// puck_core.h - everything the puck does that is not hardware: the arm's link, touch, the model's pose, the HUD's
// values, frame building. app.cpp (the ESP32) and puck/preview (WebAssembly) both run THIS code, and the PC tools.
//
// Views, in order of precedence:
//   LIVE  the arm's $G frames (GauntletLink v2, arm_link.h): the forearm's orientation turns the whole model, the
//         wrist (conj(forearm) * hand) bends the hand on it, the fingers from the curls, each plate from its servos,
//         the lights as the arm drives them, the battery. Once a $G has arrived the puck stays in LIVE (the link-lost
//         rules in arm_link.h), never DEMO.
//   ARM   only $S frames (an arm that sends no $G): the cascade replayed from the plates state.
//   STILL no arm yet (Toby, 2026-10-08: no demo): the hand still in its neutral pose - fingers relaxed, plates closed,
//         nothing lit, no events - in the home view. The scripted arm ("garm 1" on the USB console) is for testing only.
// The camera (view.h): drag = orbit, double tap = toggle the zoom (hand <-> whole arm, the orientation kept), hold =
// home (the hand, from above); a sideways drag from the left / right rim = the control panel (panel.h, 2026-10-20). The view follows the action (round 10): the hand plate / the red laser -> the hand view,
// a forearm plate / the green laser / a cascade -> the whole arm, then back to exactly the view before. Touch does
// nothing else: nothing sent to the arm, nothing zeroed. The thermal page shows while the arm says the thermal camera
// is deployed ($S flags bit 6, glink::FLAG_THERMAL). The top word comes from the arm's $E lines (DONE, BLOCKED by the
// mode switch, REFUSED); before the first $E, from $G's last event. The arm's link lost: the model turns grey.
#pragma once
#include <stdlib.h>
#include <string.h>
#include "view.h"
#include "hud.h"
#include "arm_link.h"

#ifndef ARM_POSE_WORLD
#define ARM_POSE_WORLD 1   // 1: the forearm's own orientation ($G fq) turns the whole model; 0: the forearm stays put
#endif
#ifndef HUD_DIR_DEFAULT
#define HUD_DIR_DEFAULT 1  // the HUD direction at boot: 1 BEZEL, 2 RETICLE, 3 ORBIT (USB "hud N")
#endif

namespace app {

constexpr int HUD_X = 148, HUD_Y = 170, HUD_W = 48, HUD_H = 20;   // (old sprite place, the browser preview's)
constexpr int TOAST_X = 60, TOAST_Y = 60, TOAST_W = 120, TOAST_H = 10;

enum Event : uint8_t { EV_NONE = 0, EV_LIVE, EV_NO_SIGNAL, EV_VIEW_RESET, EV_HAND_ZEROED, EV_DEMO_LEVELED, EV_BOOT, EV_ARM_UP, EV_ARM_LOST, EV_DEBUG_ON, EV_DEBUG_OFF,
                       EV_MISMATCH,
                       EV_G_TRIPLE, EV_G_DIP_UP, EV_G_DIP_DOWN, EV_G_LASER, EV_G_ARMED, EV_G_DISARMED, EV_G_CASCADE, EV_G_PLATE, EV_G_BURST,
                       EV_MODE_SAFE, EV_MODE_SHOW, EV_MODE_FLIGHT, EV_BAT_LOW, EV_SHUTDOWN, EV_THERMAL_ON, EV_THERMAL_OFF, EV_ZOOM_OUT, EV_ZOOM_IN,
                       EV_HOME, EV_COUNT };

// the word the HUD shows at the top for a moment (fits the top band: 20 characters at most); ms = how long it counts
// as news; warn = drawn in the warning colour; an empty word = nothing shown (the change speaks for itself)
inline const char* event_toast(Event e, uint32_t& ms, bool* warn = nullptr) {
  if (warn) *warn = false;
  ms = 2500;
  switch (e) {
    case EV_LIVE: return "LIVE";
    case EV_NO_SIGNAL: return "NO SIGNAL";
    case EV_BOOT: return "";
    case EV_ARM_UP: return "";       // the model's colour coming back says it (round 10: no text)
    case EV_ARM_LOST: return "";     // the model turning grey says it
    case EV_DEBUG_ON: return "LINK DEBUG ON";
    case EV_DEBUG_OFF: return "LINK DEBUG OFF";
    case EV_MISMATCH: if (warn) *warn = true; return "PROTOCOL MISMATCH";
    case EV_G_TRIPLE: return "RING";
    case EV_G_DIP_UP: return "BRIGHTER";
    case EV_G_DIP_DOWN: return "DIMMER";
    case EV_G_LASER: return "LASER";
    case EV_G_ARMED: if (warn) *warn = true; return "DRONE ARMED";
    case EV_G_DISARMED: return "DISARMED";
    case EV_G_CASCADE: return "PLATES";
    case EV_G_PLATE: return "PLATE";
    case EV_G_BURST: return "BURST";
    case EV_MODE_SAFE: return "SAFE";
    case EV_MODE_SHOW: return "SHOW";
    case EV_MODE_FLIGHT: if (warn) *warn = true; return "FULL";   // the arm's third switch position (GauntletLink calls it FLIGHT)
    case EV_BAT_LOW: if (warn) *warn = true; return "BATTERY LOW";
    case EV_SHUTDOWN: if (warn) *warn = true; return "SHUTDOWN";
    default: ms = 0; return "";
  }
}

// Touch (round 10): three ways to see a double tap, whichever comes first acts -
//   taps   the puck's own: a tap = down and up within TOUCH_TAP_MAX_MS without moving TOUCH_TAP_SLOP_PX; a double =
//          a second tap released within TOUCH_DOUBLE_MS of the first (round 7's rules, the ones that worked)
//   clicks the chip's single-click reports (gesture register 0x05, on each release): two within TOUCH_DOUBLE_MS -
//          catches a tap too short for the puck's ~16 Hz sampling
//   chip   the chip's own double click (0x0B), if it ever reports one
// The chip's double-click mode is NOT switched on any more (round 9 did, MotionMask 0xEC = 1; with it 6 of Toby's 20
// double taps registered - the chip then holds back quick taps while it decides). A second detector seeing the SAME
// double tap within TOUCH_SAME_MS is ignored; the same detector never is (round 9 lost two real doubles that way).
// Junk = a point off the 240 x 240 panel: dropped and counted. Every touch is logged when it ends (app.cpp).
constexpr uint32_t TOUCH_TAP_MAX_MS = 300, TOUCH_DOUBLE_MS = 450, TOUCH_SAME_MS = 400;
// A touch counts only if the chip reported the finger in at least TOUCH_MIN_SAMPLES reads (2026-10-21, on the arm: plates
// moved and the red laser went on and off with nobody touching - the arm's log of the puck's touch traces had taps of one
// sample, 3-22 ms, on the main page with no one near it; Toby's real taps were 38-393 ms, 2-21 samples, the touch task
// reading every 12 ms while a finger is on). Everywhere: a panel press, the thermal page's tap-off, the main page's taps
// and double taps, the chip's own click reports (they count only for a touch the puck saw with enough samples), the swipe
// into the panel (a move needs a second sample anyway). A rejected one is logged and counted (ts.noise).
constexpr int TOUCH_MIN_SAMPLES = 2;
constexpr int TOUCH_TAP_SLOP_PX = 10;
constexpr uint8_t CHIP_CLICK = 0x05, CHIP_DOUBLE_CLICK = 0x0B;
struct TouchState {
  bool down = false;
  int x0 = 0, y0 = 0, lx = 0, ly = 0, maxMove = 0, samples = 0;
  uint32_t tDown = 0, lastTap = 0, lastToggle = 0, lastClick = 0;
  char lastBy = 0;
  bool moved = false, longFired = false, tapPending = false;
  uint8_t edge = 0;             // 1 = this touch began out on the left / right side: maybe a swipe into the panel (undecided)
  int8_t side = 0;              // which way the middle is from where it began (-1 left, +1 right)
  bool acted = false;           // this touch already did its one thing (the thermal page's tap / the chip's click)
  uint32_t lastUp = 0;          // when the last touch ended
  int lastSamples = 0;          // ... and how many reads it had (the chip's click report belongs to it)
  uint32_t noise = 0;           // touches of fewer than TOUCH_MIN_SAMPLES reads, ignored
  bool used = false;            // this touch changed page: the rest of it does nothing
  uint8_t chipPrev = 0;
  uint32_t taps = 0, clicks = 0, doublesOwn = 0, doublesClicks = 0, doublesChip = 0, toggles = 0, junk = 0;
  char log[120] = "";           // the last thing touch did, for app.cpp to print (then cleared)
  char trace[120] = "";         // each touch as it ended: how long, how many samples, how far it moved, what it was
};

struct Core {
  kin::Inputs inputs;
  kin::M34 world[model::NBODY];
  CamState cam;
  gauntlet_parser_t parser;        // the older 40-byte packet (browser preview only)
  gauntlet_packet_t lastPkt;
  uint32_t lastPacketMs = 0;
  bool live = false;               // the older 40-byte packet is driving (browser preview only)
  float qZero[4] = {1, 0, 0, 0};
  float qLast[4] = {1, 0, 0, 0};
  float gFilt[3] = {0, 0, 1};      // the puck's own IMU, read but not applied (round 5)
  float gRef[3] = {0, 0, 1};
  bool imuPresent = false;
  int imuSamples = 0;
  float demoT = 0;
  TouchState ts;
  PanelState panel;                // the control panel, the second page (panel.h)
  ArmLink arm;
  bool armWasUp = false, mismatchWas = false;
  int8_t lastMode = -1; uint8_t lastBstate = 0;
  Lights lights;
  float fistK = 0;
  float qWrist[4] = {1, 0, 0, 0};  // LIVE: the wrist as drawn (eased toward each $G)
  float qArm[4] = {1, 0, 0, 0};    // LIVE: the forearm as drawn
  float plate[model::NFOURBAR] = {0, 0, 0, 0};
  rnd::Fx fx;
  uint16_t lidarHeld = 0;
  uint32_t lidarMs = 0;
  bool blank = false;              // the arm said SHUTDOWN: the screen is off
  bool thermal = false;            // the thermal page instead of the 3D view (the arm asks: $S flags bit 1)
  int8_t thermalForce = -1;        // console "thermal 1|0|auto": -1 = follow the arm
  char actWord[24] = ""; uint8_t actRes = 0; bool actWarn = false; uint32_t actMs = 0;   // the arm's last $E, as the HUD says it
  bool actDrone = false;   // BLOCKED because the drone is flying (or was just disarmed), not by the mode switch
  float platePrev[model::NFOURBAR] = {-1, -1, -1, -1}, plateGlow[model::NFOURBAR] = {};   // each plate's last position and its glow while it moves (0..1)
  bool grey = false;                  // the arm's link was up and is lost: the model is drawn grey
  uint8_t hudDir = HUD_DIR_DEFAULT;
  Event lastEvent = EV_NONE; uint32_t lastEventMs = 0;
};

inline void core_init(Core& c) {
  kin::rest_inputs(c.inputs);
  gauntlet_parser_init(&c.parser);
  memset(&c.lastPkt, 0, sizeof c.lastPkt);
}

inline bool core_is_live(const Core& c) { return c.arm.haveLive; }

// an $E action -> the word the HUD shows (DONE as it is, BLOCKED grey and struck through, REFUSED red and struck)
inline const char* act_word(uint8_t act, uint8_t res, bool& warn) {
  warn = false;
  switch (act) {
    case glink::ACT_RING_ON: return "RING";
    case glink::ACT_RING_OFF: return "RING OFF";
    case glink::ACT_RING_UP: return "BRIGHTER";
    case glink::ACT_RING_DOWN: return "DIMMER";
    case glink::ACT_CASCADE: return "PLATES";
    case glink::ACT_PLATE: return "PLATE";
    case glink::ACT_BURST: return "BURST";
    case glink::ACT_FINGERS: return "FINGER LEDS";
    case glink::ACT_VIBRATE: return "VIBRATE";
    case glink::ACT_THERMAL_ON: return "THERMAL";
    case glink::ACT_THERMAL_OFF: return "THERMAL OFF";
    case glink::ACT_LASER_RED: return "LASER";
    case glink::ACT_LASER_GREEN: return "GREEN LASER";
    case glink::ACT_ARM: if (res == 0) { warn = true; return "DRONE ARMED"; } return "ARM";
    case glink::ACT_DISARM: return "DISARMED";
    case glink::ACT_FLIP: return "FLIP";
    case glink::ACT_NONE: return "";
    default: return nullptr;   // a number this puck does not know yet: "GESTURE N"
  }
}

// the newest event / change for the HUD's top word
inline void core_event(Core& c, Event e, uint32_t ms) {
  uint32_t t; if (e == EV_NONE || !event_toast(e, t)[0]) return;
  c.lastEvent = e; c.lastEventMs = ms;
}

// ---------------------------------------------------------------- the older 40-byte packet (browser preview)
inline void core_link_byte(Core& c, uint8_t b, uint32_t ms) {
  gauntlet_packet_t p;
  if (!gauntlet_parser_push(&c.parser, b, &p)) return;
  c.lastPkt = p;
  c.lastPacketMs = ms;
  if ((p.flags & GAUNTLET_FLAG_HAND_IMU) && (p.flags & GAUNTLET_FLAG_BODY_IMU)) packet_quat(p, c.qLast);
  if (c.live) live_inputs(p, c.qZero, c.inputs);
}
inline Event core_link_update(Core& c, uint32_t ms) {
  bool fresh = c.parser.packets > 0 && (ms - c.lastPacketMs) < LINK_TIMEOUT_MS;
  if (fresh && !c.live) { c.live = true; live_inputs(c.lastPkt, c.qZero, c.inputs); return EV_LIVE; }
  if (!fresh && c.live) { c.live = false; return EV_NO_SIGNAL; }
  return EV_NONE;
}

// ---------------------------------------------------------------- the arm's link, once per loop
// The edges become HUD words: LINKED / LINK LOST, PROTOCOL MISMATCH, a mode change, BATTERY LOW, the arm's events.
inline Event core_arm_update(Core& c, uint32_t ms) {
  ArmLink& L = c.arm;
  if (ms - L.rateT0 >= 1000) {
    float s = (ms - L.rateT0) / 1000.0f;
    L.goodRate = L.rateT0 ? (L.good() - L.good0) / s : 0;
    L.badRate = L.rateT0 ? (L.bad() - L.bad0) / s : 0;
    L.liveRate = L.rateT0 ? (L.liveFrames - L.live0) / s : 0;
    L.good0 = L.good(); L.bad0 = L.bad(); L.live0 = L.liveFrames; L.rateT0 = ms;
    L.liveGapMax = L.liveGapWork; L.liveGapWork = 0;
  }
  Event e = EV_NONE;
  bool mm = L.mismatch(ms);
  bool up = L.anyUp(ms);
  int8_t mode = L.liveUp(ms) ? (int8_t)L.live.mode : (L.up(ms) ? (int8_t)L.st.mode : -1);
  uint8_t bs = L.liveUp(ms) ? L.live.bstate : (L.up(ms) ? L.st.bstate : 0);
  if (mm != c.mismatchWas) { c.mismatchWas = mm; if (mm) e = EV_MISMATCH; }
  else if (up != c.armWasUp) { c.armWasUp = up; e = up ? EV_ARM_UP : EV_ARM_LOST; }
  else if (mode >= 0 && mode != c.lastMode) { if (c.lastMode >= 0) e = (Event)(EV_MODE_SAFE + mode); c.lastMode = mode; }
  else if (bs != c.lastBstate) { if (bs >= 2 && bs <= 4 && c.lastBstate < 2) e = EV_BAT_LOW; c.lastBstate = bs; }
  else if (L.pendingGev) { uint8_t g = L.pendingGev; L.pendingGev = 0; if (!L.evSeen && g < glink::GEV_COUNT) e = (Event)(EV_G_TRIPLE + g - 1); }
  core_event(c, e, ms);
  if (L.evNew) {   // the arm's $E: what was asked, and whether it was done
    L.evNew = false;
    const char* w = act_word(L.ev.act, L.ev.res, c.actWarn);
    if (w) snprintf(c.actWord, sizeof c.actWord, "%s", w); else snprintf(c.actWord, sizeof c.actWord, "GESTURE %u", (unsigned)L.ev.act);
    c.actRes = L.ev.res; c.actMs = ms ? ms : 1;
    // BLOCKED has two meanings (the arm, 2026-10-07 late night): mode < need = the mode switch; mode = need = FULL =
    // the drone is armed or flipping, or was disarmed under 1.5 s ago (the lasers only)
    c.actDrone = L.ev.res == glink::RES_BLOCKED && L.ev.mode >= L.ev.need && L.ev.need == 2;
  }
  // the answers to the control panel's last press: the $E for its act(s) after it (the arm answers every $C; ALL LIGHTS
  // sends two - the worse answer counts)
  for (uint8_t i = 0; i < L.evqN; i++) {
    const glink::Event& ev = L.evq[i];
    if (ev.act == ACT_CAL) {   // the calibration step's answer (it comes when the arm has captured and saved it)
      PanelState& P = c.panel;
      if (P.cal >= 1 && P.cal <= CAL_STEPS && P.calSt == 1) {
        P.calSt = ev.res == glink::RES_DONE ? 2 : 3; P.calMs = ms ? ms : 1;
        if (ev.res == glink::RES_DONE) P.calSaved |= (uint8_t)(1 << (P.cal - 1));
        snprintf(P.log, sizeof P.log, "panel: calibrate step %u (%s) -> the arm: %s", (unsigned)P.cal, kCalStep[P.cal - 1].title, ev.res == glink::RES_DONE ? "saved" : "refused");
      }
      continue;
    }
    PanelPress& pp = c.panel.last;
    if (pp.btn < 0 || pp.noArm || !(ev.act == pp.act || (pp.act2 && ev.act == pp.act2)) || panel_age(ms, pp.ms) >= PANEL_ANSWER_MS + PANEL_BLINK_MS) continue;
    if (pp.res >= 0 && !(pp.act2 && ev.res > pp.res)) continue;
    if (pp.res < 0) c.panel.answered++;
    pp.res = (int8_t)ev.res; pp.need = ev.need; pp.drone = ev.res == glink::RES_BLOCKED && ev.mode >= ev.need && ev.need == 2; pp.resMs = ms ? ms : 1;
    if (pp.btn == PB_LIDAR && ev.res == glink::RES_DONE) c.panel.lidarOn = pp.a ? 1 : 0;
    snprintf(c.panel.log, sizeof c.panel.log, "panel: %s -> the arm: %s (%u ms)", pp.btn == PB_RCOLOUR || pp.btn == PB_RPREV || pp.btn == PB_RNEXT ? "ring colour" : kPanelBtn[pp.btn].label,
             panel_word(pp, ms), (unsigned)panel_age(ms, pp.ms));
  }
  L.evqN = 0;
  c.grey = (L.have || L.haveLive) && !up;
  return e;
}

// The low-battery shutdown: the arm sets $S flags bit 5 (SHUTDOWN) when the glove pack is CRITICAL and then
// deep-sleeps and sends nothing more -> the screen and the OLED go dark at once and stay dark. A $S without the
// bit (the arm powered up again) brings them back. Returns c.blank.
constexpr uint8_t S_FLAG_SHUTDOWN = 1 << 5;
inline bool core_blank_update(Core& c, uint32_t ms) {
  const ArmLink& L = c.arm;
  if (L.have) {
    bool sd = L.st.flags & S_FLAG_SHUTDOWN;
    if (sd && !c.blank) core_event(c, EV_SHUTDOWN, ms);
    c.blank = sd;
  }
  return c.blank;
}

// ---------------------------------------------------------------- IMU (the puck's own; not applied since round 5)
inline void core_imu_sample(Core& c, const float g[3]) {
  if (c.imuSamples == 0) memcpy(c.gFilt, g, sizeof c.gFilt);
  else for (int i = 0; i < 3; i++) c.gFilt[i] += (g[i] - c.gFilt[i]) * 0.2f;
  if (++c.imuSamples == DEMO_LEVEL_SAMPLES) memcpy(c.gRef, c.gFilt, sizeof c.gRef);
}

// ---------------------------------------------------------------- touch: one sample per frame (the view only)
// now / x / y: the finger as the chip reports it; chip: the chip's gesture register (0 if unknown)
inline Event touch_toggle(Core& c, uint32_t ms, const char* by) {
  TouchState& ts = c.ts;
  if (ts.lastToggle && ms - ts.lastToggle < TOUCH_SAME_MS && ts.lastBy != by[0]) {   // the same double tap, seen another way
    snprintf(ts.log, sizeof ts.log, "touch: double tap (%s) - the same one, already done", by);
    return EV_NONE;
  }
  ts.lastToggle = ms; ts.lastBy = by[0]; ts.toggles++;
  if (c.thermal) { snprintf(ts.log, sizeof ts.log, "touch: double tap (%s) - thermal page up, view unchanged", by); return EV_NONE; }
  cam_toggle_zoom(c.cam);
  snprintf(ts.log, sizeof ts.log, "touch: double tap (%s) #%u -> zoom %s", by, (unsigned)ts.toggles, c.cam.zoomTarget > 0.5f ? "out" : "in");
  return c.cam.zoomTarget > 0.5f ? EV_ZOOM_OUT : EV_ZOOM_IN;
}
// ---------------------------------------------------------------- the control panel (panel.h)
inline void panel_go(Core& c, uint8_t page, uint32_t ms, const char* why) {
  PanelState& P = c.panel;
  P.page = page; P.down = -1; P.vel = 0; P.cal = 0;
  if (page == PAGE_PANEL) P.opens++;
  TouchState& ts = c.ts;
  if (ts.down) { ts.used = true; ts.moved = true; ts.longFired = true; }   // the touch that did it does nothing else
  ts.tapPending = false; ts.lastClick = 0;
  snprintf(P.log, sizeof P.log, "panel: %s -> %s", why, page == PAGE_PANEL ? "the control panel" : "the main page");
}
// what the arm says is on: the LED groups (bit0 palm 20 mm, bit1 palm 10 mm, bit2 fingers)
inline uint8_t panel_groups(const Core& c, uint32_t ms) {
  const ArmLink& L = c.arm;
  if (L.liveUp(ms)) return L.live.groups;
  uint8_t pk = L.up(ms) ? L.st.pucks : 0;   // $S: one bit a LED driver - 1-3 group 1, 4-5 group 3 (fingers), 6-7 group 2 (the arm's PUCK_GROUP_OF)
  return (uint8_t)((pk & 0x07 ? 1 : 0) | (pk & 0x60 ? 2 : 0) | (pk & 0x18 ? 4 : 0));
}
// one press: its $C line(s) (P.tx, app.cpp sends them); the answer comes back as an $E (core_arm_update)
inline void panel_press(Core& c, int b, uint32_t ms) {
  if (b < 0 || b >= PB_COUNT) return;
  PanelState& P = c.panel;
  if (b == PB_CAL) { P.cal = 1; P.calSt = 0; P.calSaved = 0; P.presses++; snprintf(P.log, sizeof P.log, "panel: CALIBRATE - the calibration screen (step 1 of %d)", CAL_STEPS); return; }
  PanelBtnDef d = kPanelBtn[b];
  glink::Command cmd[2]; int n = 1;
  cmd[0].act = d.act; cmd[0].a = d.a; cmd[0].b = d.b;
  if (b == PB_THERMAL) { cmd[0].act = c.thermal ? glink::ACT_THERMAL_OFF : glink::ACT_THERMAL_ON; if (!c.thermal) P.thermalWait = ms ? ms : 1; }
  if (b == PB_LEDSON || b == PB_LEDSOFF) {   // the three LED-driver groups: both palm groups (one BURST, a 0) and the fingers
    uint16_t on = b == PB_LEDSON ? 1 : 0;
    cmd[0].a = 0; cmd[0].b = on; cmd[1].act = glink::ACT_FINGERS; cmd[1].a = on; n = 2;
  }
  if (b == PB_LIDAR) cmd[0].a = P.lidarOn ? 0 : 1;   // on / off by what the last answer left it
  if (b == PB_RPREV || b == PB_RNEXT || b == PB_RCOLOUR || b == PB_RINGON) {   // the ring on in the colour shown (stepped first by an arrow)
    if (b == PB_RNEXT || b == PB_RPREV) P.pick = (int8_t)((P.pick + (b == PB_RNEXT ? 1 : RING_PICKS - 1)) % RING_PICKS);   // RING ON never steps it (round 6)
    P.pickMs = ms ? ms : 1;
    cmd[0].a = (uint16_t)(kRingPick[P.pick].armIdx + 1);
  }
  P.last = PanelPress(); P.last.btn = (int8_t)b; P.last.act = cmd[0].act; P.last.act2 = n > 1 ? cmd[1].act : 0; P.last.a = cmd[0].a; P.last.ms = ms ? ms : 1; P.presses++;
  const char* name = b == PB_RCOLOUR || b == PB_RPREV || b == PB_RNEXT || b == PB_RINGON ? kRingPick[P.pick].name : d.label;
  if (!c.arm.anyUp(ms)) { P.last.noArm = true; snprintf(P.log, sizeof P.log, "panel: %s - no arm on the link, nothing sent", name); return; }
  char body[40]; int len = 0, w = snprintf(P.log, sizeof P.log, "panel: %s%s ->", b == PB_RPREV || b == PB_RNEXT || b == PB_RCOLOUR || b == PB_RINGON ? "ring " : "", name);
  for (int i = 0; i < n; i++) {
    cmd[i].seq = P.seq++;
    if (!glink::commandBody(body, sizeof body, cmd[i])) continue;
    len += (int)glink::frame(P.tx + len, sizeof P.tx - len, body);
    if (w > 0 && w < (int)sizeof P.log) w += snprintf(P.log + w, sizeof P.log - w, " $%s (%s)", body, glink::actName(cmd[i].act));
  }
  P.txLen = (uint8_t)len;
  P.last.seq = cmd[0].seq;
}
// touch on the panel: a tap on a button presses it (a small wobble allowed; the chip's own click report counts too), an
// up / down drag scrolls the list, a sideways swipe goes back (on the colour row it steps the colour instead), a tap on
// the arrow above / below the list scrolls by three rows
// the calibration screen's tap: capture this step (one $C ACT_CAL, a = the step); on the finished screen, back to the buttons
inline void cal_tap(Core& c, uint32_t ms) {
  PanelState& P = c.panel;
  P.pressed = true;
  if (P.cal > CAL_STEPS) { P.cal = 0; snprintf(P.log, sizeof P.log, "panel: calibration finished - back to the buttons"); return; }
  if (P.calSt == 1 || P.calSt == 2) return;   // capturing, or just saved: wait
  if (P.cal == CAL_THUMB_STEP && !cal_thumb_present(c.arm.liveUp(ms), c.arm.live.fmv[0])) {   // no thumb sensor yet: skip it
    P.cal++; P.calSt = 0; snprintf(P.log, sizeof P.log, "panel: calibrate - no thumb sensor yet, its step skipped"); return;
  }
  P.calMs = ms ? ms : 1;
  if (!c.arm.anyUp(ms)) { P.calSt = 5; snprintf(P.log, sizeof P.log, "panel: calibrate step %u - no arm on the link, nothing sent", (unsigned)P.cal); return; }
  glink::Command cmd; cmd.seq = P.seq++; cmd.act = ACT_CAL; cmd.a = P.cal;
  char body[40];
  P.txLen = glink::commandBody(body, sizeof body, cmd) ? (uint8_t)glink::frame(P.tx, sizeof P.tx, body) : 0;
  P.calSt = 1;
  snprintf(P.log, sizeof P.log, "panel: calibrate step %u (%s) -> $%s", (unsigned)P.cal, kCalStep[P.cal - 1].title, body);
}
inline void panel_tap(Core& c, uint32_t ms) {
  if (c.panel.cal) { cal_tap(c, ms); return; }
  PanelState& P = c.panel;
  P.pressed = true;
  if (P.start >= 0) { panel_press(c, P.start, ms); return; }
  if (P.startZone) {
    float base = P.scrollTo >= 0 ? P.scrollTo : P.scroll, to = base + P.startZone * 3.0f * PANEL_PITCH;
    P.scrollTo = to < 0 ? 0 : (to > PANEL_SCROLL_MAX ? PANEL_SCROLL_MAX : to);
  }
}
inline Event panel_touch(Core& c, bool now, int x, int y, uint32_t ms, bool click) {
  TouchState& ts = c.ts; PanelState& P = c.panel;
  const char* what = nullptr;
  if (now && !ts.down) {
    ts.down = true; ts.x0 = ts.lx = x; ts.y0 = ts.ly = y; ts.tDown = ms; ts.moved = false; ts.longFired = false; ts.maxMove = 0; ts.samples = 1;
    ts.used = false; ts.edge = 0;
    P.gesture = 0; P.lt = ms; P.scroll0 = P.scroll; P.vel = 0; P.scrollTo = -1; P.pressed = false;
    P.down = P.start = (int8_t)panel_hit(x, y, P.scroll);
    P.startZone = P.start < 0 ? panel_arrow_zone(x, y) : 0;
  } else if (now && ts.down) {
    ts.samples++;
    int dx = x - ts.x0, dy = y - ts.y0, adx = abs(dx), ady = abs(dy), mv = adx + ady;
    if (mv > ts.maxMove) ts.maxMove = mv;
    if (!ts.used) {
      if (P.gesture == 0 && mv > PANEL_SLOP_PX) {
        ts.moved = true; P.down = -1;
        P.gesture = adx > ady ? (!P.cal && panel_on_colour_row(ts.y0, P.scroll) ? 4 : 2) : (P.cal ? 3 : 1);   // 3: up / down on the calibration screen, nothing
      }
      if (P.gesture == 1 && adx >= PANEL_BACK_PX && adx > 2 * ady) { P.scroll = P.scroll0; P.gesture = 2; }   // it was a sideways swipe after all
      if (P.gesture == 1) {
        float ns = P.scroll0 - dy * PANEL_SCROLL_GAIN; ns = ns < 0 ? 0 : (ns > PANEL_SCROLL_MAX ? PANEL_SCROLL_MAX : ns);
        if (ms > P.lt) { float v = (ns - P.scroll) * 1000.0f / (float)(ms - P.lt); P.vel = P.vel * 0.4f + v * 0.6f; }
        P.scroll = ns; P.lt = ms;
      } else if (P.gesture == 2 && P.cal) {   // the calibration screen: right = back to the buttons, left = skip this step
        if (dx >= PANEL_BACK_PX) { P.cal = 0; ts.used = true; snprintf(P.log, sizeof P.log, "panel: calibration left (what was saved stays saved)"); }
        else if (dx <= -PANEL_BACK_PX) { if (P.cal <= CAL_STEPS) { P.cal++; P.calSt = 0; } ts.used = true; snprintf(P.log, sizeof P.log, "panel: calibration step skipped"); }
      } else if (P.gesture == 2 && dx >= PANEL_BACK_PX) panel_go(c, PAGE_MAIN, ms, "swipe left to right");   // back = rightwards only
      else if (P.gesture == 4 && adx >= PANEL_COLOUR_SWIPE_PX) {   // the finger going left = on along the list (a carousel)
        panel_press(c, dx < 0 ? PB_RNEXT : PB_RPREV, ms);
        ts.used = true; P.pressed = true;   // one step a swipe
      }
    }
    ts.lx = x; ts.ly = y;
  } else if (!now && ts.down && !ts.used && P.gesture == 0 && ts.samples < TOUCH_MIN_SAMPLES) {   // one read: noise, never a press
    ts.down = false; ts.lastUp = ms; ts.lastSamples = ts.samples; P.down = -1; P.lastUp = ms; P.pressed = true;
    ts.noise++;
    snprintf(ts.trace, sizeof ts.trace, "touch (panel): ignored - %d sample, %u ms (noise) at %d,%d", ts.samples, (unsigned)(ms - ts.tDown), ts.x0, ts.y0);
    return EV_NONE;
  } else if (!now && ts.down) {
    ts.down = false; ts.lastUp = ms; ts.lastSamples = ts.samples;
    uint32_t dur = ms - ts.tDown;
    what = P.gesture == 4 ? "colour swipe" : (ts.used ? "swipe (page changed)" : (P.gesture == 1 ? "scroll" : (P.gesture == 2 ? "short sideways drag" : "tap")));
    const bool quick = dur < PANEL_TAP_QUICK_MS && ts.maxMove <= PANEL_TAP_MOVE_PX;
    if (!ts.used && !P.pressed && (P.gesture == 0 || (quick && P.gesture != 4))) {   // a tap - a quick wobble counts as one
      if (P.gesture == 1) { P.scroll = P.scroll0; P.vel = 0; }
      if ((P.start >= 0 || P.startZone || P.cal) && dur < PANEL_TAP_MAX_MS) { panel_tap(c, ms); what = P.gesture ? "tap (wobbled)" : "tap"; }
      else what = P.start < 0 ? "tap off the buttons" : "held too long for a press";
    }
    if (P.gesture == 1 && ms - P.lt > 80) P.vel = 0;   // stopped before letting go: no fling
    P.down = -1; P.lastUp = ms;
  }
  // the chip's own click report (it comes with the release, or just after): a tap the puck's sampling did not see as one
  // the chip's own click report presses only for a touch the puck itself saw with TOUCH_MIN_SAMPLES reads (one its own tap
  // rules turned down - a wobble or a slow press); never for a touch the puck did not see (phantom presses, 2026-10-21)
  if (click && !P.pressed && !ts.used && ts.samples >= TOUCH_MIN_SAMPLES && ts.maxMove <= 40 && (P.start >= 0 || P.startZone || P.cal)) {
    panel_tap(c, ms);
    if (!what) snprintf(ts.trace, sizeof ts.trace, "touch (panel): the chip's click -> %s", P.start >= 0 ? kPanelBtn[P.start].label : "arrow");
  }
  if (what) {
    int b = P.start;
    snprintf(ts.trace, sizeof ts.trace, "touch (panel): %s - %u ms; %d samples; moved %d px; %d/%d -> %d/%d%s%s", what, (unsigned)(ms - ts.tDown), ts.samples,
             ts.maxMove, ts.x0, ts.y0, ts.lx, ts.ly, b >= 0 ? "; on " : (P.startZone ? "; on the arrow" : ""), b >= 0 ? (b == PB_RCOLOUR ? "the ring colour" : kPanelBtn[b].label) : "");
  }
  return EV_NONE;
}
// the panel's scroll fling, the late answer, the ring colour following the arm, the thermal camera's page, the console
inline void panel_step(Core& c, float dt, uint32_t ms) {
  PanelState& P = c.panel; ArmLink& L = c.arm;
  if (L.test.page >= 0) { panel_go(c, L.test.page ? PAGE_PANEL : PAGE_MAIN, ms, "console"); L.test.page = -1; }
  if (L.test.press >= 0) { panel_press(c, L.test.press, ms); L.test.press = -1; }
  if (P.scrollTo >= 0) {   // the arrows: glide there
    P.scroll += (P.scrollTo - P.scroll) * (1 - expf(-dt / 0.08f));
    if (fabsf(P.scrollTo - P.scroll) < 0.5f) { P.scroll = P.scrollTo; P.scrollTo = -1; }
  }
  if (!c.ts.down && P.vel != 0) {
    P.scroll += P.vel * dt; P.vel *= expf(-dt / 0.30f);
    if (P.scroll < 0) { P.scroll = 0; P.vel = 0; }
    if (P.scroll > PANEL_SCROLL_MAX) { P.scroll = PANEL_SCROLL_MAX; P.vel = 0; }
    if (fabsf(P.vel) < 8) P.vel = 0;
  }
  PanelPress& p = P.last;
  if (p.btn >= 0 && !p.noArm && p.res < 0 && !p.late && panel_age(ms, p.ms) >= PANEL_ANSWER_MS) {
    p.late = true; P.unanswered++;
    snprintf(P.log, sizeof P.log, "panel: %s - no answer from the arm in %u ms (an arm without $C, or the link)", kPanelBtn[p.btn].label, (unsigned)PANEL_ANSWER_MS);
  }
  // the colour row shows the ring's colour as the arm has it, unless it was stepped here a moment ago
  // ... only when the arm's colour CHANGES ($S ringi; repulsor / cyan are not in the row: kept), and not over a step made here
  // a moment ago (round 5: re-reading it every frame put the previous colour back for a few seconds)
  if (L.up(ms) && L.st.ringi != P.armRingi) {
    P.armRingi = L.st.ringi;
    int k = ring_pick_of(L.st.ringi);
    if (k >= 0 && (!P.pickMs || panel_age(ms, P.pickMs) > 1500)) P.pick = (int8_t)k;
  }
  if (!L.anyUp(ms)) P.lidarOn = 0;   // the arm gone: the LiDAR's state with it
  // the calibration screen: no answer in time; "SAVED" shown, then the next step
  if (P.cal && P.cal <= CAL_STEPS) {
    if (P.calSt == 1 && panel_age(ms, P.calMs) > CAL_TIMEOUT_MS) { P.calSt = 4; snprintf(P.log, sizeof P.log, "panel: calibrate step %u - no answer from the arm", (unsigned)P.cal); }
    if (P.calSt == 2 && panel_age(ms, P.calMs) > CAL_SAVED_MS) { P.cal++; P.calSt = 0; }
  }
  // THERMAL pressed here: once the camera is on, its page (the main page shows it); off again -> back to the panel
  if (P.thermalWait) {
    if (c.thermal && P.page == PAGE_PANEL) { panel_go(c, PAGE_MAIN, ms, "the thermal camera is on"); P.thermalReturn = true; P.thermalWait = 0; }
    else if (panel_age(ms, P.thermalWait) > 8000) P.thermalWait = 0;
  }
  if (P.thermalReturn && !c.thermal) { P.thermalReturn = false; if (P.page == PAGE_MAIN) panel_go(c, PAGE_PANEL, ms, "the thermal camera is off"); }
}
// what each button shows, from the arm's $G (else its $S); avail = what the mode switch allows (the arm's panel.cpp rules)
inline PanelView panel_view(const Core& c, uint32_t ms) {
  PanelView v; const ArmLink& L = c.arm; const PanelState& P = c.panel;
  v.scroll = P.scroll; v.down = P.down; v.last = P.last; v.t = ms; v.pick = P.pick;
  v.touching = c.ts.down && !c.ts.used; v.tx = c.ts.lx; v.ty = c.ts.ly;
  v.cal = P.cal; v.calSt = P.calSt; v.calSaved = P.calSaved; v.calMs = P.calMs;
  if (L.liveUp(ms)) for (int f = 0; f < 5; f++) { v.curl[f] = L.live.curl[f]; v.fmv[f] = L.live.fmv[f]; }
  v.thumb = cal_thumb_present(L.liveUp(ms), L.live.fmv[0]);
  uint8_t* s = v.state;
  const bool live = L.liveUp(ms), st = L.up(ms);
  const glink::Live& g = L.live;
  uint8_t plates = live ? g.plates : (st ? L.st.plates : 0);
  if (plates == 1) s[PB_OPEN] = 1; else if (plates == 3) s[PB_OPEN] = 2;
  if (plates == 4) s[PB_CLOSE] = 2;
  static const uint8_t kServos[4] = {0x01, 0x02, 0x0C, 0x30};   // plate 1, 2, 3 (A + B), the hand plate (A + B)
  for (int p = 0; p < 4; p++) {
    uint8_t on = 0;
    if (live) {
      for (int k = 0; k < glink::SERVOS; k++) if ((kServos[p] >> k & 1) && g.servo[k] >= 500) on = 1;
      if (g.drv & kServos[p]) on = 2;
    } else if (st) on = plates == 1 ? 1 : (plates == 3 || plates == 4 ? 2 : 0);
    s[PB_P1 + p] = on;
  }
  uint8_t lasers = live ? g.lasers : (st ? L.st.laser : 0);
  s[PB_RED] = lasers & 1; s[PB_GREEN] = (lasers >> 1) & 1;
  uint8_t groups = panel_groups(c, ms);
  s[PB_PALM20] = groups & 1; s[PB_PALM10] = (groups >> 1) & 1; s[PB_FINGERS] = (groups >> 2) & 1; s[PB_LEDSON] = (groups & 7) == 7;
  s[PB_LIDAR] = P.lidarOn;
  v.lidar = live ? g.lidar : (st ? L.st.lidar : 0);
  s[PB_THERMAL] = c.thermal;
  s[PB_VIBRATE] = live && (g.outs & 1);
  uint8_t lvl = live ? g.ringlvl : (st ? L.st.ringlvl : 0);
  s[PB_RCOLOUR] = lvl > 0; s[PB_RINGON] = lvl > 0;
  s[PB_DIM] = lvl == 1; s[PB_BRIGHT] = lvl >= 10;   // round 6: blue at the ring's dimmest / brightest (it never dims to off)
  s[PB_DISARM] = live ? (g.outs >> 4) & 1 : (st && L.st.armed);
  // the mode switch: 0 SAFE 1 SHOW 2 FULL (unknown = no arm: nothing lit); a thing that is on can always be turned off
  int mode = live ? g.mode : (st ? L.st.mode : -1);
  for (int b = 0; b < PB_COUNT; b++) {
    bool offable = s[b] && b != PB_RCOLOUR && b != PB_RINGON && b != PB_DIM && b != PB_BRIGHT && b != PB_LEDSON && b != PB_OPEN && b != PB_DISARM && !(b >= PB_P1 && b <= PB_HAND);   // the plates always need SHOW
    v.avail[b] = mode >= 0 && (mode >= kPanelBtn[b].need || offable);
  }
  return v;
}

inline Event core_touch(Core& c, bool now, int x, int y, uint32_t ms, uint8_t chip = 0) {
  TouchState& ts = c.ts;
  Event e = EV_NONE;
  if (now && (x < 0 || x > 239 || y < 0 || y > 239)) { ts.junk++; now = ts.down; x = ts.lx; y = ts.ly; }   // junk: hold the last good point
  const bool click = chip == CHIP_CLICK && chip != ts.chipPrev;   // the chip's click report, on the sample it appears
  if (c.panel.page == PAGE_PANEL) { ts.chipPrev = chip; return panel_touch(c, now, x, y, ms, click); }   // no double taps there: a tap is a press
  // the chip's reports, on the sample its gesture register CHANGES to them (a stale register never fires twice) - and only
  // for a touch the puck saw with TOUCH_MIN_SAMPLES reads (the one going on, or the one that just ended)
  const bool chipReal = ts.down ? ts.samples >= TOUCH_MIN_SAMPLES : (ms - ts.lastUp < 300 && ts.lastSamples >= TOUCH_MIN_SAMPLES);
  if (chip != ts.chipPrev && (chip == CHIP_DOUBLE_CLICK || chip == CHIP_CLICK) && !chipReal) {
    snprintf(ts.log, sizeof ts.log, "touch: the chip's %s ignored - no touch of %d+ reads with it (noise)", chip == CHIP_CLICK ? "click" : "double click", TOUCH_MIN_SAMPLES);
  } else if (chip != ts.chipPrev) {
    if (chip == CHIP_DOUBLE_CLICK) { ts.doublesChip++; Event t = touch_toggle(c, ms, "chip"); if (t) e = t; ts.lastClick = 0; }
    else if (chip == CHIP_CLICK) {
      ts.clicks++;
      if (c.thermal) {   // the thermal page: the chip's click is a tap too (one turns the camera off)
        if (!ts.down && !(ms - ts.lastUp < 300 && (ts.acted || ts.moved || ts.longFired))) { ts.acted = true; panel_press(c, PB_THERMAL, ms); snprintf(ts.log, sizeof ts.log, "touch: the chip's click on the thermal page -> camera off"); }
      } else if (ts.lastClick && ms - ts.lastClick < TOUCH_DOUBLE_MS) { ts.lastClick = 0; ts.doublesClicks++; Event t = touch_toggle(c, ms, "clicks"); if (t) e = t; }
      else ts.lastClick = ms;
    }
  }
  ts.chipPrev = chip;
  if (now && !ts.down) {
    ts.down = true; ts.x0 = ts.lx = x; ts.y0 = ts.ly = y; ts.tDown = ms; ts.moved = false; ts.longFired = false; ts.maxMove = 0; ts.samples = 1;
    ts.used = false; ts.acted = false;
    // on the left or right side, out from the middle: maybe a swipe into the panel (the chip barely reports the rim itself)
    int dx = x - 120, dy = y - 120;
    // round 4 (Toby: "a fast swipe from right to left ... should put it in the other screen, left to right puts it back"):
    // only the right side, only leftwards
    ts.edge = dx > 0 && dx * dx + dy * dy >= PANEL_EDGE_R * PANEL_EDGE_R && dx * 5 >= abs(dy) * 4 ? 1 : 0;
    ts.side = -1;   // toward the middle = leftwards
  } else if (now && ts.down) {
    ts.samples++;
    int mv = abs(x - ts.x0) + abs(y - ts.y0);
    if (mv > ts.maxMove) ts.maxMove = mv;
    if (mv > TOUCH_TAP_SLOP_PX) ts.moved = true;
    if (ts.edge == 1) {   // undecided: toward the middle quickly = the panel; anything else = an orbit after all (caught up)
      int in = (x - ts.x0) * ts.side, ady = abs(y - ts.y0);
      if (in >= PANEL_SWIPE_PX && in * 4 >= ady * 5 && ms - ts.tDown <= PANEL_SWIPE_MS) { ts.edge = 0; panel_go(c, PAGE_PANEL, ms, "swipe from the side"); }
      else if (ms - ts.tDown > PANEL_SWIPE_MS || (ts.moved && (in < 0 || ady > in + 8))) {
        ts.edge = 0;
        if (ts.moved && !c.thermal) orbit(c.cam, (float)(x - ts.x0), (float)(y - ts.y0));
      }
    } else if (!ts.used && ts.moved && !c.thermal) orbit(c.cam, (float)(x - ts.lx), (float)(y - ts.ly));
    ts.lx = x; ts.ly = y;
    if (!ts.used && !ts.moved && !ts.longFired && ms - ts.tDown > LONG_PRESS_MS) {
      ts.longFired = true; ts.tapPending = false;
      if (!c.thermal) { cam_home(c.cam); e = EV_HOME; }
      snprintf(ts.log, sizeof ts.log, "touch: hold -> home");
    }
  } else if (!now && ts.down) {
    ts.down = false; ts.lastUp = ms; ts.lastSamples = ts.samples;
    uint32_t dur = ms - ts.tDown;
    const char* what = ts.used ? "swipe into the panel" : (ts.longFired ? "hold" : (ts.moved ? "drag" : (dur < TOUCH_TAP_MAX_MS ? "tap" : "too long for a tap")));
    if (ts.used) {}
    else if (ts.samples < TOUCH_MIN_SAMPLES) {   // one read: noise, never a tap
      ts.noise++;
      snprintf(ts.trace, sizeof ts.trace, "touch: ignored - %d sample, %u ms (noise) at %d,%d", ts.samples, (unsigned)dur, ts.x0, ts.y0);
      return e;
    }
    else if (c.thermal && !ts.moved && !ts.longFired && dur < TOUCH_TAP_MAX_MS) {   // the thermal page: one tap turns the camera off (Toby, 2026-10-20)
      ts.acted = true; panel_press(c, PB_THERMAL, ms); what = "tap on the thermal page -> camera off"; ts.tapPending = false;
    } else if (!ts.moved && !ts.longFired && dur < TOUCH_TAP_MAX_MS) {
      ts.taps++;
      if (ts.tapPending && ms - ts.lastTap < TOUCH_DOUBLE_MS) {
        ts.tapPending = false; ts.doublesOwn++; what = "tap, the second";
        Event t = touch_toggle(c, ms, "taps"); if (t) e = t;
      } else { ts.tapPending = true; ts.lastTap = ms; }
    }
    snprintf(ts.trace, sizeof ts.trace, "touch: %s - %u ms; %d samples; moved %d px; %d/%d -> %d/%d", what, (unsigned)dur, ts.samples, ts.maxMove, ts.x0, ts.y0, ts.lx, ts.ly);
  }
  if (ts.tapPending && !ts.down && ms - ts.lastTap >= TOUCH_DOUBLE_MS) ts.tapPending = false;   // a single tap: nothing
  if (ts.lastClick && ms - ts.lastClick >= TOUCH_DOUBLE_MS) ts.lastClick = 0;
  return e;
}

// ---------------------------------------------------------------- frame
inline void demo_lights(const Core& c, Lights& li) {
  li = Lights();
  memcpy(li.ring, kRingRGB[1], 3); li.ringK = 1.0f; li.ringIdx = 1;
  li.green = kin::plate_frac(c.inputs, 0) > 0.9f;
  float m = 0; for (int f = 1; f < 5; f++) m += c.inputs.flex[f]; m /= 4;
  if (m > 0.7f) { li.red = true; li.lidar = 1500; }
}

static const float kRestFlex[5] = {0.10f, 0.08f, 0.08f, 0.10f, 0.12f};
static const float kIdentity9[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// LIVE: the model eases toward the last $G (LIVE_TAU_S); frozen while frames are late (LIVE_STALE_MS .. LIVE_LOST_MS);
// after LIVE_LOST_MS the wrist and fingers glide to neutral and the lights go dark, the plates and the arm's
// orientation stay. The camera is never touched here.
inline void step_live(Core& c, float dt, uint32_t ms) {
  ArmLink& L = c.arm;
  const glink::Live& g = L.live;
  const bool lost = !L.liveUp(ms), fresh = L.liveFresh(ms);
  float k = fresh ? 1 - expf(-dt / LIVE_TAU_S) : (lost ? 1 - expf(-dt / LOST_TAU_S) : 0.0f);
  float tq[4] = {1, 0, 0, 0};
  if (!lost) live_wrist_quat(g, tq);
  quat_ease(c.qWrist, tq, k);
  live_wrist_matrix(c.qWrist, c.inputs.wrist);
#if ARM_POSE_WORLD
  if (fresh && (g.imu & 1)) {   // the forearm's own orientation (lost: it stays where it was)
    float fa[4] = {g.fq[0] / 1e4f, g.fq[1] / 1e4f, g.fq[2] / 1e4f, g.fq[3] / 1e4f};
    float n = sqrtf(fa[0] * fa[0] + fa[1] * fa[1] + fa[2] * fa[2] + fa[3] * fa[3]);
    if (n > 0.5f) { for (int i = 0; i < 4; i++) fa[i] /= n; quat_ease(c.qArm, fa, k); }
  }
  live_wrist_matrix(c.qArm, c.inputs.arm);
#endif
  for (int f = 0; f < 5; f++) {
    float target = (lost || g.curl[f] < 0) ? kRestFlex[f] : g.curl[f] / 1000.0f;
    c.inputs.flex[f] += (target - c.inputs.flex[f]) * k;
  }
  if (fresh) {
    float tgt[model::NFOURBAR];
    live_plate_targets(g, c.plate, tgt);
    for (int j = 0; j < model::NFOURBAR; j++) c.plate[j] += (tgt[j] - c.plate[j]) * k;
  }
  for (int j = 0; j < model::NFOURBAR; j++) {
    const model::FourbarDef& d = model::kFourbars[j];
    c.inputs.cmd[j] = d.cmd_closed + c.plate[j] * (d.cmd_open - d.cmd_closed);
  }
  if (lost) c.lights = Lights(); else lights_from_live(g, c.lights);
}

inline void core_step(Core& c, float dt, uint32_t ms = 0) {
  ArmLink& L = c.arm;
  fake_tick(L, ms);
  if (L.test.creases >= 0) { crease_level() = (uint8_t)L.test.creases; L.test.creases = -1; }
  if (L.test.lines >= 0) { line_mode() = (uint8_t)L.test.lines; L.test.lines = -1; }
  if (L.test.hud > 0) { c.hudDir = (uint8_t)(L.test.hud > 3 ? 0 : L.test.hud); L.test.hud = -1; }
  if (L.test.thermal >= -1 && L.test.thermal != -2) { c.thermalForce = L.test.thermal; L.test.thermal = -2; }
  // the thermal page: while the arm says the thermal camera is deployed ($S flags bit 6) and the link is up - or the
  // console forces it
  c.thermal = c.thermalForce >= 0 ? c.thermalForce == 1 : (L.up(ms) && (L.st.flags & (1 << glink::FLAG_THERMAL)));
  panel_step(c, dt, ms);
  if (core_is_live(c)) {
    step_live(c, dt, ms);
  } else if (c.live) {
    c.lights = Lights();
    if (L.have) lights_from_status(L.st, ms, c.lights);
  } else if (L.have) {   // ARM: the $S frames drive the plates and the lights
    lights_from_status(L.st, ms, c.lights);
    bool greenWanted = c.lights.green;
    bool hold1 = L.test.plate1 == 1 || greenWanted;
    if (L.cas.dir) L.p1Solo = false;
    else if (hold1 && L.cas.frac[0] < 0.999f) { L.cas.solo(+1); L.p1Solo = true; }
    else if (!hold1 && L.p1Solo && L.st.plates != 1 && L.st.plates != 3) { L.cas.solo(-1); if (L.cas.frac[0] < 0.001f) L.p1Solo = false; }
    if (!hold1 && L.test.plate1 == 0 && !L.cas.dir && L.cas.frac[0] > 0.001f && L.st.plates != 1) L.cas.solo(-1);
    L.cas.step(dt * 1000.0f);
    c.lights.green = greenWanted && L.cas.frac[0] >= 0.95f;
    for (int j = 0; j < model::NFOURBAR; j++) {
      const model::FourbarDef& d = model::kFourbars[j];
      c.inputs.cmd[j] = d.cmd_closed + L.cas.frac[j] * (d.cmd_open - d.cmd_closed);
      c.plate[j] = L.cas.frac[j];
    }
    bool fist = L.test.fist >= 0 ? L.test.fist == 1 : c.lights.red;
    c.fistK += ((fist ? 1.0f : 0.0f) - c.fistK) * (1 - expf(-dt / 0.12f));
    static const float kFist[5] = {0.75f, 0.95f, 0.95f, 0.95f, 0.95f};
    for (int f = 0; f < 5; f++) c.inputs.flex[f] = kRestFlex[f] + (kFist[f] - kRestFlex[f]) * c.fistK;
    memcpy(c.inputs.wrist, kIdentity9, sizeof kIdentity9);
  } else {   // no arm yet: the hand still, in its neutral pose, nothing lit (no demo animation - Toby, 2026-10-08)
    kin::rest_inputs(c.inputs);
    for (int f = 0; f < 5; f++) c.inputs.flex[f] = kRestFlex[f];
    c.lights = Lights();
    for (int j = 0; j < model::NFOURBAR; j++) c.plate[j] = 0;
  }
  lights_apply_tests(L.test, c.lights);
  if (c.lights.lidar) { c.lidarHeld = c.lights.lidar; c.lidarMs = ms; }
  else if (c.lights.red && c.lidarHeld && ms - c.lidarMs < 1500) c.lights.lidar = c.lidarHeld;
  else c.lidarHeld = 0;
  const bool cascade = core_is_live(c) ? (L.liveUp(ms) && (L.live.outs & 4)) : (L.up(ms) && (L.st.plates == 3 || L.st.plates == 4));
  update_camera(c.cam, c.inputs, dt, c.lights.red, c.lights.green, cascade);
  // every plate glows red while it moves (the hand plate since round 10, plates 1-3 since 2026-10-10 - Toby): bright
  // with its speed, fading over ~0.25 s after it stops
  for (int j = 0; j < model::NFOURBAR; j++) {
    const float pf = kin::plate_frac(c.inputs, j);
    if (c.platePrev[j] >= 0 && dt > 0) {
      float speed = fabsf(pf - c.platePrev[j]) / dt;            // travels per second; a 0.4 s lift is ~2.5
      float want = speed > 0.15f ? fminf(1.0f, 0.45f + speed * 0.25f) : 0.0f;
      c.plateGlow[j] = want > c.plateGlow[j] ? want : c.plateGlow[j] * expf(-dt / 0.25f);
    }
    c.platePrev[j] = pf;
  }
}

// HUD values: what the arm's $G (LIVE) or $S (ARM) says, or the demo's stand-ins.
inline hud::Data core_hud(const Core& c, uint32_t ms) {
  hud::Data d;
  const ArmLink& L = c.arm;
  d.dir = c.hudDir; d.t = ms;
  if (core_is_live(c)) {
    const glink::Live& g = L.live;
    d.link = L.liveFresh(ms) ? hud::LINK_UP : hud::LINK_LOST;
    d.mode = g.mode;
    d.battery = g.bstate == 6 ? -1.0f : battery_frac(g.mv);
    d.mv = g.mv;
    d.batWarn = g.bstate >= 2 && g.bstate <= 5; d.batCrit = g.bstate == 4;
    d.plates = g.plates;
    d.rate = L.liveRate;
    d.armed = (g.outs & 16) != 0;
  } else if (L.have && !c.live) {
    const glink::Status& s = L.st;
    d.link = L.up(ms) ? hud::LINK_UP : hud::LINK_LOST;
    d.mode = s.mode;
    d.battery = s.bstate == 6 ? -1.0f : battery_frac(s.mv);
    d.mv = s.mv;
    d.batWarn = s.bstate >= 2 && s.bstate <= 5; d.batCrit = s.bstate == 4;
    d.plates = s.plates;
    d.armed = s.armed != 0;
    if (L.p1Solo) d.plates = L.cas.frac[0] >= 0.999f ? 6 : 7;
    d.rate = L.goodRate * 2;   // $S at 10 Hz: shown on the same 0..20 scale as $G
  } else {
    d.link = c.live ? hud::LINK_PACKET : hud::LINK_NONE;
    d.mode = -1;
    d.battery = -1.0f;   // no arm: unknown, never a made-up level
    d.plates = 2;
  }
  if (L.mismatch(ms)) { d.mismatch = (uint8_t)(L.theirVersion < 1 ? 1 : (L.theirVersion > 255 ? 255 : L.theirVersion)); d.link = hud::LINK_LOST; }
  bool warn = false; uint32_t hold = 0;
  d.event = event_toast(c.lastEvent, hold, &warn); d.eventWarn = warn;
  d.eventAge = c.lastEvent ? ms - c.lastEventMs : (1u << 30);
  if (c.actMs && (!c.lastEvent || (int32_t)(c.actMs - c.lastEventMs) >= 0)) {   // the arm's $E is the newer word
    if (c.actRes == glink::RES_DONE) { d.event = c.actWord; d.eventWarn = c.actWarn; d.eventAge = ms - c.actMs; }
    else if (ms - c.actMs < 3000) {
      d.blocked = c.actWord; d.blockedWarn = c.actRes == glink::RES_REFUSED || c.actRes == glink::RES_BATTERY; d.blockedAge = ms - c.actMs; d.event = "";
      d.blockedWhy = c.actDrone ? "DRONE" : (c.actRes == glink::RES_BATTERY ? "BATTERY" : "");
    }
  }
  d.batMark = battery_frac(glink::POWER_FLOOR_MV);   // the arm's floor, on the bar's own mV mapping (never hard-coded here)
  d.debug = L.debug;
  d.goodRate = L.goodRate; d.badRate = L.badRate;
  uint32_t last = L.lastMs > L.liveMs ? L.lastMs : L.liveMs;
  d.ageMs = (L.have || L.haveLive) ? ms - last : 0;
  d.uartGood = L.uart.good; d.uartBad = L.uart.dropped(); d.usbGood = L.usb.good; d.usbBad = L.usb.dropped();
  const Lights& li = c.lights;
  d.ringOn = li.ringK > 0.01f;
  if (d.ringOn) {
    for (int k = 0; k < 3; k++) d.ringRGB[k] = (uint8_t)(li.ring[k] * (0.45f + 0.55f * li.ringK));
    d.ringLvl = (uint8_t)(li.ringK * 10 + 0.5f);
    d.ringName = li.ringIdx >= 0 && li.ringIdx < 9 ? kRingName[li.ringIdx] : "WHITE";
  }
  d.pucks = li.pucks; d.red = li.red; d.green = li.green; d.strip = li.strip; d.accent = li.accent;
  if (li.strip) memcpy(d.stripRGB, li.stripRGB, 3);
  d.lidar = li.red ? li.lidar : 0;   // the range only while the red laser fires
  return d;
}
inline hud::Look core_look() { return {kStyle.hudLevel, {kStyle.hud[0], kStyle.hud[1], kStyle.hud[2]}, {kStyle.accent[0], kStyle.accent[1], kStyle.accent[2]}}; }

// Pose -> camera (pivot = the hand's centre where the arm put it) -> visible triangles. In three steps so the puck can
// build the two halves of the parts on its two cores (app.cpp): begin (pose + camera), draw_scene per half, end (lights).
struct FrameSetup { rnd::Camera cam; int lod; bool shut; SceneTone tone; };
inline FrameSetup core_begin_frame(Core& c) {
  kin::solve(c.inputs, c.world);
  float pivot[3], mid[3], Rc[9];
  hand_pivot(c.world, pivot); arm_mid(c.world, mid); cam_axes(c.cam, Rc);
  float off[3] = {0, 0, 0};
  bool zoomed = c.cam.zoom > 0.001f || c.cam.zoomTarget > 0.001f;   // the outline is only needed while zoomed out or zooming
  float want = zoomed || c.cam.fullDist <= 0 ? frame_full(Rc, mid, c.world, off) : c.cam.fullDist;
  if (!zoomed && c.cam.fullDist > 0) memcpy(off, c.cam.fullOff, sizeof off);
  bool first = c.cam.fullDist <= 0;
  c.cam.fullDist = first ? want : c.cam.fullDist + (want - c.cam.fullDist) * 0.35f;   // eased: no pumping
  for (int i = 0; i < 3; i++) c.cam.fullOff[i] = first ? off[i] : c.cam.fullOff[i] + (off[i] - c.cam.fullOff[i]) * 0.35f;
  for (int i = 0; i < 3; i++) mid[i] += c.cam.fullOff[i];
  FrameSetup f{make_camera(c.cam, pivot, mid), lod_for(c.cam), true, SceneTone()};
  f.tone.grey = c.grey; f.tone.bodyTone = PUCK_BODY_TONE;
  for (int j = 0; j < model::NFOURBAR; j++) f.tone.plateGlow[j] = c.grey ? 0 : c.plateGlow[j];
  for (int j = 0; j < model::NFOURBAR; j++) if (kin::plate_frac(c.inputs, j) > 0.02f) f.shut = false;
  return f;
}
// the calibration's arm steps: the same scene from a fixed side view - the arm's forward direction to the screen's right,
// up up - so out / down / up and the wrist's line read at a glance; the whole arm fitted to its pose (eased), then scaled
// down to CAL_ARM_SCALE and centred at CAL_ARM_CY, in the space the step's words leave
inline FrameSetup core_begin_cal_frame(Core& c) {
  kin::solve(c.inputs, c.world);
  static CamState cc;
  static const float kSide[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};   // camera x = the arm's forward (kFrameA), y = up = kFrameD (the back of the hand, up when palm down), z = kFrameL
  memcpy(cc.R, kSide, sizeof kSide); cc.zoom = cc.zoomTarget = 1;
  float pivot[3], mid[3], Rc[9], off[3] = {0, 0, 0};
  hand_pivot(c.world, pivot); arm_mid(c.world, mid); cam_axes(cc, Rc);
  float want = frame_full(Rc, mid, c.world, off);
  bool first = cc.fullDist <= 0;
  cc.fullDist = first ? want : cc.fullDist + (want - cc.fullDist) * 0.25f;
  for (int i = 0; i < 3; i++) cc.fullOff[i] = first ? off[i] : cc.fullOff[i] + (off[i] - cc.fullOff[i]) * 0.25f;
  for (int i = 0; i < 3; i++) mid[i] += cc.fullOff[i];
  FrameSetup f{make_camera(cc, pivot, mid), 1, true, SceneTone()};
  f.cam.f *= CAL_ARM_SCALE; f.cam.cy = CAL_ARM_CY;
  f.tone.grey = c.grey; f.tone.bodyTone = PUCK_BODY_TONE;
  for (int j = 0; j < model::NFOURBAR; j++) if (kin::plate_frac(c.inputs, j) > 0.02f) f.shut = false;
  return f;
}
inline bool core_cal_arm_view(const Core& c) { return c.panel.page == PAGE_PANEL && c.panel.cal >= CAL_ARM_FIRST && c.panel.cal <= CAL_STEPS; }
inline void core_end_frame(Core& c, const FrameSetup& f) { build_fx(c.fx, f.cam, c.world, c.lights); }
inline void core_build_frame(Core& c, rnd::Frame& fr) {
  FrameSetup f = core_begin_frame(c);
  draw_scene(fr, f.cam, c.world, f.lod, f.shut, 0, 1, f.tone);
  core_end_frame(c, f);
}

}  // namespace app
