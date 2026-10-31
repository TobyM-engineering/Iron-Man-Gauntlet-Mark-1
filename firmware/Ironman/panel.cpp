// panel.cpp - the puck's control panel ($C, 2026-10-20). One press = one command, done by the same functions as the
// gestures (gesturemap.cpp) under the same rules: the mode switch first (SHOW for the plates, the lights, the camera, the
// ring and the vibrator; FULL for the lasers), then whatever the function refuses itself (the battery, the heat model,
// a move running, the drone, a rest time). Turning something OFF is never blocked. Every command answers the puck with
// exactly one $E for its act; the log says what happened and why.
#include "panel.h"
#include "config.h"
#include "s3link.h"
#include "modesw.h"
#include "servos.h"
#include "pucks.h"
#include "outputs.h"
#include "thermal.h"
#include "palm.h"
#include "fly.h"
#include "calib.h"
#include "crsf.h"
#include <GauntletBenchIO.h>

static uint32_t presses = 0, refused = 0;
static uint32_t updStopAt = 0, updHb = 0;   // (2026-10-21) the puck's update: when the RC frames stopped for it (0 = not now), heartbeats seen then
uint32_t panelPresses() { return presses; }
uint32_t panelRefused() { return refused; }
void panelPoll() {   // the puck's update: the RC frames back on with its heartbeat (after the quiet minimum) or after the maximum
  if (!updStopAt) return;
  uint32_t now = millis(), hb = s3linkHeartbeats();
  if (now - updStopAt < PUCK_UPDATE_QUIET_MIN_MS) { updHb = hb; return; }
  bool back = hb != updHb, late = now - updStopAt >= PUCK_UPDATE_QUIET_MAX_MS;
  if (!back && !late) return;
  updStopAt = 0;
  crsfStream(true);
  Bench.printf("panel: puck update over (%s) - the RC frames are back on\n", back ? "its heartbeat is back" : "5 min passed");
}
bool panelPuckUpdating() { return updStopAt != 0; }
String   panelStatusLine() { return "panel: " + String(presses) + " button presses from the puck, " + String(refused) + " blocked or refused"; }

static bool wantOn(uint16_t v, bool isOn) { return v == 2 ? !isOn : v != 0; }   // 1 on / 0 off / 2 toggle
static void answer(uint8_t act, uint8_t res, uint8_t need) { if (res != glink::RES_DONE) refused++; s3linkAct(act, res, need); }
static void done(uint8_t act, bool ok, uint8_t need) { answer(act, ok ? glink::RES_DONE : glink::RES_REFUSED, need); }
static bool blockedBelow(uint8_t act, uint8_t need) {   // the mode switch: true = BLOCKED and answered
  if (need >= 2 ? modeAllowsFull() : modeAllowsShow()) return false;
  answer(act, glink::RES_BLOCKED, need);
  return true;
}

static void laser(uint8_t act, uint16_t a) {
  const bool red = act == glink::ACT_LASER_RED;
  const bool on = wantOn(a, red ? laserRedWanted() : laserGreenWanted());
  if (!on) { if (red) laserRedSet(false); else laserGreenSet(false); done(act, true, 2); return; }
  if (blockedBelow(act, 2)) return;
  if (lasersFlightHeld()) { answer(act, glink::RES_BLOCKED, 2); return; }   // the drone flies or was just disarmed
  done(act, red ? laserRedSet(true) : laserGreenSet(true), 2);
}

static void lights(uint8_t act, uint8_t g0, uint8_t g1, uint16_t v) {   // LED groups g0..g1 (0-based)
  bool any = false; for (uint8_t g = g0; g <= g1; g++) if (puckGroupIsOn(g)) any = true;
  if (!wantOn(v, any)) { for (uint8_t g = g0; g <= g1; g++) puckGroupOff(g); done(act, true, 1); return; }
  if (blockedBelow(act, 1)) return;
  bool all = true, battery = false;
  for (uint8_t g = g0; g <= g1; g++) if (!puckGroupOn(g, "the puck's panel")) { all = false; if (pucksRefusedForBattery()) battery = true; }
  answer(act, all ? glink::RES_DONE : (battery ? glink::RES_BATTERY : glink::RES_REFUSED), 1);   // (2026-10-21) BATTERY: the puck shows it was the battery
}

void panelCommand(const glink::Command& c) {
  presses++;
  const uint8_t act = c.act;
  Bench.printf("panel: %s (a %u, b %u) pressed on the puck\n", glink::actName(act), c.a, c.b);
  switch (act) {
    case glink::ACT_CASCADE: {
      bool open = c.a == 2 ? servosPlatesState() != PlatesState::Open : c.a != 0;
      if (blockedBelow(act, 1)) return;
      if (servosBusy()) { done(act, false, 1); return; }
      if (open) servosOpen(); else servosClose();
      done(act, servosBusy(), 1);                          // a sequence started = done; nothing to move or refused = the log says why
      return;
    }
    case glink::ACT_PLATE: {
      if (c.a < 1 || c.a > PLATE_GROUPS) { done(act, false, 1); return; }
      bool open = wantOn(c.b, servosPlateState((uint8_t)c.a) == 1);
      if (blockedBelow(act, 1)) return;
      done(act, servosMovePlateWith((uint8_t)c.a, open), 1);   // (2026-10-21) plate 2 brings the hand plate, plate 3 both
      return;
    }
    case glink::ACT_BURST:
      if (c.a > 2) { done(act, false, 1); return; }
      if (c.a == 0) lights(act, 0, 1, c.b); else lights(act, (uint8_t)(c.a - 1), (uint8_t)(c.a - 1), c.b);
      return;
    case glink::ACT_FINGERS: lights(act, 2, 2, c.a); return;
    case glink::ACT_LASER_RED: case glink::ACT_LASER_GREEN: laser(act, c.a); return;
    case glink::ACT_CAL: calibStart((uint8_t)c.a); return;
    case glink::ACT_PUCK_UPDATE:   // (2026-10-21) the puck restarts into its WiFi updater: the ELRS module goes quiet beside its antenna
      if (flyEnabled()) { Bench.println("panel: puck update - the drone is armed / flipping, the RC frames stay on"); done(act, false, 0); return; }
      crsfStream(false); updStopAt = millis() | 1; updHb = s3linkHeartbeats();
      Bench.printf("panel: puck update - the RC frames stop until the puck's heartbeat is back (or %lu s)\n", (unsigned long)(PUCK_UPDATE_QUIET_MAX_MS / 1000));
      done(act, true, 0); return;   // (2026-10-21) calib.cpp answers the one $E when the step is done (~2.6 s)
    case glink::ACT_LIDAR: {   // (2026-10-20) the LiDAR on its own: the lasers' rules
      const bool on = wantOn(c.a, lidarAloneWanted());
      if (!on) { lidarAloneSet(false); done(act, true, 2); return; }
      if (blockedBelow(act, 2)) return;
      if (lasersFlightHeld()) { answer(act, glink::RES_BLOCKED, 2); return; }
      done(act, lidarAloneSet(true), 2);
      return;
    }
    case glink::ACT_VIBRATE:
      if (blockedBelow(act, 1)) return;
      done(act, vibPulse(c.a ? (uint16_t)(c.a * 10) : 300, "the puck's panel"), 1);
      return;
    case glink::ACT_THERMAL_ON:  thermalOn("the puck's panel"); return;            // reports itself ($E DONE / BLOCKED / REFUSED)
    case glink::ACT_THERMAL_OFF: if (thermalIsOn() || strcmp(thermalStateName(), "off")) thermalOff("the puck's panel"); else done(act, true, 1); return;
    case glink::ACT_RING_ON:
      if (blockedBelow(act, 1)) return;
      if (c.a > 0) {   // (2026-10-20, the puck's colour buttons) a = palette index + 1: on in that colour at its level (off -> the level it had before; the default the first time)
        if (c.a - 1 >= palmColourCount()) { done(act, false, 1); return; }
        palmSet("c=" + String(c.a - 1));
        done(act, true, 1); return;
      }
      Bench.println(palmGesture(false)); done(act, true, 1); return;   // a 0: the gesture's own (off -> on, white <-> red)
    case glink::ACT_RING_OFF:  Bench.println(palmGesture(true)); palmGlowHide(); done(act, true, 1); return;   // (2026-10-21) the SAFE glow too
    case glink::ACT_RING_UP:   if (blockedBelow(act, 1)) return; Bench.println(palmDip(true)); done(act, true, 1); return;
    case glink::ACT_RING_DOWN: if (blockedBelow(act, 1)) return; Bench.println(palmDip(false)); done(act, true, 1); return;
    case glink::ACT_DISARM:    if (flyArmed()) flyDisarm("the puck's panel"); done(act, true, 0); return;
    case glink::ACT_ALL_OFF:   outputsAllOff("the puck's panel"); pucksAllOff("the puck's panel"); done(act, true, 0); return;
    default:                   // ACT_ARM / ACT_FLIP never from a button; anything else unknown
      Bench.printf("panel: %s is not a button the arm takes - refused\n", glink::actName(act));
      done(act, false, 2);
      return;
  }
}
