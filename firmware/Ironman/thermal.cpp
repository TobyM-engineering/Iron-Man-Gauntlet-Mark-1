#include "thermal.h"
#include "config.h"
#include "modesw.h"
#include "servos.h"
#include "power.h"
#include "s3link.h"
#include <GauntletBenchIO.h>
#include <GauntletLink.h>

// The camera HOLDS the hand plate open while it is on (servos.cpp: a held plate never closes - a cascade close leaves it
// up); its plate moves go through servosRequest, so they wait for a running move and keep the plates' order.
enum class Th : uint8_t { Off, Lifting, On };
static Th       st = Th::Off;
static uint32_t since = 0, turnsOn = 0;

const char* thermalStateName() { return st == Th::Off ? "off" : st == Th::Lifting ? "lifting" : "on"; }
bool thermalIsOn() { return st == Th::On; }

bool thermalOn(const char* why) {
  if (st != Th::Off) return true;
  if (!modeAllowsShow()) { s3linkAct(glink::ACT_THERMAL_ON, glink::RES_BLOCKED, 1); return false; }
  const char* r = (powerIdleForced() || powerFaultLatched()) ? powerBlockText() : !servosReady() ? "the PCA9685 is absent" : nullptr;
  if (r) { Bench.printf("thermal: refused - %s (%s)\n", r, why); s3linkAct(glink::ACT_THERMAL_ON, glink::RES_REFUSED, 1); return false; }
  servosHold(THERMAL_PLATE, SERVO_HOLD_THERMAL, true);
  since = millis();
  if (servosPlateOpen(THERMAL_PLATE)) { st = Th::On; turnsOn++; Bench.printf("thermal: ON (%s) - the hand plate is already up, the puck shows the camera\n", why); }
  else { st = Th::Lifting; servosRequest(THERMAL_PLATE, true); Bench.printf("thermal: ON (%s) - the hand plate lifts, then the puck reads the camera\n", why); }
  s3linkAct(glink::ACT_THERMAL_ON, glink::RES_DONE, 1);
  return true;
}

// OFF: the bit clears at once (the puck back to its 3D view), then the hand plate closes - only if nothing else needs
// it open (plate 2 or 3, the red laser).
bool thermalOff(const char* why) {
  if (st == Th::Off) return true;
  if (!modeAllowsShow()) { thermalStop(why); return true; }
  st = Th::Off; servosHold(THERMAL_PLATE, SERVO_HOLD_THERMAL, false);
  Bench.printf("thermal: off (%s) - the puck back to its 3D view, the hand plate closes if nothing else needs it\n", why);
  servosRequest(THERMAL_PLATE, false);
  s3linkAct(glink::ACT_THERMAL_OFF, glink::RES_DONE, 1);
  return true;
}

void thermalToggle(const char* why) { if (st != Th::Off) thermalOff(why); else thermalOn(why); }

// a mode change, goSafe, the battery: the bit off, the hold let go, NO plate move (the plates freeze where they are)
void thermalStop(const char* why) {
  if (st == Th::Off) return;
  st = Th::Off; servosHold(THERMAL_PLATE, SERVO_HOLD_THERMAL, false);
  Bench.printf("thermal: off (%s) - the hand plate stays where it is\n", why);
  s3linkAct(glink::ACT_THERMAL_OFF, glink::RES_DONE, 1);
}

void thermalPoll() {
  switch (st) {
    case Th::Lifting:
      if (servosPlateOpen(THERMAL_PLATE)) { st = Th::On; turnsOn++; Bench.println("thermal: the hand plate is up - camera on, the puck shows it full screen"); }
      else if (millis() - since > THERMAL_LIFT_TIMEOUT_MS) { st = Th::Off; servosHold(THERMAL_PLATE, SERVO_HOLD_THERMAL, false); Bench.printf("thermal: the hand plate did not come up (%s) - camera off\n", servosLastRefusal()); s3linkAct(glink::ACT_THERMAL_OFF, glink::RES_REFUSED, 1); }
      break;
    case Th::On:
      if (!servosPlatesOff() && (servosPlateState(THERMAL_PLATE) == 0 || (!servosBusy() && !servosPlateOpen(THERMAL_PLATE)))) {   // released by a stall, or put down some other way
        st = Th::Off; servosHold(THERMAL_PLATE, SERVO_HOLD_THERMAL, false); Bench.println("thermal: the hand plate is not up any more - camera off"); s3linkAct(glink::ACT_THERMAL_OFF, glink::RES_DONE, 1);
      }
      break;
    default: break;
  }
}

String thermalJson() {
  char b[64]; snprintf(b, sizeof b, "{\"state\":\"%s\",\"on\":%d,\"times\":%lu}", thermalStateName(), st == Th::On ? 1 : 0, (unsigned long)turnsOn);
  return String(b);
}
