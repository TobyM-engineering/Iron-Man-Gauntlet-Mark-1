#include "outputs.h"
#include "config.h"
#include "hal.h"
#include "servos.h"
#include "power.h"
#include "modesw.h"
#include "haptic.h"
#include "fly.h"
#include <GauntletBenchIO.h>
#include <Preferences.h>

static bool     red = false, green = false, vib = false, accent = false;
static uint32_t greenSince = 0, vibUntil = 0, shots = 0;

bool laserRedOn()   { return red; }
bool laserGreenOn() { return green; }
bool vibOn()        { return vib; }
bool accentOn()     { return accent; }
uint32_t outputsLaserShots() { return shots; }

const char* outputsBlockText() {
  static char b[96];
  if (!servosReady()) return "PCA9685 absent";
  if (!modeAllowsShow()) { snprintf(b, sizeof b, "mode %s", modeNowName()); return b; }
  if (powerIdleForced() || powerFaultLatched()) return powerBlockText();   // (2026-10-08) a latched battery FAULT: no lasers, no vibrator
  return "";
}
// The drone flying (armed or flipping) and the LASER_AFTER_FLIGHT_MS after it ends. Called every loop from outputsPoll and
// laserPoll, so the end of a flight is seen within one loop of the disarm.
static bool     wasFlying = false;
static uint32_t flightEndMs = 0;
bool lasersFlightHeld() {
  if (flyEnabled()) { wasFlying = true; return true; }
  if (wasFlying) { wasFlying = false; flightEndMs = millis() | 1; }
  return flightEndMs && millis() - flightEndMs < LASER_AFTER_FLIGHT_MS;
}
static const char* laserBlockText() {   // the lasers need FULL (anything that leaves the arm, Toby 2026-10-07 night) and stand down while the drone flies and for LASER_AFTER_FLIGHT_MS after
  const char* b = outputsBlockText(); if (b[0]) return b;
  static char m[48];
  if (!modeAllowsFull()) { snprintf(m, sizeof m, "mode %s - the lasers need FULL", modeNowName()); return m; }
  if (lasersFlightHeld()) return flyEnabled() ? "the drone is armed" : "the drone was just disarmed (1.5 s)";
  return "";
}
const char* outputsGreenBlockText() { return laserBlockText(); }   // its plate is no longer a refusal: the green laser opens plate 1 itself (below)

static void releaseOe() { if (!pcaDigitalOnCount() && !servosBusy()) servosPower(false); }

static void settleLoad();
void outputsBegin() {
  red = green = vib = accent = false;
  settleLoad();
  halPinMode(PIN_ACCENT, OUTPUT); halDigitalWrite(PIN_ACCENT, LOW); halDriveStrong(PIN_ACCENT);   // three LEDs, 14 mA: the pin's highest drive setting
  hapticBegin();
  Bench.printf("outputs: green laser ch %u (RFP30N06LE gate; %s opens first), red laser TTL ch %u (%s lifts first), vibrator on the DRV2605L, accent LEDs GPIO %u - all off\n",
               PCA_CH_LASER_GREEN, servosPlateName(LASER_GREEN_PLATE), PCA_CH_LASER_RED, servosPlateName(LASER_RED_PLATE), PIN_ACCENT);
}

// The two lasers' sequences (Toby, 2026-10-07 night): ON = the laser's plate opens first (the red laser and the LiDAR
// through the hand plate, the green laser on plate 1), the laser fires once that plate has LANDED open; OFF = the laser
// off, then its plate closes - but only if nothing else needs it open (servos.cpp: plate 2 / 3, the camera, the other
// laser), and never on a mode change or goSafe (closeAfter false: the plates freeze where they are). The plate is HELD
// open while the laser is wanted. A plate that has not landed open after LASER_PLATE_WAIT_MS = the laser gives up.
// A plate that has just been moved open is "open" when its commanded move ends, but it gets there a little later: the
// laser waits settleMs after that (2026-10-16, Toby: "it can't be moving and on"). A plate already open fires at once.
struct Beam { const char* name; uint8_t ch, plate, hold; bool* on; bool want; uint32_t since, openAt; };
// (2026-10-20) the third: the LiDAR on its own (the puck's LIDAR button) - the hand plate lifts, then it measures with no
// laser (lidar.cpp asks lidarAloneOn()); the lasers' rules (FULL, not in flight, an off never blocked). No PCA channel.
static constexpr uint8_t NO_CH = 0xFF;
static bool lidarOn = false;
static Beam beams[3] = { { "red laser", PCA_CH_LASER_RED, LASER_RED_PLATE, SERVO_HOLD_RED, &red, false, 0, 0 },
                         { "green laser", PCA_CH_LASER_GREEN, LASER_GREEN_PLATE, SERVO_HOLD_GREEN, &green, false, 0, 0 },
                         { "LiDAR", NO_CH, LASER_RED_PLATE, SERVO_HOLD_LIDAR, &lidarOn, false, 0, 0 } };
static uint32_t settleMs = LASER_PLATE_SETTLE_MS;
uint32_t laserSettleMs() { return settleMs; }
static void settleLoad() { Preferences p; if (p.begin("ironman", true)) { settleMs = p.getUInt("lset", LASER_PLATE_SETTLE_MS); p.end(); } }
void laserSettleSet(uint32_t ms) {
  settleMs = ms > 2000 ? 2000 : ms;
  Preferences p; if (p.begin("ironman", false)) { if (settleMs == LASER_PLATE_SETTLE_MS) p.remove("lset"); else p.putUInt("lset", settleMs); p.end(); }
  Bench.printf("outputs: a laser fires %u ms after its plate has opened (saved)\n", (unsigned)settleMs);
}
static void fire(Beam& b) {
  *b.on = true; if (b.ch != NO_CH) shots++; if (b.ch == PCA_CH_LASER_GREEN) greenSince = millis();
  if (b.ch != NO_CH) pcaSetDigital(b.ch, true);
  char why[48];
  if (servosPlatesOff()) snprintf(why, sizeof why, "not moved - the plates are off");
  else if (b.openAt) snprintf(why, sizeof why, "open, settled %u ms", (unsigned)settleMs);
  else snprintf(why, sizeof why, "was already open");
  Bench.printf("outputs: %s ON (%s %s)%s\n", b.name, servosPlateName(b.plate), why, b.ch == PCA_CH_LASER_GREEN ? " - 5 min guard" : "");
}
static bool beamSet(Beam& b, bool on, bool closeAfter) {
  if (on) {
    if (*b.on || b.want) return true;
    if (laserBlockText()[0]) { Bench.printf("outputs: %s refused - %s\n", b.name, laserBlockText()); return false; }
    b.want = true; b.since = millis(); b.openAt = 0; servosHold(b.plate, b.hold, true);
    if (servosPlateOpen(b.plate)) fire(b);
    else { Bench.printf("outputs: %s - %s opens first\n", b.name, servosPlateName(b.plate)); servosRequest(b.plate, true); }
    return true;
  }
  bool was = *b.on || b.want;
  b.want = false; servosHold(b.plate, b.hold, false);
  if (*b.on) { *b.on = false; if (b.ch != NO_CH) pcaSetDigital(b.ch, false); Bench.printf("outputs: %s off\n", b.name); releaseOe(); }
  if (was && closeAfter) servosRequest(b.plate, false);   // closes only if nothing else still needs it open
  return true;
}
static void beamPoll(Beam& b) {
  if (!b.want || *b.on) return;
  if (laserBlockText()[0]) { beamSet(b, false, false); return; }
  if (servosPlateOpen(b.plate)) {
    if (servosPlatesOff()) { fire(b); return; }
    if (!b.openAt) b.openAt = millis() | 1;            // the move has just ended: let the plate get there
    if (millis() - b.openAt >= settleMs) fire(b);
    return;
  }
  b.openAt = 0;
  if (millis() - b.since > LASER_PLATE_WAIT_MS) { Bench.printf("outputs: %s gave up - %s did not open (%s)\n", b.name, servosPlateName(b.plate), servosLastRefusal()); beamSet(b, false, true); }
}
bool laserRedSet(bool on, bool closeAfter)   { return beamSet(beams[0], on, closeAfter); }
bool laserGreenSet(bool on, bool closeAfter) { return beamSet(beams[1], on, closeAfter); }
bool lidarAloneSet(bool on, bool closeAfter) { return beamSet(beams[2], on, closeAfter); }
bool lidarAloneWanted() { return beams[2].want || lidarOn; }
bool lidarAloneOn()     { return lidarOn; }
bool laserRedWanted()   { return beams[0].want || red; }
bool laserGreenWanted() { return beams[1].want || green; }

bool vibPulse(uint16_t ms, const char* why) {
  if (outputsBlockText()[0]) { Bench.printf("outputs: vibrator refused - %s\n", outputsBlockText()); return false; }
  if (ms > VIB_PULSE_MAX_MS) ms = VIB_PULSE_MAX_MS;
  vib = true; vibUntil = millis() + ms;
  hapticStart();                                   // the DRV2605L drives the #1201 over I2C: no PCA channel, no transistor, no diode (v3)
  Bench.printf("outputs: vibrator %u ms (%s)\n", ms, why);
  return true;
}
void vibStop() { if (!vib) return; vib = false; hapticStop(); }
// (2026-10-21) the gestures' answer in the hand (Toby: "every time something happens ... a little buzz to tell me it went
// through ... if it doesn't work, like the battery is too low ... a double buzz"): n pulses of GMAP_BUZZ_MS, GMAP_BUZZ_GAP_MS
// apart, in ANY mode - it is the wearer's answer, not a show output (a SAFE refusal buzzes twice too). Not over a vibrator
// pulse (that one already buzzes); everything off (goSafe, the shutdown) stops it.
static uint8_t  fbLeft = 0;
static bool     fbOn = false;
static uint32_t fbAt = 0, fbCount = 0;
void hapticFeedback(uint8_t n) {
  if (!n) return;
  fbCount++;
  if (vib || fbLeft) return;
  fbLeft = n; fbOn = true; fbAt = millis() + GMAP_BUZZ_MS; hapticStart();
}
uint32_t hapticFeedbackCount() { return fbCount; }
static void feedbackPoll(uint32_t now) {
  if (!fbLeft || (int32_t)(now - fbAt) < 0) return;
  if (fbOn) { hapticStop(); fbOn = false; fbLeft--; fbAt = now + GMAP_BUZZ_GAP_MS; }
  else { hapticStart(); fbOn = true; fbAt = now + GMAP_BUZZ_MS; }
}

// The accent LEDs say "the arm is on" (Toby, 2026-10-07, 00 #33): any mode, SAFE included; refused only from LOW
// battery down, so the firmware can take them off then (they stay on GPIO 4 for exactly that).
bool accentSet(bool on) {
  PowerState ps = powerState();
  if (on && (ps == PowerState::Low || ps == PowerState::Idle || ps == PowerState::Critical)) { Bench.printf("outputs: accent LEDs refused - battery %s\n", powerStateName()); return false; }
  if (on != accent) Bench.printf("outputs: accent LEDs %s\n", on ? "on" : "off");
  accent = on; halDigitalWrite(PIN_ACCENT, on ? HIGH : LOW);
  return true;
}

// The show outputs (both lasers, the vibrator) - what the mode and the forced idle stand down. The accent LEDs are not
// one of them: they stay on in SAFE and follow the battery alone (accentSet, the idle lights in Ironman.ino). The lasers
// go off WITHOUT closing their plates (a mode change, goSafe: the plates freeze) and let go of them.
static void showOutputsOff(const char* why) {
  bool any = red || green || lidarOn || vib || beams[0].want || beams[1].want || beams[2].want;
  beamSet(beams[0], false, false); beamSet(beams[1], false, false); beamSet(beams[2], false, false);
  if (vib)   { vib = false; hapticStop(); }
  if (any) { Bench.printf("outputs: lasers and vibrator off (%s)\n", why); releaseOe(); }
}

void outputsAllOff(const char* why) {
  showOutputsOff(why);
  if (fbLeft) { fbLeft = 0; if (fbOn) { fbOn = false; hapticStop(); } }
  if (accent) { accent = false; halDigitalWrite(PIN_ACCENT, LOW); Bench.printf("outputs: accent LEDs off (%s)\n", why); }
}

void outputsPoll() {
  uint32_t now = millis();
  lasersFlightHeld();                                  // keeps the flight's end time current
  if (vib && (int32_t)(now - vibUntil) >= 0) vibStop();
  feedbackPoll(now);
  if (green && now - greenSince >= LASER_GREEN_MAX_ON_MS) { Bench.println("outputs: green laser off - the forgot-it-on guard"); laserGreenSet(false, true); }
  if ((red || green || lidarOn || vib || beams[0].want || beams[1].want || beams[2].want) && outputsBlockText()[0]) showOutputsOff(outputsBlockText());
  for (Beam& b : beams) {
    if ((*b.on || b.want) && laserBlockText()[0]) { Bench.printf("outputs: %s off - %s\n", b.name, laserBlockText()); beamSet(b, false, false); }   // the mode left FULL, the drone armed: no plate closes
    beamPoll(b);
  }
}

String outputsJson() {
  return "{\"red\":" + String(red ? 1 : 0) + ",\"green\":" + String(green ? 1 : 0) + ",\"vib\":" + String(vib ? 1 : 0) + ",\"accent\":" + String(accent ? 1 : 0) + ",\"lidar\":" + String(lidarOn ? 1 : 0) + ",\"shots\":" + String(shots) + ",\"settle\":" + String(settleMs) + ",\"block\":\"" + outputsBlockText() + "\",\"gblock\":\"" + outputsGreenBlockText() + "\"}";
}
String outputsStatusLine() {
  return String("outputs: red laser ") + (red ? "ON" : "off") + ", green laser " + (green ? "ON" : "off") + ", vibrator " + (vib ? "ON" : "off") + ", " + String(shots) + " shots; " + (outputsBlockText()[0] ? String("blocked: ") + outputsBlockText() : String("allowed")) + (outputsGreenBlockText()[0] ? String("; green: ") + outputsGreenBlockText() : String(""));
}
