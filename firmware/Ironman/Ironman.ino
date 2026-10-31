// Ironman - the gauntlet's main-board firmware (ESP32 WROOM-32 devkit), one sketch for everything the
// forearm commands: Gauntlet/ (plates, LED chains, bench test) and HandBench/ (IMUs, flex, gestures,
// laser, LiDAR, the drone link and flight) merged on the electrical design in design/README.md.
// ONE build since 2026-10-07 (Toby: "one design, one build" - the summit arm is the whole arm, the drone and both
// lasers included). WiFi on at boot (Toby, 2026-10-08), nothing depending on it:
//     ./flash.sh Ironman            compile + push over WiFi to ironman.local (min_spiffs partitions)
//     ./flash.sh Ironman --usb      over USB instead (COM11 unless --usb COMx) - the battery OFF first
//     ./flash.sh Ironman --fake     HW_FAKE: no hardware touched, the self-test runs at boot (the console, or GET :8080/selftest)
//     ./monitor.sh ironman          the console over WiFi; ./monitor.sh COM11 over USB; keys: h
//     http://ironman.local:8080/    the simulator, /hud the telemetry page, /status everything as JSON
//
// config.h is the single source of truth for every pin, address, channel and limit (design/check.ps1
// reads it). hal.* is the only place a GPIO is touched.
//
// *** PROPS OFF THE WHOOP ON THE BENCH. ANTENNA ON THE ELRS MODULE BEFORE IT EVER SEES POWER. ***
// *** NEVER USB AND THE BATTERY ON TOGETHER: the devkit has no USB diode (Toby measured 5.0 V on VIN with USB only). ***
//
// Safety, in one place (goSafe): battery IDLE / CRITICAL, mode SAFE, a firmware push, a panic key -
// the drone disarms, every servo parks, every puck, laser and the vibrator switch off, the chains go dark.

#include <GauntletBenchIO.h>
#include <WiFi.h>
#include <esp_system.h>
#include "config.h"
#include "hal.h"
#include "buses.h"
#include "servos.h"
#include "palm.h"
#include "forearm.h"
#include "current.h"
#include "power.h"
#include "modesw.h"
#include "buzzer.h"
#include "pucks.h"
#include "outputs.h"
#include "s3link.h"
#include "bench.h"
#include "shutdown.h"
#include "imu.h"
#include "flex.h"
#include "gesture.h"
#include "laser.h"
#include "lidar.h"
#include "crsf.h"
#include "fly.h"
#include "thermal.h"
#include "gesturemap.h"
#include "panel.h"
#include "calib.h"
#include "armpage.h"                              // sim/arm.html, gzipped and embedded by prebuild.sh - served at :8080/
#include "hudpage.h"                              // sim/hud.html, the same - served at :8080/hud
#include "calpage.h"                              // the /cal page: servo ends, PCA channels, battery - set on the arm, no recompile
#if HW_FAKE
#include "selftest.h"
#include "demo.h"
#endif

static constexpr uint16_t FEED_PORT = 8080;       // (IMU_FEED_PORT in config.h is the same number; the pages are written for it)
static uint32_t lastFaultBuzzMs = 0, landBelowSince = 0, lastLandWarnMs = 0, lastLowChirpMs = 0;   // (2026-10-08) the FAULT buzz, the LAND NOW warning

static const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:    return "POWERON";
    case ESP_RST_EXT:        return "EXT (external pin)";
    case ESP_RST_SW:         return "SW (software restart)";
    case ESP_RST_PANIC:      return "PANIC (crash)";
    case ESP_RST_INT_WDT:    return "INT_WDT (interrupt watchdog)";
    case ESP_RST_TASK_WDT:   return "TASK_WDT (task watchdog)";
    case ESP_RST_WDT:        return "WDT (other watchdog)";
    case ESP_RST_BROWNOUT:   return "BROWNOUT";
    default:                 return "OTHER";
  }
}

static void printResetReason() {
  esp_reset_reason_t r = esp_reset_reason();
  Bench.printf("reset reason: %s (%d)\n", resetName(r), (int)r);
  if (r == ESP_RST_BROWNOUT) Bench.println("  -> BROWN-OUT: the 3V3 fell below ~2.43 V - the logic rail dipped (a servo on the wrong rail? the pack flat?)");
  if (r == ESP_RST_PANIC)    Bench.println("  -> the firmware crashed. the backtrace is on USB serial only");
}

// ---------------- the idle lights: the accent LEDs and a few dim red strip pixels whenever the arm is on --------------
// (Toby, 2026-10-07, 00 #32 / #33) - off from LOW battery down, during the CRITICAL shutdown and while something else
// owns the strip (the bench test, a chain walk, the fake build's demo).
static bool stripBusy() {
#if HW_FAKE
  if (demoRunning()) return true;
#endif
  return forearmWalking() || benchTestRunning();
}
static void idleLights() {
  PowerState ps = powerState();
  bool low = ps == PowerState::Low || ps == PowerState::Idle || ps == PowerState::Critical;
  if (shutdownActive() || low) { accentSet(false); palmGlow(false); if (!stripBusy()) forearmOff(); return; }
  accentSet(true);
  palmGlow(modeNow() == Mode::Safe);               // SAFE's dim palm glow (Toby, 2026-10-07 night); in SHOW / FULL the ring is the wearer's
  if (!stripBusy()) forearmIdle();
}

// ---------------- the safe state --------------------------------------------------------------------
// Everything stops where it is: a running cascade is aborted and every servo released - the plates FREEZE, nothing ever
// drives them shut here (Toby, 2026-10-07 night: "never close them on a mode change, so they can't pinch a hand").
void goSafe(const char* why) {
  flyDisarm(why);
  crsfSafe();
  laserStop(why);
  thermalStop(why);
  servosAbort();
  servosParkAll();
  pucksAllOff(why);
  outputsAllOff(why);
  palmOff();
  forearmOff();
  Bench.printf("SAFE: %s\n", why);
  idleLights();                                    // the "arm is on" lights come back unless the pack is low or shutting down
}

static void onPower(PowerState from, PowerState to) {
  if (to != PowerState::Critical) shutdownCancel();   // only during the silent grace: once everything is off it carries on to sleep
  if (from == PowerState::Critical && to != PowerState::Critical) palmAlert(false);   // a sag that recovered: the CRITICAL red blink ends with the grace
  // (Toby, 2026-10-08) the arm floor, LOW and IDLE make no sound and no ring blink: the $S frame carries the state (and
  // flags bit 0), and the puck and the OLED show the warning. The only battery sound is the shutdown's chirp.
  switch (to) {
    case PowerState::Low:      palmWarning(false); Bench.printf("power: LOW (%.2f V) - the puck and the OLED warn; no sound; no LED-driver show\n", powerVolts()); break;
    case PowerState::Idle:     palmWarning(false); goSafe("battery forced idle"); break;
    case PowerState::Critical: palmWarning(false); palmAlertRate(300); palmAlert(true); goSafe("battery CRITICAL"); shutdownArm("battery CRITICAL"); break;
    case PowerState::Fault:                         // (2026-10-08) LATCHED until power-off: the pack can no longer be watched
      Bench.println("power: battery MONITOR FAULT (the divider - broken or missing?) - LATCHED until power-off: no LED bursts, lasers, vibrator, plates opening or arming; a buzz every 30 s; a flying drone is NOT disarmed for it");
      pucksAllOff("battery monitor FAULT"); laserStop("battery monitor FAULT"); laserGreenSet(false, false); vibStop();
      buzzerPlay(Buzz::Fault); lastFaultBuzzMs = millis();
      break;
    case PowerState::ArmFloor: break;               // arming refused; the puck and the OLED say it
    default:                   palmWarning(false); palmAlert(false); break;   // Ok / NoPack: back to whatever was set
  }
  idleLights();
}

// The mode switch (Toby, 2026-10-07 night). Down to SAFE: goSafe - lasers off, the drone disarmed, the camera off, the
// plates frozen where they are. FULL -> SHOW: whatever leaves the arm stops at once (both lasers, the LiDAR with the
// red one, the drone disarmed); the plates, the lights and the camera carry on. Up: nothing starts by itself.
static void onMode(Mode from, Mode to) {
  buzzerPlay(Buzz::Mode);
  if (to == Mode::Safe) goSafe("mode SAFE");
  else if (from == Mode::Flight) { laserStop("the mode left FULL"); laserGreenSet(false, false); flyDisarm("the mode switch left FULL"); }
  idleLights();
  s3linkRequestPage(to == Mode::Flight ? 4 : 1);
}

// Touch events from the puck: VIEW-ONLY (Toby, 2026-10-07: "touch on the puck is view-only for now: drag orbits, hold
// zooms out, double tap returns to the hand"). The puck uses every touch for its own 3D view; the arm only logs and
// counts them (s3linkTouches, the feed) - no plate, light, laser or zero ever moves on a touch.
static void onTouch(glink::Touch t, uint16_t x, uint16_t y) {
  Bench.printf("touch: %s at %u,%u - the puck's own view, nothing on the arm\n", glink::touchName(t), x, y);
}

// ---------------- the chain test under load (key U; bring-up step 6) -----------------------------------------
// Both LED chains walk a pixel end to end at their full wire length while the plates cascade, again and again, so a
// marginal 3.3 V data line shows up as a pixel that flickers, takes the wrong colour or never lights while the servo
// rail is working. No level shifter is fitted (Toby, 2026-10-07: no new chips); this is the proof that none is needed.
static bool chainLoad = false;
static void chainLoadStart() {
  if (!modeAllowsShow()) { Bench.printf("chain test under load: refused - mode %s (needs SHOW)\n", modeNowName()); return; }
  chainLoad = true;
  palmWalk();                                      // the ring: one dim pixel at a time, 24 steps, 400 ms each
  forearmWalkStart(40, 2);                         // the strip: one dim pixel end to end, twice
  servosOpenThenClose();
  Bench.println("chain test under load: both chains walking while the plates cascade. PASS = every pixel lights in order, none flickers or takes a colour, none stays dark");
}
static void chainLoadPoll() {
  if (!chainLoad) return;
  bool palmWalking = palmMode() == 4;
  if (!servosBusy() && (palmWalking || forearmWalking())) servosOpenThenClose();   // keep the servo rail working until both walks end
  if (!palmWalking && !forearmWalking() && !servosBusy()) { chainLoad = false; Bench.println("chain test under load: done - did every pixel light in order?"); idleLights(); }
}

// ---------------- console ----------------------------------------------------------------------------
static void help() {
  Bench.println();
  Bench.printf("=== IRONMAN ===   %s build %s %s   (host %s)\n", HW_FAKE ? "FAKE / self-test" : "full", __DATE__, __TIME__, HOSTNAME);
  Bench.println(" armor   7 OPEN cascade   8 CLOSE   9 open-pause-close   R plate 1 alone (open / close)   X abort + everything off   1/2/3 hold ch0 closed/open/meter   4 all six sweep   5 ch0 full travel (unmounted!)   6 ch0 wide (bare servo!)   W ch0 sweep   [ ] nudge closed/open");
  Bench.println(" palm    l colour (on / next)   . brighter   , dimmer   t chain test       forearm  F dot end to end and back   U both chains walking while the plates cascade (bring-up step 6)");
  Bench.println(" pucks   P repulsor burst (groups 1 + 2, the palm)   O all pucks off   outputs  G green laser on/off (FULL)   V vibrator 300 ms   I thermal camera on/off (the hand plate)");
  Bench.println(" mode    M force SAFE -> SHOW -> FULL -> follow the switch (the middle SAFE, the GPIO 36 end SHOW, the GPIO 39 end FULL)   B buzzer test   v battery line   T (capital) full-system bench test");
  Bench.println(" imu     z zero here   y zero yaw   k forget zeros   u raw   s stream on/off   r re-probe   j diagnose   g flex in the stream   f flex now");
  Bench.println(" gesture n pretend a triple   K calibrate: hand FLAT (2 s)   J calibrate: FIST (2 s)   Y wrist range (6 s)   laser  p force on/off/auto   lidar  q log on/off");
  Bench.println(" drone   e link + flight   A ARM (the drone's checks, then the glove flies it)   D DISARM   C RC frames on/off   N bind loop   Q module power lowest   + power up (locked at 100 mW)");
  Bench.printf (" setup   http://%s.local:%u/cal - each servo's closed / open end, the PCA channel map, the battery against a meter (saved on the arm, no recompile)\n", HOSTNAME, FEED_PORT);
  Bench.println(" info    m status (everything)   b boot report   i i2c scan   o wifi state   ! watchdog test (the board resets)   h this");
  Bench.printf (" WiFi    on from boot (nothing needs it): http://%s.local:%u/ simulator   /hud   /status   /plates?plate=N&open=1   ./monitor.sh %s   ./flash.sh Ironman%s\n", HOSTNAME, FEED_PORT, HOSTNAME, HW_FAKE ? "   /selftest   Z the scripted arm for the puck (on at boot)" : "");
  Bench.println();
}

static String setupLine() {                       // what the /cal page has set on this arm
  String s = "setup (/cal): PCA ";
  if (pcaMapIsIdentity()) s += "channels as designed"; else { s += "moved"; for (uint8_t c = 0; c < 16; c++) if (pcaPhysical(c) != c) s += " " + String(c) + "->" + String(pcaPhysical(c)); }
  uint8_t n = 0; for (uint8_t i = 0; i < PLATE_COUNT; i++) if (servoCalSaved(i, false) || servoCalSaved(i, true)) n++;
  s += ", servo ends saved for " + String(n) + " of " + String(PLATE_COUNT) + ", battery factor " + String(powerCalFactor(), 4);
  if (servosPlatesOff()) s += ", PLATES OFF (no plate moves; lasers / camera act at once)";
  return s;
}
static void bootReport() {
  Bench.println("--- boot report ---");
  printResetReason();
  Bench.println(setupLine());
  busesScan();
  Bench.println(powerStatusLine());
  Bench.println(modeStatusLine());
  Bench.printf("thermal: %s (key I; the puck shows the camera while it is on)   gestures blocked by the mode since boot: %lu\n", thermalStateName(), (unsigned long)s3linkEventsBlocked());
  Bench.printf("gesture map: the hand now - %s (two fingers + 3 clicks = green on, + 2 = off; fist thumb-in -> open = plates open, -> open-close = close; thumb out + two finger pulses = camera; the wrist aligned for each; 1 buzz done, 2 refused)\n", gmapPoseText());
}

static String queryArg(const String& q, const char* name) {
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
static bool hasArg(const String& q, const char* name) { return q.startsWith(String(name) + "=") || q.indexOf(String("&") + name + "=") >= 0; }

static String gauntletJson() {                    // the gauntlet's own state: the feed, /status and /tele carry it
  String s = "{\"power\":" + powerJson() + ",\"mode\":" + modeJson() + ",\"thermal\":" + thermalJson() + ",\"plates\":" + servosJson() + ",\"pucks\":" + pucksJson() + ",\"outputs\":" + outputsJson() +
             ",\"display\":" + s3linkJson() + ",\"ring\":" + palmJson() + ",\"forearm\":{\"px\":" + String(forearmCount()) + ",\"bright\":" + String(forearmBrightness()) + ",\"ma\":" + String((int)forearmMilliamps()) + "}" +
             ",\"ina\":" + currentJson() +
#if HW_FAKE
             ",\"demo\":" + demoJson() +
#endif
             ",\"buzzer\":" + String(buzzerSounding() ? 1 : 0) + ",\"fake\":" + String(HW_FAKE) + ",\"heap\":" + String(ESP.getFreeHeap()) + ",\"t\":" + String(millis()) + "}";
  return s;
}

static String feedJson() {                        // the 40 Hz feed the simulator polls: HandBench's frame plus "gauntlet"
  String s = imuJson();
  if (s.endsWith("}")) s.remove(s.length() - 1);
  s += ",\"gauntlet\":" + gauntletJson() + "}";
  return s;
}

static String infoJson() {
  static String md5;
  if (!md5.length()) md5 = ESP.getSketchMD5();
  char b[560];
  snprintf(b, sizeof b,
           "{\"host\":\"%s\",\"ip\":\"%s\",\"rssi\":%d,\"up\":%u,\"build\":\"%s %s\",\"sketch\":\"%s\",\"size\":%u,\"chip\":\"%s rev %u\",\"heap\":%u,\"heapmin\":%u,\"core\":\"%d.%d.%d\",\"reset\":\"%s\",\"variant\":\"%s\","
           "\"palm\":{\"pin\":%u,\"px\":%u,\"cap\":%u},\"pins\":{\"thumb\":%u,\"vbat\":%u,\"sda1\":%u,\"scl1\":%u,\"sda2\":%u,\"scl2\":%u,\"oe\":%u,\"crsf\":%u,\"accent\":%u}}",
           HOSTNAME, WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (unsigned)millis(), __DATE__, __TIME__, md5.c_str(), (unsigned)ESP.getSketchSize(),
           ESP.getChipModel(), (unsigned)ESP.getChipRevision(), (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
           resetName(esp_reset_reason()), HW_FAKE ? "fake" : "full",
           (unsigned)PIN_PALM_LED, (unsigned)PALM_COUNT, (unsigned)((PALM_CAP * 100 + 127) / 255),
           (unsigned)PIN_FLEX_THUMB, (unsigned)PIN_VBAT, (unsigned)PIN_I2C1_SDA, (unsigned)PIN_I2C1_SCL, (unsigned)PIN_I2C2_SDA, (unsigned)PIN_I2C2_SCL, (unsigned)PIN_PCA_OE, (unsigned)PIN_CRSF, (unsigned)PIN_ACCENT);
  return String(b);
}

static String teleJson(const String& q) {         // the telemetry page's one request: the feed + the gesture state + the gauntlet
  String s; s.reserve(3000);
  s += "{\"imu\":";      s += feedJson();
  s += ",\"gesture\":";  s += gestureJson();
  s += ",\"servo\":" + servosJson();
  s += ",\"low\":" + String(powerState() == PowerState::Low || powerState() == PowerState::Idle || powerState() == PowerState::Critical ? 1 : 0) + ",\"heap\":" + String(ESP.getFreeHeap());
  if (hasArg(q, "log"))            { s += ",\"log\":";  s += Bench.logJson(queryArg(q, "log")); }
  if (queryArg(q, "info") == "1")  { s += ",\"info\":"; s += infoJson(); }
  s += "}";
  return s;
}

static String platesSet(const String& q) {        // GET :8080/plates?open=1 | close=1 | cycle=1 | abort=1 | plate=N&open=1|close=1 (N 1-3 forearm, 4 the hand plate: that plate alone)
  String msg;
  if (queryArg(q, "abort") == "1") { servosAbort(); servosParkAll(); msg = "cascade aborted, servos parked"; }
  else if (hasArg(q, "open") || hasArg(q, "close") || hasArg(q, "cycle")) {
    if (!modeAllowsShow()) msg = String("refused - mode ") + modeNowName();
    else if (powerIdleForced()) msg = String("refused - ") + powerBlockText();
    else if (servosBusy()) msg = "a cascade is already running";
    else if (hasArg(q, "plate")) {
      int plate = queryArg(q, "plate").toInt();
      bool open = queryArg(q, "open") == "1";
      if (plate < 1 || plate > PLATE_GROUPS) msg = "plate must be 1-" + String(PLATE_GROUPS) + " (4 = the hand plate)";
      else if (!hasArg(q, "open") && !hasArg(q, "close")) msg = "plate=N needs open=1 or close=1";
      else if (servosMovePlate((uint8_t)plate, open)) msg = String("plate ") + plate + (open ? " opening" : " closing") + " alone";
      else msg = String("refused - ") + servosLastRefusal();
    }
    else if (queryArg(q, "open") == "1") { servosOpen(); msg = "opening"; }
    else if (queryArg(q, "close") == "1") { servosClose(); msg = "closing"; }
    else { servosOpenThenClose(); msg = "open, pause, close"; }
  }
  String s = servosJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

static String pucksSet(const String& q) {         // GET :8080/pucks?on=G | off=G (group 1-3) | show=repulsor|fingers|all | alloff=1
  String msg;
  if (hasArg(q, "on"))      { int g = queryArg(q, "on").toInt() - 1;  msg = puckGroupOn(g, "page") ? "group " + String(g + 1) + " on" : pucksLastRefusal(); }
  if (hasArg(q, "off"))     { int g = queryArg(q, "off").toInt() - 1; puckGroupOff(g); msg = "group " + String(g + 1) + " off"; }
  if (hasArg(q, "show"))    { String n = queryArg(q, "show"); msg = pucksShow(n.c_str()) ? n + " burst" : String("refused - ") + (pucksBlockText()[0] ? pucksBlockText() : pucksLastRefusal()); }
  if (queryArg(q, "alloff") == "1") { pucksAllOff("page"); msg = "all pucks off"; }
  String s = pucksJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

static String outputsSet(const String& q) {       // GET :8080/outputs?green=1|0 | red=1|0 | vib=MS | accent=1|0 | settle=MS
  String msg;
  if (hasArg(q, "settle")) { laserSettleSet((uint32_t)queryArg(q, "settle").toInt()); msg = "a laser fires " + String(laserSettleMs()) + " ms after its plate has opened (saved)"; }
  if (hasArg(q, "green")) { bool on = queryArg(q, "green") == "1"; msg = laserGreenSet(on) ? (on ? "green laser on" : "green laser off") : String("refused - ") + outputsGreenBlockText(); }
  if (hasArg(q, "red"))   { bool on = queryArg(q, "red") == "1";   msg = laserRedSet(on)   ? (on ? "red laser on" : "red laser off")     : String("refused - ") + outputsBlockText(); }
  if (hasArg(q, "vib"))   { int ms = queryArg(q, "vib").toInt();   msg = vibPulse(ms, "page") ? "vibrator " + String(ms) + " ms" : String("refused - ") + outputsBlockText(); }
  if (hasArg(q, "accent")) { bool on = queryArg(q, "accent") == "1"; msg = accentSet(on) ? (on ? "accent LEDs on" : "accent LEDs off") : String("refused - ") + outputsBlockText(); }
  String s = outputsJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

// GET :8080/panel?act=N&a=A&b=B - a control-panel button as if pressed on the puck (glink::ACT_* numbers; $C in GauntletLink.h)
static String panelPress(const String& q) {
  glink::Command c; static uint16_t pseq = 0;
  c.seq = ++pseq; c.act = (uint8_t)queryArg(q, "act").toInt(); c.a = (uint16_t)queryArg(q, "a").toInt(); c.b = (uint16_t)queryArg(q, "b").toInt();
  if (!hasArg(q, "act")) return "{\"msg\":\"panel: act=N (glink::ACT_*), a=, b= - see GauntletLink.h $C\"}";
  panelCommand(c);
  String ev = s3linkLastEventLine(); int star = ev.indexOf('*'); if (star > 0) ev = ev.substring(0, star);   // "$E,seq,act,res,mode,need,ver" without its CRC / newline
  return "{\"act\":\"" + String(glink::actName(c.act)) + "\",\"presses\":" + String(panelPresses()) + ",\"refused\":" + String(panelRefused()) + ",\"event\":\"" + ev + "\"}";
}

static String modeSet(const String& q) {          // GET :8080/mode?force=safe|show|flight|switch
  String f = queryArg(q, "force"), msg;
  if (f == "safe")   { modeForce(Mode::Safe);   msg = "forced SAFE"; }
  else if (f == "show")   { modeForce(Mode::Show);   msg = "forced SHOW"; }
  else if (f == "full" || f == "flight") { modeForce(Mode::Flight); msg = "forced FULL"; }
  else if (f == "switch") { modeFollowSwitch();      msg = "following the switch"; }
  String s = modeJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

// ---- the /cal page's routes (2026-10-13): set on the arm, kept in NVS, no recompile ----
static const char* calBlock() {                   // why a servo may not be moved for calibration now ("" = it may)
  if (!servosReady()) return "PCA9685 not found";
  if (!modeAllowsShow()) return "the mode is SAFE - put the switch on SHOW (or Force SHOW)";
  if (powerIdleForced()) return powerBlockText();
  if (flyArmed()) return "the drone is armed";
  if (servosBusy()) return "a plate move is running";
  return "";
}
// GET :8080/servocal?s=N (0-5, PLATES[]) & us=US | step=+-N | save=closed|open | clear=1 | test=open|close ; release=1
static String calSet(const String& q) {
  String msg;
  int s = hasArg(q, "s") ? queryArg(q, "s").toInt() : -1;
  const char* blk = calBlock();
  if (hasArg(q, "path")) { const char* e = servosPathSet(queryArg(q, "path")); msg = *e ? String("refused - ") + e : String("plate 3 path ") + servosPathJson(); }
  else if (hasArg(q, "follow")) { servosFollowSet(queryArg(q, "follow") != "0"); msg = servosFollowOn() ? "plate 3B follows 3A along its path" : "plate 3B moves on its own (finding its closed)"; }
  else if (hasArg(q, "plate") && (hasArg(q, "ms") || hasArg(q, "ease"))) {   // (2026-10-20) a plate's move time / motion: tried, or kept with keep=1
    String ea = queryArg(q, "ease"); int lin = ea == "linear" ? 1 : (ea == "smooth" ? 0 : -1);
    uint8_t g = (uint8_t)queryArg(q, "plate").toInt();
    const char* e = servosPlateMotion(g, hasArg(q, "ms") ? queryArg(q, "ms").toInt() : -1, lin, queryArg(q, "keep") == "1");
    msg = *e ? String("refused - ") + e : String(servosPlateName(g)) + " " + servosPlateMotionJson(g) + (queryArg(q, "keep") == "1" ? " (saved)" : " (to try - keep=1 saves it)");
  }
  else if (hasArg(q, "plates")) { bool off = queryArg(q, "plates") == "off"; servosSetPlatesOff(off); msg = off ? "plates OFF - no plate moves; the lasers and the camera act with the plates where they are" : "plates ON - the plates move again"; }
  else if (queryArg(q, "release") == "1") { servoCalRelease(); msg = "servo released"; }
  else if (s >= 0 && s < (int)PLATE_COUNT) {
    const char* nm = PLATES[s].name;
    if (hasArg(q, "sweep")) {
      if (*blk) msg = String("refused - ") + blk;
      else {
        uint16_t to = (uint16_t)queryArg(q, "sweep").toInt(), fr = hasArg(q, "from") ? (uint16_t)queryArg(q, "from").toInt() : (servoLastUs((uint8_t)s) ? servoLastUs((uint8_t)s) : to);
        uint32_t ms = hasArg(q, "ms") ? (uint32_t)queryArg(q, "ms").toInt() : 500;
        String pq2 = queryArg(q, "pair"); uint8_t pr2 = pq2 == "1" ? 1 : (pq2 == "mirror" ? 2 : 0);
        const char* e = servoCalSweep((uint8_t)s, fr, to, ms, pr2);
        if (!*e && hasArg(q, "then")) { String t = queryArg(q, "then"); int c1 = t.indexOf(","), c2 = t.indexOf(",", c1 + 1), c3 = t.indexOf(",", c2 + 1);
          if (c1 > 0 && c2 > c1 && c3 > c2) servoCalSweepThen((uint8_t)t.substring(0, c1).toInt(), (uint16_t)t.substring(c2 + 1, c3).toInt(), (uint16_t)t.substring(c1 + 1, c2).toInt(), (uint32_t)t.substring(c3 + 1).toInt()); }
        msg = *e ? String("refused - ") + e : String(nm) + " sweeping " + fr + " -> " + to + " us in " + ms + " ms";
      }
    }
    else if (hasArg(q, "end")) {   // (2026-10-20) an end by number, no move: keep=1 saves it, otherwise it is tried (the plate moves use it) until a reboot
      bool open = queryArg(q, "end") == "open", keep = queryArg(q, "keep") == "1";
      const char* e = servoCalSetEnd((uint8_t)s, open, queryArg(q, "us").toInt(), keep);
      msg = *e ? String("refused - ") + e : String(nm) + (open ? " OPEN" : " CLOSED") + " = " + servoEndUs((uint8_t)s, open) + " us" + (keep ? " (saved)" : " (to try - not saved: a reboot forgets it, keep=1 saves it)");
    }
    else if (hasArg(q, "us") || hasArg(q, "step")) {      if (*blk) msg = String("refused - ") + blk;
      else {
        uint16_t now = servoChannelUs(PLATES[s].ch), last = servoLastUs((uint8_t)s);
        int base = now ? now : (last ? last : servoEndUs((uint8_t)s, false));
        int us = hasArg(q, "us") ? queryArg(q, "us").toInt() : base + queryArg(q, "step").toInt();
        if (us < SERVO_US_MIN) us = SERVO_US_MIN;
        if (us > SERVO_US_MAX) us = SERVO_US_MAX;
        String pq = queryArg(q, "pair"); uint8_t pr = pq == "1" ? 1 : (pq == "mirror" ? 2 : 0);
        const char* e = servoCalMove((uint8_t)s, (uint16_t)us, pr);
        msg = *e ? String("refused - ") + e : String(nm) + " held at " + us + " us" + (servosFollowOn() && s == PLATE3_LEADER ? String(" - 3B follows at ") + servoChannelUs(PLATES[PLATE3_FOLLOWER].ch) + " us" : String("")) + (pr ? (servoCalPair() >= 0 ? String(" with ") + PLATES[servoCalPair()].name + " at " + servoChannelUs(PLATES[servoCalPair()].ch) + " us" : String(" (no pair)")) : String(""));
      }
    }
    else if (hasArg(q, "save")) { bool open = queryArg(q, "save") == "open"; const char* e = servoCalSave((uint8_t)s, open, queryArg(q, "both") == "1"); msg = *e ? String("refused - ") + e : String(nm) + (open ? " OPEN" : " CLOSED") + " saved: " + servoEndUs((uint8_t)s, open) + " us"; }
    else if (queryArg(q, "clear") == "1") { servoCalClear((uint8_t)s); msg = String(nm) + " back to config.h's ends"; }
    else if (hasArg(q, "test")) {
      bool open = queryArg(q, "test") == "open";
      if (*blk) msg = String("refused - ") + blk;
      else { servoCalRelease(); uint8_t g = PLATE_GROUP_OF[s]; msg = servosMovePlate(g, open) ? String(servosPlateName(g)) + (open ? " opening" : " closing") + " with the saved ends" : String("refused - ") + servosLastRefusal(); }
    }
  }
  String j = "{\"held\":" + String(servoCalHeld()) + ",\"pair\":" + String(servoCalPair()) + ",\"platesoff\":" + String(servosPlatesOff() ? 1 : 0) + ",\"follow\":" + String(servosFollowOn() ? 1 : 0) + ",\"path\":" + servosPathJson() + ",\"sweeping\":" + String(servoCalSweeping() ? 1 : 0) + ",\"mode\":\"" + modeNowName() + "\",\"canmove\":" + (*blk ? "0" : "1") + ",\"why\":\"" + blk + "\",\"servos\":[";
  for (uint8_t i = 0; i < PLATE_COUNT; i++) {
    bool sc = servoCalSaved(i, false), so = servoCalSaved(i, true);
    j += String(i ? "," : "") + "{\"name\":\"" + PLATES[i].name + "\",\"ch\":" + PLATES[i].ch + ",\"pca\":" + pcaPhysical(PLATES[i].ch) +
         ",\"closed\":" + servoEndUs(i, false) + ",\"open\":" + servoEndUs(i, true) + ",\"sc\":" + (sc ? 1 : 0) + ",\"so\":" + (so ? 1 : 0) +
         ",\"src\":\"" + ((PLATES[i].usClosed || PLATES[i].usOpen) ? "config.h" : "default") + "\",\"now\":" + servoChannelUs(PLATES[i].ch) + ",\"last\":" + servoLastUs(i) + "}";
  }
  return j + "],\"msg\":\"" + msg + "\"}";
}
// GET :8080/pcamap  ?set=6:13,9:6 (each channel:output a swap) | set=reset
static String pcamapSet(const String& q) {
  String msg;
  if (hasArg(q, "set")) { const char* e = pcaMapSet(queryArg(q, "set")); msg = *e ? String("refused - ") + e : (pcaMapIsIdentity() ? String("every channel on its own output") : String("channel map saved")); }
  String j = "{\"identity\":" + String(pcaMapIsIdentity() ? 1 : 0) + ",\"map\":[";
  for (uint8_t c = 0; c < 16; c++) j += String(c ? "," : "") + pcaPhysical(c);
  j += "],\"names\":[";
  for (uint8_t c = 0; c < 16; c++) { const char* nm = ""; for (const PcaChan& pc : PCA_CHANNELS) if (pc.ch == c) nm = pc.name; j += String(c ? "," : "") + "\"" + nm + "\""; }
  return j + "],\"msg\":\"" + msg + "\"}";
}
// GET :8080/power  ?meter=V (the meter's reading of the pack now; 0 = forget)
static String powerSet(const String& q) {
  String msg;
  if (hasArg(q, "meter")) { const char* e = powerCalMeter(queryArg(q, "meter").toFloat()); msg = *e ? String("refused - ") + e : String("battery reading calibrated: factor ") + String(powerCalFactor(), 4); }
  String s = powerJson(); s.remove(s.length() - 1);
  return s + ",\"msg\":\"" + msg + "\"}";
}

static void statusAll() {
  Bench.println(powerStatusLine());
  Bench.println(modeStatusLine());
  Bench.printf("plates: %s%s, %lu cascades, PCA %s\n", servosPlatesStateName(), servosBusy() ? " (moving)" : "", (unsigned long)servosCascades(), servosReady() ? "ok" : "ABSENT");
  Bench.println(pucksStatusLine());
  Bench.println(outputsStatusLine());
  palmStatus();
  Bench.printf("forearm: %u px, brightness %u/255 (cap %u), ~%d mA now\n", forearmCount(), forearmBrightness(), LED_BRIGHTNESS_CAP, (int)forearmMilliamps());
  Bench.println(s3linkStatusLine());
  if (currentPresent()) Bench.printf("ina219: %.2f A now (servos %.2f A), peak %.2f A, rail %.2f V; %lu stalls caught%s%s\n", currentNowA(), currentServoA(), currentPeakA(), currentBusV(), (unsigned long)servosStallCount(), servosLastStall()[0] ? " - last: " : "", servosLastStall());
  else Bench.println("ina219: absent - stall detection off (optional)");
  imuStatus(); flexPrint(); Bench.println(flexRateLine());
  Bench.printf("watchdog: %s, %lu ms, %lu passes; PCA9685 MODE2 0x%02X\n", halWdtOn() ? "on" : "OFF", (unsigned long)halWdtTimeoutMs(), (unsigned long)halWdtFeeds(), servosMode2());
  Bench.println(gestureStatusLine()); Bench.println(laserStatusLine()); Bench.println(lidarStatusLine()); Bench.println(panelStatusLine()); Bench.println(crsfStatusLine()); Bench.println(flyStatusLine());
}

// ---------------- setup / loop -----------------------------------------------------------------------
// The battery's warnings (loop(), and the self-test's tick): the latched FAULT's buzz every BATT_FAULT_BUZZ_EVERY_MS,
// and LAND NOW while armed under BATT_LAND_WARN_V (2026-10-08). The low-battery chirp every 5 s is gone (Toby,
// 2026-10-08: "LOW: a warning on the puck and the OLED only, no sound").
void batteryAlarmsPoll() {
  { uint32_t now = millis();
    PowerState ps = powerState();
    if (powerFaultLatched() && !shutdownActive() && now - lastFaultBuzzMs >= BATT_FAULT_BUZZ_EVERY_MS) { lastFaultBuzzMs = now; buzzerPlay(Buzz::Fault); }   // the FAULT's own buzz (not a low battery: the pack can no longer be watched)
    // (2026-10-20) LOW and IDLE: a short chirp every BATT_LOW_CHIRP_EVERY_MS until the shutdown (Toby: a beep every 5 s from 7 V)
    if ((ps == PowerState::Low || ps == PowerState::Idle) && !powerFaultLatched() && !shutdownActive()) { if (!lastLowChirpMs || now - lastLowChirpMs >= BATT_LOW_CHIRP_EVERY_MS) { lastLowChirpMs = now; buzzerPlay(Buzz::Low); } }
    else lastLowChirpMs = 0;
    // LAND NOW (2026-10-08): armed, the pack under BATT_LAND_WARN_V for BATT_PERSIST_MS - the ELRS module stops at 7.0 V.
    // The vibrator only (felt in flight, no sound); the puck and the OLED show the low pack.
    if (flyArmed() && !powerFaultLatched() && (ps == PowerState::Ok || ps == PowerState::ArmFloor || ps == PowerState::Low) && powerVolts() < BATT_LAND_WARN_V) {
      if (!landBelowSince) landBelowSince = now;
      if (now - landBelowSince >= BATT_PERSIST_MS && now - lastLandWarnMs >= BATT_LAND_WARN_EVERY_MS) {
        lastLandWarnMs = now; vibPulse(VIB_ARM_MS * 2, "land now");
        Bench.printf("power: LAND NOW - the glove pack reads %.2f V, under %.1f V (the ELRS module stops at 7.0 V)\n", powerVolts(), BATT_LAND_WARN_V);
      }
    } else landBelowSince = 0;
  }
}

void setup() {
  // (2026-10-08) a timer wake out of the CRITICAL shutdown's sleep does ONE thing: a chirp, then asleep again for
  // SHUTDOWN_CHIRP_EVERY_MS - every pad is still held (OE high, the LED data lines and the accent LEDs low), nothing is
  // re-initialised. Only the master switch (a power-off) ends it.
  if (halWokeFromShutdown()) halShutdownChirpAndSleep(SHUTDOWN_CHIRP_HZ, SHUTDOWN_CHIRP_MS, SHUTDOWN_CHIRP_EVERY_MS);
  // The safety lines first, before anything else runs: every PCA output off (OE high - the 1 k pull-up
  // already did this from power-on), the accent LEDs dark, the buzzer quiet (its module sounds on LOW), both chains dark.
  halReleaseHolds();                  // pads a CRITICAL shutdown held through deep sleep (a reset clears them anyway)
  halPinMode(PIN_PCA_OE, OUTPUT);     halDigitalWrite(PIN_PCA_OE, HIGH);
  halPinMode(PIN_ACCENT, OUTPUT);     halDigitalWrite(PIN_ACCENT, LOW);
  halPinMode(PIN_BUZZER, OUTPUT);     halDigitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW);
  palmBegin();
  forearmBegin();
  Serial.begin(115200);
  delay(600);
  Bench.println();
  Bench.printf("IRONMAN  %s build %s %s\n", HW_FAKE ? "FAKE" : "full", __DATE__, __TIME__);
  printResetReason();
  busesBegin();
  busesScan();
  servosBegin();                                   // parks every channel; OE stays high
  currentBegin();                                  // optional INA219, bus 1
  powerBegin();
  Bench.println(setupLine());                      // what the /cal page has set on this arm (PCA map, servo ends, battery factor)
  modeswBegin();
  buzzerBegin();
  pucksBegin();
  outputsBegin();
  s3linkBegin();
  imuBegin();
  flexBegin();
  gestureBegin();
  laserBegin();
  lidarBegin();
  crsfBegin();                                     // RC frames to the ELRS module from here on: sticks centred, throttle min, disarmed
  flyBegin();
  powerOnChange(onPower);
  modeOnChange(onMode);
  s3linkOnTouch(onTouch);
  s3linkOnCommand(panelCommand);                  // the puck's control panel ($C, 2026-10-20): its buttons act, under the gestures' rules
  Bench.onUpdateStart([] {
    halWdtPause(true);                             // a flash write can hold the loop for seconds: the watchdog stands down for the OTA (the outputs are off first)
    imuStream(false);
    s3linkSetFlag(2, true);
    goSafe("firmware update");
    crsfStream(false);                             // (2026-10-08) no more RC frames during a push: with no "handset" the ELRS module should stop its
                                                   // RC packets, whose RF beside the ESP32's antenna stalled pushes on HandBench (it had to switch the
                                                   // module's supply off; the arm cannot). 05 checks it; the frames start again with the reboot.
  });
  // the feed port: the simulator, the telemetry page and every control route
  Bench.liveFeed(FEED_PORT, feedJson);
  Bench.livePageGz("/", ARM_GZ, ARM_GZ_LEN);
  Bench.livePageGz("/hud", HUD_GZ, HUD_GZ_LEN);
  Bench.onLive("/ring", palmSet, "application/json");
  Bench.onLive("/gesture", gestureSet, "application/json");
  Bench.onLive("/laser", laserSet, "application/json");
  Bench.onLive("/fly", flySet, "application/json");
  Bench.onGet("/imu", imuJson, "application/json");
  Bench.onLive("/tele", teleJson, "application/json");
  Bench.onLive("/info", [](const String&) { return infoJson(); }, "application/json");
  Bench.onLive("/status", [](const String&) { return gauntletJson(); }, "application/json");
  Bench.onLive("/plates", platesSet, "application/json");
  Bench.onLive("/servo", platesSet, "application/json");    // the simulator's old route name
  Bench.onLive("/pucks", pucksSet, "application/json");
  Bench.onLive("/outputs", outputsSet, "application/json");
  Bench.onLive("/panel", panelPress, "application/json");
  Bench.onLive("/mode", modeSet, "application/json");
  Bench.onLive("/power", powerSet, "application/json");          // ?meter=V calibrates the reading (NVS)
  Bench.onLive("/servocal", calSet, "application/json");       // the /cal page: one servo at a time, its ends saved (NVS)
  Bench.onLive("/pcamap", pcamapSet, "application/json");      // the PCA9685 channel map (NVS)
  Bench.livePage("/cal", CAL_HTML);
  Bench.onLive("/bench", [](const String&) { return benchTestReport(); }, "text/plain");
  Bench.onGet("/status", gauntletJson, "application/json");
#if HW_FAKE
  Bench.onLive("/selftest", [](const String&) { return selftestReport(); }, "text/plain");
  Bench.onLive("/demo", [](const String& q) { if (queryArg(q, "on") == "1") demoSet(true); else if (queryArg(q, "on") == "0") demoSet(false); return demoJson(); }, "application/json");
  if (SELFTEST_AT_BOOT) selftestRun();             // before the network: the report is kept, the summary printed again below
#endif
  Bench.begin(HOSTNAME, 0, NET_WIFI_AT_BOOT);      // last; WiFi joins in the background - the loop runs from the first moment, with or without a network
  buzzerPlay(Buzz::Boot);
  help();
  Bench.println(modeStatusLine());
  idleLights();                                    // the accent LEDs and the strip's few dim red pixels: "the arm is on"
  halWdtBegin(WDT_TIMEOUT_MS);                     // (2026-10-08) last: a loop that stops for WDT_TIMEOUT_MS resets the ESP32 - OE floats high, every PCA output LOW
  Bench.printf("watchdog: the loop is watched - %lu ms without a pass resets the board (key ! tests it)\n", (unsigned long)WDT_TIMEOUT_MS);
#if HW_FAKE
  Bench.println(selftestSummary());
#endif
}

void loop() {
  halWdtFeed();
  Bench.poll();
  powerPoll();
  modeswPoll();
  servosPoll();
  pucksPoll();
  outputsPoll();
  buzzerPoll();
  s3linkPoll();
  palmPoll();
  currentSample();
  imuPoll();
  flexPoll();
  gesturePoll();
  lidarPoll();
  laserPoll();
  crsfPoll();
  flyPoll();
  gmapPoll();                                      // (2026-10-21) Toby's map: the plates' fist / open, the camera's two pulses (after flyPoll: the fingers' measure)
  forearmWalkPoll();
  chainLoadPoll();
  thermalPoll();                                   // the thermal camera: the hand plate up -> on, closing -> off
  shutdownPoll();                                  // the CRITICAL shutdown: alarm, everything off, deep sleep
#if HW_FAKE
  demoPoll();
#endif
  batteryAlarmsPoll();                             // the FAULT buzz, LAND NOW (the vibrator)
  calibPoll();                                     // the puck's CALIBRATE steps (2026-10-21)
  panelPoll();                                     // the RC frames back on after a puck update
  if (!Bench.available()) { delay(1); return; }
  int raw = Bench.read();
  if (raw == '\n' || raw == '\r' || raw == ' ') return;
  switch (raw) {                                   // capitals first (case-sensitive keys), then the rest lower-cased
    case 'T': benchTestRun(); idleLights(); return;
    case 'K': gestureCaptureStart(false); return;    // calibrate: hand FLAT (2 s) - the page's step 4, no computer needed
    case 'J': gestureCaptureStart(true); return;     // calibrate: FIST (2 s) - the page's step 5
    case 'Y': Bench.println(flySet("range=1")); return;   // the wrist range (the page's step 7): 6 s of the wrist's extremes
    case 'U': chainLoadStart(); return;              // the chain test under load (bring-up step 6)
    case 'R': if (!modeAllowsShow()) Bench.printf("plates: refused - mode %s\n", modeNowName());
              else if (servosBusy()) Bench.println("plates: busy");
              else servosMovePlate(1, servoPosPermille(0) < 500);   // plate 1 alone: open if it is closed, else close
              return;
    case 'X': goSafe("key X"); return;
    case '!': Bench.printf("watchdog test: the loop stops NOW - the board must reset within %lu ms (OE goes high, every PCA output off)\n", (unsigned long)WDT_TIMEOUT_MS);
              Serial.flush(); { uint32_t t0 = (millis)(); while ((millis)() - t0 < 4 * WDT_TIMEOUT_MS) { } }
              Bench.println("watchdog test: FAILED - no reset (is the watchdog on? see the boot lines)"); return;
    case 'W': servoSweepOnce(); return;
    case 'F': Bench.println("forearm: one dot end to end and back, twice (x stops)"); forearmBounce(2, 25, [] { Bench.poll(); return !(Bench.available() && Bench.peek() == 'x'); }); idleLights(); return;
    case 'P': if (!pucksShow("repulsor")) Bench.printf("pucks: refused - %s\n", pucksBlockText()[0] ? pucksBlockText() : pucksLastRefusal()); return;
    case 'O': pucksAllOff("key O"); return;
    case 'G': if (!laserGreenSet(!laserGreenWanted())) Bench.printf("outputs: green laser refused - %s\n", outputsGreenBlockText()); return;
    case 'V': vibPulse(300, "key V"); return;
    case 'I': thermalToggle("key I (the bench - the gesture is 00 #31)"); return;
    case 'M': { Mode m = modeNow(); if (!modeForced()) modeForce(Mode::Safe); else if (m == Mode::Safe) modeForce(Mode::Show); else if (m == Mode::Show) modeForce(Mode::Flight); else modeFollowSwitch(); return; }
    case 'B': buzzerPlay(Buzz::Test); return;
#if HW_FAKE
    case 'Z': demoSet(!demoRunning()); return;
#endif
    case 'D': flyDisarm("key D"); return;
    case 'A': Bench.println(flyBenchArm(FLY_BENCH_ARM_S)); return;
    case 'C': crsfStream(!crsfStreaming()); return;
    case 'N': crsfBindLoop(!crsfBindBursts()); return;
    case 'Q': crsfQuiet(); return;
    default: break;
  }
  int c = tolower(raw);
  switch (c) {
    case '7': if (modeAllowsShow()) servosOpen(); else Bench.printf("plates: refused - mode %s\n", modeNowName()); break;
    case '8': if (modeAllowsShow()) servosClose(); else Bench.printf("plates: refused - mode %s\n", modeNowName()); break;
    case '9': if (modeAllowsShow()) servosOpenThenClose(); else Bench.printf("plates: refused - mode %s\n", modeNowName()); break;
    case '1': servoHold(servoUsClosed()); break;
    case '2': servoHold(servoUsOpen());   break;
    case '3': servoHoldForMeter();        break;
    case '4': if (SERVO_RAW_TOOLS) servosAllSweep(); else Bench.println("servos: the all-six sweep ignores the plates' order and could jam them - set SERVO_RAW_TOOLS in config.h with the plates OFF the servos"); break;
    case '5': servoFullTravel();          break;
    case '6': servoWideTravel();          break;
    case '[': servoNudgeClosed(+25);      break;
    case ']': servoNudgeOpen(-25);        break;
    case 'x': if (servosBusy()) { servosAbort(); servosParkAll(); } palmOff(); forearmOff(); Bench.println("LEDs off"); break;
    case 'l': palmNextColour();           break;
    case '.': palmBrighter();             break;
    case ',': palmDimmer();               break;
    case 't': palmWalk();                 break;
    case 'v': Bench.println(powerStatusLine()); break;
    case 'n': gestureFake();              break;
    case 'p': laserKeyCycle();            break;
    case 'q': lidarLog(!lidarLogging());  break;
    case 'e': crsfStatus(); Bench.println(flyStatusLine()); break;
    case '+': crsfPowerUp();              break;
    case 'z': imuZeroHere();              break;
    case 'y': imuZeroYaw();               break;
    case 'k': imuZeroForget();            break;
    case 'u': imuRaw();                   break;
    case 's': imuStream(!imuStreaming()); break;
    case 'r': imuBegin();                 break;
    case 'j': imuDiagnose();              break;
    case 'f': flexPrint();                break;
    case 'g': { static bool on = false; on = !on; imuStreamFlex(on); } break;
    case 'm': statusAll();                break;
    case 'b': bootReport();               break;
    case 'i': busesScan();                break;
    case 'o': if (Bench.wifiUp()) Bench.printf("wifi up  %s  http://%s.local/\n", Bench.ip().c_str(), HOSTNAME);
              else if (!Bench.wifiEnabled()) Bench.println("wifi OFF (the CRITICAL shutdown turned it off)");
              else                Bench.println("wifi down - retrying in the background"); break;
    case 'h': help();                     break;
    default:
      if (flyArmed()) { flyDisarm("unknown key = panic disarm"); break; }
      Bench.printf("unknown '%c' - press h\n", (char)c); break;
  }
}
