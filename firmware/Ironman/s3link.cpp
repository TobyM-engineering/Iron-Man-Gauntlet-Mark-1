#include "s3link.h"
#include "config.h"
#include "hal.h"
#include "power.h"
#include "modesw.h"
#include "servos.h"
#include "palm.h"
#include "forearm.h"
#include "pucks.h"
#include "outputs.h"
#include "current.h"
#include "buzzer.h"
#include "bench.h"
#include "fly.h"
#include "crsf.h"
#include "lidar.h"
#include "imu.h"
#include "flex.h"
#include "gesture.h"
#include "laser.h"
#include "thermal.h"
#include <GauntletBenchIO.h>

static_assert(S3_BAUD == glink::BAUD, "config.h S3_BAUD and GauntletLink BAUD differ");
static_assert(S3_STATUS_MS == glink::S_PERIOD_MS && S3_LIVE_MS == glink::G_PERIOD_MS, "config.h S3_*_MS and GauntletLink's periods differ");
static_assert(FOREARM_COUNT <= 100, "GauntletLink $G strippos / striplit carry 0..100 pixels");
static_assert(PLATE_COUNT == glink::SERVOS, "GauntletLink $G carries one position per servo in PLATES[]");
static_assert(PUCK_GROUPS == 3, "GauntletLink $G groups carries three LED groups");
static_assert(S3_TX_BUF >= 4 * glink::MAX_LINE, "the TX ring must hold a few whole frames");

static glink::Parser parser;
static uint32_t lastSendMs = 0, lastLiveMs = 0, lastHbMs = 0, heartbeats = 0, sent = 0, liveSent = 0, touches = 0, lastTouchMs = 0, logLines = 0;
static uint32_t txDrops = 0, txBytes = 0, rxBytes = 0, versionBad = 0, fieldBad = 0, unknownType = 0, lastVerLogMs = 0, lastV1Seen = 0;
static long     theirVersion = -1;               // the puck's protocol version from its last heartbeat (-1 = none heard)
static uint32_t pollMaxUs = 0, pollMaxEverUs = 0, writeMaxUs = 0, pollN = 0; static uint64_t pollSumUs = 0;
static uint32_t pollWinStart = 0, pollMaxLastWin = 0;
static uint16_t seq = 0, gseq = 0, touchX = 0, touchY = 0;
static glink::Touch lastTouch = glink::Touch::None;
static uint8_t  pageReq = 0, flags = 0;
static glink::Heartbeat hb;
static char     lastLine[glink::MAX_LINE] = "", lastLive[glink::MAX_LINE] = "";
static void (*onTouch)(glink::Touch, uint16_t, uint16_t) = nullptr;
static void (*onCommand)(const glink::Command&) = nullptr;
static_assert((uint16_t)(BATT_ARM_FLOOR_V * 1000.0f + 0.5f) == glink::POWER_FLOOR_MV && (uint16_t)(LIGHT_FULL_FLOOR_V * 1000.0f + 0.5f) == glink::POWER_FLOOR_MV,
              "config.h BATT_ARM_FLOOR_V / LIGHT_FULL_FLOOR_V and GauntletLink.h POWER_FLOOR_MV (the puck's one battery-bar mark) disagree");
static_assert((uint16_t)(BATT_CRITICAL_V * 1000.0f + 0.5f) == glink::BATT_EMPTY_MV, "config.h BATT_CRITICAL_V and GauntletLink.h BATT_EMPTY_MV (the puck's 0 %) disagree");
static int32_t  lastCmdSeq = -1;
static uint32_t commands = 0, cmdRepeats = 0;
static bool     wasUp = false, testMode = false, uartStarted = false;
// gesture / event detection for $G gev / gn
static uint8_t  gev = glink::GEV_NONE; static uint16_t gn = 0;
static uint32_t prevTriple = 0, prevDipUp = 0, prevDipDown = 0, prevBursts = 0, prevStarts = 0;
static bool     prevLaser = false, prevArmed = false, eventsPrimed = false;

bool     s3linkUp()           { return lastHbMs && millis() - lastHbMs <= S3_HEARTBEAT_LOST_MS; }
uint32_t s3linkHeartbeats()   { return heartbeats; }
uint32_t s3linkStatusSent()   { return sent; }
uint32_t s3linkLiveSent()     { return liveSent; }
uint32_t s3linkTouches()      { return touches; }
uint32_t s3linkTxDrops()      { return txDrops; }
uint32_t s3linkPollMaxUs()    { return pollMaxEverUs; }
uint32_t s3linkWriteMaxUs()   { return writeMaxUs; }
long     s3linkTheirVersion() { return theirVersion; }
uint32_t s3linkVersionRejects() { return versionBad + parser.v1; }
const glink::Parser& s3linkParser() { return parser; }
void     s3linkRequestPage(uint8_t p) { pageReq = p; }
void     s3linkSetFlag(uint8_t bit, bool on) { if (on) flags |= (1 << bit); else flags &= ~(1 << bit); }
const char* s3linkLastStatusLine() { return lastLine; }
const char* s3linkLastLiveLine()   { return lastLive; }
void     s3linkOnTouch(void (*fn)(glink::Touch, uint16_t, uint16_t)) { onTouch = fn; }
void     s3linkOnCommand(void (*fn)(const glink::Command&)) { onCommand = fn; }
uint32_t s3linkCommands()     { return commands; }
void     s3linkTestMode(bool on) { testMode = on; }
bool     s3linkUartOn() { return uartStarted; }
glink::Touch s3linkLastTouch(uint16_t* x, uint16_t* y, uint32_t* age) { if (x) *x = touchX; if (y) *y = touchY; if (age) *age = lastTouchMs ? millis() - lastTouchMs : 0; return lastTouch; }

// The one place a frame goes out. Never waits: the frame goes into the TX ring only when the ring has room for all of
// it (plus the FIFO's 128 bytes - the core reports the FIFO's room instead of the ring's when the ring is full, and a
// write the ring cannot take would wait), else it is dropped and counted. In the self-test (testMode) nothing goes
// to the wire: the test's clock runs faster than the line.
static bool sendLine(const char* line, size_t n) {
  if (testMode || !uartStarted) { txBytes += n; return true; }
  uint32_t t0 = micros();
  int room = Serial2.availableForWrite();
  if (room < (int)(n + 129)) { txDrops++; return false; }
  size_t w = Serial2.write((const uint8_t*)line, n);
  uint32_t us = micros() - t0; if (us > writeMaxUs) writeMaxUs = us;
  txBytes += w;
  if (w != n) { txDrops++; return false; }
  return true;
}

void s3linkBegin() {
#if HW_FAKE
  // The fake build can also run on a bench devkit whose GPIO 18 is a palm ring's DATA line: UART bytes there would
  // light random pixels. Put that board's MAC here and it gets no UART (all zeros = no such board).
  { uint64_t m = ESP.getEfuseMac(); const uint8_t hb[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; bool same = true;
    for (int i = 0; i < 6; i++) if (((m >> (8 * i)) & 0xFF) != hb[i]) same = false;
    if (same) { Bench.println("s3link: this is the HandBench devkit (GPIO 18 = its palm ring data) - the puck UART stays OFF in the fake build here"); return; } }
#endif
  Serial2.setTxBufferSize(S3_TX_BUF);             // before begin(): a TX ring, so write() copies and returns
  Serial2.setRxBufferSize(S3_RX_BUF);
  Serial2.begin(S3_BAUD, SERIAL_8N1, PIN_S3_RX, PIN_S3_TX);   // the core keeps its internal pull-up on RX (default on in 3.3.x)
  uartStarted = true;
  Bench.printf("s3link: the puck (round display) on UART2, protocol v%u (CRC-16) - TX GPIO %u -> its header pin 8 (GPIO16), RX GPIO %u <- its pin 7 (GPIO15), %lu baud; $S every %lu ms, $G every %lu ms, heartbeat timeout %lu ms; never blocks (TX ring %u B, frames dropped when full)\n",
               glink::VERSION, PIN_S3_TX, PIN_S3_RX, (unsigned long)S3_BAUD, (unsigned long)S3_STATUS_MS, (unsigned long)S3_LIVE_MS, (unsigned long)S3_HEARTBEAT_LOST_MS, (unsigned)S3_TX_BUF);
}

static uint8_t bstateOf() {
  switch (powerState()) { case PowerState::Ok: return 0; case PowerState::ArmFloor: return 1; case PowerState::Low: return 2; case PowerState::Idle: return 3;
                          case PowerState::Critical: return 4; case PowerState::Fault: return 5; case PowerState::NoPack: return 6; default: return 0; }
}

static void buildStatus(glink::Status& s) {
  s.seq = seq; s.mode = (uint8_t)modeNow();
  float v = powerVolts(); s.mv = (uint16_t)(v < 0 ? 0 : v > 9.999f ? 9999 : v * 1000 + 0.5f);
  s.bstate = bstateOf();
  s.plates = (uint8_t)servosPlatesState();
  s.ringi = palmColourIndex(); s.ringlvl = palmLevel();
  s.laser = (laserRedOn() ? 1 : 0) | (laserGreenOn() ? 2 : 0);
  s.pucks = 0; for (uint8_t i = 0; i < PUCK_COUNT; i++) if (puckIsOn(i)) s.pucks |= (1 << i); if (pucksBranchOn()) s.pucks |= 0x80;
  s.armed = flyArmed() ? 1 : 0; s.lq = crsfLinkUp() ? crsfLQ() : 0; float cv = crsfCraftVolts(); s.cv100 = (uint16_t)(cv < 0 ? 0 : cv > 655 ? 65535 : cv * 100 + 0.5f);
  strncpy(s.fcmode, crsfFcSeen() ? crsfFcMode() : "-", sizeof s.fcmode - 1); s.fcmode[sizeof s.fcmode - 1] = 0;
  s.lidar = lidarMm();
  if (flyEnabled() && !flyArmed()) flags |= (1 << 3); else flags &= ~(1 << 3);
  if (benchTestRunning()) flags |= (1 << 4); else flags &= ~(1 << 4);
  if (thermalIsOn()) flags |= (1 << glink::FLAG_THERMAL); else flags &= ~(1 << glink::FLAG_THERMAL);   // the camera is deployed: the puck shows its image full screen
  if (powerState() != PowerState::Ok && powerState() != PowerState::NoPack && powerState() != PowerState::Unknown) flags |= 1; else flags &= ~1;
  s.flags = flags; s.page = pageReq;
}

// Events for $G: every counter or edge the modules keep, checked every loop() so none is missed between frames
// (two in one frame = gn steps twice, the newest is named).
static void event(uint8_t e) { gev = e; gn++; }
static void watchEvents() {
  uint32_t starts = servosStarts(), bursts = pucksBursts();
  bool laser = laserRedOn();
  uint32_t tri = gestureCount(), up = gestureDipUpCount(), down = gestureDipDownCount(); bool armed = flyArmed();
  if (!eventsPrimed) { eventsPrimed = true; prevTriple = tri; prevDipUp = up; prevDipDown = down; prevBursts = bursts; prevStarts = starts; prevLaser = laser; prevArmed = armed; return; }
  if (tri != prevTriple)   { prevTriple = tri; event(glink::GEV_TRIPLE); }
  if (up != prevDipUp)     { prevDipUp = up; event(glink::GEV_DIP_UP); }
  if (down != prevDipDown) { prevDipDown = down; event(glink::GEV_DIP_DOWN); }
  if (laser && !prevLaser) event(glink::GEV_LASER);
  prevLaser = laser;
  if (armed != prevArmed)  { prevArmed = armed; event(armed ? glink::GEV_ARMED : glink::GEV_DISARMED); }
  if (starts != prevStarts) { prevStarts = starts; event(servosMovingPlate() ? glink::GEV_PLATE : glink::GEV_CASCADE); }
  if (bursts != prevBursts) { prevBursts = bursts; event(glink::GEV_BURST); }
}

static void buildLive(glink::Live& g) {
  g.seq = gseq; g.t = millis();
  { float fq[4], hq[4]; g.imu = imuSegPair(fq, hq); for (int i = 0; i < 4; i++) { g.fq[i] = glink::q10k(fq[i]); g.hq[i] = glink::q10k(hq[i]); } }
  for (uint8_t i = 0; i < 5; i++) {
    float c = i == 0 ? gestureThumbCurlFull() : gestureFingerCurl(i);   // (2026-10-21) the model's thumb: its full curl (calibration step 3)
    g.curl[i] = (isnan(c) || flexFault(i)) ? -1 : (int16_t)lroundf((c < 0 ? 0 : c > 1 ? 1 : c) * 1000.0f);
    g.fmv[i] = flexMv(i);
  }
  g.hand = isnan(gestureHandOpenMv()) ? 0 : gestureFist() ? 2 : 1;
  g.lidar = lidarMm();
  if (flyArmed()) g.outs |= 16;
  for (uint8_t i = 0; i < glink::SERVOS; i++) g.servo[i] = servoPosPermille(i);
  g.drv = servosDrivenMask(); g.plates = (uint8_t)servosPlatesState();
  g.ring = palmShownRgb(); g.ringset = palmSetRgb(); g.ringlvl = palmLevel(); g.ringmode = palmMode();
  { int8_t first; forearmSummary(g.strip, g.striplvl, g.striplit, first); g.strippos = first; }
  for (uint8_t k = 0; k < 3; k++) if (puckGroupIsOn(k)) g.groups |= (1 << k);
  g.accent = accentOn() ? 1 : 0;
  g.lasers = (laserRedOn() ? 1 : 0) | (laserGreenOn() ? 2 : 0);
  g.mode = (uint8_t)modeNow();
  g.gev = gev; g.gn = gn;
  float v = powerVolts(); g.mv = (uint16_t)(v < 0 ? 0 : v > 9.999f ? 9999 : v * 1000 + 0.5f); g.bstate = bstateOf();
  g.ma = currentPresent() ? (int16_t)lroundf(currentNowA() * 1000.0f) : -1;
  if (vibOn()) g.outs |= 1;
  if (buzzerSounding()) g.outs |= 2;
  if (servosBusy()) g.outs |= 4;
  if (benchTestRunning()) g.outs |= 8;
  if (modeForced()) g.outs |= 32;
  if (flags & 4) g.outs |= 64;
}

static void handle() {
  const char* f[glink::MAX_FIELDS]; uint8_t n = parser.fields(f, glink::MAX_FIELDS);
  if (!n) return;
  uint32_t now = millis();
  switch (f[0][0]) {
    case 'H': {
      long ver = -1; glink::Heartbeat h;
      glink::Result r = glink::heartbeatParse(f, n, h, &ver);
      if (ver >= 0) theirVersion = ver;
      if (r == glink::Result::BadVersion) {
        versionBad++;
        if (!lastVerLogMs || now - lastVerLogMs >= 10000) { lastVerLogMs = now; Bench.printf("s3link: the puck speaks protocol v%ld, the arm v%u - its frames are IGNORED. Build both from the same lib/GauntletLink/src/GauntletLink.h\n", ver, glink::VERSION); }
        break;
      }
      if (r != glink::Result::Ok) { fieldBad++; break; }
      hb = h; heartbeats++; lastHbMs = now;
      if (!wasUp) { wasUp = true; Bench.printf("s3link: puck UP, protocol v%ld (page %u, thermal %s, oled %s, %u fps)\n", ver, hb.page, hb.mlx ? "ok" : "absent", hb.oled ? "ok" : "absent", hb.fps); }
      break;
    }
    case 'T': {
      long sq, x, y;
      if (n != 5 || f[0][1] || !glink::toLong(f[1], 0, 65535, sq) || !glink::toLong(f[3], 0, 239, x) || !glink::toLong(f[4], 0, 239, y)) { fieldBad++; break; }
      glink::Touch t = glink::touchFrom(f[2]);
      if (t == glink::Touch::None) { fieldBad++; break; }
      touchX = (uint16_t)x; touchY = (uint16_t)y; lastTouch = t; lastTouchMs = now; touches++;
      Bench.printf("s3link: touch %s at %u,%u\n", glink::touchName(t), touchX, touchY);
      if (onTouch) onTouch(t, touchX, touchY);
      break;
    }
    case 'C': {   // a button on the puck's control panel (2026-10-20): acted on once per seq
      glink::Command c; glink::Result r = glink::commandParse(f, n, c);
      if (r == glink::Result::BadVersion) { versionBad++; break; }
      if (r != glink::Result::Ok) { fieldBad++; break; }
      if (lastCmdSeq == (int32_t)c.seq) { cmdRepeats++; break; }   // the same line twice = one press
      lastCmdSeq = c.seq; commands++;
      if (onCommand) onCommand(c);
      break;
    }
    case 'L': if (n >= 2) { logLines++; Bench.printf("puck: %s\n", f[1]); } break;
    case 'A': break;
    default:  unknownType++; break;
  }
}

static void feed(char c) { if (parser.feed(c)) handle(); }

// $E: gesture / action events (2026-10-07 night), each sent once, as soon as the TX ring has room - a burst of events
// never blocks and never delays $G / $S. The queue is small on purpose: more than S3_EVENT_QUEUE pending = dropped, counted.
static constexpr uint8_t S3_EVENT_QUEUE = 8;
static glink::Event evq[S3_EVENT_QUEUE]; static uint8_t evHead = 0, evN = 0; static uint16_t eseq = 0; static uint32_t evSent = 0, evDropped = 0, evBlocked = 0;
static char     lastEvent[glink::MAX_LINE] = "";
static const char* modeWord(uint8_t m) { return m == 0 ? "SAFE" : m == 1 ? "SHOW" : "FULL"; }
void s3linkAct(uint8_t act, uint8_t res, uint8_t need) {
  if (res == glink::RES_BLOCKED) {
    evBlocked++;
    if ((uint8_t)modeNow() >= need) Bench.printf("gesture: %s - BLOCKED, %s; told the puck\n", glink::actName(act), flyEnabled() ? "the drone is armed" : "the drone was just disarmed");   // a laser in flight (LASER_AFTER_FLIGHT_MS)
    else Bench.printf("gesture: %s - BLOCKED by the mode (needs %s, the switch is on %s); told the puck\n", glink::actName(act), modeWord(need), modeNowName());
  }
  if (evN >= S3_EVENT_QUEUE) { evDropped++; return; }
  glink::Event e; e.seq = eseq++; e.act = act; e.res = res; e.mode = (uint8_t)modeNow(); e.need = need;
  evq[(evHead + evN) % S3_EVENT_QUEUE] = e; evN++;
}
uint32_t s3linkEventsSent()    { return evSent; }
uint32_t s3linkEventsBlocked() { return evBlocked; }
const char* s3linkLastEventLine() { return lastEvent; }
static void sendEvents(char* body, char* line) {
  while (evN) {
    if (!glink::eventBody(body, glink::MAX_LINE, evq[evHead])) { evHead = (evHead + 1) % S3_EVENT_QUEUE; evN--; continue; }
    size_t n = glink::frame(line, glink::MAX_LINE, body);
    if (!n || !sendLine(line, n)) return;            // no room this loop: it goes in a later one
    memcpy(lastEvent, line, n + 1); evSent++;
    evHead = (evHead + 1) % S3_EVENT_QUEUE; evN--;
  }
}

void s3linkPoll() {
  uint32_t t0 = micros(), now = millis();
  if (!testMode && uartStarted) {                  // at most S3_RX_MAX_PER_POLL bytes a loop, whatever arrives
    uint16_t k = 0;
    while (k < S3_RX_MAX_PER_POLL && Serial2.available() > 0) { int c = Serial2.read(); if (c < 0) break; feed((char)c); k++; }
    rxBytes += k;
  } else if (uartStarted) {
    while (Serial2.available() > 0) Serial2.read();   // the self-test owns the link: whatever a real puck sends meanwhile is thrown away
  }
  watchEvents();
  char body[glink::MAX_LINE], line[glink::MAX_LINE];
  sendEvents(body, line);
  if (now - lastLiveMs >= S3_LIVE_MS) {
    lastLiveMs = now;
    glink::Live g; buildLive(g);
    if (glink::liveBody(body, sizeof body, g)) {
      size_t n = glink::frame(line, sizeof line, body);
      if (n && sendLine(line, n)) { memcpy(lastLive, line, n + 1); liveSent++; gseq++; }
    }
  }
  if (now - lastSendMs >= S3_STATUS_MS) {
    lastSendMs = now;
    glink::Status s; buildStatus(s);
    if (glink::statusBody(body, sizeof body, s)) {
      size_t n = glink::frame(line, sizeof line, body);
      if (n && sendLine(line, n)) { memcpy(lastLine, line, n + 1); sent++; seq = (seq + 1) % 10000; if (pageReq) pageReq = 0; }
    }
  }
  if (wasUp && !s3linkUp()) { wasUp = false; Bench.println("s3link: puck link DOWN (no heartbeat for 3 s) - nothing else changes, frames keep going out"); }
  if (parser.v1 != lastV1Seen && (!lastVerLogMs || now - lastVerLogMs >= 10000)) { lastV1Seen = parser.v1; lastVerLogMs = now; Bench.printf("s3link: %lu lines from a protocol-v1 puck (two-digit CRC) IGNORED - flash the puck with GauntletLink v%u\n", (unsigned long)parser.v1, glink::VERSION); }
  uint32_t us = micros() - t0;
  pollN++; pollSumUs += us;
  if (us > pollMaxUs) pollMaxUs = us;
  if (us > pollMaxEverUs && !testMode) pollMaxEverUs = us;
  if (now - pollWinStart >= 10000) { pollWinStart = now; pollMaxLastWin = pollMaxUs; pollMaxUs = 0; }
}

// The self-test's proof that the link cannot stall the arm (selftest.cpp t_link_nonblocking): tries to push `frames`
// widest-size $G frames through the REAL UART back to back - far faster than the line can carry them - and reports the
// slowest single attempt and how many were dropped. Then the TX ring drains by itself.
void s3linkFloodTest(uint16_t frames, uint32_t& maxUs, uint32_t& drops, uint32_t& accepted) {
  bool tm = testMode; testMode = false;
  glink::Live g; for (int i = 0; i < 4; i++) { g.fq[i] = -10000; g.hq[i] = -10000; } for (int i = 0; i < 5; i++) { g.curl[i] = 1000; g.fmv[i] = 3300; }
  for (int i = 0; i < glink::SERVOS; i++) g.servo[i] = 1000; g.seq = 65535; g.t = 4294967295UL; g.lidar = 40000; g.mv = 9999; g.ma = 9999; g.gn = 65535;
  char body[glink::MAX_LINE], line[glink::MAX_LINE]; glink::liveBody(body, sizeof body, g); size_t n = glink::frame(line, sizeof line, body);
  uint32_t d0 = txDrops; maxUs = 0; accepted = 0;
  for (uint16_t i = 0; i < frames; i++) {
    uint32_t t0 = micros();
    if (sendLine(line, n)) accepted++;
    uint32_t us = micros() - t0; if (us > maxUs) maxUs = us;
  }
  drops = txDrops - d0;
  testMode = tm;
}

String s3linkJson() {
  char b[760];                                     // 34 fields at their widest is ~600
  uint32_t age = 0; glink::Touch t = s3linkLastTouch(nullptr, nullptr, &age);
  snprintf(b, sizeof b, "{\"up\":%d,\"ver\":%u,\"their\":%ld,\"hb\":%lu,\"sent\":%lu,\"live\":%lu,\"drops\":%lu,\"txb\":%lu,\"rxb\":%lu,\"good\":%lu,\"bad\":%lu,\"stray\":%lu,\"cut\":%lu,\"long\":%lu,\"v1\":%lu,\"verbad\":%lu,\"fieldbad\":%lu,\"touch\":\"%s\",\"tx\":%u,\"ty\":%u,\"tn\":%lu,\"tage\":%lu,\"page\":%u,\"fps\":%u,\"mlx\":%u,\"oled\":%u,\"heap\":%lu,\"uptime\":%lu,\"pollmax\":%lu,\"pollmax10s\":%lu,\"pollavg\":%lu,\"writemax\":%lu,\"gev\":%u,\"gn\":%u}",
           s3linkUp() ? 1 : 0, glink::VERSION, theirVersion, (unsigned long)heartbeats, (unsigned long)sent, (unsigned long)liveSent, (unsigned long)txDrops, (unsigned long)txBytes, (unsigned long)rxBytes,
           (unsigned long)parser.good, (unsigned long)parser.bad, (unsigned long)parser.stray, (unsigned long)parser.truncated, (unsigned long)parser.tooLong, (unsigned long)parser.v1, (unsigned long)versionBad, (unsigned long)fieldBad,
           glink::touchName(t), touchX, touchY, (unsigned long)touches, (unsigned long)age, hb.page, hb.fps, hb.mlx, hb.oled, (unsigned long)hb.heap, (unsigned long)hb.uptimeS,
           (unsigned long)pollMaxEverUs, (unsigned long)(pollMaxLastWin > pollMaxUs ? pollMaxLastWin : pollMaxUs), (unsigned long)(pollN ? pollSumUs / pollN : 0), (unsigned long)writeMaxUs, gev, gn);
  return String(b);
}
String s3linkStatusLine() {
  char b[400];
  snprintf(b, sizeof b, "s3link: puck %s (protocol v%u, the puck's %s), %lu heartbeats, %lu $S + %lu $G sent, %lu dropped (TX ring full), %lu bad / %lu cut / %lu v1 / %lu wrong-version lines, %lu touches (last %s); "
           "puck page %u, thermal %s, oled %s, %u fps; link poll max %lu us (avg %lu), write max %lu us",
           s3linkUp() ? "UP" : "down", glink::VERSION, theirVersion < 0 ? "unheard" : (theirVersion == glink::VERSION ? "matches" : "DIFFERS"), (unsigned long)heartbeats, (unsigned long)sent, (unsigned long)liveSent, (unsigned long)txDrops,
           (unsigned long)parser.bad, (unsigned long)parser.truncated, (unsigned long)parser.v1, (unsigned long)versionBad, (unsigned long)touches, glink::touchName(lastTouch),
           hb.page, hb.mlx ? "ok" : "absent", hb.oled ? "ok" : "absent", hb.fps, (unsigned long)pollMaxEverUs, (unsigned long)(pollN ? pollSumUs / pollN : 0), (unsigned long)writeMaxUs);
  return String(b);
}
#if HW_FAKE
void s3linkInject(const char* line) { while (*line) feed(*line++); }
#endif
