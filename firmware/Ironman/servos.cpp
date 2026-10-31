#include "servos.h"
#include "config.h"
#include "hal.h"
#include "current.h"
#include "power.h"
#include <Wire.h>
#include <Preferences.h>
#include <GauntletBenchIO.h>
#if !HW_FAKE
#include <Adafruit_PWMServoDriver.h>
static Adafruit_PWMServoDriver pca(0x40, Wire);
#endif

static bool     ready = false;
static uint16_t usClosed = SERVO_US_CLOSED_DEFAULT;
static uint16_t usOpen   = SERVO_US_OPEN_DEFAULT;
static uint16_t chanUs[16];                 // the pulse each servo channel is getting, 0 = released
static uint16_t lastUs[16];                 // the last pulse each servo channel was given (it stays there once released), 0 = never since boot
static bool     digOn[16];                  // the static channels' state
static bool     oeOn = false;
static uint8_t  mode2Read = 0;              // MODE2 as read back at boot (2026-10-08: OUTNE 00 = every output LOW while OE is high)
uint8_t servosMode2() { return mode2Read; }

bool servosReady() { return ready; }
uint16_t servoUsClosed() { return usClosed; }
uint16_t servoUsOpen()   { return usOpen; }
void servoNudgeClosed(int16_t d) { usClosed += d; Bench.printf("us_closed = %u\n", usClosed); }
void servoNudgeOpen(int16_t d)   { usOpen   += d; Bench.printf("us_open   = %u\n", usOpen); }

// ---- set on the day, kept in NVS, no recompile (Toby, 2026-10-13: "make sure everything is ready to just be shoved in")
// (1) The channel map: the PCA9685 output each of the firmware's channels is wired to. Identity by default; Toby changed
// a couple of channels on the arm (2026-10-13, 00 #40) - set them with /pcamap?set=6=13 (each L=P a swap, so the map
// stays one-to-one). (2) Each servo's closed / open pulse, found on the real plate with the /cal page: they win over
// PLATES[] in config.h. Both live in the "ironman" namespace ("pcam", "sc0".."sc5" / "so0".."so5").
static uint8_t  chMap[16] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static uint16_t calC[PLATE_COUNT], calO[PLATE_COUNT];
static int8_t   calServo = -1;              // the servo the /cal page is holding now, -1 none
static int8_t   calPair = -1;               // (2026-10-16) its pair partner, moved with it (the page's "pair" box): plate 3, the hand plate
static bool     platesOff = false;          // (2026-10-16) the plates switched off on the /cal page: no plate ever moves (the servos are not set up yet);
                                            // the lasers and the camera then treat their plate as open and act at once. NVS "nopl".
static uint32_t calAt = 0;                  // when it was last moved (released after SERVO_CAL_HOLD_MS)
// (2026-10-20) a plate's move set on the arm: its time (0 = PLATES[].moveMs) and its motion (1 = constant speed - no ease in
// or out, so it leaves fast and lands fast: "momentum is your friend" - 0 = smoothstep); NVS "pm1".."pm4" / "pl1".."pl4"
static uint16_t plateMs[PLATE_GROUPS + 1] = { 0 };
static uint8_t  plateLin[PLATE_GROUPS + 1] = { 0, 0, 0, 0, 1 };   // PLATE_LINEAR_DEFAULT until liveLoad reads NVS
static uint32_t durOfPlate(uint8_t g);       // below (the planner)
static float    moveMaxA = 0, holdMaxA = 0;  // the servos' current at its highest while this sequence moved / held (the log line at its end)
static void pathSet(float a1, float b1, float a2, float b2);   // plate 3s path, live (below)
static void closeSet(float len, float ratio);
static void liveLoad() {
  Preferences p; if (!p.begin("ironman", true)) return;
  uint8_t m[16];
  if (p.getBytes("pcam", m, 16) == 16) { bool seen[16] = {}; bool ok = true; for (uint8_t i = 0; i < 16; i++) { if (m[i] > 15 || seen[m[i]]) ok = false; else seen[m[i]] = true; } if (ok) memcpy(chMap, m, 16); }
  for (uint8_t i = 0; i < PLATE_COUNT; i++) { char k[4] = { 's', 'c', (char)('0' + i), 0 }; calC[i] = p.getUShort(k, 0); k[1] = 'o'; calO[i] = p.getUShort(k, 0); }
  platesOff = p.getBool("nopl", false);
  for (uint8_t g = 1; g <= PLATE_GROUPS; g++) { char k[4] = { 'p', 'm', (char)('0' + g), 0 }; plateMs[g] = p.getUShort(k, 0); k[1] = 'l'; plateLin[g] = p.getUChar(k, PLATE_LINEAR_DEFAULT[g] ? 1 : 0); }
  { float v[6] = { 0, 0, 0, 0, PLATE3_CLOSE_LEN, PLATE3_CLOSE_RATIO }; size_t n = p.getBytes("p3p", v, sizeof v); if ((n == 16 || n == 24) && v[0] > 0 && v[2] > v[0]) { pathSet(v[0], v[1], v[2], v[3]); closeSet(v[4], v[5]); } }
  p.end();
}
uint8_t pcaPhysical(uint8_t ch) { return ch < 16 ? chMap[ch] : ch; }
bool pcaMapIsIdentity() { for (uint8_t i = 0; i < 16; i++) if (chMap[i] != i) return false; return true; }
bool servosPlatesOff() { return platesOff; }

// The one place the chip is written. on/off are the 12-bit counts; (4096, 0) = full on, (0, 4096) = full off.
// Every write goes to the output the channel is wired to (chMap).
static void pcaWrite(uint8_t ch, uint16_t on, uint16_t off) {
#if !HW_FAKE
  if (ready && ch < 16) pca.setPWM(chMap[ch], on, off);
#else
  (void)ch; (void)on; (void)off;
#endif
}

void servosPower(bool on) {
  // OE is active LOW. OE_DRIVE_LOW_ALWAYS was the 2026-09 bench state (an unwired OE); the build has it false,
  // so the outputs are live only while something is meant to be moving or on.
  // Switching OFF waits one PWM period first (2026-10-16): a released channel's "full off" takes effect at the end of
  // the chip's current 20 ms cycle, but OE cuts the outputs at once - raised in the middle of a pulse it left the servo
  // a runt pulse, read as "the far end": plate 1 flew fully open the moment it was let go after closing (Toby saw it,
  // the log had no move). One period later every released channel is already quiet.
  if (!on && oeOn && !OE_DRIVE_LOW_ALWAYS) delay(SERVO_OE_OFF_WAIT_MS);
  oeOn = on || OE_DRIVE_LOW_ALWAYS;
  halDigitalWrite(PIN_PCA_OE, oeOn ? LOW : HIGH);
}

static bool followWriting = false, followOff = false;   // followOff: the /cal page moves 3B on its own (finding its closed)
static uint16_t closedSameWay(uint8_t i) {              // a servo's closed pulse, config.h's mirror flag ignored (plate 3's turn the same way)
  if (calC[i]) return calC[i];
  if (PLATES[i].usClosed) return PLATES[i].usClosed;
  return (uint16_t)(1500 - PLATES[i].travelDeg * SERVO_US_PER_DEG / 2.0f);
}
// the path in use: PLATE3_PATH, or the knee and the back's stop set live on the /cal page (NVS "p3p": a1 b1 a2 b2)
static float path3[4][2] = { { PLATE3_PATH[0][0], PLATE3_PATH[0][1] }, { PLATE3_PATH[1][0], PLATE3_PATH[1][1] }, { PLATE3_PATH[2][0], PLATE3_PATH[2][1] }, { PLATE3_PATH[3][0], PLATE3_PATH[3][1] } };
static float closeLen = PLATE3_CLOSE_LEN, closeRatio = PLATE3_CLOSE_RATIO;   // the closing path (below)
static void closeSet(float len, float ratio) { closeLen = len; closeRatio = ratio; }
static void pathSet(float a1, float b1, float a2, float b2) { path3[0][0] = 0; path3[0][1] = 0; path3[1][0] = a1; path3[1][1] = b1; path3[2][0] = a2; path3[2][1] = b2; path3[3][0] = 180; path3[3][1] = b2; }
const char* servosPathSet(const String& spec) {          // "a1,b1,a2,b2[,cl,cr]" (degrees: the knee; where 3B stops; closing: 3A's first cl deg at cr:1) or "reset"
  Preferences p;
  if (spec == "reset") { pathSet(PLATE3_PATH[1][0], PLATE3_PATH[1][1], PLATE3_PATH[2][0], PLATE3_PATH[2][1]); closeSet(PLATE3_CLOSE_LEN, PLATE3_CLOSE_RATIO); if (p.begin("ironman", false)) { p.remove("p3p"); p.end(); } return ""; }
  float v[6] = { 0, 0, 0, 0, closeLen, closeRatio }; int from = 0, nv = 0;
  for (int k = 0; k < 6 && from <= (int)spec.length(); k++) { int c = spec.indexOf(',', from); String s = c < 0 ? spec.substring(from) : spec.substring(from, c); s.trim(); if (!s.length()) break; v[k] = s.toFloat(); nv++; from = c < 0 ? spec.length() + 1 : c + 1; }
  if (nv != 4 && nv != 6) return "write it as a1,b1,a2,b2 or a1,b1,a2,b2,cl,cr (degrees; cr = the front's degrees per the back's while closing)";
  if (!(v[4] >= 0 && v[4] <= 120 && v[5] >= 1 && v[5] <= 6)) return "closing: cl 0-120 deg, cr 1-6";
  if (!(v[0] > 0 && v[2] > v[0] && v[1] >= 0 && v[3] >= v[1] && v[3] <= 175 && v[2] <= 175)) return "a1 > 0, a2 > a1, b2 >= b1 >= 0, b2 <= 175, a2 <= 175";
  pathSet(v[0], v[1], v[2], v[3]); closeSet(v[4], v[5]);
  if (p.begin("ironman", false)) { p.putBytes("p3p", v, sizeof v); p.end(); }
  Bench.printf("plate 3 path: 3B %.1f deg at 3A %.1f, then 1:%.2f to 3B %.1f deg at 3A %.1f, then 3B stays\n", v[1], v[0], (v[3] - v[1]) / (v[2] - v[0]), v[3], v[2]);
  return "";
}
String servosPathJson() { char b[96]; snprintf(b, sizeof b, "[%.1f,%.1f,%.1f,%.1f,%.1f,%.2f]", path3[1][0], path3[1][1], path3[2][0], path3[2][1], closeLen, closeRatio); return b; }
static float pathDeg(float a) {                          // the path: 3B's degrees for 3A's (the first slope extends below closed)
  constexpr uint8_t n = 4;
  #define PLATE3_PATH path3
  if (a <= PLATE3_PATH[0][0]) return PLATE3_PATH[0][1] + (a - PLATE3_PATH[0][0]) * (PLATE3_PATH[1][1] - PLATE3_PATH[0][1]) / (PLATE3_PATH[1][0] - PLATE3_PATH[0][0]);
  for (uint8_t k = 1; k < n; k++)
    if (a <= PLATE3_PATH[k][0]) return PLATE3_PATH[k - 1][1] + (a - PLATE3_PATH[k - 1][0]) * (PLATE3_PATH[k][1] - PLATE3_PATH[k - 1][1]) / (PLATE3_PATH[k][0] - PLATE3_PATH[k - 1][0]);
  return PLATE3_PATH[n - 1][1];
  #undef PLATE3_PATH
}
struct Frame3 { float cA, dA, cB, dB; };
static Frame3 frame3() {                                 // each servo's closed pulse and its opening direction
  constexpr uint8_t L = PLATE3_LEADER, F = PLATE3_FOLLOWER;
  Frame3 f; f.cA = closedSameWay(L);
  float oA = calO[L] ? calO[L] : (PLATES[L].usOpen ? PLATES[L].usOpen : f.cA + PLATES[L].travelDeg * SERVO_US_PER_DEG);
  f.dA = oA >= f.cA ? 1.0f : -1.0f;
  f.cB = closedSameWay(F); f.dB = (calC[F] && calO[F]) ? (calO[F] >= calC[F] ? 1.0f : -1.0f) : f.dA;
  return f;
}
static uint16_t usOfB(const Frame3& f, float degB) { int us = (int)(f.cB + f.dB * degB * SERVO_US_PER_DEG + 0.5f); return (uint16_t)(us < (int)SERVO_US_MIN ? SERVO_US_MIN : (us > (int)SERVO_US_MAX ? SERVO_US_MAX : us)); }
uint16_t servosFollowerUs(uint16_t leaderUs) {           // 3B's pulse for 3A at leaderUs on the OPENING path
  Frame3 f = frame3(); return usOfB(f, pathDeg(((float)leaderUs - f.cA) * f.dA / SERVO_US_PER_DEG));
}
// Closing is not opening backwards (Toby, 2026-10-16: "on the way down the front closes more - about two to one at first,
// until about 45 deg, then they close together"): from where the close starts (cAs, cBs), 3A's first closeLen degrees go
// at closeRatio:1 against 3B's, then the two run straight to closed together.
static int8_t leadDir = 1;                               // the leader's last move: +1 opening, -1 closing
static float cAs = 0, cBs = 0;                           // where the current close started (deg)
static float closeDeg(float a) {
  float L = closeLen < cAs * 0.6f ? closeLen : cAs * 0.6f;
  float ak = cAs - L, bk = cBs - L / closeRatio; if (bk < 0) bk = 0;
  if (a >= ak) return cBs - (cAs - a) / closeRatio;
  return ak > 0.5f ? bk * a / ak : 0;
}
static uint16_t followerFor(uint16_t us, uint16_t prevUs) {
  Frame3 f = frame3();
  float a = ((float)us - f.cA) * f.dA / SERVO_US_PER_DEG, ap = prevUs ? ((float)prevUs - f.cA) * f.dA / SERVO_US_PER_DEG : a;
  if (a < ap - 0.05f && leadDir > 0) {                   // a close starts here: from where the two are now
    uint16_t bUs = lastUs[PLATES[PLATE3_FOLLOWER].ch];
    leadDir = -1; cAs = ap; cBs = bUs ? ((float)bUs - f.cB) * f.dB / SERVO_US_PER_DEG : pathDeg(ap);
  } else if (a > ap + 0.05f) leadDir = 1;
  return usOfB(f, leadDir < 0 ? closeDeg(a) : pathDeg(a));
}
void servosFollowSet(bool on) { followOff = !on; }
bool servosFollowOn() { return PLATE3_FOLLOW && !followOff; }
void servoWriteUs(uint8_t ch, uint16_t us) {
  if (ch >= SERVO_COUNT) return;                        // 6-15 are static outputs, never pulses
  if (servosFollowOn() && ch == PLATES[PLATE3_FOLLOWER].ch && !followWriting) return;   // 3B moves only with 3A (PLATE3_PATH)
  if (us < SERVO_US_MIN) us = SERVO_US_MIN;             // the envelope, always
  if (us > SERVO_US_MAX) us = SERVO_US_MAX;
  uint32_t ticks = ((uint32_t)us * 4096UL) / (1000000UL / PCA_FREQ);   // 4096 counts per 20 ms
  uint16_t prevUs = lastUs[ch];
  chanUs[ch] = us; lastUs[ch] = us;
  pcaWrite(ch, 0, (uint16_t)ticks);
  if (servosFollowOn() && ch == PLATES[PLATE3_LEADER].ch) { followWriting = true; servoWriteUs(PLATES[PLATE3_FOLLOWER].ch, followerFor(us, prevUs)); followWriting = false; }
}

void servoRelease(uint8_t ch) {
  if (ch < 16) { chanUs[ch] = 0; if (ch >= SERVO_COUNT) digOn[ch] = false; }
  pcaWrite(ch, 0, 4096);   // "full off" = no pulses
  if (servosFollowOn() && ch == PLATES[PLATE3_LEADER].ch) { uint8_t f = PLATES[PLATE3_FOLLOWER].ch; chanUs[f] = 0; pcaWrite(f, 0, 4096); }   // 3B goes limp with 3A (2026-10-16: it stayed driven after a chained move)
}   // "full off" = no pulses
uint16_t servoChannelUs(uint8_t ch) { return ch < SERVO_COUNT ? chanUs[ch] : 0; }

bool pcaSetDigital(uint8_t ch, bool on) {
  if (ch < SERVO_COUNT || ch > 15 || !ready) return false;
  digOn[ch] = on;
  if (on) {
    if (!oeOn) servosPower(true);                       // a static output needs OE live
    pcaWrite(ch, 4096, 0);
  } else {
    pcaWrite(ch, 0, 4096);
    // leave OE to whoever else needs it: the cascade releases it when it ends, pucksPoll/outputs call servosPower(false) when all off
  }
  return true;
}
bool    pcaDigitalState(uint8_t ch) { return ch < 16 ? digOn[ch] : false; }
uint8_t pcaDigitalOnCount() { uint8_t n = 0; for (uint8_t c = SERVO_COUNT; c < 16; c++) if (digOn[c]) n++; return n; }

void servosParkAll() {
  calServo = -1; calPair = -1;
  for (uint8_t ch = 0; ch < 16; ch++) servoRelease(ch);
  servosPower(false);
}

void i2cScan() {
#if !HW_FAKE
  uint8_t n = 0;
  for (uint8_t a = 8; a < 120; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Bench.printf("  0x%02X", a);
      if (a == 0x40) Bench.print("   <- PCA9685");
      if (a == 0x41) Bench.print("   <- INA219");
      if (a == 0x4A || a == 0x4B) Bench.print("   <- BNO085 (forearm)");
      if (a == 0x70) Bench.print("   <- PCA9685 All Call (same chip, normal)");
      Bench.println();
      n++;
    }
  }
  if (!n) Bench.println("  nothing on the bus");
#else
  Bench.println("  (fake build: no bus)");
#endif
}

bool servosBegin() {
  // OE first - before the chip is touched nothing can twitch. (hal / setup() already parked it high at power-on.)
  halPinMode(PIN_PCA_OE, OUTPUT);
  servosPower(false);
  for (uint8_t c = 0; c < 16; c++) { chanUs[c] = 0; digOn[c] = false; }
  liveLoad();   // the channel map and the calibrated servo ends saved on the day (NVS)
#if !HW_FAKE
  Wire.beginTransmission(0x40);
  ready = (Wire.endTransmission() == 0);
  if (!ready) { Bench.println("PCA9685 NOT responding on bus 1 - servo, laser, vibrator and puck commands locked out"); return false; }
  pca.begin();
  pca.setOscillatorFrequency(PCA_OSC_HZ);
  pca.setPWMFreq(PCA_FREQ);
  // MODE2 written every boot, not left to the chip's power-on default: a reset of the ESP32 does not reset the PCA9685, and
  // what its outputs do while OE is high is this register's OUTNE bits (00 = LOW). Then read back.
  Wire.beginTransmission(0x40); Wire.write(0x01); Wire.write(PCA_MODE2_VALUE); Wire.endTransmission();
  Wire.beginTransmission(0x40); Wire.write(0x01); Wire.endTransmission(false);
  mode2Read = (Wire.requestFrom(0x40, 1) == 1) ? (uint8_t)Wire.read() : 0xFF;
#else
  ready = true;
  mode2Read = PCA_MODE2_VALUE;                // the fake chip keeps what it is given
#endif
  Bench.printf("PCA9685 MODE2 0x%02X - %s\n", mode2Read, mode2Read == PCA_MODE2_VALUE ? "totem pole, every output LOW while OE is high (a reset: OE floats high on its 1 k pull-up = all off)" : "NOT what was written (0x04) - check bus 1");
  for (uint8_t ch = 0; ch < 16; ch++) servoRelease(ch);
  Bench.printf("PCA9685 ready, %u Hz, channels 0-5 parked, 6-15 off, OE %s\n", PCA_FREQ,
               OE_DRIVE_LOW_ALWAYS ? "driven LOW (outputs enabled)" : "HIGH until a move or an output");
  return true;
}

// Waits for 'x' on the console while keeping the network alive. Returns true if 'x' arrived.
static bool waitOrX(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    Bench.poll(); halWdtFeed(); delay(20);
    if (Bench.available() && tolower(Bench.read()) == 'x') return true;
  }
  return false;
}

void servoSweepOnce() {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.printf("servo ch0: %u -> %u us, easing. watch the rail.\n", usClosed, usOpen);
  servosPower(true);
  delay(20);
  const int steps = 60;
  for (int i = 0; i <= steps; i++) {
    float t = i / (float)steps; t = t * t * (3.0f - 2.0f * t);
    servoWriteUs(0, usClosed + (int)((usOpen - usClosed) * t));
    halWdtFeed(); delay(12);
  }
  delay(400);
  for (int i = steps; i >= 0; i--) {
    float t = i / (float)steps; t = t * t * (3.0f - 2.0f * t);
    servoWriteUs(0, usClosed + (int)((usOpen - usClosed) * t));
    halWdtFeed(); delay(12);
  }
  delay(300);
  servoRelease(0);           // a stalled MG90S that keeps getting pulses cooks itself
  if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("sweep done - servo released (normal)");
}

void servoHold(uint16_t us) {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.printf("holding ch0 at %u us for 1.5 s\n", us);
  servosPower(true); delay(20);
  servoWriteUs(0, us);
  delay(1500);
  servoRelease(0); if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("released");
}

void servoHoldForMeter() {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.printf("holding ch0 at %u us for 20 s - METER NOW (x ends early)\n", usClosed);
  Bench.println("  expect: PCA V+ ~5 V | ch0 signal pin ~0.2 V avg");
  servosPower(true); delay(20);
  servoWriteUs(0, usClosed);
  waitOrX(20000);
  servoRelease(0); if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("hold done - servo released (normal)");
}

// Full travel on channel 0, 1000 <-> 2000 us, three times. ONLY for a servo that is not attached to anything.
void servoFullTravel() {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.println("ch0 FULL TRAVEL 1000 <-> 2000 us x3 - unmounted servo only");
  servosPower(true);
  delay(20);
  for (int rep = 0; rep < 3; rep++) {
    for (int us = 1000; us <= 2000; us += 10) { servoWriteUs(0, us); halWdtFeed(); delay(6); }
    delay(300);
    for (int us = 2000; us >= 1000; us -= 10) { servoWriteUs(0, us); halWdtFeed(); delay(6); }
    delay(300);
  }
  servoRelease(0);
  if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("full travel done - servo released");
}

// Find the servo's real limits: slow 600 <-> 2400 us on channel 0, once. Bare servo ONLY.
void servoWideTravel() {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.printf("ch0 WIDE %u -> %u -> %u us (the envelope), slow. bare servo only\n", SERVO_US_MIN, SERVO_US_MAX, SERVO_US_MIN);
  servosPower(true);
  delay(20);
  servoWriteUs(0, 1500); delay(600);
  for (int us = 1500; us >= SERVO_US_MIN; us -= 10) { servoWriteUs(0, us); halWdtFeed(); delay(15); }
  Bench.printf("  at %u us\n", SERVO_US_MIN);  delay(1200);
  for (int us = SERVO_US_MIN; us <= SERVO_US_MAX; us += 10) { servoWriteUs(0, us); halWdtFeed(); delay(15); }
  Bench.printf("  at %u us\n", SERVO_US_MAX); delay(1200);
  for (int us = SERVO_US_MAX; us >= 1500; us -= 10) { servoWriteUs(0, us); halWdtFeed(); delay(15); }
  delay(400);
  servoRelease(0);
  if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("wide travel done - servo released");
}

// ---------------- the cascade -------------------------------------------------
// A sequence is a handful of moves with start times relative to t0. Each move eases from -> to over
// its plate's moveMs. servosPoll() advances all active moves every 10 ms, holds after the last one
// lands, then releases. A stalled servo gets no more than the hold plus CASCADE_STALL_GRACE_MS.
struct Move { uint8_t plate; bool open; uint32_t start; uint16_t dur; int16_t from, to; bool done; bool lin; };
static Move        seq[8];
static uint8_t     seqN = 0;
static uint32_t    seqT0 = 0, seqLastEnd = 0, seqLastTick = 0;
static bool        seqActive = false, seqThenClose = false;
static const char* seqName = "";
static PlatesState platesState = PlatesState::Closed;   // (2026-10-20) after a boot every plate is taken as CLOSED - where they rest when the arm is switched on (unknown blocked a single close and made a move start from the far end)
static uint32_t    cascades = 0, plateMoves = 0, seqStarts = 0;   // completed cascades, completed single-plate moves, sequences started (every kind)
static int8_t      landed[PLATE_COUNT] = { 0, 0, 0, 0, 0, 0 };   // where each servo last arrived: 1 open, 0 closed, -1 unknown (stopped mid-move); closed at boot (2026-10-20)
static uint32_t    pendingCloseAt = 0;      // open-pause-close: the close starts at this time (0 = none) - a timed step, never a delay() in the loop
static uint8_t     seqPlate = 0;            // 0 = a cascade; 1..4 = one plate moving alone (PLATE_GROUP_OF)
static PlatesState fromLanded() {           // the plates as a whole, from where each servo arrived
  bool allOpen = true, allClosed = true;
  for (uint8_t i = 0; i < PLATE_COUNT; i++) { if (landed[i] < 0) return PlatesState::Unknown; if (landed[i] != 1) allOpen = false; if (landed[i] != 0) allClosed = false; }
  return allOpen ? PlatesState::Open : allClosed ? PlatesState::Closed : PlatesState::Mixed;
}
// Stall detection on the INA219 (current.cpp): the servos' own current over SERVO_STALL_TOTAL_A while moving, or
// over SERVO_HOLD_STALL_A once every move has landed, for SERVO_STALL_MS = every servo of the cascade released.
static uint32_t    stallOverSince = 0, stalls = 0;
static char        lastStall[110] = "";
uint32_t    servosStallCount() { return stalls; }
const char* servosLastStall()  { return lastStall; }

bool servosBusy() { return seqActive || pendingCloseAt; }   // the pause of open-pause-close counts: nothing else may start in it
PlatesState servosPlatesState() { return platesState; }
const char* servosPlatesStateName() {
  switch (platesState) { case PlatesState::Open: return "open"; case PlatesState::Closed: return "closed"; case PlatesState::Opening: return "opening"; case PlatesState::Closing: return "closing"; case PlatesState::Mixed: return "mixed"; default: return "unknown"; }
}
uint32_t servosCascades() { return cascades; }
uint32_t servosPlateMoves() { return plateMoves; }
uint32_t servosStarts()     { return seqStarts; }

static uint16_t plateUs(uint8_t i, bool open) {
  const PlateDef& p = PLATES[i];
  uint16_t saved = open ? calO[i] : calC[i];                    // calibrated on the real plate with the /cal page (NVS)
  if (saved) return saved;
  uint16_t configured = open ? p.usOpen : p.usClosed;
  if (configured) return configured;
  uint16_t half = (uint16_t)(p.travelDeg * SERVO_US_PER_DEG / 2.0f);
  bool up = open != p.mirror;                                   // the mirrored servo of a pair turns the other way
  return up ? 1500 + half : 1500 - half;
}

// ---- the /cal page: one servo at a time, on the real plate; the ends it saves win over config.h ----
uint16_t servoEndUs(uint8_t i, bool open) { return i < PLATE_COUNT ? plateUs(i, open) : 0; }   // in effect now
uint16_t servoCalSaved(uint8_t i, bool open) { return i < PLATE_COUNT ? (open ? calO[i] : calC[i]) : 0; }
int8_t   servoCalHeld() { return calServo; }
int8_t   servoCalPair() { return calPair; }
uint16_t servoLastUs(uint8_t i) { return i < PLATE_COUNT ? lastUs[PLATES[i].ch] : 0; }
// hold servo i at us (the envelope clamps it); pair 1 / 2 = its partner on the same plate too, so the two never drag
// each other through the plate (Toby, 2026-10-16, plate 3). Until BOTH servos of the pair have saved ends: 1 = the same
// pulse (Toby: "plate 3's two open the same way as plate 2 - open them the same amount"; config.h's mirror flag on
// plate 3B is not true of his arm), 2 = mirrored about 1500 us. With both saved: the same fraction of each one's own
// closed -> open, whichever way each turns. "" = done
const char* servoCalMove(uint8_t i, uint16_t us, uint8_t pair) {
  if (i >= PLATE_COUNT) return "no such servo";
  if (!ready) return "PCA9685 not found";
  if (seqActive || pendingCloseAt) return "a plate move is running";
  int8_t j = -1;
  if (pair) for (uint8_t k = 0; k < PLATE_COUNT; k++) if (k != i && PLATE_GROUP_OF[k] == PLATE_GROUP_OF[i]) j = (int8_t)k;
  if (calServo >= 0 && calServo != (int8_t)i && calServo != j) servoRelease(PLATES[calServo].ch);
  if (calPair >= 0 && calPair != (int8_t)i && calPair != j) servoRelease(PLATES[calPair].ch);
  servosPower(true);
  servoWriteUs(PLATES[i].ch, us);
  calServo = (int8_t)i; calPair = -1; calAt = millis();
  if (j >= 0) {
    int uj = pair == 2 ? 3000 - (int)us : (int)us;
    if (calC[i] && calO[i] && calC[j] && calO[j] && calO[i] != calC[i]) {
      float f = ((float)us - calC[i]) / ((float)calO[i] - calC[i]);
      uj = (int)(calC[j] + f * ((float)calO[j] - calC[j]) + 0.5f);
    }
    if (uj < (int)SERVO_US_MIN) uj = SERVO_US_MIN;
    if (uj > (int)SERVO_US_MAX) uj = SERVO_US_MAX;
    servoWriteUs(PLATES[j].ch, (uint16_t)uj);
    calPair = j;
  }
  return "";
}
static void saveEnd(uint8_t i, bool open, uint16_t us) {
  (open ? calO : calC)[i] = us;
  Preferences p; if (p.begin("ironman", false)) { char k[4] = { 's', (char)(open ? 'o' : 'c'), (char)('0' + i), 0 }; p.putUShort(k, us); p.end(); }
  Bench.printf("cal: %s %s = %u us (saved)\n", PLATES[i].name, open ? "OPEN" : "CLOSED", us);
}
// (2026-10-20) an end set by number, no move - the hand plate's pair is not symmetric, so the mirrored nudge cannot put
// its two servos at two different fractions (Toby: left a little short of halfway, right a little past). keep false =
// tried until a reboot (every plate move uses it at once), keep true = saved in NVS like a Save on the page.
const char* servoCalSetEnd(uint8_t i, bool open, int us, bool keep) {
  if (i >= PLATE_COUNT) return "no such servo";
  if (us < (int)SERVO_US_MIN || us > (int)SERVO_US_MAX) return "outside the servo envelope";
  if (seqActive || pendingCloseAt) return "a plate move is running";
  if (keep) saveEnd(i, open, (uint16_t)us);
  else { (open ? calO : calC)[i] = (uint16_t)us; Bench.printf("cal: %s %s = %d us (to try - not saved)\n", PLATES[i].name, open ? "OPEN" : "CLOSED", us); }
  return "";
}
// (2026-10-20) a plate's move set on the arm: ms 0 = config.h's PLATES[].moveMs, -1 = unchanged; lin 1 constant speed, 0 smoothstep,
// -1 unchanged; keep = saved in NVS, else tried until a reboot
const char* servosPlateMotion(uint8_t g, int ms, int lin, bool keep) {
  if (g < 1 || g > PLATE_GROUPS) return "plate 1-4";
  if (ms > 0 && (ms < 80 || ms > 3000)) return "a move time of 80-3000 ms (0 = config.h's)";
  if (seqActive || pendingCloseAt) return "a plate move is running";
  if (ms >= 0) plateMs[g] = (uint16_t)ms;
  if (lin >= 0) plateLin[g] = lin ? 1 : 0;
  if (keep) { Preferences p; if (p.begin("ironman", false)) { char k[4] = { 'p', 'm', (char)('0' + g), 0 }; if (plateMs[g]) p.putUShort(k, plateMs[g]); else p.remove(k); k[1] = 'l'; if ((plateLin[g] != 0) != PLATE_LINEAR_DEFAULT[g]) p.putUChar(k, plateLin[g]); else p.remove(k); p.end(); } }
  Bench.printf("plates: %s moves in %u ms, %s%s\n", servosPlateName(g), (unsigned)durOfPlate(g), plateLin[g] ? "constant speed" : "smoothstep", keep ? " (saved)" : " (to try - not saved)");
  return "";
}
bool     servosPlateLinear(uint8_t g) { return g >= 1 && g <= PLATE_GROUPS && plateLin[g]; }
uint32_t servosPlateMs(uint8_t g)     { return g >= 1 && g <= PLATE_GROUPS ? durOfPlate(g) : 0; }
String servosPlateMotionJson(uint8_t g) { return g < 1 || g > PLATE_GROUPS ? String("{}") : "{\"plate\":" + String(g) + ",\"ms\":" + String(durOfPlate(g)) + ",\"lin\":" + String(plateLin[g]) + "}"; }
// the pulse servo i is holding now becomes its closed / open end; with both, its pair partner's too
const char* servoCalSave(uint8_t i, bool open, bool both) {
  if (i >= PLATE_COUNT) return "no such servo";
  if (calServo != (int8_t)i || !chanUs[PLATES[i].ch]) return "move that servo to the end first (it is not being held)";
  saveEnd(i, open, chanUs[PLATES[i].ch]);
  if (both && calPair >= 0 && chanUs[PLATES[calPair].ch]) saveEnd((uint8_t)calPair, open, chanUs[PLATES[calPair].ch]);
  return "";
}
void servoCalClear(uint8_t i) {
  if (i >= PLATE_COUNT) return;
  calC[i] = calO[i] = 0;
  Preferences p; if (p.begin("ironman", false)) { char k[4] = { 's', 'c', (char)('0' + i), 0 }; p.remove(k); k[1] = 'o'; p.remove(k); p.end(); }
  Bench.printf("cal: %s back to config.h's ends (%u / %u us)\n", PLATES[i].name, plateUs(i, false), plateUs(i, true));
}
void servosSetPlatesOff(bool off) {            // the /cal page's switch: off = no plate ever moves (kept in NVS)
  if (off && !platesOff) { servosAbort(); servoCalRelease(); }
  platesOff = off;
  Preferences p; if (p.begin("ironman", false)) { if (off) p.putBool("nopl", true); else p.remove("nopl"); p.end(); }
  Bench.println(off ? "plates: switched OFF - no plate moves (cascade, gestures, the lasers' and the camera's plates); the /cal page still moves one servo" : "plates: switched ON - the plates move again");
}
// the /cal page's quick move (Toby, 2026-10-16: plate 3 "slides - it is a momentum thing, do it quickly"): servo i from
// from to to in ms, smoothstep, run by servosPoll - plate 3's back servo follows along the path the whole way; held
// at the end afterwards (let go as any hold: release=1, 15 s, the current guard)
static struct { bool on; uint8_t i; uint16_t from, to; uint32_t t0, ms; uint8_t pair; } calSw = { false, 0, 0, 0, 0, 0, 0 };
static struct { bool on; uint8_t i; uint16_t from, to; uint32_t ms; } calNext = { false, 0, 0, 0, 0 };   // then= (Toby: "plate 2, then plate 3 the instant it lands")
void servoCalSweepThen(uint8_t i, uint16_t from, uint16_t to, uint32_t ms) { calNext.on = i < PLATE_COUNT; calNext.i = i; calNext.from = from; calNext.to = to; calNext.ms = ms; }
const char* servoCalSweep(uint8_t i, uint16_t from, uint16_t to, uint32_t ms, uint8_t pair) {
  const char* e = servoCalMove(i, from, pair);
  if (*e) return e;
  calSw.pair = (pair && calPair >= 0) ? pair : 0;
  calSw.on = true; calSw.i = i; calSw.from = from; calSw.to = to; calSw.t0 = millis(); calSw.ms = ms < 60 ? 60 : (ms > 5000 ? 5000 : ms);
  return "";
}
static int pairUs(uint8_t i, int8_t j, uint16_t us, uint8_t pair) {   // the partner's pulse for servo i at us
  int uj = pair == 2 ? 3000 - (int)us : (int)us;
  if (calC[i] && calO[i] && calC[j] && calO[j] && calO[i] != calC[i]) {   // both saved: the same fraction of each one's travel
    float f = ((float)us - calC[i]) / ((float)calO[i] - calC[i]);
    uj = (int)(calC[j] + f * ((float)calO[j] - calC[j]) + 0.5f);
  }
  return uj < (int)SERVO_US_MIN ? SERVO_US_MIN : (uj > (int)SERVO_US_MAX ? SERVO_US_MAX : uj);
}
static void calSweepPoll() {
  if (!calSw.on) return;
  if (calServo != (int8_t)calSw.i) { calSw.on = false; calNext.on = false; return; }   // released or taken over
  float t = (float)(millis() - calSw.t0) / calSw.ms;
  if (t >= 1.0f) {
    servoWriteUs(PLATES[calSw.i].ch, calSw.to); if (calSw.pair && calPair >= 0) servoWriteUs(PLATES[calPair].ch, (uint16_t)pairUs(calSw.i, calPair, calSw.to, calSw.pair));
    calSw.on = false; calAt = millis();
    if (calNext.on) { calNext.on = false; servoCalSweep(calNext.i, calNext.from, calNext.to, calNext.ms); }   // the next plate, at once
    return;
  }
  float e = t * t * (3.0f - 2.0f * t);
  uint16_t u = (uint16_t)(calSw.from + ((float)calSw.to - calSw.from) * e + 0.5f);
  servoWriteUs(PLATES[calSw.i].ch, u);
  if (calSw.pair && calPair >= 0) servoWriteUs(PLATES[calPair].ch, (uint16_t)pairUs(calSw.i, calPair, u, calSw.pair));   // the partner in step
  calAt = millis();
}
bool servoCalSweeping() { return calSw.on; }
static uint32_t calOverSince = 0;                // a /cal hold that draws too much is let go on the board
static void calGuardPoll() {
  if (calServo < 0 || !currentPresent()) { calOverSince = 0; return; }
  if (currentServoA() > SERVO_CAL_STALL_A) {
    if (!calOverSince) calOverSince = millis();
    else if (millis() - calOverSince > SERVO_CAL_STALL_MS) { Bench.printf("cal: %s let go - the servos drew %.2f A for %u ms\n", PLATES[calServo].name, currentServoA(), (unsigned)SERVO_CAL_STALL_MS); calSw.on = false; servoCalRelease(); calOverSince = 0; }
  } else calOverSince = 0;
}
void servoCalRelease() {
  if (calPair >= 0) { servoRelease(PLATES[calPair].ch); calPair = -1; }
  if (calServo == PLATE3_LEADER || calServo == PLATE3_FOLLOWER) { servoRelease(PLATES[PLATE3_LEADER].ch); servoRelease(PLATES[PLATE3_FOLLOWER].ch); }
  if (calServo < 0) return;
  servoRelease(PLATES[calServo].ch); calServo = -1;
  if (!seqActive && !pcaDigitalOnCount()) servosPower(false);
}
// the channel map: "6=13,9=6" (each L=P a swap, so it stays one-to-one) or "reset"; only with everything parked
const char* pcaMapSet(const String& spec) {
  if (seqActive || pendingCloseAt) return "a plate move is running";
  for (uint8_t c = 0; c < 16; c++) if (chanUs[c] || digOn[c]) return "something is being driven - everything off first (key X)";
  uint8_t m[16]; memcpy(m, chMap, 16);
  if (spec == "reset") { for (uint8_t i = 0; i < 16; i++) m[i] = i; }
  else {
    int from = 0; bool any = false;
    while (from <= (int)spec.length()) {
      int comma = spec.indexOf(',', from); if (comma < 0) comma = spec.length();
      String tok = spec.substring(from, comma); tok.trim(); from = comma + 1;
      if (!tok.length()) continue;
      int eq = tok.indexOf('='); if (eq < 0) eq = tok.indexOf(':');   // 6=13 or 6:13 (the page sends ':', no URL encoding needed)
      if (eq < 1) return "write each as LOGICAL=PCA, e.g. 6=13";
      int L = tok.substring(0, eq).toInt(), P = tok.substring(eq + 1).toInt();
      if (L < 0 || L > 15 || P < 0 || P > 15 || !isDigit(tok[0]) || !isDigit(tok[eq + 1])) return "channels are 0-15";
      uint8_t j = 0; while (m[j] != P) j++;
      m[j] = m[L]; m[L] = (uint8_t)P; any = true;
    }
    if (!any) return "nothing to set";
  }
  servosParkAll();                                 // everything off under the old map
  memcpy(chMap, m, 16);
  Preferences p; if (p.begin("ironman", false)) { p.putBytes("pcam", chMap, 16); p.end(); }
  servosParkAll();                                 // and under the new one
  String s = "PCA map:"; for (uint8_t i = 0; i < 16; i++) if (chMap[i] != i) s += " " + String(i) + "->" + String(chMap[i]);
  Bench.println(pcaMapIsIdentity() ? "PCA map: every channel on its own output (identity)" : s.c_str());
  return "";
}

static uint32_t endOf(uint8_t idx) { return seq[idx].start + seq[idx].dur; }

// A move starts from the pulse the servo was last given (a released MG90S stays where it was put), so a plate that
// is already open and is told to open just holds; never driven since boot = from the far end, as before.
static void addMove(uint8_t plate, bool open, uint32_t start) {
  if (seqN >= 8 || plate >= PLATE_COUNT) return;
  uint16_t last = lastUs[PLATES[plate].ch];
  int16_t  from = last ? (int16_t)last : (int16_t)plateUs(plate, false);   // no pulse yet (a boot): from closed, never from the far end (2026-10-20: a close after a reboot snapped plate 3 open first)
  const uint8_t g = PLATE_GROUP_OF[plate];
  seq[seqN++] = { plate, open, start, (uint16_t)(plateMs[g] ? plateMs[g] : PLATES[plate].moveMs), from, (int16_t)plateUs(plate, open), false, plateLin[g] != 0 };
}

static void startSeq(const char* name) {
  calServo = -1; calPair = -1;                                  // a plate move takes over a servo the /cal page was holding
  if (!ready) { Bench.println("PCA9685 not found - no cascade"); seqN = 0; return; }
  seqName = name;
  seqLastEnd = 0;
  for (uint8_t i = 0; i < seqN; i++) if (endOf(i) > seqLastEnd) seqLastEnd = endOf(i);
  Bench.printf("cascade %s (%lu ms):\n", name, (unsigned long)seqLastEnd);
  for (uint8_t i = 0; i < seqN; i++)
    Bench.printf("  t+%4lu ms  %-10s ch%u  %u -> %u us  (%u ms)\n", (unsigned long)seq[i].start,
                 PLATES[seq[i].plate].name, PLATES[seq[i].plate].ch, seq[i].from, seq[i].to, seq[i].dur);
  servosPower(true);
  seqT0 = millis(); seqLastTick = 0; seqActive = true; stallOverSince = 0; seqStarts++; moveMaxA = holdMaxA = 0;
}

// Releases every servo of the sequence; moves that had started and not landed leave their servo "unknown".
static void endSeq() {
  uint32_t t = millis() - seqT0;
  for (uint8_t i = 0; i < seqN; i++) {
    servoRelease(PLATES[seq[i].plate].ch);
    if (!seq[i].done && t >= seq[i].start) landed[seq[i].plate] = -1;
  }
  if (!pcaDigitalOnCount()) servosPower(false);
  seqActive = false; seqThenClose = false; seqPlate = 0;
}

// The plates: 0 plate 1, 1 plate 2, 2 + 3 plate 3's pair (A, B mirrored), 4 + 5 the hand plate's pair (A, B
// mirrored). Each pair moves in ONE step - same start, same travel, same duration (Toby, 2026-10-07, 00 #28;
// config.h checks the pairs match). The ORDER is mechanical (Toby, 2026-10-07 night; config.h PLATE_NEEDS /
// PLATE_OPEN_ORDER): plate 1 is on its own, plate 2 needs the hand plate open, plate 3 needs the hand plate and plate 2.
// Opening runs PLATE_OPEN_ORDER: the hand plate first, plate 2 once it has landed (PLATE_CLEAR_TRAVEL 1; 30 % until 2026-10-31 - the two caught each other) with plate 1
// alongside (PLATE_WITH), plate 3 the instant plate 2 lands; closing runs it backwards: plate 3, then plate 2 + plate 1,
// then the hand plate once plate 2 is home (2026-10-20, the hand plate fitted). A plate
// closes only when nothing still needs it open: a plate that depends on it, or a HOLD (the thermal camera, a laser).
// Every path goes through planOpen / planClose: the cascade, one plate, the lasers, the camera.
static void addPlate(uint8_t plateNo, bool open, uint32_t start) {   // every servo of one plate (PLATE_GROUP_OF), together
  for (uint8_t i = 0; i < PLATE_COUNT; i++) if (PLATE_GROUP_OF[i] == plateNo) addMove(i, open, start);
}
static uint32_t endOfPlate(uint8_t plateNo) {
  uint32_t e = 0;
  for (uint8_t k = 0; k < seqN; k++) if (PLATE_GROUP_OF[seq[k].plate] == plateNo && endOf(k) > e) e = endOf(k);
  return e;
}
static uint8_t  holds[PLATE_GROUPS + 1] = { 0 };   // [plate] = who needs it open (SERVO_HOLD_*)
static uint8_t  reqOpen = 0, reqClose = 0;         // plates asked for while a move was running (the lasers, the camera): planned when it ends
static char     refusal[110] = "";
const char* servosLastRefusal() { return refusal; }
const char* servosPlateName(uint8_t g) { static const char* const N[] = { "?", "plate 1", "plate 2", "plate 3", "the hand plate" }; return g <= PLATE_GROUPS ? N[g] : "?"; }
static const char* holderText(uint8_t h) { return (h & SERVO_HOLD_THERMAL) ? "the thermal camera is on" : (h & SERVO_HOLD_RED) ? "the red laser is on" : (h & SERVO_HOLD_GREEN) ? "the green laser is on" : (h & SERVO_HOLD_LIDAR) ? "the LiDAR is measuring" : "held"; }
void    servosHold(uint8_t g, uint8_t who, bool on) { if (g < 1 || g > PLATE_GROUPS) return; if (on) holds[g] |= who; else holds[g] &= (uint8_t)~who; }
uint8_t servosHolds(uint8_t g) { return (g >= 1 && g <= PLATE_GROUPS) ? holds[g] : 0; }
int8_t servosPlateState(uint8_t g) {             // 1 open, 0 closed, -1 not known (boot, stopped mid-move, moving now, its servos disagree)
  if (g < 1 || g > PLATE_GROUPS) return -1;
  if (seqActive) for (uint8_t k = 0; k < seqN; k++) if (PLATE_GROUP_OF[seq[k].plate] == g && !seq[k].done) return -1;
  int8_t s = 2;
  for (uint8_t i = 0; i < PLATE_COUNT; i++) if (PLATE_GROUP_OF[i] == g) { if (landed[i] < 0) return -1; if (s == 2) s = landed[i]; else if (s != landed[i]) return -1; }
  return s == 2 ? -1 : s;
}
bool servosPlateOpen(uint8_t g) { return platesOff || servosPlateState(g) == 1; }   // plates off: nothing is in the way
static const char* openBlock(uint8_t g, uint8_t opening) {   // nullptr = it may open now (its prerequisites open, or opening first in this plan)
  for (uint8_t p = 1; p <= PLATE_GROUPS; p++)
    if ((PLATE_NEEDS[g] & (1 << p)) && !(opening & (1 << p)) && servosPlateState(p) != 1) { snprintf(refusal, sizeof refusal, "%s needs %s open first", servosPlateName(g), servosPlateName(p)); return refusal; }
  return nullptr;
}
static const char* closeBlock(uint8_t g, uint8_t closing) {  // nullptr = nothing needs it open any more (its dependents closed, or closing first in this plan)
  if (holds[g]) { snprintf(refusal, sizeof refusal, "%s stays open - %s", servosPlateName(g), holderText(holds[g])); return refusal; }
  for (uint8_t d = 1; d <= PLATE_GROUPS; d++)
    if ((PLATE_NEEDS[d] & (1 << g)) && !(closing & (1 << d)) && servosPlateState(d) != 0) { snprintf(refusal, sizeof refusal, "%s stays open - %s is not closed", servosPlateName(g), servosPlateName(d)); return refusal; }
  return nullptr;
}
// strict: one blocked plate refuses the whole plan (one plate, the lasers, the camera); else a blocked plate is skipped
// with a line in the log and the rest still moves (the cascade).
// (2026-10-20) the time, as a fraction of a move, at which the smoothstep easing has done `travel` of the way
static float easeTimeAt(float travel, bool lin = false) {
  if (lin) return travel < 0 ? 0 : travel > 1 ? 1 : travel;   // constant speed: the time is the travel
  if (travel <= 0) return 0; if (travel >= 1) return 1;
  float lo = 0, hi = 1;
  for (int i = 0; i < 24; i++) { float m = 0.5f * (lo + hi); if (m * m * (3.0f - 2.0f * m) < travel) lo = m; else hi = m; }
  return 0.5f * (lo + hi);
}
static uint32_t durOfPlate(uint8_t g) { if (plateMs[g]) return plateMs[g]; uint32_t d = 0; for (uint8_t i = 0; i < PLATE_COUNT; i++) if (PLATE_GROUP_OF[i] == g && PLATES[i].moveMs > d) d = PLATES[i].moveMs; return d; }
static void startWith(const uint32_t startG[], uint8_t planned) {   // PLATE_WITH: a plate starts with its partner when both are in the plan
  for (uint8_t g = 1; g <= PLATE_GROUPS; g++) {
    uint8_t w = PLATE_WITH[g];
    if (!w || !(planned & (1 << g)) || !(planned & (1 << w))) continue;
    for (uint8_t k = 0; k < seqN; k++) if (PLATE_GROUP_OF[seq[k].plate] == g) seq[k].start = startG[w];
  }
}
static bool planOpen(uint8_t want, bool strict) {
  seqN = 0; refusal[0] = 0;
  if (powerFaultLatched()) { snprintf(refusal, sizeof refusal, "the battery monitor FAULT - no plate opens until a power-off (closing still works)"); return false; }   // (2026-10-08)
  uint32_t clearG[PLATE_GROUPS + 1] = { 0 }, startG[PLATE_GROUPS + 1] = { 0 }; uint8_t opening = 0;
  for (uint8_t k = 0; k < PLATE_GROUPS; k++) {
    uint8_t g = PLATE_OPEN_ORDER[k];
    if (!(want & (1 << g)) || servosPlateState(g) == 1) continue;
    const char* b = openBlock(g, opening);
    if (b) { if (strict) { seqN = 0; return false; } Bench.printf("plates: %s - skipped\n", b); continue; }
    uint32_t start = 0;
    for (uint8_t p = 1; p <= PLATE_GROUPS; p++) if ((PLATE_NEEDS[g] & (1 << p)) && (opening & (1 << p)) && clearG[p] > start) start = clearG[p];   // once they are far enough open
    addPlate(g, true, start); startG[g] = start; opening |= (1 << g);
    clearG[g] = start + (uint32_t)(easeTimeAt(PLATE_CLEAR_TRAVEL[g], plateLin[g]) * (endOfPlate(g) - start) + 0.5f);
  }
  startWith(startG, opening);
  return seqN > 0;
}
static bool planClose(uint8_t want, bool strict) {
  seqN = 0; refusal[0] = 0;
  uint32_t endG[PLATE_GROUPS + 1] = { 0 }, startG[PLATE_GROUPS + 1] = { 0 }; uint8_t closing = 0;
  for (int8_t k = PLATE_GROUPS - 1; k >= 0; k--) {
    uint8_t g = PLATE_OPEN_ORDER[k];
    if (!(want & (1 << g)) || servosPlateState(g) == 0) continue;
    const char* b = closeBlock(g, closing);
    if (b) { if (strict) { seqN = 0; return false; } Bench.printf("plates: %s\n", b); continue; }
    // the mirror of the opening rule: g passes its clear point (PLATE_CLEAR_TRAVEL open) only once the plates needing it are home
    uint32_t lead = (uint32_t)(easeTimeAt(1.0f - PLATE_CLEAR_TRAVEL[g], plateLin[g]) * durOfPlate(g) + 0.5f), start = 0;
    for (uint8_t d = 1; d <= PLATE_GROUPS; d++) if ((PLATE_NEEDS[d] & (1 << g)) && (closing & (1 << d)) && endG[d] > lead && endG[d] - lead > start) start = endG[d] - lead;
    addPlate(g, false, start); endG[g] = endOfPlate(g); startG[g] = start; closing |= (1 << g);
  }
  startWith(startG, closing);
  return seqN > 0;
}
static constexpr uint8_t ALL_PLATES = (uint8_t)(((1 << (PLATE_GROUPS + 1)) - 1) & ~1);

void servosOpen() {
  if (platesOff) { Bench.println("cascade OPEN: refused - the plates are switched off (the /cal page)"); return; }
  seqThenClose = false; pendingCloseAt = 0; seqPlate = 0;
  if (!planOpen(ALL_PLATES, false)) { Bench.printf("cascade OPEN: nothing to move%s%s\n", refusal[0] ? " - " : " (every plate is open)", refusal); return; }
  startSeq("OPEN");
  if (seqActive) platesState = PlatesState::Opening;
}

void servosClose() {
  if (platesOff) { Bench.println("cascade CLOSE: refused - the plates are switched off (the /cal page)"); return; }
  seqThenClose = false; pendingCloseAt = 0; seqPlate = 0;
  if (!planClose(ALL_PLATES, false)) { Bench.printf("cascade CLOSE: nothing to move%s%s\n", refusal[0] ? " - " : " (every plate that may close is closed)", refusal); return; }
  startSeq("CLOSE");
  if (seqActive) platesState = PlatesState::Closing;
}

void servosOpenThenClose() { servosOpen(); seqThenClose = seqActive; }

static bool startPlate(uint8_t plateNo, bool open) {
  static char name[24];
  snprintf(name, sizeof name, "PLATE %u %s", plateNo, open ? "OPEN" : "CLOSE");
  startSeq(name);
  if (!seqActive) return false;
  seqPlate = plateNo;
  platesState = open ? PlatesState::Opening : PlatesState::Closing;
  return true;
}
// One plate (bench R, the route, the gesture map): refused - with the reason in servosLastRefusal() - when it would
// jam: plate 2 or 3 without what it needs open, a plate something still needs. A plate already there: true, no move.
bool servosMovePlate(uint8_t plateNo, bool open) {
  refusal[0] = 0;
  if (platesOff) { snprintf(refusal, sizeof refusal, "the plates are switched off (the /cal page)"); return false; }
  if (!ready) { snprintf(refusal, sizeof refusal, "the PCA9685 is absent"); return false; }
  if (seqActive || pendingCloseAt) { snprintf(refusal, sizeof refusal, "a plate move is running"); return false; }
  if (plateNo < 1 || plateNo > PLATE_GROUPS) { snprintf(refusal, sizeof refusal, "no plate %u", plateNo); return false; }
  if (servosPlateState(plateNo) == (open ? 1 : 0)) return true;
  seqThenClose = false;
  bool ok = open ? planOpen(1 << plateNo, true) : planClose(1 << plateNo, true);
  if (!ok) { if (refusal[0]) Bench.printf("plates: refused - %s\n", refusal); return false; }
  return startPlate(plateNo, open);
}
// (2026-10-21, Toby: "if I click plate 2, automatically open the hand plate, then plate 2 ... plate 3: the hand plate,
// plate 2, then plate 3") The puck's plate buttons: opening a plate opens what it needs first (PLATE_NEEDS, all the way
// down), closing one closes what needs it first (plate 3, then plate 2, then the hand plate) - one plan, in the planner's
// order and timing, strict (a held plate refuses the whole press, as for one plate).
static uint8_t needsOf(uint8_t g) {        // every plate g needs open, through the ones they need
  uint8_t m = 0, add = PLATE_NEEDS[g];
  while (add) { m |= add; uint8_t nx = 0; for (uint8_t p = 1; p <= PLATE_GROUPS; p++) if (add & (1 << p)) nx |= PLATE_NEEDS[p]; add = nx & (uint8_t)~m; }
  return m;
}
static uint8_t dependentsOf(uint8_t g) {   // every plate that needs g open, through the ones that need those
  uint8_t m = 0; bool grew = true;
  while (grew) { grew = false; for (uint8_t d = 1; d <= PLATE_GROUPS; d++) if (!(m & (1 << d)) && (PLATE_NEEDS[d] & ((1 << g) | m))) { m |= (1 << d); grew = true; } }
  return m;
}
bool servosMovePlateWith(uint8_t plateNo, bool open) {
  refusal[0] = 0;
  if (platesOff) { snprintf(refusal, sizeof refusal, "the plates are switched off (the /cal page)"); return false; }
  if (!ready) { snprintf(refusal, sizeof refusal, "the PCA9685 is absent"); return false; }
  if (seqActive || pendingCloseAt) { snprintf(refusal, sizeof refusal, "a plate move is running"); return false; }
  if (plateNo < 1 || plateNo > PLATE_GROUPS) { snprintf(refusal, sizeof refusal, "no plate %u", plateNo); return false; }
  if (servosPlateState(plateNo) == (open ? 1 : 0)) return true;
  seqThenClose = false;
  const uint8_t want = (uint8_t)((1 << plateNo) | (open ? needsOf(plateNo) : dependentsOf(plateNo)));
  bool ok = open ? planOpen(want, true) : planClose(want, true);
  if (!ok) { if (refusal[0]) Bench.printf("plates: refused - %s\n", refusal); return false; }
  if (want != (1 << plateNo)) {
    Bench.printf("plates: %s %s - with", servosPlateName(plateNo), open ? "opens" : "closes");
    for (uint8_t g = 1; g <= PLATE_GROUPS; g++) if (g != plateNo && (want & (1 << g))) Bench.printf(" %s", servosPlateName(g));
    Bench.printf(" %s\n", open ? "first" : "closing first");
  }
  return startPlate(plateNo, open);
}
// The lasers and the camera: the same, but asked for while a move is running it waits for that move to end; a close is
// only ever done if nothing needs the plate (else it quietly stays open - that is the rule, not an error).
void servosRequest(uint8_t plateNo, bool open) {
  if (plateNo < 1 || plateNo > PLATE_GROUPS || platesOff) return;   // plates off: the laser / camera acts with its plate where it is
  if (open) { reqClose &= (uint8_t)~(1 << plateNo); } else { reqOpen &= (uint8_t)~(1 << plateNo); }
  if (servosPlateState(plateNo) == (open ? 1 : 0) && !seqActive) return;
  if (seqActive || pendingCloseAt) { if (open) reqOpen |= (1 << plateNo); else reqClose |= (1 << plateNo); return; }
  if (open) servosMovePlate(plateNo, true);
  else if (!closeBlock(plateNo, 0)) servosMovePlate(plateNo, false);
  else Bench.printf("plates: %s\n", refusal);
}
static void servePending() {                      // called when the sequencer goes idle
  if (seqActive || pendingCloseAt) return;
  for (uint8_t k = 0; k < PLATE_GROUPS && !seqActive; k++) { uint8_t g = PLATE_OPEN_ORDER[k]; if (reqOpen & (1 << g)) { reqOpen &= (uint8_t)~(1 << g); servosRequest(g, true); } }
  for (int8_t k = PLATE_GROUPS - 1; k >= 0 && !seqActive; k--) { uint8_t g = PLATE_OPEN_ORDER[k]; if (reqClose & (1 << g)) { reqClose &= (uint8_t)~(1 << g); servosRequest(g, false); } }
}
uint8_t servosMovingPlate() { return seqActive ? seqPlate : 0; }

void servosAbort() {
  reqOpen = reqClose = 0;                        // nothing asked for earlier moves after an abort (goSafe, a mode change)
  if (pendingCloseAt) { pendingCloseAt = 0; Bench.println("cascade: the close after the pause cancelled"); }
  if (!seqActive) return;
  endSeq();
  platesState = fromLanded();                                    // the servos that stopped mid-move are unknown
  Bench.printf("cascade %s ABORTED\n", seqName);
}

void servosPoll() {
  calSweepPoll(); calGuardPoll();
  if (calServo >= 0 && millis() - calAt > SERVO_CAL_HOLD_MS) { Bench.printf("cal: %s released (%u s without a move)\n", PLATES[calServo].name, (unsigned)(SERVO_CAL_HOLD_MS / 1000)); servoCalRelease(); }
  if (pendingCloseAt && (int32_t)(millis() - pendingCloseAt) >= 0) { pendingCloseAt = 0; servosClose(); }
  if (!seqActive) { servePending(); return; }    // the lasers' and the camera's plates, asked for while a move ran
  uint32_t t = millis() - seqT0;
  if (t - seqLastTick < 10) return;
  seqLastTick = t;
  bool allDone = true;
  for (uint8_t i = 0; i < seqN; i++) {
    Move& m = seq[i];
    if (m.done || t < m.start) { if (!m.done) allDone = false; continue; }
    float p = (t - m.start) / (float)m.dur;
    if (p >= 1.0f) { p = 1.0f; m.done = true; landed[m.plate] = m.open ? 1 : 0; } else allDone = false;
    float e = m.lin ? p : p * p * (3.0f - 2.0f * p);            // constant speed, or smoothstep
    servoWriteUs(PLATES[m.plate].ch, (uint16_t)(m.from + (m.to - m.from) * e));
  }
  if (currentPresent()) {                                       // the INA219's stall check
    bool  holding = allDone && t >= seqLastEnd;
    float a = currentServoA(), limit = holding ? SERVO_HOLD_STALL_A : SERVO_STALL_TOTAL_A;
    if (holding) { if (a > holdMaxA) holdMaxA = a; } else if (a > moveMaxA) moveMaxA = a;
    if (a <= limit) stallOverSince = 0;
    else if (!stallOverSince) stallOverSince = millis();
    else if (millis() - stallOverSince >= SERVO_STALL_MS) {
      stalls++; stallOverSince = 0;
      snprintf(lastStall, sizeof lastStall, "%s: %.2f A %s for %lu ms (limit %.2f A)", seqName, a, holding ? "while holding" : "while moving", (unsigned long)SERVO_STALL_MS, limit);
      // every move had landed: the plates are where they were sent, a servo is pressing on a stop; else the ones mid-move are unknown
      endSeq();
      platesState = fromLanded();
      Bench.printf("cascade %s STALL - %s - every servo released (check the plate's end stops, or a pair working against itself); moving peak %.2f A\n", seqName, lastStall, moveMaxA);
      return;
    }
  }
  bool stalled = t >= seqLastEnd + CASCADE_HOLD_MS + CASCADE_STALL_GRACE_MS;   // a move that never reported done (cannot happen with the clock, but the release is unconditional anyway)
  if ((allDone && t >= seqLastEnd + CASCADE_HOLD_MS) || stalled) {
    bool thenClose = seqThenClose, single = seqPlate != 0;
    endSeq();
    if (single) plateMoves++; else cascades++;
    platesState = fromLanded();
    Bench.printf("cascade %s done - servos released%s, plates %s; peak %.2f A moving, %.2f A holding\n", seqName, stalled ? " (stall grace)" : "", servosPlatesStateName(), moveMaxA, holdMaxA);
    if (thenClose) pendingCloseAt = millis() + CASCADE_PAUSE_MS;   // the pause is a time, not a delay(): the loop keeps running
  }
}

int16_t servoPosPermille(uint8_t i) {
  if (i >= PLATE_COUNT) return -1;
  uint16_t us = lastUs[PLATES[i].ch];
  if (!us) return -1;
  int32_t c = plateUs(i, false), o = plateUs(i, true);
  if (c == o) return 0;
  int32_t p = (int32_t)(((int32_t)us - c) * 1000L / (o - c));
  return (int16_t)(p < 0 ? 0 : p > 1000 ? 1000 : p);
}
uint8_t servosDrivenMask() {
  uint8_t m = 0;
  for (uint8_t i = 0; i < PLATE_COUNT; i++) if (chanUs[PLATES[i].ch]) m |= (uint8_t)(1u << i);
  return m;
}

void servosAllSweep() {
  if (!ready) { Bench.println("PCA9685 not found - aborting"); return; }
  Bench.println("ALL 6 servo channels sweeping 1200<->1800 us for 30 s (x ends)");
  servosPower(true);
  uint32_t t0 = millis(); bool side = false;
  while (millis() - t0 < 30000) {
    uint16_t us = side ? 1800 : 1200;
    for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) servoWriteUs(ch, us);
    side = !side;
    if (waitOrX(700)) break;
  }
  for (uint8_t ch = 0; ch < SERVO_COUNT; ch++) servoRelease(ch);
  if (!pcaDigitalOnCount()) servosPower(false);
  Bench.println("all-channel sweep done - servos released");
}

String servosJson() {
  String s = "{\"state\":\""; s += servosPlatesStateName(); s += "\",\"busy\":" + String(seqActive ? 1 : 0) + ",\"oe\":" + String(oeOn ? 1 : 0) + ",\"ok\":" + String(ready ? 1 : 0) + ",\"n\":" + String(cascades) + ",\"us\":[";
  for (uint8_t c = 0; c < SERVO_COUNT; c++) { if (c) s += ","; s += String(chanUs[c]); }
  s += "],\"dig\":[";
  for (uint8_t c = SERVO_COUNT; c < 16; c++) { if (c > SERVO_COUNT) s += ","; s += String(digOn[c] ? 1 : 0); }
  s += "],\"pos\":[";
  for (uint8_t i = 0; i < PLATE_COUNT; i++) { if (i) s += ","; s += String(servoPosPermille(i)); }
  s += "],\"plate\":" + String(servosMovingPlate()) + ",\"wait\":" + String(pendingCloseAt ? 1 : 0) + ",\"single\":" + String(plateMoves);
  s += ",\"stalls\":" + String(stalls) + ",\"stall\":\"" + lastStall + "\"}";
  return s;
}

// The CRITICAL shutdown: every channel released, OE high, then the chip's own SLEEP bit (its oscillator stops; it
// draws microamps). Only a reboot wakes it (servosBegin() runs again) - servosWake() is for the fake build's test.
static bool asleep = false;
void servosSleep() {
  servosAbort(); servosParkAll();
#if !HW_FAKE
  if (ready) pca.sleep();
#endif
  asleep = true;
}
bool servosAsleep() { return asleep; }
void servosWake() {
#if !HW_FAKE
  if (ready) pca.wakeup();
#endif
  asleep = false;
}
