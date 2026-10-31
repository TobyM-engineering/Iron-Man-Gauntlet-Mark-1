#include "config.h"
#include "crsf.h"
#include "config.h"
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <GauntletBenchIO.h>
#include <math.h>

// ---------------- channels -----------------------------------------------
static constexpr uint8_t CH_ROLL = 0, CH_PITCH = 1, CH_THROTTLE = 2, CH_YAW = 3;
static portMUX_TYPE      mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint16_t channels[16];
static volatile bool     streaming  = true;
static volatile uint32_t framesSent = 0;
static volatile uint32_t lastFeedMs = 0;     // when fly last wrote the sticks: the task's watchdog
static volatile bool     watchdog   = false; // the task had to take the sticks down itself

// One outbound "handset" frame (ping / param read / write). The CRSF task sends it in place
// of the next RC frame so nothing else touches the UART.
static uint8_t           pendingTx[16];
static volatile uint8_t  pendingLen  = 0;
static volatile uint32_t pendingSent = 0;

#if !HW_FAKE
static HardwareSerial crsf(1);
#else
static uint8_t  fakeRx[512]; static uint16_t fakeRxHead = 0, fakeRxTail = 0;   // bytes the test injects "from the module"
static uint8_t  fakeTx[64];  static uint8_t  fakeTxLen = 0; static uint32_t fakeTxCount = 0;   // the last frame "sent"
void crsfFakeInject(const uint8_t* b, uint8_t n) { for (uint8_t i = 0; i < n; i++) { fakeRx[fakeRxHead] = b[i]; fakeRxHead = (fakeRxHead + 1) % sizeof fakeRx; } }
uint8_t crsfFakeLastTx(uint8_t* out, uint8_t cap) { uint8_t n = fakeTxLen < cap ? fakeTxLen : cap; memcpy(out, fakeTx, n); return n; }
uint32_t crsfFakeTxCount() { return fakeTxCount; }
#endif

static void setChannel(uint8_t ch, uint16_t v) { portENTER_CRITICAL(&mux); channels[ch] = v; portEXIT_CRITICAL(&mux); }
static uint16_t getChannel(uint8_t ch)         { portENTER_CRITICAL(&mux); uint16_t v = channels[ch]; portEXIT_CRITICAL(&mux); return v; }
static uint16_t clampCh(uint16_t v)            { return v < CRSF_MIN ? CRSF_MIN : v > CRSF_MAX ? CRSF_MAX : v; }

void     crsfSetSticks(uint16_t r, uint16_t p, uint16_t t, uint16_t y) {
  portENTER_CRITICAL(&mux);
  channels[CH_ROLL] = clampCh(r); channels[CH_PITCH] = clampCh(p); channels[CH_THROTTLE] = clampCh(t); channels[CH_YAW] = clampCh(y);
  lastFeedMs = millis();
  portEXIT_CRITICAL(&mux);
}
void     crsfSetAux(uint8_t i, bool high) { if (i >= 4 && i < 16) setChannel(i, high ? CRSF_MAX : CRSF_MIN); }
uint16_t crsfChannel(uint8_t i)           { return i < 16 ? getChannel(i) : 0; }
bool     crsfWatchdogTripped()            { bool w = watchdog; watchdog = false; return w; }
void     crsfStream(bool on)              { streaming = on; Bench.printf("crsf: RC frames %s\n", on ? "on" : "OFF - the module has lost its handset; the drone (if linked) goes to failsafe"); }
bool     crsfStreaming()                  { return streaming; }
uint32_t crsfFramesSent()                 { return framesSent; }

void crsfSafe() {
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < 16; i++) channels[i] = CRSF_MID;
  channels[CH_THROTTLE] = CRSF_MIN;
  for (uint8_t i = 4; i < 16; i++) channels[i] = CRSF_MIN;   // every AUX low = disarmed, angle mode
  portEXIT_CRITICAL(&mux);
}

// ---------------- CRSF plumbing ------------------------------------------
static inline uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  return crc;
}

// ESP32 gotcha: gpio_set_direction() re-routes the pad to plain GPIO and silently disconnects
// the UART TX signal. Taking the wire = park the GPIO output high, enable output, then hand the
// pad to UART1 TX. Releasing = direction input, which puts plain GPIO back with output off, so
// the module can drive the wire with its telemetry.
static inline void takeWire() {
  gpio_set_level((gpio_num_t)PIN_CRSF, 1);
  gpio_set_direction((gpio_num_t)PIN_CRSF, GPIO_MODE_INPUT_OUTPUT);
  esp_rom_gpio_connect_out_signal(PIN_CRSF, U1TXD_OUT_IDX, false, false);
}
static inline void releaseWire() { gpio_set_direction((gpio_num_t)PIN_CRSF, GPIO_MODE_INPUT); }

static void sendRaw(const uint8_t* f, uint8_t n) {
#if HW_FAKE
  if (n > sizeof fakeTx) n = sizeof fakeTx;
  memcpy(fakeTx, f, n); fakeTxLen = n; fakeTxCount++;
#else
  takeWire();
  crsf.write(f, n);
  uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(10));   // last stop bit is out
  releaseWire();
#endif
}

// 0xC8 <len> 0x16 <22 bytes of 16 x 11-bit> <crc>
void crsfBuildRcFrame(uint8_t* buf) {        // the 26 bytes of the RC frame from the channels as they are now (the self-test decodes it back)
  buf[0] = 0xC8; buf[1] = 24; buf[2] = 0x16;
  uint16_t snap[16];
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < 16; i++) snap[i] = channels[i];
  portEXIT_CRITICAL(&mux);
  uint8_t* p = &buf[3];
  memset(p, 0, 22);
  uint32_t bit = 0;
  for (uint8_t i = 0; i < 16; i++) {
    uint16_t v = snap[i] & 0x07FF;
    for (uint8_t b = 0; b < 11; b++) { if (v & (1u << b)) p[bit >> 3] |= (uint8_t)(1u << (bit & 7)); bit++; }
  }
  uint8_t crc = 0;
  for (uint8_t i = 2; i < 25; i++) crc = crc8_dvb_s2(crc, buf[i]);
  buf[25] = crc;
}
static void sendRcFrame() { uint8_t buf[26]; crsfBuildRcFrame(buf); sendRaw(buf, 26); }
uint8_t crsfCrc8(const uint8_t* p, uint8_t n) { uint8_t c = 0; for (uint8_t i = 0; i < n; i++) c = crc8_dvb_s2(c, p[i]); return c; }

// Core-0 task: WiFi work, the pages and the WS2812 writes on core 1 cannot starve it - a gap in
// CRSF is a failsafe on the drone. The watchdog lives here for the same reason.
static void taskStep() {                     // one period of the core-0 task: the watchdog, then one frame (a pending command, else the RC frame)
  {
    uint32_t now = millis();
    portENTER_CRITICAL(&mux);
    if (lastFeedMs && now - lastFeedMs > CRSF_FEED_STALE_MS) {         // loop() has stopped writing the sticks
      bool hot = channels[CH_THROTTLE] != CRSF_MIN || channels[FLY_ARM_AUX] != CRSF_MIN;
      channels[CH_THROTTLE] = CRSF_MIN; channels[FLY_ARM_AUX] = CRSF_MIN;
      channels[CH_ROLL] = channels[CH_PITCH] = channels[CH_YAW] = CRSF_MID;
      if (hot) watchdog = true;
    }
    portEXIT_CRITICAL(&mux);
    if (pendingLen) {
      uint8_t f[16], n;
      portENTER_CRITICAL(&mux);
      n = pendingLen; memcpy(f, pendingTx, n); pendingLen = 0;
      portEXIT_CRITICAL(&mux);
      sendRaw(f, n);
      pendingSent = pendingSent + 1;
    } else if (streaming) {
      sendRcFrame();
      framesSent = framesSent + 1;
    }
  }
}
#if !HW_FAKE
static void crsfTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(CRSF_PERIOD_MS);
  for (;;) { taskStep(); vTaskDelayUntil(&last, period); }
}
#endif

// ---------------- listening (same wire) ----------------------------------
static volatile uint32_t rxEchoOk = 0, rxEchoBad = 0, rxModOk = 0, rxModBad = 0, rxRawBytes = 0;
static volatile uint32_t rxLastModMs = 0;
static volatile uint8_t  linkLQ = 0, linkRfMode = 0, downLQ = 0;
static volatile int8_t   linkRssi = 0, downRssi = 0;
static volatile bool     linkUp = false;
static uint8_t  rxBuf[64];
static uint8_t  rxLen = 0;
static uint8_t  rawRing[256];                 // every byte seen on the wire, ours and theirs, for /fly?raw=1
static uint16_t rawIdx = 0;
static volatile uint32_t strayBytes = 0;      // bytes outside any frame (no sync byte, or a silly length): another baud rate, a log, noise
static uint32_t rateRaw = 0, rateStray = 0;   // bytes per second over the last 10 s window
static uint32_t heardSince = 0;               // when the module last appeared (first frame after silence)
static bool     quietDone  = false;           // the automatic Max Power cap + Dynamic power has been sent for this appearance
static uint32_t lastLinkMs = 0, lastAutoBindMs = 0;   // auto-bind: the module on but no drone for CRSF_AUTO_BIND_MS -> the bind loop by itself
static uint32_t lastLinkStatsMs = 0;                  // the last LINK_STATISTICS frame: the module stops sending them when the receiver vanishes, so silence = the link is gone

static volatile uint32_t cntLink = 0, cntSync = 0, cntDevInfo = 0, cntEntry = 0, cntOther = 0, cntTelem = 0;
static volatile uint8_t  lastOtherType = 0, lastOtherAddr = 0;
static char     fcMode[16] = "";          // flight controller mode string: "ANGL*" = disarmed and ready
static uint32_t fcModeMs = 0;
static float    fcVolts = 0, fcAmps = 0;  // craft battery, via telemetry
static uint8_t  fcPct = 0;
static float    attRoll = 0, attPitch = 0, attYaw = 0;
static uint32_t attMs = 0;
// MSP over CRSF (what the ELRS Lua and the Betaflight passthrough use): MSP_REQ 0x7A to the flight controller (ext dest
// 0xC8), MSP_RESP 0x7B back, each chunked behind a status byte (bit 4 = start, bits 5-6 = MSP version, bits 0-3 = a
// sequence). A v1 request is [status][size][cmd][payload][xor]; a v1 reply is [status][size][payload][xor] - no cmd byte,
// the asker remembers it. Asked only while the FC says '!': MSP_STATUS (101) carries its arming-disable flags - WHY it
// will not arm (2026-10-01 night: eight refused arms after a battery change and no record of the reason).
static uint8_t  mspBuf[96]; static uint8_t mspLen = 0; static bool mspOpen = false; static uint8_t mspLastCmd = 0, mspSeq = 0;
static uint32_t armFlags = 0, armFlagsMs = 0, mspReqMs = 0, mspReqs = 0, mspResps = 0; static uint8_t armFlagCount = 0;
static bool     mspDump = false;                 // the next reply's bytes go to the log (a manual /fly?msp=1 asks for that)
static uint32_t accAckMs = 0;                    // the FC acknowledged MSP_ACC_CALIBRATION (an empty reply to cmd 205)
static float    fcTempC = NAN; static uint32_t fcTempMs = 0; static uint8_t fcTempSrc = 0;   // CRSF TEMP frame 0x0D (Betaflight 2025+: ESC or core temperature), the hottest value
static uint16_t fcCycleUs = 0, fcLoadPct = 0;    // from MSP_STATUS (the first bytes, stable across versions)
static int16_t  fcCoreC = 0; static uint32_t fcCoreMs = 0;   // the FC's chip (MCU core) temperature from MSP_STATUS, API 1.46+, degC
static uint32_t cntRpm = 0, cntTemp = 0;

// module identity and parameter list (DEVICE_INFO 0x29, SETTINGS_ENTRY 0x2B)
static volatile uint8_t  modParamCount = 0;
static char              modName[24] = "";
static uint8_t           modSwVer[3] = { 0, 0, 0 };
static volatile uint8_t  lastEntryIdx = 0, lastEntryType = 0;
static char              lastEntryName[24] = "";
static volatile uint8_t  bindFieldIdx = 0, powerFieldIdx = 0, telemFieldIdx = 0, dynFieldIdx = 0;
static char              powerOptions[48] = "";   // the Max Power field's option text, e.g. "10;25;50;100;250;500" (mW)
static char              dynOptions[48]   = "";   // the Dynamic field's option text, "Off;Dyn;AUX9;AUX10;AUX11;AUX12" (ELRS 3.x)
static bool              dynamicSet = false;      // we have set Dynamic -> Dyn since the module last appeared
// The "Dyn" option of the Dynamic field (ExpressLRS: Off = fixed at Max Power, Dyn = dynamic, AUX9-12 = dynamic while
// that channel is high); 1 when the option text has not been read.
uint8_t crsfDynamicIdx() {
  if (!dynOptions[0]) return 1;
  uint8_t idx = 0; const char* p = dynOptions;
  while (*p) { if (strncmp(p, "Dyn", 3) == 0 && (p[3] == ';' || p[3] == 0)) return idx; const char* s = strchr(p, ';'); if (!s) break; p = s + 1; idx++; }
  return 1;
}
const char* crsfDynamicOptions() { return dynOptions; }
bool        crsfDynamicOn()      { return dynamicSet; }
// The highest Max Power option at or under CRSF_POWER_MAX_MW - the lock (design/01-power.md section 4).
uint8_t crsfPowerMaxIdx() {
  if (!powerOptions[0]) return CRSF_POWER_MAX_IDX_FALLBACK;
  uint8_t idx = 0, best = 0; bool any = false; const char* p = powerOptions;
  while (*p) { int v = atoi(p); if (v > 0 && v <= (int)CRSF_POWER_MAX_MW) { best = idx; any = true; } const char* s = strchr(p, ';'); if (!s) break; p = s + 1; idx++; }
  return any ? best : 0;
}
const char* crsfPowerOptions() { return powerOptions; }
static volatile uint8_t  lastCmdStatus = 0;
static char              lastCmdInfo[24] = "";

bool        crsfModuleHeard()  { return rxLastModMs && millis() - rxLastModMs <= CRSF_QUIET_MS; }
uint32_t    crsfModuleAgeMs()  { return rxLastModMs ? millis() - rxLastModMs : 0; }
bool        crsfLinkUp()       { return linkUp && crsfModuleHeard(); }
uint8_t     crsfLQ()           { return linkLQ; }
int8_t      crsfRssi()         { return linkRssi; }
uint8_t     crsfRfMode()       { return linkRfMode; }
uint8_t     crsfDownLQ()       { return downLQ; }
int8_t      crsfDownRssi()     { return downRssi; }
const char* crsfFcMode()       { return fcMode; }
bool        crsfFcSeen()       { return fcMode[0] != 0; }
uint32_t    crsfFcModeAgeMs()  { return fcModeMs ? millis() - fcModeMs : 0xFFFFFFFF; }
bool        crsfCraftArmed()   { return fcMode[0] && !strchr(fcMode, '*') && !strchr(fcMode, '!'); }
bool        crsfCraftReady()   { return fcMode[0] && strchr(fcMode, '*'); }
bool        crsfCraftBlocked() { return fcMode[0] && strchr(fcMode, '!'); }
float       crsfCraftVolts()   { return fcVolts; }
uint8_t     crsfCraftPct()     { return fcPct; }
float       crsfCraftAmps()    { return fcAmps; }
bool        crsfAttitude(float& r, float& p, float& y, uint32_t& age) { if (!attMs) return false; r = attRoll; p = attPitch; y = attYaw; age = millis() - attMs; return true; }

static void handleFrame(const uint8_t* f, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t i = 2; i < n - 1; i++) crc = crc8_dvb_s2(crc, f[i]);
  bool ok = (crc == f[n - 1]);
  // Ours: RC channels, or our handset commands (origin 0xEA) - the wire echoes what we send.
  // Anything else with a 0xC8 sync byte is the flight controller's telemetry forwarded.
  bool ours = (f[0] == 0xC8 && f[2] == 0x16) ||
              (f[0] == 0xEE && f[2] >= 0x28 && f[2] <= 0x2D && n > 4 && f[4] == 0xEA) ||
              (f[0] == 0xEE && f[2] == 0x7A && n > 4 && f[4] == 0xEA);
  if (ours) { if (ok) rxEchoOk++; else rxEchoBad++; return; }
  if (!ok)  { rxModBad++; return; }
  rxModOk++;
  uint32_t now = millis();
  if (!rxLastModMs) Bench.println("crsf: the module is talking - the wire and the baud rate are right");
  if (!rxLastModMs || now - rxLastModMs > CRSF_QUIET_MS) { heardSince = now; quietDone = false; }   // (re)appeared: a power cycle - its power setting may be back up
  rxLastModMs = now;
  switch (f[2]) {
    case 0x14:                                   // LINK_STATISTICS
      if (n >= 14) {
        cntLink++; lastLinkStatsMs = now;
        linkRssi   = ((int8_t)f[3] < 0) ? (int8_t)f[3] : -(int8_t)f[3];
        linkLQ     = f[5];
        linkRfMode = f[8];
        downRssi   = ((int8_t)f[10] < 0) ? (int8_t)f[10] : -(int8_t)f[10];
        downLQ     = f[11];
        bool up = linkLQ > 0;
        if (up) lastLinkMs = now;
        if (up != linkUp) {
          linkUp = up;
          if (up) Bench.printf("crsf: DRONE LINKED  LQ %u  RSSI %d dBm\n", linkLQ, linkRssi);
          else    Bench.println("crsf: drone link LOST (LQ 0)");
        }
      }
      break;
    case 0x3A: cntSync++; break;                 // handset sync
    case 0x29: {                                 // DEVICE_INFO: dest, origin, name\0, serial4, hw4, sw4, paramCount, paramVer
      cntDevInfo++;
      uint8_t i = 5, k = 0;
      while (i < n - 1 && f[i] && k < sizeof(modName) - 1) modName[k++] = (char)f[i++];
      modName[k] = 0;
      i += 1;
      if (i + 12 < n - 1) { modSwVer[0] = f[i + 9]; modSwVer[1] = f[i + 10]; modSwVer[2] = f[i + 11]; modParamCount = f[i + 12]; }
      break;
    }
    case 0x2B: {                                 // SETTINGS_ENTRY: dest, origin, idx, chunksLeft, parent, type, name\0, ...
      cntEntry++;
      lastEntryIdx = f[5]; lastEntryType = f[8];
      uint8_t i = 9, k = 0;
      while (i < n - 1 && f[i] && k < sizeof(lastEntryName) - 1) lastEntryName[k++] = (char)f[i++];
      lastEntryName[k] = 0;
      if ((lastEntryType & 0x7F) == 9 && strcmp(lastEntryName, "Max Power") == 0 && i + 1 < n - 1) {   // TEXT_SELECTION: the options string follows the name - "10;25;50;100;250;500"
        uint8_t j = i + 1, m = 0;
        while (j < n - 1 && f[j] && m < sizeof(powerOptions) - 1) powerOptions[m++] = (char)f[j++];
        powerOptions[m] = 0;
      }
      if ((lastEntryType & 0x7F) == 9 && strcmp(lastEntryName, "Dynamic") == 0 && i + 1 < n - 1) {     // TEXT_SELECTION: "Off;Dyn;AUX9;AUX10;AUX11;AUX12"
        uint8_t j = i + 1, m = 0;
        while (j < n - 1 && f[j] && m < sizeof(dynOptions) - 1) dynOptions[m++] = (char)f[j++];
        dynOptions[m] = 0;
      }
      if ((lastEntryType & 0x7F) == 13 && i + 2 < n - 1) {   // COMMAND: [status][timeout][info\0]
        lastCmdStatus = f[i + 1];
        uint8_t j = i + 3, m = 0;
        while (j < n - 1 && f[j] && m < sizeof(lastCmdInfo) - 1) lastCmdInfo[m++] = (char)f[j++];
        lastCmdInfo[m] = 0;
        if (lastEntryIdx == bindFieldIdx) Bench.printf("crsf: bind - module says status=%u '%s'\n", lastCmdStatus, lastCmdInfo);
      }
      break;
    }
    case 0x21: {                                 // FLIGHT_MODE text from the FC
      char m[16]; uint8_t k = 0;
      for (uint8_t i = 3; i < n - 1 && f[i] && k < sizeof(m) - 1; i++) m[k++] = (char)f[i];
      m[k] = 0;
      fcModeMs = now;
      if (strcmp(m, fcMode) != 0) {
        strncpy(fcMode, m, sizeof(fcMode)); fcMode[sizeof(fcMode) - 1] = 0;
        const char* why = strchr(fcMode, '!') ? " (disarmed, arming BLOCKED on the drone)"
                        : strchr(fcMode, '*') ? " (disarmed, ready to arm)" : " (ARMED)";
        Bench.printf("crsf: drone mode '%s'%s\n", fcMode, why);
      }
      cntTelem++;
      break;
    }
    case 0x08:                                   // BATTERY_SENSOR: V(0.1, BE), A(0.1, BE), mAh(24), %
      if (n >= 12) { fcVolts = ((f[3] << 8) | f[4]) / 10.0f; { uint16_t ra = (uint16_t)((f[5] << 8) | f[6]); fcAmps = ra == 0xFFFF ? -1.0f : ra / 10.0f; }   /* 0xFFFF = no current sensor (it read 6553.5 A) */ fcPct = f[10]; }
      cntTelem++;
      break;
    case 0x1E:                                   // ATTITUDE: pitch, roll, yaw as int16 big-endian, radians x 10000
      if (n >= 9) {
        attPitch = (int16_t)((f[3] << 8) | f[4]) / 10000.0f * 57.29578f;
        attRoll  = (int16_t)((f[5] << 8) | f[6]) / 10000.0f * 57.29578f;
        attYaw   = (int16_t)((f[7] << 8) | f[8]) / 10000.0f * 57.29578f;
        attMs = now;
      }
      cntTelem++;
      break;
    case 0x0D: {                                 // TEMP: source id, then int16 big-endian values in 0.1 degC (ESCs, or the FC's core) - keep the hottest
      cntTemp++; cntTelem++;
      if (n >= 7) { float hot = -1000; for (uint8_t i = 4; i + 1 < n - 1; i += 2) { float t = (int16_t)((f[i] << 8) | f[i + 1]) / 10.0f; if (t > hot) hot = t; } if (hot > -1000) { fcTempC = hot; fcTempMs = now; fcTempSrc = f[3]; } }
      break;
    }
    case 0x0C: cntRpm++; cntTelem++; break;      // RPM: source id, int24 values - counted, not kept
    case 0x7B: {                                 // MSP_RESP from the flight controller: [dest][src][status][msp bytes...]
      if (n < 7) break;
      uint8_t st = f[5];
      if (st & 0x10) { mspLen = 0; mspOpen = true; }                     // start of a reply
      if (!mspOpen) break;
      for (uint8_t i = 6; i < n - 1 && mspLen < sizeof mspBuf; i++) mspBuf[mspLen++] = f[i];
      if (mspLen >= 1) {
        uint8_t size = mspBuf[0];
        uint8_t hdr = (mspLen >= 2 && mspBuf[1] == mspLastCmd) ? 2 : 1;   // Betaflight 4.3+ (this 2026.6 build too) puts the command byte after the size in a v1 reply; the cycle time read 14437 = 0x3865 with the cmd (101 = 0x65) taken as data
        if (mspLen >= (uint16_t)size + hdr) {                             // the whole v1 reply is in (its xor may trail; not checked - the CRSF CRC covered the wire)
          mspOpen = false; mspResps++;
          const uint8_t* p = mspBuf + hdr;
          if (mspLastCmd == 205) { accAckMs = now; Bench.println("crsf: the drone acknowledged the accelerometer calibration - it levels itself now, keep it still 2 s"); }
          if (mspDump || mspResps <= 2) {                                 // the bytes themselves, for checking the layout against this Betaflight (2026.6-alpha: the first read came back 0x1d = a count, not flags)
            char h[200]; int k = 0; for (uint8_t j = 0; j < size && k < (int)sizeof h - 3; j++) k += snprintf(h + k, sizeof h - k, "%02x", p[j]);
            Bench.printf("crsf: msp reply to cmd %u, %u bytes: %s\n", mspLastCmd, size, h); mspDump = false;
          }
          if (mspLastCmd == 101 && size >= 17) {                          // MSP_STATUS: cycle u16, i2c u16, sensors u16, modes u32, profile u8, load u16, gyro cycle u16, [n][n more mode bytes], [count] flags u32, config u8 ...
            fcCycleUs = (uint16_t)(p[0] | (p[1] << 8)); fcLoadPct = (uint16_t)(p[11] | (p[12] << 8));
            uint8_t more = p[15] & 0x0F, i = 16 + more;
            int8_t off = -1;                                              // the count byte is 20..40 in every Betaflight since 4.0 and the flags fit under it; if the byte at i is not that, this build's layout differs by a byte or two - look just past it
            for (int8_t d = 0; d <= 3 && off < 0; d++) {
              int8_t j = i + d; if (j + 5 > size) break;
              uint8_t cnt = p[j]; uint32_t fl = (uint32_t)p[j + 1] | ((uint32_t)p[j + 2] << 8) | ((uint32_t)p[j + 3] << 16) | ((uint32_t)p[j + 4] << 24);
              if (cnt >= 20 && cnt <= 40 && (cnt >= 32 || (fl >> cnt) == 0)) off = j;
            }
            if (off >= 0) {
              uint32_t fl = (uint32_t)p[off + 1] | ((uint32_t)p[off + 2] << 8) | ((uint32_t)p[off + 3] << 16) | ((uint32_t)p[off + 4] << 24);
              bool changed = fl != armFlags || !armFlagsMs;
              armFlagCount = p[off]; armFlags = fl; armFlagsMs = now;
              if (off + 8 <= size) { int16_t tc = (int16_t)(p[off + 6] | (p[off + 7] << 8)); if (tc > -40 && tc < 150) { fcCoreC = tc; fcCoreMs = now; } }   // after the flags and the config byte: the chip temperature, degC (API 1.46+)
              if (changed) { char t[160]; crsfArmingFlagText(t, sizeof t); Bench.printf("crsf: the drone's arming-disable flags: 0x%08lx (%u defined, count at byte %d%s) = %s\n", (unsigned long)fl, armFlagCount, off, off == i ? "" : " - not where 4.5 puts it", t[0] ? t : "none"); }
            } else if (mspResps <= 3) Bench.printf("crsf: msp status reply - no flag count found near byte %u of %u\n", i, size);
          }
        }
      }
      break;
    }
    default: cntOther++; lastOtherType = f[2]; lastOtherAddr = f[0]; break;
  }
}

#if HW_FAKE
static bool rxAvailable() { return fakeRxTail != fakeRxHead; }
static uint8_t rxRead() { uint8_t b = fakeRx[fakeRxTail]; fakeRxTail = (fakeRxTail + 1) % sizeof fakeRx; return b; }
#else
static bool rxAvailable() { return crsf.available() > 0; }
static uint8_t rxRead() { return (uint8_t)crsf.read(); }
#endif
static void pumpRx() {
  while (rxAvailable()) {
    uint8_t b = rxRead();
    rxRawBytes++;
    rawRing[rawIdx++ & 255] = b;
    if (rxLen == 0) { if (b == 0xC8 || b == 0xEA || b == 0xEE) rxBuf[rxLen++] = b; else strayBytes++; continue; }
    if (rxLen == 1) { if (b < 2 || b > 62) { rxLen = 0; strayBytes += 2; continue; } rxBuf[rxLen++] = b; continue; }
    rxBuf[rxLen++] = b;
    if (rxLen == rxBuf[1] + 2) { handleFrame(rxBuf, rxLen); rxLen = 0; }
    if (rxLen >= sizeof(rxBuf)) rxLen = 0;
  }
}

// ---------------- module parameters (what the Lua does) --------------------
static void queueFrame(uint8_t type, const uint8_t* payload, uint8_t n) {
  uint8_t f[16];
  f[0] = 0xEE; f[1] = n + 2; f[2] = type;           // 0xEE = to the TX module, as a real handset addresses these
  memcpy(&f[3], payload, n);
  uint8_t crc = 0;
  for (uint8_t i = 2; i < 3 + n; i++) crc = crc8_dvb_s2(crc, f[i]);
  f[3 + n] = crc;
  portENTER_CRITICAL(&mux);
  memcpy(pendingTx, f, n + 4); pendingLen = n + 4;
  portEXIT_CRITICAL(&mux);
}
static void sendPing()                             { uint8_t p[2] = { 0x00, 0xEA };         queueFrame(0x28, p, 2); }
static void sendParamRead(uint8_t idx)             { uint8_t p[4] = { 0xEE, 0xEA, idx, 0 }; queueFrame(0x2C, p, 4); }
static void sendParamWrite(uint8_t idx, uint8_t v) { uint8_t p[4] = { 0xEE, 0xEA, idx, v }; queueFrame(0x2D, p, 4); }
void crsfMspSend(uint8_t cmd) {                   // a v1 MSP request with no payload to the FC: [dest FC][src handset][0x30 | seq = start, v1][size 0][cmd][xor = cmd]
  uint8_t p[6] = { 0xC8, 0xEA, (uint8_t)(0x30 | (mspSeq++ & 0x0F)), 0, cmd, cmd };
  queueFrame(0x7A, p, 6); mspLastCmd = cmd; mspReqMs = millis(); mspReqs++;
}
void crsfMspAsk()       { crsfMspSend(101); mspDump = true; }   // MSP_STATUS now, and log the reply's bytes
void crsfAccCalibrate() { crsfMspSend(205); Bench.println("crsf: MSP_ACC_CALIBRATION sent to the drone - it must sit still and level for about 3 s (disarmed); its attitude should read 0/0 on that surface afterwards"); }   // after a frame change the board sits differently: Betaflight's 'level' is wherever it was calibrated
// Betaflight's arming-disable flag names, bit order of 4.5 / 2025 (bit 20 is DSHOT_TELEM there, RPMFILTER before). The
// raw value is logged too, in case this build numbers them differently.
static const char* ARM_FLAG_NAMES[] = { "NOGYRO", "FAILSAFE", "RXLOSS", "BADRX", "BOXFAILSAFE", "RUNAWAY", "CRASH", "THROTTLE", "ANGLE", "BOOTGRACE", "NOPREARM", "LOAD", "CALIB", "CLI", "CMS", "BST", "MSP", "PARALYZE", "GPS", "RESC", "DSHOT_TELEM", "REBOOT_REQD", "DSHOT_BBANG", "NO_ACC_CAL", "MOTOR_PROTO", "ARMSWITCH" };
static const char* armFlagHint(uint8_t bit) {
  switch (bit) {
    case 2: case 3: return "no receiver signal";
    case 7:  return "throttle not low";
    case 8:  return "not level";
    case 9:  return "just booted";
    case 12: return "gyro calibrating - hold it still";
    case 16: return "configurator attached";
    case 20: return "a motor's ESC not answering - prop jammed? motor plug?";
    case 25: return "arm switch was on - it must see it off first";
    case 5:  return "runaway takeoff tripped";
    case 6:  return "crash recovery";
    case 1: case 4: return "failsafe";
    default: return nullptr;
  }
}
bool crsfArmingFlagText(char* out, size_t n) {   // "CALIB (gyro calibrating - hold it still) + MSP (configurator attached)"; false (empty) when unknown or stale
  out[0] = 0;
  if (!armFlagsMs || millis() - armFlagsMs > CRSF_MSP_FLAGS_FRESH_MS) return false;
  if (!armFlags) { snprintf(out, n, "no flag set"); return true; }
  size_t k = 0;
  for (uint8_t b = 0; b < 32 && k < n - 1; b++) {
    if (!(armFlags & (1UL << b))) continue;
    const char* name = b < sizeof(ARM_FLAG_NAMES) / sizeof(ARM_FLAG_NAMES[0]) ? ARM_FLAG_NAMES[b] : "?";
    const char* hint = armFlagHint(b);
    k += snprintf(out + k, n - k, "%s%s", k ? " + " : "", name);
    if (hint && k < n - 1) k += snprintf(out + k, n - k, " (%s)", hint);
    if (!strcmp(name, "?") && k < n - 1) k += snprintf(out + k, n - k, "bit %u", b);
  }
  return true;
}
uint32_t crsfAccCalAckMs() { return accAckMs; }
float    crsfCoreTempC(uint32_t* ageMs) { if (ageMs) *ageMs = fcCoreMs ? millis() - fcCoreMs : 0xFFFFFFFF; return fcCoreMs ? (float)fcCoreC : NAN; }
float    crsfTempC(uint32_t* ageMs, uint8_t* src) { if (ageMs) *ageMs = fcTempMs ? millis() - fcTempMs : 0xFFFFFFFF; if (src) *src = fcTempSrc; return fcTempMs ? fcTempC : NAN; }
uint16_t crsfFcCycleUs() { return fcCycleUs; }
uint16_t crsfFcLoadPct() { return fcLoadPct; }
const char* crsfModuleName() { return modName; }
void     crsfModuleVersion(uint8_t* v3) { v3[0] = modSwVer[0]; v3[1] = modSwVer[1]; v3[2] = modSwVer[2]; }
uint32_t crsfTempFrames() { return cntTemp; }
uint32_t crsfArmingFlags(uint32_t* ageMs) { if (ageMs) *ageMs = armFlagsMs ? millis() - armFlagsMs : 0xFFFFFFFF; return armFlags; }

// Walk the parameter list once (ping, then read fields until the goal's field is known), then
// act. Goals: 'n' bind, 'q' quiet, 'p' power up, 'D' the cap + Dynamic power, 'e' telemetry 1:2, 'r' telemetry std.
static uint8_t  walkStep = 0, walkIdx = 0, walkTries = 0;
static uint32_t walkDeadline = 0;
static char     walkGoal = 0;
static uint8_t  powerLevel = 0;

static bool goalKnown(char g) {
  if (g == 'n') return bindFieldIdx;
  if (g == 'q' || g == 'p') return powerFieldIdx;
  if (g == 'D') return powerFieldIdx && dynFieldIdx;
  if (g == 'e' || g == 'r') return telemFieldIdx;
  return true;
}

static void doGoal() {
  switch (walkGoal) {
    case 'n':
      if (!bindFieldIdx) { Bench.println("crsf: no field named Bind - does this module have a binding phrase set?"); return; }
      Bench.printf("crsf: sending Bind (field %u) - the receiver must be in bind mode (whoop power-cycled three times)\n", bindFieldIdx);
      sendParamWrite(bindFieldIdx, 1);
      lastCmdStatus = 0; lastCmdInfo[0] = 0; lastEntryIdx = 0;
      walkStep = 3; walkTries = 0; walkDeadline = millis() + 350;   // read it back
      break;
    case 'q':
      if (!powerFieldIdx) { Bench.println("crsf: no field named Max Power"); return; }
      powerLevel = 0;
      Bench.printf("crsf: Max Power -> lowest (field %u)\n", powerFieldIdx);
      sendParamWrite(powerFieldIdx, 0);
      break;
    case 'p':
      if (!powerFieldIdx) { Bench.println("crsf: no field named Max Power"); return; }
      if (powerLevel >= crsfPowerMaxIdx()) {              // the lock: never past CRSF_POWER_MAX_MW (the fan, 0.5 A, and the jammed WiFi start above it)
        Bench.printf("crsf: Max Power stays at option %u - locked at %u mW (options %s)\n", powerLevel, (unsigned)CRSF_POWER_MAX_MW, powerOptions[0] ? powerOptions : "unknown, index cap " + String(CRSF_POWER_MAX_IDX_FALLBACK));
        return;
      }
      powerLevel++;
      Bench.printf("crsf: Max Power -> option %u (0 = lowest, %u = the %u mW lock)\n", powerLevel, crsfPowerMaxIdx(), (unsigned)CRSF_POWER_MAX_MW);
      sendParamWrite(powerFieldIdx, powerLevel);
      break;
    case 'D':
      if (!powerFieldIdx) { Bench.println("crsf: no field named Max Power"); return; }
      powerLevel = crsfPowerMaxIdx();
      Bench.printf("crsf: Max Power -> option %u, the %u mW cap (field %u)\n", powerLevel, (unsigned)CRSF_POWER_MAX_MW, powerFieldIdx);
      sendParamWrite(powerFieldIdx, powerLevel);
      if (!dynFieldIdx) { Bench.println("crsf: no field named Dynamic - the power stays fixed at the cap"); dynamicSet = false; return; }
      walkStep = 4; walkDeadline = millis() + 150;           // the Dynamic write once the Max Power write is on the wire (one pending frame)
      break;
    case 'e':
      if (!telemFieldIdx) { Bench.println("crsf: no field named Telem Ratio"); return; }
      Bench.printf("crsf: Telem Ratio -> 1:2 (field %u)\n", telemFieldIdx);   // Std;Off;1:128;..;1:2;Race -> 8
      sendParamWrite(telemFieldIdx, 8);
      break;
    case 'r':
      if (!telemFieldIdx) { Bench.println("crsf: no field named Telem Ratio"); return; }
      Bench.printf("crsf: Telem Ratio -> Std (field %u)\n", telemFieldIdx);
      sendParamWrite(telemFieldIdx, 0);
      break;
  }
}

static void walkStart(char goal) {
  walkGoal = goal;
  if (goalKnown(goal)) { doGoal(); return; }
  modParamCount = 0; lastEntryIdx = 0; lastEntryName[0] = 0;
  walkStep = 1; walkTries = 0; walkDeadline = millis() + 800;
  sendPing();
  Bench.println("crsf: pinging the module ...");
}

static void walkPoll() {
  if (!walkStep) return;
  uint32_t now = millis();
  if (walkStep == 4) {                                   // after the cap: Dynamic -> Dyn
    if ((int32_t)(now - walkDeadline) < 0 || pendingLen) return;
    sendParamWrite(dynFieldIdx, crsfDynamicIdx()); dynamicSet = true;
    Bench.printf("crsf: Dynamic -> option %u (options '%s'): the module starts at its minimum and steps up only as the link needs, never past %u mW\n",
                 crsfDynamicIdx(), dynOptions[0] ? dynOptions : "not read", (unsigned)CRSF_POWER_MAX_MW);
    walkStep = 0; return;
  }
  if (walkStep == 3) {                                   // after Bind: read the command back
    if ((int32_t)(now - walkDeadline) < 0) return;
    if (lastEntryIdx == bindFieldIdx) {
      if (lastCmdStatus == 3) {                          // asks for confirmation - give it
        Bench.println("crsf: bind - module asked to confirm, confirming");
        sendParamWrite(bindFieldIdx, 4);
        lastEntryIdx = 0; walkDeadline = now + 350; walkTries = 0;
        return;
      }
      Bench.printf("crsf: bind done from our side (status %u) - the module double-blinks while bind packets go out\n", lastCmdStatus);
      walkStep = 0; return;
    }
    if (++walkTries > 4) { Bench.println("crsf: bind - no read-back from the module"); walkStep = 0; return; }
    sendParamRead(bindFieldIdx); walkDeadline = now + 400;
    return;
  }
  if (walkStep == 1) {                                   // waiting for DEVICE_INFO
    if (modParamCount) {
      Bench.printf("crsf: module '%s' ELRS %u.%u.%u, %u fields - reading them\n", modName, modSwVer[0], modSwVer[1], modSwVer[2], modParamCount);
      walkStep = 2; walkIdx = 1; walkTries = 0; lastEntryIdx = 0;
      sendParamRead(walkIdx); walkDeadline = now + 400;
    } else if ((int32_t)(now - walkDeadline) >= 0) {
      if (++walkTries > 3) { Bench.println("crsf: no answer to the ping - module powered (7-13 V)? CRSF pad on GPIO 23? in WiFi mode (power-cycle it)? see key e"); walkStep = 0; return; }
      sendPing(); walkDeadline = now + 800;
    }
    return;
  }
  bool advance = false;                                  // walkStep 2: reading fields
  if (lastEntryIdx == walkIdx) {
    if (strcmp(lastEntryName, "Bind") == 0)        bindFieldIdx  = walkIdx;
    if (strcmp(lastEntryName, "Max Power") == 0)   powerFieldIdx = walkIdx;
    if (strcmp(lastEntryName, "Telem Ratio") == 0) telemFieldIdx = walkIdx;
    if (strcmp(lastEntryName, "Dynamic") == 0)     dynFieldIdx   = walkIdx;
    advance = true;
  } else if ((int32_t)(now - walkDeadline) >= 0) {
    if (++walkTries > 2) advance = true;                 // never answered; skip it
    else { sendParamRead(walkIdx); walkDeadline = now + 400; }
  }
  if (!advance) return;
  if (goalKnown(walkGoal) || walkIdx >= modParamCount) {
    Bench.printf("crsf: parameter walk done: bind=%u maxpower=%u telem=%u dynamic=%u\n", bindFieldIdx, powerFieldIdx, telemFieldIdx, dynFieldIdx);
    walkStep = 0;
    doGoal();
    return;
  }
  walkIdx++; walkTries = 0; lastEntryIdx = 0;
  sendParamRead(walkIdx); walkDeadline = now + 400;
}

void crsfBind()         { walkStart('n'); }
static uint32_t bindStartMs = 0, bindLastMs = 0; static uint8_t bindBursts = 0;
uint8_t crsfBindBursts() { return bindStartMs ? bindBursts : 0; }
void crsfBindLoop(bool on) {
  if (!on) { if (bindStartMs) Bench.printf("crsf: bind loop stopped after %u bursts\n", bindBursts); bindStartMs = 0; bindBursts = 0; return; }
  bindStartMs = millis(); bindLastMs = 0; bindBursts = 0;
  Bench.printf("crsf: bind loop - a Bind command every %lu s for %lu s, stops when the drone links. put the whoop in bind mode now (power it 3 times in a row, its LED double-blinks)\n", (unsigned long)(CRSF_BIND_EVERY_MS / 1000), (unsigned long)(CRSF_BIND_FOR_MS / 1000));
}
static void bindPoll() {
  if (!bindStartMs) return;
  uint32_t now = millis();
  if (crsfLinkUp()) { Bench.printf("crsf: bind loop - the drone is LINKED (LQ %u) after %u bursts, stopping\n", linkLQ, bindBursts); bindStartMs = 0; bindBursts = 0; return; }
  if (now - bindStartMs > CRSF_BIND_FOR_MS) { Bench.printf("crsf: bind loop - gave up after %u bursts and %lu s, no link. whoop in bind mode? (3 power cycles, fast double blink) module powered?\n", bindBursts, (unsigned long)(CRSF_BIND_FOR_MS / 1000)); bindStartMs = 0; bindBursts = 0; return; }
  if (walkStep) return;                                          // a walk / read-back in progress: wait for it
  if (bindLastMs && now - bindLastMs < CRSF_BIND_EVERY_MS) return;
  bindLastMs = now; bindBursts++;
  Bench.printf("crsf: bind burst %u\n", bindBursts);
  walkStart('n');
}
void crsfQuiet()        { walkStart('q'); }
void crsfDynamic()      { walkStart('D'); }
void crsfPowerUp()      { walkStart('p'); }
void crsfTelemetryOn()  { walkStart('e'); }
void crsfTelemetryStd() { walkStart('r'); }

// ---------------- status -------------------------------------------------
String crsfRawJson() {                        // the ring oldest first
  static const char* hex = "0123456789abcdef";
  String s; s.reserve(700);
  s += "{\"rawbytes\":" + String((unsigned long)rxRawBytes) + ",\"stray\":" + String((unsigned long)strayBytes) + ",\"ours\":" + String((unsigned long)(rxEchoOk * 26)) +
       ",\"secs\":" + String(millis() / 1000) + ",\"echo\":[" + String((unsigned long)rxEchoOk) + "," + String((unsigned long)rxEchoBad) + "],\"module\":[" + String((unsigned long)rxModOk) + "," + String((unsigned long)rxModBad) + "],\"raw\":\"";
  for (uint16_t k = 0; k < 256; k++) { uint8_t b = rawRing[(rawIdx + k) & 255]; s += hex[b >> 4]; s += hex[b & 15]; }
  s += "\"}";
  return s;
}
String crsfStatusLine() {
  char b[300];
  if (rxModOk == 0 && rxModBad == 0)
    snprintf(b, sizeof b, "crsf: GPIO %u %lu baud %u Hz, %lu frames sent%s; module SILENT - nothing heard (powered 7-13 V? CRSF pad on the wire? in WiFi mode after 60 s alone - power-cycle it)",
             PIN_CRSF, (unsigned long)CRSF_BAUD, (unsigned)(1000 / CRSF_PERIOD_MS), (unsigned long)framesSent, streaming ? "" : " (frames OFF)");
  else
    snprintf(b, sizeof b, "crsf: GPIO %u %lu baud %u Hz, %lu frames sent%s; module heard %lu ms ago (%lu good, %lu bad); drone link %s LQ %u RSSI %d dBm rf %u (down LQ %u %d dBm); drone '%s' %.1f V %u %%%s",
             PIN_CRSF, (unsigned long)CRSF_BAUD, (unsigned)(1000 / CRSF_PERIOD_MS), (unsigned long)framesSent, streaming ? "" : " (frames OFF)",
             (unsigned long)(millis() - rxLastModMs), (unsigned long)rxModOk, (unsigned long)rxModBad, crsfLinkUp() ? "UP" : "down", linkLQ, linkRssi, linkRfMode, downLQ, downRssi,
             fcMode[0] ? fcMode : "(no telemetry)", fcVolts, fcPct, attMs ? "" : ", no attitude yet");
  return String(b);
}
void crsfStatus() {
  Bench.println(crsfStatusLine());
  Bench.printf("crsf: sticks roll %u pitch %u throttle %u yaw %u | ARM AUX%u = %u | AUX2 %u (angle)\n",
               getChannel(CH_ROLL), getChannel(CH_PITCH), getChannel(CH_THROTTLE), getChannel(CH_YAW), (unsigned)(FLY_ARM_AUX - 3), getChannel(FLY_ARM_AUX), getChannel(5));
  Bench.printf("crsf: wire - our own frames heard back ok=%lu bad=%lu; raw bytes=%lu (%lu/s lately), stray (no frame) %lu (%lu/s lately)%s%s\n", (unsigned long)rxEchoOk, (unsigned long)rxEchoBad, (unsigned long)rxRawBytes,
               (unsigned long)rateRaw, (unsigned long)strayBytes, (unsigned long)rateStray,
               (rxEchoOk == 0 && rxEchoBad == 0) ? "  <- not hearing our own frames: GPIO / UART problem" : "",
               (rateStray > 200) ? "  <- SOMETHING is transmitting on the wire that is not CRSF at this baud: the module in WiFi mode (power-cycle it) or at another baud rate; /fly?raw=1 shows the bytes" : "");
  if (bindStartMs) Bench.printf("crsf: bind loop running, burst %u\n", bindBursts);
  { String h; for (uint16_t k = 208; k < 256; k++) { char t[4]; snprintf(t, sizeof t, "%02x ", rawRing[(rawIdx + k) & 255]); h += t; } Bench.println("crsf: last 48 bytes on the wire: " + h); }
  Bench.printf("crsf: frame types - linkstats=%lu sync=%lu devinfo=%lu entries=%lu telemetry=%lu (temp=%lu rpm=%lu) other=%lu (last 0x%02X from 0x%02X) | commands sent=%lu | fc cycle %u us load %u %%, temp %s\n",
               (unsigned long)cntLink, (unsigned long)cntSync, (unsigned long)cntDevInfo, (unsigned long)cntEntry, (unsigned long)cntTelem, (unsigned long)cntTemp, (unsigned long)cntRpm, (unsigned long)cntOther, lastOtherType, lastOtherAddr, (unsigned long)pendingSent,
               fcCycleUs, fcLoadPct, fcTempMs ? (String(fcTempC, 1) + " C (source " + String(fcTempSrc) + ")").c_str() : "never received");
  if (modParamCount) Bench.printf("crsf: module '%s' ELRS %u.%u.%u, %u fields; bind=%u maxpower=%u telem=%u\n",
                                  modName, modSwVer[0], modSwVer[1], modSwVer[2], modParamCount, bindFieldIdx, powerFieldIdx, telemFieldIdx);
  if (attMs) Bench.printf("crsf: drone attitude roll %.0f pitch %.0f yaw %.0f deg, %lu ms ago\n", attRoll, attPitch, attYaw, (unsigned long)(millis() - attMs));
  { char t[160]; crsfArmingFlagText(t, sizeof t); Bench.printf("crsf: msp - %lu status requests, %lu replies; arming-disable flags %s%s\n", (unsigned long)mspReqs, (unsigned long)mspResps, armFlagsMs ? "" : "never received", armFlagsMs ? (String("0x") + String((unsigned long)armFlags, HEX) + " (" + String(armFlagCount) + " defined) = " + (t[0] ? t : "(stale)") + ", " + String((millis() - armFlagsMs) / 1000) + " s ago").c_str() : ""); }
}

// ---------------- bring-up --------------------------------------------------
void crsfBegin() {
  crsfSafe();
#if HW_FAKE
  Bench.printf("crsf: FAKE - no UART on GPIO %u; frames are built, not sent; the test injects the module's bytes\n", PIN_CRSF);
#else
  // One wire, true half duplex, no resistor: TX and RX both on PIN_CRSF. The Arduino pin manager
  // refuses one GPIO as both UART RX and TX, so the pad is routed at IDF level. Push-pull only
  // while a frame goes out.
  crsf.setRxBufferSize(1024);
  crsf.begin(CRSF_BAUD, SERIAL_8N1, -1, PIN_CRSF);
  uart_set_pin(UART_NUM_1, PIN_CRSF, PIN_CRSF, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  gpio_pullup_en((gpio_num_t)PIN_CRSF);
  gpio_set_direction((gpio_num_t)PIN_CRSF, GPIO_MODE_INPUT);
  xTaskCreatePinnedToCore(crsfTask, "crsf", 3072, nullptr, 3, nullptr, 0);
  Bench.printf("crsf: ELRS Nano TX on GPIO %u, %lu baud 8N1, RC frames at %u Hz, one wire half duplex; sticks centred, throttle min, every AUX low (disarmed)\n",
               PIN_CRSF, (unsigned long)CRSF_BAUD, (unsigned)(1000 / CRSF_PERIOD_MS));
#endif
}

void crsfPoll() {
#if HW_FAKE
  { static uint32_t lastStep = 0; uint32_t now = millis(); if (now - lastStep >= CRSF_PERIOD_MS) { lastStep = now; taskStep(); } }   // what the core-0 task does, on the fake clock
#endif
  pumpRx();
  walkPoll();
  bindPoll();
  { bool blocked = crsfCraftBlocked() && crsfFcModeAgeMs() <= 1500;   // MSP_STATUS from the FC: often while it says '!' (why?), steadily otherwise (cycle time, load, flags)
    if (crsfLinkUp() && crsfFcSeen() && !pendingLen && !walkStep && millis() - mspReqMs >= (blocked ? CRSF_MSP_EVERY_MS : CRSF_MSP_POLL_MS)) crsfMspSend(101); }
  if (linkUp && lastLinkStatsMs && millis() - lastLinkStatsMs > CRSF_LINKSTATS_LOST_MS) {   // the drone (or its battery) went away: no statistics for 2 s
    linkUp = false; linkLQ = 0; downLQ = 0;
    Bench.println("crsf: drone link LOST - no link statistics from the module for 2 s (drone off, battery out, or out of range)");
  }
  if (CRSF_AUTO_DYNAMIC && !quietDone && heardSince && crsfModuleHeard() && millis() - heardSince >= CRSF_AUTO_DYNAMIC_MS && !walkStep) {   // the module (re)appeared: the cap and Dynamic power
    quietDone = true; dynamicSet = false;
    Bench.printf("crsf: module appeared - Max Power to the %u mW cap with Dynamic power on (CRSF_AUTO_DYNAMIC; Q = lowest for flashing)\n", (unsigned)CRSF_POWER_MAX_MW);
    crsfDynamic();
  }
  if (crsfModuleHeard() && !crsfLinkUp() && !bindStartMs && heardSince && !walkStep) {   // the module is on and hears no drone: bind in case the whoop sits in bind mode (3 quick power cycles put it there)
    uint32_t now2 = millis(), since = lastLinkMs > heardSince ? lastLinkMs : heardSince;
    if (now2 - since >= CRSF_AUTO_BIND_MS && (!lastAutoBindMs || now2 - lastAutoBindMs >= CRSF_AUTO_BIND_AGAIN_MS)) { lastAutoBindMs = now2; Bench.println("crsf: module on, no drone for 20 s - running the bind loop by itself (harmless if the drone is simply off)"); crsfBindLoop(true); }
  }
  static uint32_t winMs = 0; static uint32_t winRaw = 0, winStray = 0;   // the recent wire rates (the boot average hid a clean wire behind the module's start-up garbage)
  uint32_t now = millis();
  if (now - winMs >= 10000) { rateRaw = (rxRawBytes - winRaw) / ((now - winMs) / 1000); rateStray = (strayBytes - winStray) / ((now - winMs) / 1000); winRaw = rxRawBytes; winStray = strayBytes; winMs = now; }
}
uint8_t  crsfPowerLevel() { return powerLevel; }   // (declared after the parameter walk, so defined down here)

