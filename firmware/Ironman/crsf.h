// CRSF to the BetaFPV ELRS Nano TX V2 on ONE wire (PIN_CRSF, GPIO 23), half duplex, and
// everything the module says back: link statistics, the module's own parameters (bind, power,
// telemetry ratio) and the flight controller's telemetry it forwards (mode string, battery,
// attitude). The HandBench copy of the gauntlet's radio.cpp, which proved all of this on
// 2026-09-06 (400000 baud; the pad is handed to UART1 only while a frame goes out).
//
// It streams RC frames from crsfBegin() on: every stick centred, throttle at minimum, every
// AUX low = disarmed. fly.cpp writes the sticks 50 times a second; if it stops (loop stalled,
// firmware being written) the CRSF task itself puts throttle and ARM back down.
#pragma once
#include <Arduino.h>

static constexpr uint16_t CRSF_MIN = 172, CRSF_MID = 992, CRSF_MAX = 1811;   // 1000 / 1500 / 2000 us

void     crsfBegin();                         // UART1 on PIN_CRSF, the core-0 task; frames go out from the first moment
void     crsfPoll();                          // every loop(): parse what the module sends, run a pending module command

// the sticks (AETR: 0 roll, 1 pitch, 2 throttle, 3 yaw; AUX1 = index 4)
void     crsfSetSticks(uint16_t roll, uint16_t pitch, uint16_t throttle, uint16_t yaw);   // 172..1811; also feeds the task's watchdog
void     crsfSetAux(uint8_t index, bool high);
void     crsfSafe();                          // everything centred, throttle min, every AUX low
uint16_t crsfChannel(uint8_t index);
bool     crsfWatchdogTripped();               // the task had to drop throttle/ARM because loop() stopped feeding it (cleared on read)
void     crsfStream(bool on);                 // key C: stop / start the RC frames (the module then loses the "handset")
bool     crsfStreaming();
uint32_t crsfFramesSent();

// what the module says
bool     crsfModuleHeard();                   // a good frame from the module inside CRSF_QUIET_MS
uint32_t crsfModuleAgeMs();
bool     crsfLinkUp();                        // the module reports LQ > 0 to the receiver
uint8_t  crsfLQ();  int8_t crsfRssi();  uint8_t crsfRfMode();  uint8_t crsfDownLQ();  int8_t crsfDownRssi();
// what the flight controller says, through the module
const char* crsfFcMode();                     // Betaflight's mode string: "ANGL*" ready, "ANGL!" arming blocked, "ANGL" armed; "" = never seen
bool     crsfFcSeen();
uint32_t crsfFcModeAgeMs();
bool     crsfCraftArmed();  bool crsfCraftReady();  bool crsfCraftBlocked();
float    crsfCraftVolts();  uint8_t crsfCraftPct();  float crsfCraftAmps();
bool     crsfAttitude(float& rollDeg, float& pitchDeg, float& yawDeg, uint32_t& ageMs);   // ATTITUDE 0x1E; false until seen
// why it will not arm: MSP_STATUS over CRSF (0x7A / 0x7B through the module), asked every CRSF_MSP_EVERY_MS while the mode string says '!'
void     crsfMspAsk();                        // one request now (GET :8080/fly?msp=1), the reply's bytes logged
void     crsfMspSend(uint8_t cmd);            // any no-payload MSP v1 request to the FC
void     crsfAccCalibrate();                  // MSP_ACC_CALIBRATION (205): the drone still and level on the floor for 3 s - Betaflight's "level" is wherever this was last done (GET :8080/fly?acccal=1, the page's "level the drone")
bool     crsfArmingFlagText(char* out, size_t n);   // "CALIB (gyro calibrating - hold it still) + MSP (configurator attached)"; false = none known / stale
uint32_t crsfArmingFlags(uint32_t* ageMs);    // the raw bits
uint32_t crsfAccCalAckMs();                   // millis() when the FC last acknowledged the accelerometer calibration (0 never)
float    crsfTempC(uint32_t* ageMs, uint8_t* src);   // the hottest value of the last CRSF TEMP frame (0x0D) in degC, NAN if never; src = the frame's source id
float    crsfCoreTempC(uint32_t* ageMs);      // the FC's chip temperature from MSP_STATUS (degC), NAN if never
uint16_t crsfFcCycleUs();  uint16_t crsfFcLoadPct();   // from MSP_STATUS, polled every CRSF_MSP_POLL_MS while linked
const char* crsfModuleName();  void crsfModuleVersion(uint8_t* v3);  uint8_t crsfPowerLevel();   // the module as it introduced itself; the Max Power index we last set (0 = lowest)
uint32_t crsfTempFrames();

// module commands (what the ELRS Lua does, over the same wire)
void     crsfBind();                          // one Bind command (receiver in bind mode first)
void     crsfBindLoop(bool on);               // key N / the page: a Bind command every CRSF_BIND_EVERY_MS for CRSF_BIND_FOR_MS, stops itself once the drone is linked (2026-09-06 took ~28 bursts while the whoop was power-cycled into bind mode)
uint8_t  crsfBindBursts();                    // > 0 while the loop runs (bursts so far), 0 idle
void     crsfQuiet();                         // Max Power -> lowest (key Q, for flashing with the module on)
void     crsfDynamic();                       // Max Power -> the CRSF_POWER_MAX_MW cap and Dynamic -> Dyn (automatic when the module appears)
bool     crsfDynamicOn();                     // Dynamic -> Dyn has been sent since the module last appeared
uint8_t  crsfDynamicIdx();                    // the "Dyn" option index (1 until the option text is read)
const char* crsfDynamicOptions();             // the Dynamic field's option text, "" until read
void     crsfPowerUp();                       // Max Power one notch up (wraps)
void     crsfTelemetryOn();                   // Telem Ratio 1:2
void     crsfTelemetryStd();

String   crsfRawJson();                       // GET :8080/fly?raw=1 -> the last 256 bytes seen on the wire as hex, stray-byte counts: what IS the module sending?
String   crsfStatusLine();                    // key m: one line
void     crsfStatus();                        // key e: the detail (wire, module, link, craft, counters)
void     crsfBuildRcFrame(uint8_t* buf26);     // the RC frame from the channels as they are (the self-test decodes it back)
uint8_t  crsfCrc8(const uint8_t* p, uint8_t n); // CRC8 DVB-S2 over p
uint8_t  crsfPowerMaxIdx();                   // the Max Power option index the lock allows (CRSF_POWER_MAX_MW)
const char* crsfPowerOptions();               // the module's option text, "" until the parameter walk has read it
#if HW_FAKE
void     crsfFakeInject(const uint8_t* b, uint8_t n);   // bytes "from the module" for the parser (whole frames, sync byte first)
uint8_t  crsfFakeLastTx(uint8_t* out, uint8_t cap);     // the last frame the task "sent"
uint32_t crsfFakeTxCount();
#endif
