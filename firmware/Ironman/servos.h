// The PCA9685 (I2C bus 1, 0x40): six MG90S on channels 0-5 (the armor cascade, proven 2026-09-07)
// and static outputs on channels 6-15 driven with the full-on / full-off bits: 6 the green laser's RFP30N06LE gate, 7 the red laser's TTL input, 9-11 the three puck-group D4184 TRIG inputs (8 and 12-15 spare) -
// on / off only. OE on PIN_PCA_OE holds every channel at 0 V from power-on until
// servosBegin() has parked them, and again whenever nothing is moving.
#pragma once
#include <Arduino.h>

bool servosBegin();               // PCA9685 init on an already-running Wire; parks every channel. false = not found (commands locked out)
bool servosReady();
void servosPower(bool on);        // OE: on = outputs live. Static channels (6-15) that are on keep it live too (see pcaSetDigital)
void servoWriteUs(uint8_t ch, uint16_t us);   // clamped to SERVO_US_MIN..MAX, servo channels only
void servoRelease(uint8_t ch);    // no pulses on that channel
void servosParkAll();             // no pulses on any servo channel, every static channel off, OE off - the safe state
uint16_t servoChannelUs(uint8_t ch);          // the pulse a servo channel is getting right now, 0 = released
// static outputs
bool pcaSetDigital(uint8_t ch, bool on);      // channels 6-15: full-on / full-off. false = not a static channel or PCA absent
bool pcaDigitalState(uint8_t ch);
uint8_t pcaDigitalOnCount();                  // how many static channels are on (they keep OE live)
void i2cScan();

// Bench moves on channel 0
void servoSweepOnce();            // closed -> open -> closed with easing, then release
void servoHold(uint16_t us);      // 1.5 s
void servoHoldForMeter();         // 20 s, 'x' ends early
void servosAllSweep();            // all six servo channels 1200<->1800 for 30 s, 'x' ends early
void servoFullTravel();           // ch0 1000<->2000 x3 - unmounted servo only
void servoWideTravel();           // ch0 600<->2400 once, slow - bare servo only, finds its stops

// The armor cascade (plates in config.h). Non-blocking: call servosPoll() every loop().
// Opening: plate 1 + the hand plate, then plate 2 once the hand plate has landed, then plate 3; closing the reverse.
void servosOpen();
void servosClose();
void servosOpenThenClose();       // open, CASCADE_PAUSE_MS, close - a timed step in servosPoll(), never a delay()
bool servosMovePlate(uint8_t plateNo, bool open);   // ONE plate (1-3 forearm, 4 the hand plate; PLATE_GROUP_OF), its servos together. false = busy / bad number / no PCA
bool servosMovePlateWith(uint8_t plateNo, bool open);   // (2026-10-21) the puck's plate buttons: the same, with what it needs opened first / what needs it closed first
void servosAbort();               // stop a running cascade (or the pause before its close) and park - used by goSafe()
void servosPoll();
bool servosBusy();                // a move is running, or open-pause-close is in its pause
uint32_t servosStallCount();      // stalls the INA219 has caught since boot (current.cpp, config SERVO_STALL_*)
const char* servosLastStall();    // the last one in words, "" if none
// Unknown = some servo has not arrived anywhere since boot (or stopped mid-move); Mixed = all arrived, not all at one end
// (one plate opened alone). The numbers are the link's (GauntletLink $S / $G "plates"): keep the order.
enum class PlatesState : uint8_t { Unknown, Open, Closed, Opening, Closing, Mixed };
PlatesState servosPlatesState();
const char* servosPlatesStateName();
uint32_t servosCascades();        // cascades completed since boot
uint32_t servosPlateMoves();      // single-plate moves completed since boot
uint32_t servosStarts();          // sequences started since boot, cascades and single plates (the puck link's events)
int16_t  servoPosPermille(uint8_t i);   // servo i of PLATES[]: 0 = its closed pulse, 1000 = its open pulse (the last pulse it was given), -1 = never driven since boot
uint8_t  servosDrivenMask();      // bit i = servo i of PLATES[] is getting pulses right now
uint8_t  servosMovingPlate();     // the plate (1-4) moving alone right now, 0 = none (or a cascade)
// The plates' mechanical order (config.h PLATE_NEEDS / PLATE_OPEN_ORDER, Toby 2026-10-07 night) and who holds a plate open
constexpr uint8_t SERVO_HOLD_THERMAL = 1, SERVO_HOLD_RED = 2, SERVO_HOLD_GREEN = 4, SERVO_HOLD_LIDAR = 8;   // LIDAR: the puck's LIDAR button (2026-10-20)
int8_t      servosPlateState(uint8_t plateNo);   // 1 open, 0 closed, -1 not known (boot, stopped mid-move, moving now)
bool        servosPlateOpen(uint8_t plateNo);
void        servosHold(uint8_t plateNo, uint8_t who, bool on);   // SERVO_HOLD_*: while held a plate never closes
uint8_t     servosHolds(uint8_t plateNo);
void        servosRequest(uint8_t plateNo, bool open);   // the lasers / the camera: open or close it now, or when the running move ends; a close only if nothing needs it
const char* servosLastRefusal();             // why the last plan was refused or a plate was skipped
const char* servosPlateName(uint8_t plateNo); // "plate 1" .. "the hand plate"
uint16_t servoUsClosed();
uint16_t servoUsOpen();
void servoNudgeClosed(int16_t d);
void servoNudgeOpen(int16_t d);
String servosJson();              // {"state":"open","busy":0,"oe":1,"us":[...6],"dig":[...10 for ch 6-15]}
void servosSleep();               // the CRITICAL shutdown: everything parked, OE high, the PCA9685 asleep (a reboot wakes it)
bool servosAsleep();
void servosWake();                // the fake build's self-test only
uint8_t     servosMode2();                 // the PCA9685's MODE2 as read back at boot (PCA_MODE2_VALUE 0x04 = totem pole, outputs LOW while OE is high)

// Set on the day, kept in NVS - no recompile (2026-10-13). The channel map: the PCA9685 output each channel is wired to
// (identity by default; /pcamap). Each servo's closed / open end found on the real plate with the /cal page - it wins
// over PLATES[] in config.h. A held servo is released after SERVO_CAL_HOLD_MS without a move.
uint8_t     pcaPhysical(uint8_t ch);
bool        pcaMapIsIdentity();
const char* pcaMapSet(const String& spec);    // "6=13,9=6" (each L=P a swap) or "reset"; "" = done, else why not
uint16_t    servoEndUs(uint8_t i, bool open);    // servo i of PLATES[]: the end in effect (saved, else config.h, else derived)
uint16_t    servoCalSaved(uint8_t i, bool open); // 0 = none saved
int8_t      servoCalHeld();                      // the servo the /cal page holds now, -1 none
uint16_t    servoLastUs(uint8_t i);              // the last pulse servo i was given, 0 = never since boot
const char* servoCalMove(uint8_t i, uint16_t us, uint8_t pair = 0);    // pair 1 = its partner too on the same pulse, 2 = mirrored (until both have saved ends)
const char* servoCalSave(uint8_t i, bool open, bool both = false);    // both: the partner's held pulse saved too
const char* servoCalSetEnd(uint8_t i, bool open, int us, bool keep); // an end by number, no move: keep = saved in NVS, else tried until a reboot (/servocal?s=N&end=open|closed&us=X[&keep=1])
const char* servosPlateMotion(uint8_t g, int ms, int lin, bool keep);   // a plate's move time (0 = config.h's, -1 unchanged) and motion (1 constant speed, 0 smoothstep, -1 unchanged), tried or kept (/servocal?plate=N&ms=M&ease=linear|smooth[&keep=1])
String      servosPlateMotionJson(uint8_t g);
bool        servosPlateLinear(uint8_t g);        // the plate moves at constant speed (PLATE_LINEAR_DEFAULT / the /cal page)
uint32_t    servosPlateMs(uint8_t g);            // its move time now (the /cal page's, else config.h's)
int8_t      servoCalPair();                      // the partner held with it, -1 none
uint16_t    servosFollowerUs(uint16_t leaderUs);  // plate 3: 3B's pulse for 3A's (PLATE3_PATH in config.h)
void        servosFollowSet(bool on);              // off = the /cal page moves 3B on its own (finding its closed); not saved
bool        servosFollowOn();
const char* servoCalSweep(uint8_t i, uint16_t from, uint16_t to, uint32_t ms, uint8_t pair = 0);   // the quick smooth move, run on the board (pair: its partner in step)
bool        servoCalSweeping();
void        servoCalSweepThen(uint8_t i, uint16_t from, uint16_t to, uint32_t ms);   // a second quick move the instant the first lands
const char* servosPathSet(const String& spec);   // plate 3's path live: a1,b1,a2,b2 (deg) or reset (NVS p3p)
String      servosPathJson();
void        servoCalClear(uint8_t i);
void        servoCalRelease();
bool        servosPlatesOff();                   // the plates switched off on the /cal page: no plate moves; lasers / camera act at once
void        servosSetPlatesOff(bool off);
