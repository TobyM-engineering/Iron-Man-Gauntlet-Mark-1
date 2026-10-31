// The glove's battery: the 2S pack through 100k / 22k on PIN_VBAT, read every poll, 1 s average,
// and the voltage rules of design/01-power.md section 10 with hysteresis and persistence:
//   OK        above every threshold
//   (Toby, 2026-10-08) no sound and no ring blink from ARMFLOOR to IDLE: the puck and the OLED show the state ($S)
//   ARMFLOOR  < 7.5 V  arming refused (7.2 until 2026-10-08; the ELRS module is a 7-13 V part);
//             armed and < BATT_LAND_WARN_V 7.3 V: LAND NOW, the vibrator every 5 s (Ironman.ino)
//   LOW       < 7.0 V  the high-power LED show refused (BuckPuck headroom), the idle lights off
//   IDLE      < 6.8 V  forced idle: plates park, lasers and pucks off, drone disarmed
//   CRITICAL  < 6.6 V  shutdown: a silent 5 s grace (ring blinks red fast), everything off, asleep, a chirp every 5 s
//   FAULT     < 4.6 or > 9.5 V for 2 s: the MONITOR is broken (snapped divider leg, missing) - LATCHED until power-off
//             (2026-10-08): no new show, no arming, a distinct buzz every 30 s; in flight a warning only
//   NOPACK    a steady 4.6 - 5.8 V: USB or a 5 V bench feed, no pack; no rules apply
#pragma once
#include <Arduino.h>

enum class PowerState : uint8_t { Unknown, Ok, ArmFloor, Low, Idle, Critical, Fault, NoPack };

void        powerBegin();
void        powerPoll();                   // every loop()
float       powerVolts();                  // filtered pack voltage
PowerState  powerState();
const char* powerStateName();
bool        powerArmOk();                  // arming allowed by the pack
bool        powerShowOk();                 // the BuckPuck show allowed by the pack
bool        powerIdleForced();             // IDLE or worse
bool        powerCritical();
bool        powerFaultLatched();           // a FAULT was seen since power-up (it never clears by itself)
const char* powerBlockText();              // why arming / the show is refused ("" when allowed)
void        powerOnChange(void (*fn)(PowerState from, PowerState to));   // one listener (Ironman.ino: goSafe, buzzer, ring)
String      powerJson();                   // {"v":7.86,"state":"ok","pin":1.42,"armok":1,"showok":1,"k":1.0000}
float       powerCalFactor();              // the meter calibration (1 = none; NVS "bk")
const char* powerCalMeter(float meter);    // the meter reads this many volts on the pack now (0 = forget); "" = done, else why not
String      powerStatusLine();
#if HW_FAKE
void        powerFakeVolts(float v);       // the pack reads this from now on
void        powerFakeFaultReset();         // the self-test: forget a latched FAULT (a real arm needs a power-off)
#endif
