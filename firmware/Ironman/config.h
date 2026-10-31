// Ironman - the main-board firmware: Gauntlet/ (plates, chains, bench test) + HandBench/ (flight,
// gestures, laser, LiDAR) merged, on the design in design/README.md. ONE config.h is the source
// of truth for every pin, address, channel and limit; design/check.ps1 parses the tables below.
//
// ONE design, ONE build (Toby, 2026-10-07): the arm at the summit is the whole arm, everything in it used -
// both BNO085s, every flex sensor, the plates, the palm LEDs, the strip, the round screen with the puck app, the
// thermal camera, the OLED, BOTH LASERS and THE DRONE (Toby, 2026-10-08: the organisers approved the drone and the
// lasers). The only variant (flash.sh --fake):
//   HW_FAKE 1       no hardware touched at all: every sensor is scripted, every actuator recorded, the
//                   self-test runs at boot (selftest.cpp) and prints its report on the serial console
#pragma once
#include <Arduino.h>

// flash.sh writes Ironman/variant.h (gitignored) with the build's defines (--fake -> HW_FAKE 1) instead
// of passing -D on the compile line: a -D changes every file's flags and throws the ESP32 core's build
// cache away (a ten-minute rebuild); a header only recompiles the sketch.
#if __has_include("variant.h")
#include "variant.h"
#endif

#ifndef HW_FAKE
#define HW_FAKE 0
#endif
#ifdef BUILD_SUMMIT
#error "the summit build is retired (2026-10-07: one design, one build) - delete BUILD_SUMMIT from variant.h / the compile line"
#endif

constexpr char HOSTNAME[] = "ironman";      // ironman.local: flash.sh, monitor.sh, the sim page (:8080/) and the HUD (:8080/hud)
// WiFi ON AT BOOT (Toby, 2026-10-08: "no WiFi" was a misread - the arm must work on its own with no computer, WiFi stays):
// one build, no key or switch position needed. It joins in the background (the loop runs from the first moment) and
// brings back the over-the-air flashing, the console over HTTP, the sim page, the HUD and every HTTP route - for testing,
// watching and coding the arm while it runs on its battery. NOTHING depends on it: every control is a gesture, the mode
// switch or a key on the USB console, and the arm runs the same with no network. The CRITICAL shutdown turns the radio
// off before the deep sleep (shutdown.cpp). The ELRS module's OWN WiFi stays disabled (its Auto On cleared, 01 section 5).
constexpr bool NET_WIFI_AT_BOOT = true;

// =====================================================================================================
// PIN TABLE - the single source of truth (design/03-pins-and-buses.md). design/check.ps1 parses the
// constants AND the table: a pin double-booked, a strap / flash / console pin, or an input-only pin
// declared as an output fails the check. Never put anything on 0, 2, 5, 12, 15 (boot straps: 12
// high at boot bricks the start, 5 pulls up at boot), 6-11 (flash), 1/3 (USB console).
// Usable on the WROOM-32 devkit: 4 13 14 16 17 18 19 21 22 23 25 26 27 32 33 (GPIO) + 34 35 36 39
// (input-only, no internal pulls). NOTE: 16/17 are free on the WROOM-32 only - on a WROVER they are
// the PSRAM's. This firmware is for the WROOM-32 devkit.
// =====================================================================================================
constexpr uint8_t PIN_I2C1_SDA     = 21;   // bus 1 (forearm): PCA9685 0x40, forearm BNO08X (no straps since 2026-10-07: its own address, 0x4B on the bench pair - 0x4A and 0x4B both probed, 00 #27), INA219 0x41 (A0 bridged; the servo branch's current, stall detection)
constexpr uint8_t PIN_I2C1_SCL     = 22;
constexpr uint8_t PIN_I2C2_SDA     = 16;   // bus 2 (palm, across the wrist): hand BNO08X 0x4A, ADS1115 0x48 (ADDR to GND), DRV2605L 0x5A, LIDAR-Lite v3 0x62
constexpr uint8_t PIN_I2C2_SCL     = 17;
constexpr uint8_t PIN_PCA_OE       = 23;   // PCA9685 output enable, active LOW; 1 k pull-up to 3V3 at the PCA: the Adafruit board's own R7 (10 k to GND) makes 1 k -> 3.0 V a solid "off", 10 k only 1.65 V
constexpr uint8_t PIN_CRSF         = 25;   // one wire to the ELRS Nano TX V2, half duplex (UART1 TX while a frame goes out)
constexpr uint8_t PIN_ARM_LED      = 13;   // forearm WS2812B strip data, straight from the pin through 330 R (as the first bench test), pixel 0 at the elbow
constexpr uint8_t PIN_PALM_LED     = 27;   // palm puck data (19 px: 12-ring + 7-jewel), straight from the pin through 330 R, across the wrist
constexpr uint8_t PIN_VBAT         = 35;   // battery divider 100k / 22k from the SWITCHED bus, 100 nF at the pin (input-only)
constexpr uint8_t PIN_FLEX_THUMB   = 34;   // thumb flex divider (47 k + 100 nF at the pin), ADC1 (input-only). Four fingers are on the ADS1115
constexpr uint8_t PIN_S3_TX        = 18;   // UART2 TX -> round display GPIO16
constexpr uint8_t PIN_S3_RX        = 19;   // UART2 RX <- round display GPIO15 (touch events, heartbeat)
constexpr uint8_t PIN_BUZZER       = 26;   // MusRock passive buzzer module (its own PNP, LOW-LEVEL TRIGGER, VCC 3.3 V): LEDC tones with the output inverted, the pin idles HIGH
constexpr uint8_t PIN_MODE_A       = 36;   // mode switch pole A (ON-OFF-ON, common to GND), 10 k pull-up + 100 nF. POLLED, never an interrupt (36/39 glitch while ADC1 samples - ESP32 errata)
constexpr uint8_t PIN_MODE_B       = 39;   // mode switch pole B, same
constexpr uint8_t PIN_ACCENT       = 4;    // three red 2 mm accent LEDs in the palm, one 270 R each (14 mA from the pin at the highest drive setting). v3 (2026-10-06): the LED-branch MOSFET is gone, this pin is free and has no boot pulse
constexpr uint8_t PIN_IMU_INT_ARM  = 32;   // forearm BNO085 INT (optional - read only when the sensor says it has data)
constexpr uint8_t PIN_IMU_INT_HAND = 33;   // hand BNO085 INT, across the wrist (optional)
constexpr uint8_t PIN_SPARE        = 14;   // nothing on it (the #1054 red-laser fallback would sink here, 00 #23). 5 is a boot strap and stays unused on purpose.

enum class PinDir : uint8_t { In, Out, InOut, Analog };
struct PinDef { const char* name; uint8_t gpio; PinDir dir; const char* note; };
constexpr PinDef PINS[] = {
  { "I2C1_SDA",     PIN_I2C1_SDA,     PinDir::InOut,  "bus 1 forearm" },
  { "I2C1_SCL",     PIN_I2C1_SCL,     PinDir::InOut,  "bus 1 forearm" },
  { "I2C2_SDA",     PIN_I2C2_SDA,     PinDir::InOut,  "bus 2 palm (WROOM-32 only)" },
  { "I2C2_SCL",     PIN_I2C2_SCL,     PinDir::InOut,  "bus 2 palm (WROOM-32 only)" },
  { "PCA_OE",       PIN_PCA_OE,       PinDir::Out,    "high at boot = every PCA output off" },
  { "CRSF",         PIN_CRSF,         PinDir::InOut,  "half duplex UART1" },
  { "ARM_LED",      PIN_ARM_LED,      PinDir::Out,    "WS2812 data, 330 R, direct" },
  { "PALM_LED",     PIN_PALM_LED,     PinDir::Out,    "WS2812 data, 330 R, direct" },
  { "VBAT",         PIN_VBAT,         PinDir::Analog, "100k/22k divider" },
  { "FLEX_THUMB",   PIN_FLEX_THUMB,   PinDir::Analog, "47k divider" },
  { "S3_TX",        PIN_S3_TX,        PinDir::Out,    "UART2 to the display" },
  { "S3_RX",        PIN_S3_RX,        PinDir::In,     "UART2 from the display" },
  { "BUZZER",       PIN_BUZZER,       PinDir::Out,    "LEDC tone, inverted (module)" },
  { "MODE_A",       PIN_MODE_A,       PinDir::In,     "polled, external 10k pull-up" },
  { "MODE_B",       PIN_MODE_B,       PinDir::In,     "polled, external 10k pull-up" },
  { "ACCENT",       PIN_ACCENT,       PinDir::Out,    "3 red LEDs, 270 R each" },
  { "IMU_INT_ARM",  PIN_IMU_INT_ARM,  PinDir::In,     "optional" },
  { "IMU_INT_HAND", PIN_IMU_INT_HAND, PinDir::In,     "optional" },
};
constexpr uint8_t PIN_COUNT = sizeof(PINS) / sizeof(PINS[0]);

// ---------------- I2C ----------------------------------------------------------------------------
// Bus 1 is short and inside the gauntlet: 400 kHz (the PCA9685 and the OLED ran there at 400 since
// 2026-09-07). Bus 2 crosses the wrist: 100 kHz is what the bench rig proved for two weeks on a long
// wire; raise it to 400 kHz once bring-up step 8 passes with the real harness. Pull-ups: 2.2 k on
// bus 1 at the ESP32; NONE added on bus 2 - the LIDAR-Lite v3's internal 3 k plus the two boards'
// 10 k give about 1.9 k (3 || 10 || 10), and 2.2 k on top would be 1.0 k and 3.2 mA.
constexpr uint32_t I2C1_HZ = 400000;
constexpr uint32_t I2C2_HZ = 100000;
constexpr uint32_t I2C_HZ  = I2C2_HZ;        // (the name the HandBench modules use: the palm bus speed)

struct I2cDev { const char* name; uint8_t bus; uint8_t addr; bool optional; };
constexpr I2cDev I2C_DEVICES[] = {
  { "PCA9685",             1, 0x40, false },
  { "PCA9685 all-call",    1, 0x70, false },   // the same chip answers here until the driver clears it
  { "BNO085 forearm",      1, 0x4A, true  },   // Teyleten GY-BNO08X on four wires, no straps (Toby, 2026-10-07): the board's own address - the bench pair answered 0x4B; BNO_ADDRS probes both, so this entry only names the scan (00 #27)
  { "INA219 servo branch", 1, 0x41, true  },   // A0 bridged (0x40 is the PCA9685); in series at the head of the servo branch: the six servos + the palm feed (current.cpp)
  { "BNO085 hand",         2, 0x4A, true  },   // the same: no straps
  { "ADS1115 fingers",     2, 0x48, true  },   // ADDR -> GND
  { "LIDAR-Lite v3",       2, 0x62, true  },
  { "DRV2605L haptic",     2, 0x5A, true  },   // Adafruit #2305 on the palm board: it drives the #1201 motor itself - no PCA channel, no transistor (haptic.cpp)
};
constexpr uint8_t I2C_DEVICE_COUNT = sizeof(I2C_DEVICES) / sizeof(I2C_DEVICES[0]);
constexpr uint8_t ADS1115_ADDR = 0x48;
constexpr uint8_t LIDAR_ADDR   = 0x62;
constexpr uint8_t INA219_ADDR  = 0x41;
// The INA219's shunt, as fitted: the HiLetgo board's stock R100 (0.1 ohm), kept (Toby, 2026-10-07). It reads to
// 3.2 A, above both stall limits below, and drops 0.17 V in a normal cascade (01 section 11). current.cpp keeps the
// library's 0.1-ohm calibration and scales by 0.1 / INA219_SHUNT_OHM (1.0 with the stock shunt).
constexpr float    INA219_SHUNT_OHM    = 0.1f;
// Servo stall detection on the INA219 (servos.cpp): the servos' own current = the INA219 reading less the palm
// ring's estimate (the ring rides the same palm feed). 01 section 3: a MG90S moving a plate ~0.45 A, stalled ~0.8 A.
// TUNE on the bench (05 step 4): run cascades with the log's "I x.xx A" lines on and set these above a healthy one.
constexpr float    SERVO_STALL_TOTAL_A = 2.5f;   // while a cascade runs: the servos together above this ...
constexpr float    SERVO_HOLD_STALL_A  = 0.6f;   // ... or above this once every move has landed (holding should be ~0.1 A) ...
constexpr uint32_t SERVO_STALL_MS      = 300;    // ... for this long = a stall: every servo of the cascade released at once

// ---------------- PCA9685: servos on 0-5, static outputs on 6-15 -----------------------------------
// Channels 6-15 are logic levels (the chip's full-ON / full-OFF bits), 3.3 V totem-pole, 10 mA source /
// 25 mA sink: a laser's TTL input or a MOSFET's gate - never a load. The arm has five MOSFETs (Toby, 2026-10-06
// final plan): the IRF4905 master switch, the green laser's RFP30N06LE (gate on ch 6) and three D4184 modules
// switching the BuckPuck groups (TRIG on ch 9-11). While OE is high every channel reads 0 V.
constexpr uint16_t PCA_FREQ    = 50;          // MG90S
constexpr uint32_t SERVO_OE_OFF_WAIT_MS = 1000 / PCA_FREQ + 10;   // OE goes high only this long after a release: no runt pulse (servos.cpp servosPower)
constexpr uint32_t PCA_OSC_HZ  = 27000000;    // measured ~27 MHz on these boards (pulses were 8 % short at the library's 25)
// Toby, 2026-10-05: THREE forearm plates + the hand plate, six servos. Plate 1 and plate 2 have one servo
// each (the second arm is a helper), plate 3 has TWO (channels 2 + 3), the hand plate has TWO (4 + 5, across
// the wrist). There is no pod-door servo. The constant names keep their old spelling so nothing else moves.
constexpr uint8_t  PCA_CH_PLATE1       = 0;   // forearm plate 1 (one servo)
constexpr uint8_t  PCA_CH_PLATE2       = 1;   // forearm plate 2 (one servo)
constexpr uint8_t  PCA_CH_PLATE3       = 2;   // forearm plate 3, servo A
constexpr uint8_t  PCA_CH_PLATE4       = 3;   // forearm plate 3, servo B (was "4 outside" / "the pod door" until 2026-10-05)
constexpr uint8_t  PCA_CH_PALM_A       = 4;   // hand plate servo A, via the wrist (W19 SRV_A)
constexpr uint8_t  PCA_CH_PALM_B       = 5;   // hand plate servo B (W20 SRV_B)
constexpr uint8_t  PCA_CH_LASER_GREEN  = 6;   // 1 k -> gate of the RFP30N06LE low-side switch in the pod (100 k gate-source pull-down; OE holds the channel low from power-on): the Quarton VLM-520-74 on the servo rail, 50 mA measured (Toby, 2026-10-06)
constexpr uint8_t  PCA_CH_LASER_RED    = 7;   // the #1056's TTL input, straight (00 #23: confirm the 3.3 V level on the bench)
constexpr uint8_t  PCA_CH_VIBRATOR     = 8;   // spare since 2026-10-06 - the vibration motor is on the DRV2605L (haptic.cpp); the name stays for the JSON
constexpr uint8_t  PUCK_COUNT          = 7;   // seven 3023-D-N-1000 BuckPucks, one LED each
constexpr uint8_t  PUCK_GROUPS         = 3;   // switched together, one D4184 module per group on the group's Vin- (low side)
constexpr uint8_t  PCA_CH_PUCK_GROUP[PUCK_GROUPS] = { 9, 10, 11 };   // D4184 #1 / #2 / #3 TRIG; 12-15 spare
struct PcaChan { const char* name; uint8_t ch; bool servo; };
constexpr PcaChan PCA_CHANNELS[] = {
  { "plate 1",             PCA_CH_PLATE1,      true  },
  { "plate 2",             PCA_CH_PLATE2,      true  },
  { "plate 3 servo A",     PCA_CH_PLATE3,      true  },
  { "plate 3 servo B",     PCA_CH_PLATE4,      true  },
  { "hand plate servo A",  PCA_CH_PALM_A,      true  },
  { "hand plate servo B",  PCA_CH_PALM_B,      true  },
  { "green laser",         PCA_CH_LASER_GREEN, false },
  { "red laser",           PCA_CH_LASER_RED,   false },
  { "spare (was vibrator)",PCA_CH_VIBRATOR,    false },
  { "LED group 1 (D4184 #1)",  9,              false },   // the three 20 mm palm LEDs (XM-L2, pucks 1-3)
  { "LED group 2 (D4184 #2)",  10,             false },   // the two square palm LEDs (10 mm, pucks 6-7)
  { "LED group 3 (D4184 #3)",  11,             false },   // the two finger LEDs (index + middle, pucks 4-5)
  { "spare",               12,                 false },
  { "spare",               13,                 false },
  { "spare",               14,                 false },
  { "spare",               15,                 false },
};
constexpr uint8_t PCA_CHANNEL_COUNT = sizeof(PCA_CHANNELS) / sizeof(PCA_CHANNELS[0]);
// OE: wired to GPIO 23 with a 1 k pull-up, so outputs are OFF from power-on until servosBegin() has
// parked every channel. true would drive OE low permanently (the old bench state) - never in the build.
constexpr bool     OE_DRIVE_LOW_ALWAYS = false;

// ---------------- battery (design/01-power.md section 10) ------------------------------------------
// 100 k top, 22 k bottom, from the switched RAW bus to GPIO 35 - the 22 k is what Toby built (2026-10-13; the design
// had 33 k): 8.7 V full -> 1.57 V at the pin, 6.6 V -> 1.19 V. DIV_R_BOTTOM must be what is built (the bench board
// had 32 k = 10 k + 22 k): a wrong ratio reads a full pack as 6.3 V - under CRITICAL. Two-point calibration, 2026-09-07.
constexpr float    DIV_R_TOP    = 100000.0f;
constexpr float    DIV_R_BOTTOM = 22000.0f;   // R3 (Toby, 2026-10-13: 22 k, not 33 k)
constexpr float    DIV_RATIO_NOMINAL = (DIV_R_TOP + DIV_R_BOTTOM) / DIV_R_BOTTOM;
// The two-point calibration below was measured on 2026-09-07 with the bench board's 100 k / 32 k divider. The
// rebuilt 100 k / 22 k divider (in line under heat-shrink) needs it measured again (05 step 1): until
// then it corrects the ADC the same way and the resistor tolerance is uncorrected (about +-1 %).
constexpr float    BATT_RAW1 = 7.770f, BATT_REAL1 = 7.630f;
constexpr float    BATT_RAW2 = 7.129f, BATT_REAL2 = 7.027f;
constexpr float    BATT_A = (BATT_REAL1 - BATT_REAL2) / (BATT_RAW1 - BATT_RAW2);
constexpr float    BATT_B = BATT_REAL1 - BATT_A * BATT_RAW1;
// The voltage rules. Each threshold has hysteresis (clears BATT_HYST_V above) and persistence
// (BATT_PERSIST_MS below it) so a servo burst cannot flap a state.
// The battery makes NO sound until the shutdown (Toby, 2026-10-08: "no 5 s continuous beep anywhere; LOW: a warning on
// the puck and the OLED only, no sound"): from the arm floor down to IDLE the $S frame carries the state and its flags
// bit 0, and the puck and the OLED show it - no buzzer, no ring blink. The only battery sound is the shutdown's chirp.
constexpr float    BATT_ARM_FLOOR_V  = 7.5f;   // below: arming refused (silent; the puck and the OLED say it) (7.2 until 2026-10-08: the ELRS Nano TX V2 is a 7-13 V part (BetaFPV) and the switch, fuse and wires drop up to ~0.3 V under load - 7.2 left it no margin)
constexpr float    BATT_LAND_WARN_V  = 7.3f;   // (2026-10-08) armed and under this for BATT_PERSIST_MS: LAND NOW - the vibrator every 5 s (no sound since Toby's no-sound rule), a flight-log line. The module's 7.0 V minimum is 0.3 V away
constexpr float    BATT_LOW_V        = 7.0f;   // LOW: the puck and the OLED warn, a chirp every BATT_LOW_CHIRP_EVERY_MS (2026-10-20), the high-power LED show refused (BuckPuck headroom)
constexpr uint32_t BATT_LOW_CHIRP_EVERY_MS = 5000;   // (2026-10-20, Toby: "a 7 volts low battery warning, I need a beep every like 5 seconds") LOW and IDLE: one short chirp this often, until the shutdown (whose own chirp then carries on); the 2026-10-08 rule was silence until the shutdown
constexpr float    BATT_IDLE_V       = 6.8f;   // forced idle: plates park, lasers off, drone disarmed (no sound)
constexpr float    BATT_CRITICAL_V   = 6.6f;   // shutdown: a silent grace, then everything off and the ESP32 in deep sleep, chirping (shutdown.cpp)
constexpr uint32_t SHUTDOWN_GRACE_MS        = 5000;   // before anything goes off, SILENT (it was a 5 s buzzer alarm until 2026-10-08): the loads are already off (goSafe), so a sag that recovers cancels the shutdown here; the ring blinks red fast meanwhile
constexpr uint32_t SHUTDOWN_TELL_MS         = 1500;   // $S frames with the shutdown flag (bit 5) before the deep sleep - the puck blanks its screen
constexpr uint32_t SHUTDOWN_CHIRP_EVERY_MS  = 5000;   // asleep after the shutdown, the ESP32 wakes this often, chirps once (SHUTDOWN_CHIRP_MS) and sleeps again - until the master switch is off (the regulators, the ELRS module and the puck still drain the pack: 01 section 15). 30 s until Toby's 2026-10-08 change
constexpr uint32_t SHUTDOWN_CHIRP_MS        = 120;
constexpr uint16_t SHUTDOWN_CHIRP_HZ        = 2200;
constexpr uint32_t BATT_FAULT_BUZZ_EVERY_MS = 30000;  // (2026-10-08) the battery reading under BATT_FAULT_LOW_V / over BATT_FAULT_HIGH_V for BATT_PERSIST_MS LATCHES a FAULT until power-off: no new show, no arming, this distinct buzz
constexpr uint32_t BATT_LAND_WARN_EVERY_MS  = 5000;   // the LAND NOW warning (BATT_LAND_WARN_V) repeats this often while armed
constexpr uint32_t WDT_TIMEOUT_MS           = 3000;   // (2026-10-08) the task watchdog on loop() (hal.h): a stalled loop resets the ESP32 - OE floats high, every PCA output goes LOW
constexpr uint8_t  PCA_MODE2_VALUE          = 0x04;   // MODE2: OUTDRV 1 (totem pole), OUTNE 00 = every output LOW while OE is high (NXP PCA9685 datasheet, MODE2) - written at every boot and read back
static_assert((PCA_MODE2_VALUE & 0x03) == 0 && (PCA_MODE2_VALUE & 0x04), "config.h PCA_MODE2_VALUE: OUTNE (bits 1:0) must be 00 = outputs LOW while OE is high, OUTDRV (bit 2) 1 = totem pole");
constexpr float    BATT_HYST_V       = 0.15f;
constexpr uint32_t BATT_PERSIST_MS   = 2000;
constexpr float    BATT_FAULT_LOW_V  = 4.6f;   // under this the MONITOR is broken (bench supply, snapped leg) - not a low pack
constexpr float    BATT_FAULT_HIGH_V = 9.5f;   // the Ovonic Rebel pack is LiHV: 4.35 V per cell = 8.7 V full (Toby, 2026-10-06); 9.0 left the ADC no margin

// ---------------- LEDs ------------------------------------------------------------------------------
// One brightness cap for both WS2812 chains, 30 % of full: 119 pixels white at 100 % would be 7 A on the
// 5 A servo rail; at the cap 2.1 A. Nothing may set a chain above it.
constexpr uint8_t  LED_BRIGHTNESS_CAP = 77;
constexpr uint16_t LED_RAMP_MS        = 200;   // every "on" ramps up over this (inrush on the shared rail)
constexpr uint16_t PALM_OUTER = 12;            // the 37 mm DIYmall 12-LED outer ring: pixels 0-11 - data, 5 V and ground come into it first (design 04 section 9) ...
constexpr uint16_t PALM_INNER = 7;             // ... and the 23 mm 7-LED inner disc chained after it: pixels 12-18 (three short wires from the ring)
constexpr uint16_t FOREARM_COUNT = 100;        // SEZO WS2812B ECO, 1 m, 100/m (the listing says 60; the strip on the bench is 100 - Toby), pixel 0 at the elbow, driven straight from the pin through 330 R
constexpr uint8_t  FOREARM_BRIGHTNESS = LED_BRIGHTNESS_CAP;
// The strip's look whenever the arm is on (Toby, 2026-10-07, 00 #32): a few LEDs, dim, red. Change these freely:
constexpr uint8_t  FOREARM_IDLE_FIRST   = 0;          // the first lit pixel (0 = the elbow end)
constexpr uint8_t  FOREARM_IDLE_COUNT   = 3;          // how many are lit
constexpr uint8_t  FOREARM_IDLE_SPACING = 1;          // 1 = side by side, 2 = every other pixel ...
constexpr uint8_t  FOREARM_IDLE_LEVEL   = 40;         // 0-255 of the colour, then the 30 % cap: 40 = about 5 % of full drive
constexpr uint32_t FOREARM_IDLE_RGB     = 0xFF0000;   // red
static_assert(FOREARM_IDLE_FIRST + (FOREARM_IDLE_COUNT ? (FOREARM_IDLE_COUNT - 1) * FOREARM_IDLE_SPACING : 0) < FOREARM_COUNT, "config.h FOREARM_IDLE_*: past the strip's end");

// ---------------- servos: the armor cascade (Gauntlet/servos.cpp, proven 2026-09-07) ---------------
constexpr uint8_t  SERVO_COUNT = 6;
constexpr uint16_t SERVO_US_MIN = 650;         // hard envelope: measured 600 <-> 2400 us = ~180 deg on the bench MG90S, 50 us inside
constexpr uint16_t SERVO_US_MAX = 2400;   // (2350 until 2026-10-16: plate 3 front closes at ~2369; the bench MG90S settles quietly to 2400; the hand plate's right servo closes at 2400)
constexpr uint16_t SERVO_US_CLOSED_DEFAULT = 1300;
constexpr uint16_t SERVO_US_OPEN_DEFAULT   = 1700;
constexpr uint32_t SERVO_CAL_HOLD_MS       = 15000;   // the /cal page holds one servo this long after its last move, then releases it (a stalled MG90S must not cook)
// mirror = the servo of a PAIR that faces the other one: it turns the opposite way (open = 1500 - half travel). With
// usClosed / usOpen set (calibrated on the real plate at bring-up, 05 step 5) mirror is ignored: the numbers say it.
struct PlateDef { const char* name; uint8_t ch; float travelDeg; uint16_t moveMs; uint16_t usClosed; uint16_t usOpen; bool mirror; };
constexpr PlateDef PLATES[] = {                // usClosed/usOpen 0 = derive from travelDeg centred on 1500 us
  // The two pairs (Toby, 2026-10-07, 00 #28): plate 3's servos A + B and the hand plate's A + B move in ONE step -
  // the same start, the same travel, the same duration - and servo B, the mirrored one, turns the opposite way.
  // Each servo's closed / open pulse is calibrated on the real plate at bring-up (05 step 5); the travel and
  // timing below are the 2026-09-07 defaults until then.
  { "plate 1",   PCA_CH_PLATE1, 100, 150, 1769, 750, false },   // calibrated 2026-10-16: opens with the pulse going DOWN; quick (momentum shuts it)
  { "plate 2",   PCA_CH_PLATE2,  80, 300, 1400, 2180, false },   // calibrated on the arm 2026-10-16 (the /cal page; NVS wins)
  { "plate 3A",  PCA_CH_PLATE3,  80, 500, 2350, 1189, false },   // calibrated 2026-10-16: OPENS with the pulse going DOWN; 0.5 s each way (proved on the arm)
  { "plate 3B",  PCA_CH_PLATE4,  80, 500, 2255,  716, true  },  // follows 3A along PLATE3_PATH (opens DOWN too; the open end here only sets the direction)
  { "hand A",    PCA_CH_PALM_A,  90, 350, 1400, 1866, false }, // left (output 0): tuned on the arm with the plate fitted (2026-10-20): closed 1400 (higher than its stop, so the right can bring the front down), open 1866 (~70 %); 0.35 s at constant speed (PLATE_LINEAR_DEFAULT). Its servo range is 760-2196
  { "hand B",    PCA_CH_PALM_B,  90, 350, 2400, 1472, true  }, // right (output 1): closed 2400 (all the way down - it pulls the front down), open 1472 (where the pair stops fighting: 0.1-0.6 A holding, no twist on release) - mirrored. Range 2400-955
};
constexpr uint8_t  PLATE_COUNT         = sizeof(PLATES) / sizeof(PLATES[0]);
// The physical plate each servo above belongs to (1 = plate 1, 2 = plate 2, 3 = plate 3, 4 = the hand plate):
// servosMovePlate(n) moves every servo of plate n together, alone (plate 1 can open on its own - 2026-10-07).
constexpr uint8_t  PLATE_GROUP_OF[]    = { 1, 2, 3, 3, 4, 4 };
constexpr uint8_t  PLATE_GROUPS        = 4;
// the thermal camera (thermal.cpp): the plate whose lift deploys it - the hand plate (Toby, 2026-10-07 night: "it lifts
// the hand plate") - and how long its lift may take before the camera gives up
constexpr uint8_t  THERMAL_PLATE           = 4;
constexpr uint32_t THERMAL_LIFT_TIMEOUT_MS = 3000;
static_assert(THERMAL_PLATE >= 1 && THERMAL_PLATE <= PLATE_GROUPS, "THERMAL_PLATE must be a plate group 1..PLATE_GROUPS");
// The plates' mechanical order (Toby, 2026-10-07 night - enforced on every path: the cascade, one plate, the lasers, the
// thermal camera, mode changes). Plate 1 (the side plate, it carries the green laser) opens on its own; plate 2 needs the
// hand plate open; plate 3 needs the hand plate AND plate 2. Open in that order, close in reverse; a plate closes only
// when nothing still needs it open - a plate that depends on it, the thermal camera, a laser (servos.cpp's holds). A
// plate starts only once its prerequisites have LANDED (the old 40 ms overlap is gone). Bits = plate numbers (1..4).
constexpr uint8_t  PLATE_NEEDS[PLATE_GROUPS + 1] = { 0, 0, (1 << 4), (1 << 2) | (1 << 4), 0 };   // [plate] = the plates it needs open - Toby, 2026-10-20 (the hand plate fitted, it sits over plate 2): plate 2 needs the hand plate, plate 3 the hand plate and plate 2, plate 1 nothing
constexpr float    PLATE_CLEAR_TRAVEL[PLATE_GROUPS + 1] = { 1, 1, 1, 1, 1 };   // a plate needing this one may start once it is this far through its TRAVEL (1 = landed). The hand plate fully (Toby, 2026-10-31: "the hand plate has to open and then the plate 2 ... closing, plate 2 has to close and then the hand plate" - at 30 % (2026-10-16) the two overlapped and caught each other); closing mirrors it - the hand plate starts down only once what needs it is home
constexpr bool     PLATE_LINEAR_DEFAULT[PLATE_GROUPS + 1] = { false, false, false, false, true };   // (2026-10-20) a plate moving at constant speed (momentum) unless the /cal page set it otherwise (NVS "pl1".."pl4"): the hand plate
constexpr uint8_t  PLATE_WITH[PLATE_GROUPS + 1] = { 0, 2, 0, 0, 0 };   // in one plan plate 1 starts with plate 2 (Toby's forearm order, kept: 1 + 2 together, 3 the instant 2 lands; closing 3, then 2 + 1, then the hand plate)
constexpr uint8_t  PLATE_OPEN_ORDER[PLATE_GROUPS] = { 1, 4, 2, 3 };    // every plate after the ones it needs; closing runs it backwards
constexpr bool plateOrderOk() {
  for (uint8_t k = 0; k < PLATE_GROUPS; k++) {
    uint8_t g = PLATE_OPEN_ORDER[k];
    for (uint8_t p = 1; p <= PLATE_GROUPS; p++) if (PLATE_NEEDS[g] & (1 << p)) { bool before = false; for (uint8_t j = 0; j < k; j++) if (PLATE_OPEN_ORDER[j] == p) before = true; if (!before) return false; }
  }
  return true;
}
static_assert(plateOrderOk(), "config.h PLATE_OPEN_ORDER: a plate comes before one it needs");
constexpr uint8_t  LASER_RED_PLATE   = 4;      // the red laser and the LiDAR fire through the hand plate: it lifts first (Toby, 2026-10-07 night)
constexpr uint8_t  LASER_GREEN_PLATE = 1;      // the green laser rides plate 1 (the side plate): it opens first
constexpr uint32_t LASER_PLATE_WAIT_MS = 2500; // a laser whose plate has not landed open by then gives up (a stall, a move that would jam)
constexpr uint32_t LASER_PLATE_SETTLE_MS = 400; // and fires only this long after its plate's move has ended: the plate gets there after the command (Toby, 2026-10-16: "it can't be moving and on"); live: /outputs?settle=MS, NVS "lset"
constexpr bool     SERVO_RAW_TOOLS   = false;  // bench keys 4 (all six swept): they ignore the plates' order - true only with the plates OFF the servos
// Plate 3's two servos move along ONE path (Toby, 2026-10-16, watching it on the arm): the back servo (3B, PCA output 15)
// opens twice as fast as the front one (3A, output 14) until it is ~45 deg open, then 1:1, then it stops at its stop a
// little past 90 deg while 3A goes on to its own stop; closing is the same path backwards. Both turn the SAME way (not
// mirrored, whatever PLATES[] says about 3B). So 3B is never commanded on its own: servoWriteUs() drives it from 3A's
// pulse - 3B's degrees from ITS closed = PLATE3_PATH(3A's degrees from ITS closed), piecewise linear between the knots.
constexpr bool     PLATE3_FOLLOW     = true;
constexpr float    SERVO_CAL_STALL_A  = 1.5f;  // a /cal hold (one servo, or plate 3's pair) drawing more than this ...
constexpr uint32_t SERVO_CAL_STALL_MS = 400;   // ... this long is let go on the board
constexpr uint8_t  PLATE3_LEADER = 2, PLATE3_FOLLOWER = 3;    // PLATES[] indices: 3A leads, 3B follows
constexpr float    PLATE3_PATH[][2]  = { { 0, 0 }, { 22.5f, 45 }, { 123, 155 }, { 180, 155 } };   // { 3A deg, 3B deg } while OPENING - as tuned on the arm 2026-10-16 (live: the /cal page path=)
// Closing is not opening backwards (Toby): from where it starts, 3A's first PLATE3_CLOSE_LEN deg at PLATE3_CLOSE_RATIO:1
// against 3B, then both straight to closed together. All of it live on the /cal page (path=a1,b1,a2,b2,cl,cr; NVS p3p).
constexpr float    PLATE3_CLOSE_LEN   = 45.0f;
constexpr float    PLATE3_CLOSE_RATIO = 2.0f;
static_assert(sizeof(PLATE_GROUP_OF) == PLATE_COUNT, "config.h PLATE_GROUP_OF[]: one entry per servo in PLATES[]");
constexpr bool pairsMatch() {                  // the two servos of a plate: same travel, same duration, exactly one mirrored
  for (uint8_t i = 0; i < PLATE_COUNT; i++) for (uint8_t j = i + 1; j < PLATE_COUNT; j++)
    if (PLATE_GROUP_OF[i] == PLATE_GROUP_OF[j] && (PLATES[i].travelDeg != PLATES[j].travelDeg || PLATES[i].moveMs != PLATES[j].moveMs || PLATES[i].mirror == PLATES[j].mirror)) return false;
  return true;
}
static_assert(pairsMatch(), "config.h PLATES[]: the two servos of a plate need the same travel and duration, and one of them mirrored");
constexpr float    SERVO_US_PER_DEG    = (2400 - SERVO_US_MIN) / 180.0f;   // the bench MG90S: 650..2400 us = 180 deg - fixed, not the envelope (2026-10-20: the envelope's top moved, plate 3's path in degrees must not)
constexpr uint16_t CASCADE_HOLD_MS     = 600;
constexpr uint16_t CASCADE_PAUSE_MS    = 1500;
constexpr uint32_t CASCADE_STALL_GRACE_MS = 1500;   // a cascade still "busy" this long after its planned end is released unconditionally (bounded stall time)

// ---------------- BuckPuck bank (design/01-power.md sections 6-7, 14) --------------------------------
// Seven 3023-D-N-1000 (1 A fixed, no CTRL wire) in THREE groups (Toby, 2026-10-06 final plan): one D4184 MOSFET module
// per group cuts the group's Vin- (low side), its TRIG on PCA_CH_PUCK_GROUP[g]. ON / OFF ONLY - never PWM a group:
// the only dimming in the arm is the palm ring's and the strip's brightness, in firmware. Bursts only.
// (2026-10-20, Toby fitted 3 mm aluminium behind every LED: "update the math ... I need to run it longer") the heat model
// below governs a burst now; these are backstops only: about the model's longest burst from cold, then a short rest.
constexpr uint32_t PUCK_ON_MAX_MS_PALM   = 90000;   // groups 1 and 2 (the model ends them first: ~63 s / ~27 s from cold) ...
constexpr uint32_t PUCK_OFF_MIN_MS_PALM  = 10000;   // ... then this long off (15 s / 45 s until 2026-10-20)
constexpr uint32_t PUCK_ON_MAX_MS_FINGER = 200000;  // group 3 (~190 s from cold on its 13.5 mm of aluminium; 10 s until 2026-10-20) ...
constexpr uint32_t PUCK_OFF_MIN_MS_FINGER = 10000;  // (30 s until 2026-10-20)
constexpr float    PUCK_DUTY_MAX         = 0.75f;   // cumulative on-time over PUCK_DUTY_WINDOW_MS (25 % until 2026-10-20)
constexpr uint32_t PUCK_DUTY_WINDOW_MS   = 120000;
constexpr float    PUCK_SHOW_FLOOR_V     = BATT_LOW_V;   // the show is refused below this (BuckPuck headroom: Vin >= Vf + 2.5 V) - and a running burst cut
constexpr float    LIGHT_FULL_FLOOR_V    = 7.5f;   // (2026-10-21, Toby: one bar, "anything powerful" - the drone, all the lights at once - stops under
                                                   // it; single groups still start) a group starts only above PUCK_SHOW_FLOOR_V + (this - it) x the
                                                   // LEDs that would then be lit / PUCK_COUNT: 2 LEDs 7.14 V, 3 7.21 V, 5 7.36 V, all 7 7.5 V (their sag
                                                   // grows with the load; one measurement: 7.45 V resting sagged to 6.9 V with 5 on). = the puck's mark
constexpr uint32_t PUCK_DUTY_SLACK_MS    = 500;     // a burst overruns its limit by a poll period; three exact 10 s bursts are still 25 %
constexpr uint8_t  PUCK_GROUP_OF[PUCK_COUNT] = { 0, 0, 0, 2, 2, 1, 1 };   // puck 1-3 XM-L2 -> group 1, 4-5 fingers -> group 3, 6-7 palm 10 mm -> group 2
constexpr bool     PUCK_GROUP_IS_FINGER[PUCK_GROUPS] = { false, false, true };
constexpr const char* PUCK_GROUP_NAME[PUCK_GROUPS] = { "palm 20 mm x3", "palm 10 mm x2", "fingers x2" };
// The palm plate capped near 50 C (Toby, 2026-10-07: the shells are PETG; the audit: a bare plate a stranger holds for
// ten seconds). No sensor: a first-order model per body, dT/dt = (P - (T - ambient) / R) / C, started at ambient at
// power-on. A group refuses to start above PLATE_START_MAX_C and a running burst ends at PLATE_CAP_C. The numbers are
// conservative estimates until the temperature log at bring-up (05 step 12) measures the real plate and tabs.
// (2026-10-20) one body per LED group, from the aluminium Toby fitted (3 mm plate, 2.43 J per cm3 per K); his concern is
// the PETG round the metal softening and an LED working loose, not his skin ("I could just lift my hand"), so the cap went
// 50 -> 55 C and the start limit 44 -> 48 C. From cold a burst lasts about: palm 20 mm 63 s, palm 10 mm 27 s, fingers 190 s.
constexpr float    PLATE_AMBIENT_C       = 30.0f;    // inside a glove, on a warm stage
constexpr float    PLATE_CAP_C           = 55.0f;    // a burst ends here (50 until 2026-10-20)
constexpr float    PLATE_START_MAX_C     = 48.0f;    // no burst starts above this (44 until 2026-10-20)
constexpr float    PALM20_J_PER_K        = 14.9f;    // group 1: three 23 x 23 x 3 mm aluminium plates (3.86 J/K each) + the three 20 mm stars (~1.1 each)
constexpr float    PALM20_K_PER_W        = 30.0f;    // to the air through the PETG - an ESTIMATE until a measured burst
constexpr float    PALM10_J_PER_K        = 4.4f;     // group 2: two 25 x 10 x 3 mm plates (1.82 J/K each) + the two 10 mm boards - the least metal
constexpr float    PALM10_K_PER_W        = 60.0f;
constexpr float    FINGER_J_PER_K        = 13.5f;    // group 3, EACH finger: 4.5 layers of 3 mm (13.5 mm) aluminium, taken as 20 x 20 mm (the old tab's
                                                     // footprint - Toby did not know the size) + its 10 mm board; it was 1.7 J/K (one 1.5 mm tab)
constexpr float    FINGER_K_PER_W        = 40.0f;
constexpr float    LED_HEAT_W            = 2.1f;     // per LED at 1 A: about 3 W in, 70 % of it heat
// (2026-10-08, the audits) the LED drivers' own loss heats a THIRD body, the driver bank in the forearm: 4.5 W in, about
// 3 W into the LED, so about 1.5 W per driver (the docs had said 0.4 W) - 10.5 W with all seven on. The bank's mass and
// its path to the air are ESTIMATES until 05 step 7 measures them; it is held to the same 44 / 50 C as the plates.
constexpr float    DRIVER_HEAT_W         = 1.5f;
constexpr float    BANK_J_PER_K          = 35.0f;    // the seven BuckPucks together (~5 g each, potting ~1 J/(g K)) - an ESTIMATE
constexpr float    BANK_K_PER_W          = 10.0f;    // to the air inside the forearm shell - an ESTIMATE
constexpr uint8_t  PUCK_GROUP_DRIVERS[PUCK_GROUPS] = { 3, 2, 2 };   // drivers per group: groups 1 / 2 / 3
// (2026-10-20) the model is kept in RTC memory across a reset (an OTA push, a brownout, the watchdog, the shutdown's deep
// sleep: a reboot is not a cool-down); a real power-up (the master switch) starts it at PLATE_AMBIENT_C. The 2026-10-08
// rule - every boot at the cap - would now mean ~3 minutes of no light after each power-up with this much aluminium.
constexpr uint8_t  PUCK_GROUP_LEDS[PUCK_GROUPS] = { 3, 2, 1 };   // LEDs heating each group's body; a finger's body takes its one LED (the two fingers alike)

// ---------------- lasers, vibrator (outputs.cpp) -------------------------------------------------------
constexpr uint32_t LASER_GREEN_MAX_ON_MS = 300000;  // the pod laser's forgot-it-on guard, like the red one's LASER_MAX_ON_MS
constexpr uint32_t LASER_AFTER_FLIGHT_MS = 1500;    // (Toby, 2026-10-07 night) both lasers stay blocked while the drone flies (armed or flipping) and this long after: a wrist jerk during the finger-gun disarm must never fire the green laser. Every path (gesture, key, route); a gesture seen meanwhile is reported $E BLOCKED
constexpr uint32_t VIB_PULSE_MAX_MS      = 2000;    // the longest single buzz
constexpr uint32_t VIB_ARM_MS            = 120;     // haptic ticks: arm, disarm, LAND NOW (x2)
constexpr uint32_t VIB_DISARM_MS         = 300;
constexpr bool     LASER_ACTIVE_HIGH     = true;    // (the HandBench name: the red laser's module switches on with a HIGH input - true for a PCA channel too)

// ---------------- mode switch (polled, debounced) ------------------------------------------------------
// The mode switch (Toby, 2026-10-08, replacing the 2026-10-07 night order): the MIDDLE = SAFE (both pins high - open),
// the GPIO 36 end = SHOW (pole A low), the GPIO 39 end = FULL (pole B low); both low is impossible on an ON-OFF-ON and
// reads SAFE. So a broken wire, a lost common or a bad contact reads both high = SAFE, never a live mode. Going from
// SHOW to FULL (or back) passes the middle, i.e. SAFE, for as long as the lever is there. The code keeps the
// name Flight for FULL; everything on a screen or in the log says FULL.
//   SAFE  for handing it round: the idle lights only (accent LEDs, the few dim strip pixels, a dim palm glow); every
//         gesture is still detected and reported to the puck ($E BLOCKED), nothing acts; nothing moves or emits
//   SHOW  plates, palm burst, finger LEDs, ring changes, vibrator, the thermal camera (the lasers, the LiDAR and
//         the drone are FULL's)
//   FULL  everything: SHOW plus the lasers, the LiDAR and drone arming - anything that leaves the arm lives here only;
//         all of it used at the summit (Toby, 2026-10-08: the organisers approved the drone and the lasers)
enum class Mode : uint8_t { Safe, Show, Flight };
constexpr uint32_t MODE_DEBOUNCE_MS  = 30;
constexpr uint32_t MODE_POLL_MS      = 5;
constexpr bool     MODE_POWERUP_NEEDS_SAFE = true;   // power-up stays SAFE until the switch has been seen in the middle (SAFE) once: switching on in SHOW or FULL never goes live

// ---------------- buzzer ----------------------------------------------------------------------------------
constexpr uint8_t  BUZZER_LEDC_CH    = 6;           // an LEDC channel the NeoPixel RMT and the servo PCA do not use
constexpr bool     BUZZER_ACTIVE_LOW = true;        // the MusRock module's PNP sounds on LOW: the pin idles HIGH, the LEDC output is inverted (hal)

// ---------------- haptic: the DRV2605L drives the #1201 coin motor over I2C (haptic.cpp) --------------------
constexpr uint8_t  DRV2605_ADDR      = 0x5A;        // Adafruit #2305 on bus 2, VIN on the palm's 3V3, the motor on its OUT pins
constexpr uint8_t  HAPTIC_RTP_LEVEL  = 0x60;        // open-loop ERM amplitude for a tick: ~75 % of VDD = ~2.5 V on the 2.5-3.8 V #1201
constexpr uint16_t BUZZ_HZ_LOW       = 880;
constexpr uint16_t BUZZ_HZ_HIGH      = 1760;

// ---------------- the round display link (s3link.cpp, lib/GauntletLink) --------------------------------
// Protocol v2 (2026-10-07): $S 10 Hz, $G 20 Hz out, $H / $T / $L in. Never blocking: frames go into a TX ring of
// S3_TX_BUF bytes and are DROPPED (counted) when it has no room - a UART has no handshake, so an unplugged or dead
// puck changes nothing on the TX side; at most S3_RX_MAX_PER_POLL bytes are parsed per loop() however much arrives
// (a babbling puck costs a bounded few hundred microseconds); the RX pin keeps the core's internal pull-up (on by
// default in core 3.3.x), so a floating wire idles high instead of reading noise.
constexpr uint32_t S3_BAUD           = 115200;
static_assert(S3_BAUD == 115200, "config.h S3_BAUD: the puck runs GauntletLink BAUD (115200) - change both or neither");
constexpr uint32_t S3_STATUS_MS      = 100;         // $S ten times a second
constexpr uint32_t S3_LIVE_MS        = 50;          // $G twenty times a second
constexpr uint32_t S3_HEARTBEAT_LOST_MS = 3000;     // no heartbeat from the display for this long = "display link down" on the console / HUD
constexpr size_t   S3_TX_BUF         = 1024;        // the TX ring (89 ms of line); the link averages 50 % of the line
constexpr size_t   S3_RX_BUF         = 512;
constexpr uint16_t S3_RX_MAX_PER_POLL = 256;        // bytes parsed per poll at most (the puck sends ~60 a second)
constexpr uint32_t IMU_LINK_LIVE_MS  = 500;         // a sensor quiet for this long loses its "live" bit on the link
#if HW_FAKE
// The fake build's scripted arm (demo.cpp) that keeps the link busy after the self-test - for the puck's desk test.
constexpr bool     DEMO_AT_BOOT       = true;
constexpr uint32_t DEMO_CYCLE_MS      = 50000;      // one pass of the script (the palm LED groups rest 45 s between bursts)
constexpr uint32_t DEMO_DRAIN_MS      = 360000;     // the pack from full to nearly LOW in six minutes, then a fresh one
constexpr float    DEMO_PACK_FULL_V   = 8.60f;
constexpr float    DEMO_PACK_EMPTY_V  = 6.95f;      // under LOW (7.0), above IDLE (6.8, which would park everything)
constexpr uint16_t DEMO_FLAT_MV = 1200, DEMO_CLOSED_MV = 600;               // index..pinky
constexpr uint16_t DEMO_THUMB_FLAT_MV = 800, DEMO_THUMB_CLOSED_MV = 600;    // the thumb, closed as far as a fist allows
#endif

// ---------------- flex: the thumb on the ESP32's ADC, four fingers on the ADS1115 ---------------------
// 3V3 -> sensor -> input, input -> FLEX_DIVIDER_OHM -> sensor ground: the voltage FALLS as the finger bends. The thumb
// keeps the proven raw 10 ms samples on GPIO 34 (the flick detector reads them); index..pinky are
// ADS1115 AIN0..3, single-shot at 860 SPS round-robin (one conversion every ~1.2 ms), FSR 4.096 V.
constexpr const char* FLEX_NAMES[5]  = { "thumb", "index", "middle", "ring", "pinky" };
constexpr uint16_t    FLEX_SAMPLE_MS = 10;          // the thumb at 100 Hz; each finger is fresh every ~6 ms
// The divider resistor (R14-R18): 47 k since 2026-10-07 (the audit, Toby: the Adafruit #182s read 29-55 k on the rig, so
// 10 k threw away half the swing - the middle finger 126 mV with 10 k, about 220 mV with 47 k). The sensor's resistance is
// R = FLEX_DIVIDER_OHM x (3300 / mV - 1). Every mV calibration (the page's, or keys K / J) must be captured again on 47 k.
constexpr float       FLEX_DIVIDER_OHM = 47000.0f;
constexpr float       FLEX_ALPHA     = 0.25f;
constexpr uint16_t    ADS_FSR_MV     = 4096;        // PGA +-4.096 V: 0.125 mV per LSB
constexpr uint16_t    ADS_CONV_MS    = 1;           // a conversion at 860 SPS is 1.16 ms (1.29 at -10 %): poll its ready bit from 1 ms, the next one starts at once (2 until 2026-10-08: ~70 Hz a finger)
constexpr float       FLEX_SMOOTH_TAU_MS = 34.8f;   // (2026-10-08) the smoothing as a TIME constant = HandBench's 0.25 per 10 ms sample (-10 / ln 0.75), whatever the rate
// (2026-10-08, the audits) a flex reading that cannot be a working sensor is a FAULT: the finger then reads OPEN for every
// gesture (never bent), the thumb's clicks are ignored, the throttle uses the other fingers; logged and sent as curl -1
constexpr uint16_t    FLEX_FAULT_LOW_MV   = 300;    // a #182 on 47 k reads >= ~600 mV even past a fist (125 kOhm): under this = open sensor / broken wire (~0 V)
constexpr uint16_t    FLEX_FAULT_HIGH_MV  = 3100;   // over this = a shorted sensor (3.3 V)
constexpr uint16_t    FLEX_FAULT_FLAT_MARGIN_MV   = 150;   // or above the calibrated flat by more than this (or half the swing)
constexpr uint16_t    FLEX_FAULT_CLOSED_MARGIN_MV = 250;   // or below the calibrated closed by more than this (or the whole swing - a hard squeeze reads past the fist)
constexpr uint32_t    FLEX_STALE_MS       = 60;     // no new sample this long (the ADS1115 silent) = faulted too ...
constexpr uint16_t    FLEX_STALE_POLLS    = 12;     // ... AND none in this many flexPoll() calls in a row (a finger comes every 4 conversions): a stall of the loop itself is not a silent sensor
constexpr uint32_t    FLEX_FAULT_CLEAR_MS = 1000;   // plausible and fresh this long before a faulted finger counts again
constexpr uint32_t    FLEX_FAULT_REPORT_MS = 200;   // ... logged once a fault has lasted this long (no line per glitch)

// ---------------- self-test (HW_FAKE builds) ----------------------------------------------------------------
constexpr bool     SELFTEST_AT_BOOT  = true;        // run the suite in setup() and keep the report for GET /selftest

// =====================================================================================================
// Everything below is the HandBench rig's tuning, kept verbatim (2026-09-21 .. 2026-10-03): the IMU
// zero poses, the thumb gesture, the finger dips, the laser gesture, the LiDAR timing, the CRSF link
// and the flight feel. Do not retune here without the bench.
// =====================================================================================================

// ---------------- BNO085 ------------------------------------------------------
constexpr uint8_t  BNO_ADDRS[]     = { 0x4A, 0x4B };   // both probed: unstrapped the GY-BNO08X answers at its own default (0x4B on the bench pair, Toby 2026-10-07: no straps); AD0 to GND would give 0x4A
constexpr uint16_t IMU_REPORT_MS   = 20;               // game rotation vector cadence from the sensor (50 Hz) - what /imu serves the simulator
constexpr uint16_t IMU_PRINT_MS    = 100;              // console stream cadence (~10 Hz)
constexpr uint16_t IMU_FEED_PORT   = 8080;             // keep-alive JSON feed for the simulator (Bench.liveFeed); /imu on port 80 is the slow copy
constexpr uint8_t  IMU_BEGIN_TRIES = 3;                // a BNO that acks but is still booting gets another go
constexpr uint32_t IMU_STALE_MS    = 2000;             // no report for this long -> warn
constexpr uint32_t IMU_ZERO_AVG_MS = 3000;             // key z averages the pose over this long (an outstretched arm wobbles)
// The sensor tilt at the "zero" pose, [0] = arm, [1] = hand, as the BNO's
// native ZYX roll/pitch in degrees (what raw mode, key u, prints). Key z
// measures and saves the real thing to NVS, which overrides these; key k drops
// the NVS values so these apply. Measured 2026-09-22, Toby's arm held straight
// out, 9 s average of 89 samples (wobble: arm 1.6 roll / 5.8 pitch, hand 4.2 /
// 1.6). The first hand-only figure on 2026-09-21 was 29.3 / 22.5 in another
// pose. Yaw is never stored - it restarts with the sensor; key y re-zeroes it.
constexpr float    IMU_ZERO_DEG[2][2] = { { -8.14f, -12.75f }, { 31.07f, 6.16f } };


constexpr uint16_t PALM_COUNT         = PALM_OUTER + PALM_INNER;   // 19
constexpr uint8_t  PALM_CAP           = LED_BRIGHTNESS_CAP;   // 30 % of 255, the one cap for both chains
constexpr uint8_t  PALM_LEVELS        = 10;      // . and , step the cap in tenths
constexpr uint8_t  PALM_DEFAULT_LEVEL = 10;      // the first switch-on after boot: the cap (30 % of full). Was 5; Toby 2026-09-28: "start at 10/10". After that a switch-on comes back at the level it had before the last off (Toby 2026-09-29)
constexpr uint8_t  PALM_GLOW_COLOUR   = 1;       // SAFE's dim palm glow (an idle light, Toby 2026-10-07 night): "repulsor" in the palette - shown only while the ring's own setting is off
constexpr uint8_t  PALM_GLOW_LEVEL    = 1;       // ... at 1/10 of the cap (3 % of full, ~25 mA)
constexpr uint16_t PALM_RAMP_MS       = 150;     // every change eases over this: no current step for the boost
constexpr float    PALM_MA_PER_CHANNEL = 20.0f;  // per pixel per colour channel at full drive - for the estimate in the log
constexpr float    PALM_MA_IDLE        = 1.0f;   // per pixel while dark (the chips themselves)
// Chain test (key t / the page's "test"): one dim white pixel at a time, in chain
// order, so a break in the chain shows as "the last pixel that lit". It drives
// PALM_WALK_EXTRA pixels past PALM_COUNT too, so a chain longer than config says
// shows up as well.
constexpr uint8_t  PALM_WALK_EXTRA     = 5;
constexpr uint16_t PALM_WALK_MS        = 400;    // per pixel
constexpr uint8_t  PALM_WALK_LEVEL     = 60;     // dim white, ~14 mA for the one pixel


// ---------------- thumb gesture ---------------------------------------------------
// Two fast thumb crunches: hand open = ring off -> white -> red -> white, fist = off.
// A crunch counts only when it is FAST: the thumb's millivolts must fall by
// GESTURE_FLICK_FRAC of its flat-to-fist swing within GESTURE_FLICK_MS (a slow curl,
// however deep, never does), then come back up by GESTURE_RELEASE_FRAC of that drop
// before the next one can count. Two such flicks starting within GESTURE_WINDOW_MS
// fire the action. The swing (thumb flat and thumb closed as far as a fist allows)
// and the closed thresholds of the other fingers come from the sim page's
// calibration (GET :8080/gesture?tf=N&tc=N&f=a,b,c,d) and live in NVS.
// Added 2026-09-26; speed rule and fist thresholds the same evening (Toby: "the
// whole point is that I do it super fast").
// Tuned the same night from Toby's report ("sometimes I do it double and it
// won't change ... it goes up a little, or down and then down again"): the fall
// need only be 15 % of the swing but must happen inside 100 ms (speed, not
// depth, is the test); a flick ends when the thumb comes back up by
// GESTURE_RELEASE_FRAC of its own excursion (at least GESTURE_RELEASE_MIN_MV)
// OR simply stops falling for GESTURE_PLATEAU_MS, so "down, down again" is two
// flicks; consecutive pairs fire back to back (four flicks = two toggles).
// Third tuning, late 2026-09-26 (his hand resting on the desk changed the colour;
// "only deliberate movements ... same amplitude ... they can't build on each
// other"): a flick is a whole crunch-and-release - it starts with a fast fall
// (GESTURE_FLICK_FRAC of the swing inside GESTURE_FLICK_MS), must reach
// GESTURE_AMP_MIN_FRAC of the swing in all, and must come back up by
// GESTURE_RELEASE_FRAC of its own excursion within GESTURE_FLICK_MAX_MS, or it is
// discarded (too small = a wobble; never released = a held or stair-stepped
// crunch, which cannot build into anything). The three flicks must be in
// unison (gaps 150-500 ms and alike) AND alike in size (largest at most
// GESTURE_AMP_RATIO_X10/10 x the smallest).
// Seventh, 2026-09-29 (Toby: "about half can turn it off and about two thirds can turn it on ... make it
// more accurate"): his own triples, measured, have gaps of 230-460 ms in one sitting, so a 500 ms window
// and a 1.6x evenness threw real ones out; the opening fall gets 80 ms (inside a fist the thumb is
// cramped and slower); sizes may differ 3x. The micro filter (a QUICK opening fall) is what keeps the
// desk wobbles out, and that stays.
constexpr uint32_t GESTURE_WINDOW_MS      = 600;   // the most one gap between two flicks of the run may be (was 500)
constexpr uint16_t GESTURE_EVEN_RATIO_X10 = 30;    // the longer gap may be at most 3x the shorter (1.6x until 2026-09-29, 2.2x until 2026-10-07 - his clicks inside a fist ran 181 / 478 ms apart, 2.6x, and failed "not in one rhythm"; "two quick and a third after half a second", 150 / 500 = 3.3x, still fails)
constexpr uint16_t GESTURE_AMP_RATIO_X10  = 30;    // the biggest flick may be at most 3x the smallest (was 2.5x): "a big one and two small ones is not a change", fist crunches vary
constexpr uint32_t GESTURE_COOLDOWN_MS    = 600;   // after a hit, flicks this soon are dropped - a fourth in the same burst starts nothing
constexpr uint32_t GESTURE_CAPTURE_MS     = 2000;  // keys K / J: the on-arm finger calibration averages this long (the page's steps 4 / 5 did the same)
// Fourth tuning (Toby: "too unsensitive now ... I was just talking about filtering
// out the super small variations, the sensor being static, going up and down ...
// when closing it doesn't work anymore"): inside a fist the thumb only moves a
// little, so the size floor sits just above the sensor's own jitter, not at a
// third of the open-hand swing, and a flick need only come back up 40 % of itself.
// The median of five and the unison + alike-size rules do the rest of the
// filtering. `GET :8080/gesture?trace=1` hands back the last 2.5 s of the thumb
// (raw and filtered) so these numbers can be set from what the sensor really does.
// Fifth tuning (Toby: "it was right ... I still need the super small movements
// that are quick"): the sensitive version, with only the noise filtering kept.
// No size floor beyond the opening fall; the opening fall must happen inside
// 80 ms, which small quick crunches do and the sensor's slow wander does not
// (measured: filtered jitter 6-8 mV, wander ~30 mV over seconds, 7 openings in
// 40 s at rest with a 100 ms window); a flick counts
// once the thumb comes back up a little (15 %, at least 8 mV); a fall that
// simply settles into a new level (closing the fist) is discarded.
// Sixth and final tuning (Toby: "it was perfectly fine ... you needed a super
// micro filtering and that was it ... go back to what it was"): the version he
// called perfectly fine, exactly - a flick ends and COUNTS when the thumb comes
// back up a little OR simply stops falling for 80 ms, no size floor, no release
// requirement, no settle rule - plus the one micro filter: the opening fall must
// be QUICK, 20 mV inside 60 ms. A real crunch, however small, is that quick; the
// sensor's wander with a still hand (30 mV over a second) never is.
constexpr float    GESTURE_FLICK_FRAC     = 0.12f; // the fast fall that opens a flick (12 %: a bigger calibrated swing must not price out his smallest crunches) ...
constexpr uint16_t GESTURE_FLICK_MIN_MV   = 20;    // ... but never less than this (the filtered jitter is 6-8 mV)
constexpr uint16_t GESTURE_FLICK_MS       = 80;    // ... and it must happen inside this: the micro filter (60 until 2026-09-29; the wander is ~30 mV a second, so 80 ms still sees under 3 mV of it)
constexpr float    GESTURE_AMP_MIN_FRAC   = 0.0f;  // no size floor beyond the opening fall
constexpr uint16_t GESTURE_AMP_MIN_MV     = 20;    // (= the opening floor)
constexpr float    GESTURE_RELEASE_FRAC   = 0.15f; // a flick ends (and counts) once it comes back up this much of its own excursion ...
constexpr uint16_t GESTURE_RELEASE_MIN_MV = 8;     // ... at least this ...
constexpr uint32_t GESTURE_PLATEAU_MS     = 80;    // ... OR once it has stopped falling for this long ("down, pause, down" is two flicks, as it was)
constexpr uint32_t GESTURE_FLICK_MAX_MS   = 700;   // a fall still making new lows after this long is a slow curl, not a flick - discarded
constexpr uint32_t GESTURE_MIN_GAP_MS     = 150;   // two flicks closer than this are one crunch that stalled and went on (a thumb cannot go down-up-down faster); Toby's real doubles run 230-460 ms
constexpr uint8_t  GESTURE_FLICKS         = 3;     // flicks that make the gesture (the detector keeps the last three and judges them on every new flick). Was 2; Toby 2026-09-26: "I could accidentally do it with two, three is much better"
// The closed hand (middle, ring, pinky) that turns the ring off: a finger counts as closed once it
// has come GESTURE_FIST_FRAC of the way, in millivolts, from its flat reading to its closed one.
// The page sends flat and closed per finger (GET :8080/gesture?fl=a,b,c,d&fx=a,b,c,d, index..pinky).
// Half the millivolts is about a third of the bend (the divider gives most of its swing early), so
// a normal fist is far past it and a relaxed open hand well short of it. Until 2026-09-29 the line
// was the page's half-closed pose, which physically sat deep in the swing - Toby had to "close my
// fingers super hard" and a finger a little short turned "off" into a colour change.
constexpr float    GESTURE_FIST_FRAC      = 0.50f;
// (2026-10-21) the puck's CALIBRATE screen ($C ACT_CAL, calib.cpp): each step waits CAL_SETTLE_MS (the tap on the puck jolts
// the arm), then averages CAL_AVG_MS; steps 4 / 5 (the arm down / up) refuse a pose under IMU_AXIS_MIN_DEG from the zero, and
// use the hanging pose alone when the two disagree by more than IMU_AXIS_AGREE_DEG (HandBench's sim page rules)
// (2026-10-21) the puck's update ($C ACT_PUCK_UPDATE): the RC frames stop, and start again when the puck's heartbeat comes
// back after at least PUCK_UPDATE_QUIET_MIN_MS (heartbeats just after the press are still the old firmware's), or after MAX
constexpr uint32_t PUCK_UPDATE_QUIET_MIN_MS = 8000;
constexpr uint32_t PUCK_UPDATE_QUIET_MAX_MS = 300000;
constexpr uint32_t CAL_SETTLE_MS          = 600;
constexpr uint32_t CAL_AVG_MS             = 2000;
constexpr float    IMU_AXIS_MIN_DEG       = 60.0f;
constexpr float    IMU_AXIS_AGREE_DEG     = 15.0f;
constexpr uint16_t GESTURE_FIST_MIN_MV    = 30;    // a finger whose flat-to-closed swing is under this is unknown (sensor not moving)
// ---------------- finger dips: ring brightness (2026-09-29) ------------------------
// Toby: "my pinky will move down and up, and every time it does that it increases the brightness
// one out of the ten; every time my pointer finger goes up and down it decreases it". A dip is one
// finger on its own: it falls DIP_FRAC of its flat-to-closed swing (at least DIP_MIN_MV) inside
// DIP_FALL_MS and comes back up DIP_BACK_FRAC of that excursion within DIP_MAX_MS of starting,
// while the other three fingers and the thumb each move less than DIP_STILL_FRAC of their own
// swing (at least DIP_STILL_MIN_MV) - so closing the hand, a squeeze, or the thumb's crunches are
// never a dip. Only while the ring is on; down stops at 1/10 (off is the closed hand + crunches).
constexpr float    DIP_FRAC         = 0.20f;   // (25 % missed quick dips in the replay: the 35 ms smoothing shaves a 120 ms dip)
constexpr uint16_t DIP_MIN_MV       = 30;
constexpr uint32_t DIP_FALL_MS      = 250;
constexpr float    DIP_BACK_FRAC    = 0.50f;
constexpr uint32_t DIP_MAX_MS       = 800;
constexpr float    DIP_STILL_FRAC   = 0.15f;
constexpr uint16_t DIP_STILL_MIN_MV = 25;
constexpr uint32_t DIP_GAP_MS       = 300;   // two dips on one finger closer than this are one


// Toby, 2026-09-28, after the first cut (35 deg, any finger movement stopped it): "my hand is in a line with my arm, then I
// close my hand and SLIGHTLY move my wrist down a little bit - that's when the laser should turn on. When I let go of my
// fingers a little bit - a little bit more than a little bit - it'll turn off."
// Third cut, same evening (Toby: "aligned just means my wrist isn't to the side or up or down or twisted ... I could do it
// facing up, facing down, anywhere ... the down movement is super minor, another key to the password, but quick ... once
// it's on I could move anywhere, the moment I let go of my fingers more than half it turns off, otherwise it stays on"):
// the wrist is the HAND RELATIVE TO THE FOREARM (imuRelative: conj(d_arm) * d_hand, both zeroed together at step 1), so it
// reads the same whatever the arm does; "in line" = flat in all three; a small quick flex either way with the fist fires;
// only letting the fingers go stops it.
constexpr float    LASER_INLINE_DEG   = 8.0f;    // |flexion| under this ...
constexpr float    LASER_SIDE_DEG     = 35.0f;   // (20 until the jolt cut)   // ... and |side-to-side| under this ...
constexpr float    LASER_TWIST_DEG    = 50.0f;   // (30 until the jolt cut)   // ... and |twist| under this = the wrist is flat / in line: where a shot has to start
constexpr float    LASER_FIRE_DEG     = 12.0f;   // back to 12 (20 for an hour on 2026-09-30 made it "one in ten"): the drone gesture is kept apart by LASER_FRESH_OPEN_MS instead   // a super-minor flick: this much flexion (either way) ...
// 2026-09-30 late (Toby: "it only works when I am horizontal ... my hand has to be perfectly straight"): the trigger is a
// JOLT - the wrist flex changes by LASER_JOLT_DEG inside LASER_JOLT_MS from a roughly straight start (flex within
// LASER_START_FLEX_DEG, side within LASER_SIDE_DEG, twist within LASER_TWIST_DEG) with a fresh fist held - no flat line to
// sit inside first. And the heading healer runs in ANY arm pose (see laser.cpp): the two sensors' headings drift apart
// and restart at every power-up; horizontal that read as the hand pointing sideways, arm down or up it read as twist
// and flex and the laser (and the model) went wrong until a step 1.
constexpr float    LASER_JOLT_DEG       = 12.0f;   // (10 of flex alone until 2026-10-05; now the whole rotation of the hand against the forearm, any direction, so a little more)
constexpr uint32_t LASER_JOLT_MS        = 400;
// The trigger, the way Toby says it (third flight): "my fingers go from open to close really fast, all four, and I jolt
// my hand a little - down and up maybe even, an obvious change - and my hand decently in line with my arm". So: a FRESH
// fist that formed from an open hand within LASER_CLOSE_FAST_MS, a wrist flex excursion of LASER_JOLT_DEG peak to peak
// inside the last LASER_JOLT_WINDOW_MS (before or after the fist formed), side and twist within their limits, judged
// until LASER_AFTER_FIST_MS after the fist formed. The drone gesture's first fist is formed calmly (no jolt) and its
// second is not fresh.
constexpr uint32_t LASER_CLOSE_FAST_MS  = 400;
constexpr uint32_t LASER_JOLT_WINDOW_MS = 600;
constexpr uint32_t LASER_AFTER_FIST_MS  = 1000;
constexpr float    LASER_START_FLEX_DEG = 25.0f;
constexpr uint32_t LASER_FLICK_MS     = 500;     // ... within this long of leaving the in-line zone (slower = a droop, not a flick)
constexpr uint32_t LASER_FRESH_OPEN_MS = 1000;  // the fist that fires must be a FRESH one: made after the hand was not a fist for at least this long. The drone's arm gesture (a fist, a quick open, a fist again) re-closes within a second and so can never fire the laser; Toby's laser gesture closes from an open hand and flicks
constexpr uint32_t LASER_FIST_GRACE_MS = 700;    // the fist may close this long after the flick got there (flick first, squeeze second, still counts) ...
constexpr uint32_t LASER_FIST_HOLD_MS = 100;     // ... and must stay closed this long before it fires (Toby: "hold my fingers there for a tenth of a second" - a filter that also cuts accidents)
constexpr uint16_t LASER_RELEASE_MV   = 25;      // "let go more than half": middle/ring/pinky must read this far ABOVE their half-curl thresholds on average
                                                 // to switch it off - a finger twitching at the threshold does not
constexpr uint32_t LASER_OPEN_MS      = 250;     // ... and stay there this long (150 until 2026-10-05: a quick relaxation after the jolt is not a let-go)
constexpr uint32_t LASER_STALE_MS     = 500;     // no fresh sample from either IMU for this long = off, fail safe
constexpr uint32_t LASER_MAX_ON_MS    = 300000;  // a forgot-it-on guard, not a demo limit: 5 minutes in one go (forced-on drops back to auto)
constexpr uint32_t LASER_BOOT_HOLD_MS = 3000;    // nothing fires in the first seconds after boot
constexpr uint32_t LASER_ZERO_MS      = 800;     // GET /laser?zero=1 (the sim page's step 1, hand in line) zeroes BOTH sensors over this window (imuZeroAsync)
constexpr float    LASER_HEAL_LEVEL_DEG = 30.0f; // the yaw healer runs only with the forearm within this of level (there a heading nudge is a wrist-yaw nudge) ...
constexpr float    LASER_HEAL_MAX_DEG = 40.0f;   // ... only on drift up to this (more = do step 1 again) ...
constexpr float    LASER_HEAL_GAIN    = 0.25f;   // per 100 ms, of the angle still to take out (0.02 until the pose-independent healer)   // ... taking this fraction of the side angle out every 100 ms while the hand is flat, open and the laser off (about 5 s to settle)

constexpr uint32_t LIDAR_PERIOD_MS    = 100;     // ~10 measurements a second
constexpr uint32_t LIDAR_STATUS_MS    = 5;       // how often to ask "still busy?" (shares the bus with the arm BNO085)
constexpr uint32_t LIDAR_BUSY_MAX_MS  = 250;     // a measurement still busy after this is abandoned
constexpr uint32_t LIDAR_PROBE_MS     = 1000;    // re-probe an absent sensor this often
constexpr uint32_t LIDAR_LOST_MS      = 2000;    // declared absent only after this long without one successful transaction (a NACK or two mid-measurement is not "gone")
constexpr uint16_t LIDAR_MIN_CM       = 5;       // outside MIN..MAX = no return (the v3 ranges 5 cm to 40 m; 10 m is the most this rig cares about)
constexpr uint16_t LIDAR_MAX_CM       = 1000;
constexpr bool     LIDAR_LOG_DEFAULT  = true;    // "lidar: NNNN mm" in the console at the measurement rate (key q toggles)

constexpr uint32_t CRSF_BAUD          = 400000;
constexpr uint32_t CRSF_PERIOD_MS     = 20;      // RC frames at 50 Hz (Toby's spec; the gauntlet ran 200 Hz, the module takes either)
constexpr uint32_t CRSF_FEED_STALE_MS = 300;     // the CRSF task drops throttle and ARM itself if loop() has not fed the sticks for this long (stalled, flashing)
constexpr uint32_t CRSF_QUIET_MS      = 1500;    // nothing heard from the module for this long = "silent" on the pages
constexpr uint32_t CRSF_LINKSTATS_LOST_MS = 2000; // no LINK_STATISTICS from the module for this long = the drone link is gone (the module stops sending them when the receiver vanishes - the board believed "LQ 100" after the drone's battery was pulled, 2026-10-01)
constexpr uint32_t CRSF_BIND_EVERY_MS = 4000;    // the bind loop: a Bind command to the module this often ...
constexpr uint32_t CRSF_BIND_FOR_MS   = 90000;   // ... for this long at most (it stops as soon as the drone links)
// Every time the module shows up (power-cycled), once it has talked for CRSF_AUTO_DYNAMIC_MS: Max Power to the
// CRSF_POWER_MAX_MW cap and Dynamic to "Dyn" (Toby, 2026-10-06: "dynamic power, max 100 mW"). ExpressLRS then starts
// at its minimum power and steps up only as the link needs it (it needs telemetry on - Telem Ratio not Off / Race),
// so at bench distance it sits low and no longer jams the ESP32's WiFi; it never passes the cap.
constexpr bool     CRSF_AUTO_DYNAMIC    = true;
constexpr uint32_t CRSF_AUTO_DYNAMIC_MS = 3000;
constexpr uint32_t CRSF_AUTO_BIND_MS  = 20000;   // the module on and no drone linked for this long = run the bind loop by itself (Toby: "if it's not binded, maybe bind it, so I don't have to manually do it"; his power-cycling puts the whoop in bind mode now and then). Repeats every CRSF_AUTO_BIND_AGAIN_MS while still unlinked
constexpr uint32_t CRSF_AUTO_BIND_AGAIN_MS = 120000;
constexpr uint32_t FLY_BENCH_ARM_S    = 45;      // the page's "arm test": hand-driven once armed, disarms by itself after this
// ---------------- flight: the hand as the sticks (fly.*) ------------------------------
// Toby (2026-09-30): "the throttle being my fingers up and down, crunching just my full front
// four; opening my hand arms it, only if it's armable - horizontal and capable of being armed;
// moving my hand left/right rolls, forward/back pitches; moving my arm up and down does
// nothing; it has to be pretty unsensitive so it doesn't throttle crazy high or low if I move my
// fingers just a little". Throttle = the mean curl of index, middle, ring and pinky (the page's
// flat/closed per finger, in resistance like the model): open = minimum, so an open hand is
// idle and Betaflight will arm. Roll = wrist twist, pitch = wrist flexion, both hand relative
// to forearm (imuRelative), so the arm can be anywhere. Arm = the hand was closed, then opened
// and held open - while the module hears the drone, the drone says it is ready and level, and
// flight mode is on (the page's "fly on" / key E; it turns the ring and laser gestures off).
// Disarm = a thumb crunch, the page's DISARM, key D, or any fail-safe. ARM is AUX1 (index 4)
// as the whoop's Modes tab says; AUX2 low = angle mode.
constexpr uint8_t  FLY_ARM_AUX          = 4;
constexpr float    FLY_THR_MAX          = 0.70f;  // the most throttle the fingers can ask for (of full): the OPEN hand, "more than hovering"; a fist is minimum, hover in between (Toby 2026-09-30 night - the first cut had it the other way round)
constexpr float    FLY_THR_DEAD         = 0.10f;  // the first tenth of the curl is still minimum: a relaxed open hand is idle
constexpr float    FLY_THR_TOP          = 0.55f;  // minimum throttle from this much curl on: a loose fist is already "lowest" (0.85, then 0.60)
constexpr float    FLY_THR_EXPO         = 0.30f;  // 0 linear .. 1 cubic: a little gentler near the bottom
constexpr float    FLY_THR_SLEW_PER_S   = 2.0f;   // the throttle moves at most this much of its range per second: a twitch cannot throw it (1.2 felt unresponsive)
constexpr float    FLY_STICK_DEAD_DEG   = 3.0f;   // wrist within this of straight = stick centred (6, then 4)
constexpr float    FLY_STICK_FULL_DEG   = 25.0f;  // this much wrist = the full stickmax (35 before the first flight: "it almost did nothing")
constexpr float    FLY_STICK_MAX        = 0.50f;  // of full deflection (angle mode: about half the lean limit)
constexpr float    FLY_STICK_EXPO       = 0.0f;   // LINEAR: the same tilt always gives the same lean ("sometimes it does little, sometimes a lot" was the expo plus the dead band)
constexpr float    FLY_STICK_SLEW_PER_S = 4.0f;
constexpr int8_t   FLY_ROLL_SIGN        = +1;     // flip if the drone rolls the wrong way (twist + = right side of the hand down)
constexpr int8_t   FLY_PITCH_SIGN       = +1;     // flip if it pitches the wrong way (flex + = fingers down = forward)
// The arm gesture, third cut (Toby 2026-09-30 night: "my hand is open, then I close it really quick and
// open it really fast - open, closed, open within a quarter of a second, just my four fingers"): a
// SQUEEZE PULSE. From an open hand (mean curl under FLY_OPEN_CURL) the fingers close past
// FLY_SQUEEZE_CURL and are open again within FLY_ARM_PULSE_MS of leaving open, then stay open
// FLY_ARM_OPEN_MS, with the hand level and the wrist in line at the end, and the drone's checks.
constexpr float    FLY_OPEN_CURL        = 0.15f;  // "open" (the gesture starts and ends here; the throttle is at its most here too)
constexpr float    FLY_SQUEEZE_CURL     = 0.35f;  // the pulse must reach at least this mean curl
constexpr uint32_t FLY_ARM_PULSE_MS     = 2000;   // leaving the fist to being back in it inside this (700, then 1500 until 2026-10-07: two of his last four tries took 1511 and 1522 ms)
constexpr float    FLY_THR_CAPTURE      = 0.25f;  // the page's ARM only: the throttle waits at minimum until the fingers ask for no more than this, then it is theirs (a gesture arm starts from the fist and needs no wait; 0.05 was never reached by a normal fist - "50 % and 0 % were the same")
// Fourth cut, later the same night (Toby: "throttle is my hands out, so arming with my hands out would start
// at 100 % - that's stupid. Now: my hand in a fist, then really quickly I open my hand and close it, that
// arms it at the lowest throttle; lifting my fingers turns the throttle up; turning my wrist in the
// horizontal plane is yaw; disarm is a quick double click of my thumb"): from a FIST (mean curl >=
// FLY_CLOSED_CURL) the fingers come out past FLY_PULSE_OPEN_CURL and are back in a fist within
// FLY_ARM_PULSE_MS, held FLY_ARM_OPEN_MS, hand level and wrist in line at the end, the drone's checks.
// The laser keeps its fist + wrist jolt, now a heavier one (LASER_FIRE_DEG 20): the drone gesture needs
// the wrist still (flex within 12), so a band of 12-20 deg separates the two.
constexpr float    FLY_CLOSED_CURL      = 0.40f;  // a fist, for the gesture (a loose one will do; 0.55 armed one time in twenty, 2026-09-30 night)
constexpr float    FLY_PULSE_OPEN_CURL  = 0.35f;  // the fingers must come out at least this far (curl at or under this; 0.30 asked for a full extension inside a quick flick)
constexpr int8_t   FLY_YAW_SIGN         = -1;     // yaw = the hand turned left / right of the forearm (relative side angle, + = left); stick + = turn right. Flip if it turns the wrong way
constexpr float    FLY_YAW_FULL_DEG     = 30.0f;  // this much side angle = FLY_YAW_MAX
constexpr float    FLY_YAW_DEAD_DEG     = 8.0f;   // yaw has its own, wider dead band: the side angle is heading-based and drifts (no magnetometer); it is also re-zeroed at the moment of arming
constexpr float    FLY_YAW_MAX          = 0.50f;
constexpr uint32_t FLY_ARM_CLOSED_MS    = 120;    // ... for this long, then ... (300 missed a quick fist-and-open, 2026-09-30 evening)
constexpr uint32_t FLY_ARM_OPEN_MS      = 300;    // ... open (under FLY_THR_DEAD) and held for this long (500 until the glove-on cut) ...
constexpr uint32_t FLY_ARM_WITHIN_MS    = 2500;   // ... starting within this of the closed spell ending
constexpr float    FLY_LEVEL_DEG        = 35.0f;  // "armable": the drone's own attitude (telemetry 0x1E) within this of level
constexpr uint8_t  FLY_MIN_LQ           = 50;     // and the uplink quality at least this
constexpr uint32_t FLY_TELEM_FRESH_MS   = 1500;   // the drone's mode string and attitude this recent, or no arming
constexpr uint32_t FLY_TELEM_LOST_MS    = 2000;   // armed and the drone's telemetry stops for this long = disarm
constexpr uint32_t FLY_HAND_STALE_MS    = 300;    // the hand or forearm IMU older than this = no arming / disarm
constexpr float    FLY_DISARM_THUMB_CURL = 0.50f; // a thumb crunch past half its (fist-limited) swing ...   // (unused since the fourth cut: disarm is a thumb DOUBLE crunch, gesture.cpp counts them)
constexpr uint32_t FLY_DISARM_THUMB_MS  = 100;    // ... held this long = disarm
constexpr bool     FLY_NEED_TELEMETRY   = true;   // false = arm without hearing the drone (bench only, props off)
// Arm gesture, second cut (Toby 2026-09-30, glove on): "a fist and then glove out super fast, my glove
// straight out - like the laser's orientation, except genuinely horizontal and in a line with my wrist".
constexpr uint32_t FLY_ARM_OPEN_FAST_MS = 400;    // closed -> open in at most this ("super fast")
constexpr float    FLY_ARM_LEVEL_DEG    = 60.0f;  // the hand within this of level: roll and pitch from its page step-1 pose (15, then 20; his straight-out hand sits 15-20 deg from his step 1 - one arm refused at 20)
constexpr float    FLY_ARM_FLEX_DEG     = 35.0f;  // and the wrist in line with the forearm: flexion ... (12 before; the laser needs a QUICK 20 so the two still do not meet)
constexpr float    FLY_ARM_SIDE_DEG     = 35.0f;  // ... side ...
constexpr float    FLY_ARM_TWIST_DEG    = 60.0f;  // ... twist within these
// ---------------- flight tuning, live (2026-09-30 night) --------------------------------
// Toby: "the difference between all the way down and all the way up is just slightly above hovering;
// hovering is slightly below my fingers straight ... everything else less sensitive as well; fine
// movements equal very fine drone movements ... the throttle has to be quite perfect, it has to follow
// my fingers very accurately". So: the throttle is a STRAIGHT LINE from the fist (minimum) to the
// straight fingers (FLY_THR_TOP_PCT of full), followed through a 60 ms filter, no rate limit; the
// sticks get small maxima. These three live in NVS and the page's drone row has - / + for them
// (GET :8080/fly?thrmax=N&stickmax=N&yawmax=N, percent of full), because the hover point differs with
// the battery and the weight. The defaults below are the first guess.
constexpr uint8_t  FLY_THR_TOP_PCT_DEFAULT   = 37;    // straight fingers = this much throttle (the board's slider, NVS "fthr", 2026-10-07; 50 before - 58 climbed "from mid to all the way up super quickly" on the first flight)
constexpr uint8_t  FLY_STICK_MAX_PCT_DEFAULT = 66;    // roll / pitch at full wrist, percent of full stick (the board's slider, NVS "fstk", 2026-10-07; 70 before - 30, then 50 "almost did nothing")
constexpr uint8_t  FLY_YAW_MAX_PCT_DEFAULT   = 47;    // yaw at full turn, percent of full stick (the board's slider, NVS "fyaw", 2026-10-07; 70 before)
constexpr float    FLY_FILTER_S              = 0.06f; // every stick follows its target through this time constant
constexpr float    FLY_CRAFT_LOW_V           = 3.30f; // the drone's own battery (telemetry) at or under this = LOW: the throttle is capped below hover (it comes down) and the palm ring blinks red (Toby: "when it hits 3.3 volts, turn the throttle a lot down and blink my glove"). The pages go amber from FLY_CRAFT_WARN_V
constexpr uint32_t FLY_CRAFT_LOW_MS          = 3000;  // ... held this long (a throttle burst sags a fine cell for a moment)
constexpr float    FLY_CRAFT_WARN_V           = 3.45f; // the chip goes amber under this
constexpr float    FLY_LOW_THR_FACTOR        = 0.80f; // battery LOW: the throttle may not exceed this x the hover setting - a controlled descent, the sticks still work
// The throttle curve, hover-centred (Toby after the first flight: "more like an exponential graph - more of the range in
// the middle, so it hovers over a decent amount of my fingers; all the way down goes down a lot, all the way up goes up a
// lot"): the middle of the finger range sits at the HOVER throttle (thrhover, live, NVS "fhov") and the output moves away
// from it with FLY_THR_MID_EXPO (0 = straight lines to the ends, 1 = cubic: nearly flat around hover), to minimum at the
// fist and to thrmax at straight fingers.
constexpr uint8_t  FLY_THR_HOVER_PCT_DEFAULT = 42;    // a first guess for an Air65 on a fresh cell; set it where the drone actually hovers
constexpr float    FLY_THR_MID_EXPO          = 0.70f;
// The finger window of the throttle (Toby, third flight: "the range I want is literally between fingers all the way down
// and fingers slightly ever so up - make that little movement the entire range"): minimum at thrclose % curl and
// above, the top at thropen % curl and below, hover in the middle of the window. Both live (NVS "fclo" "fopn").
// Fourth flight: the window ANCHORS ITSELF to the fist at the moment of arming (and to any tighter fist after), because
// "a millimetre up and it flies to the roof" meant the fixed window did not sit where his fist reads. Minimum from the
// anchor down to FLY_THR_FIST_DEAD below it, then the whole range over the next thrspan % of curl (live, NVS "fspn"),
// hover in the middle of that lift.
// Fifth flight, 2026-10-01 (Toby: "your values don't make sense to me ... one bar for the range, 0 to whatever percent I
// choose, and one for sensitivity; bigger percent = more sensitive"): the throttle is LINEAR from the fist (0) to the top
// (thrmax) over a lift that thrsens sets - sensitivity 0 = a 64 % lift, 100 = a 4 % lift. No hover setting, no curve.
constexpr uint8_t  FLY_THR_SENS_PCT_DEFAULT  = 9;     // -> a lift of about 59 % of the fingers' travel for the whole range (the board's slider, NVS "fsen", 2026-10-07; 30 = 46 % before)
// (HandBench 2026-10-07 night, bba6932: what lived only in its board's NVS, now the defaults - the sliders above and Toby's
// measured wrist range. Its finger calibration (GESTURE_CAL_* in mV) is NOT taken: those millivolts are HandBench's 10 k
// dividers on its own ADC pins; the gauntlet has 47 k dividers and the ADS1115 - calibrate on the arm with K / J.)
constexpr float    FLY_RANGE_DEFAULT_NEG[3] = { 28, 36, 40 };   // the wrist range (step 7, NVS "frng"): full stick at this much tilt / bend / turn the minus way ...
constexpr float    FLY_RANGE_DEFAULT_POS[3] = { 19, 35, 20 };   // ... and the plus way, degrees (FLY_STICK_FULL_DEG / FLY_YAW_FULL_DEG each way until 2026-10-07)
constexpr uint8_t  FLY_THR_SPAN_PCT_DEFAULT  = 15;    // (fourth flight, unused since thrsens)
constexpr float    FLY_THR_FIST_DEAD         = 0.08f; // a fist a little looser than the anchor is still minimum (0.05 before the fifth flight)
constexpr float    FLY_FLIP_DEG              = 75.0f; // the drone's own attitude past this = it is flipping or has crashed: disarm at once (Toby: "if it starts flipping, turn it off immediately")
constexpr float    FLY_THR_ANCHOR_DEFAULT    = 0.65f; // the anchor before any fist has been seen (the page's ARM with an open hand)
constexpr uint8_t  FLY_THR_CLOSE_PCT_DEFAULT = 60;    // (third flight, unused since the anchor)
constexpr uint8_t  FLY_THR_OPEN_PCT_DEFAULT  = 40;    // "slightly up": the top from here on
// ---------------- flight feel, 2026-10-01 (Toby: "the settings were tuned for the battery, then I changed the battery and
// 30 % should mean something slightly different at 3.6 V than at 3.4 V ... the throttle should sit around perfect, slightly
// up or down jumps a lot ... make it easy to fly, it's a software issue ... keep my sliders") -------------------------
// Voltage compensation: motor thrust at a given throttle scales with the cell voltage (RPM with voltage, thrust with
// RPM squared, so the duty needed for one thrust goes about as 1/V); the throttle sent is the commanded one times
// FLY_VREF over the drone's reported voltage, clamped, so the range and sensitivity sliders mean the same thrust on a
// fresh cell as on a tired one. Betaflight's own vbat_sag_compensation would do the same inside the drone - leave it off.
constexpr float    FLY_VREF         = 3.70f;  // the voltage the sliders are "true" at
constexpr float    FLY_VCOMP_MIN    = 0.80f;  // the factor's limits (4.6 V .. 2.96 V equivalent)
constexpr float    FLY_VCOMP_MAX    = 1.25f;
constexpr float    FLY_VOLT_TAU_S   = 1.0f;   // the voltage is filtered this much before use (a throttle burst sags it)
// The throttle curve, not a slider: thr = top x (1 - (1 - x)^FLY_THR_GAMMA) over the lift x. Steep at the bottom (a fist
// drops it fast), flat near the top where hover lives, so most of the fingers' resolution is spent around hover.
constexpr float    FLY_THR_GAMMA    = 1.0f;   // 1 = a straight line from the floor to the top (2 until the seventh round, when the bottom of the range was Betaflight minimum)
constexpr float    FLY_THR_SLEW     = 0.5f;   // the throttle may move at most this much of full per second (2.0 until the seventh round): a 0.2 s finger dip is a few percent; floor to top (about 15 %) still takes 0.3 s
// Seventh round (2026-10-01, after the sixth flight). Toby: "an atom too closed and it drops the drone, then I overshoot
// climbing back; there is never a time I want to drop the throttle that quickly - average my throttles, keep it real
// time, nothing crazy"; "I turn my whole hand left as far as it goes and barely anything happens, right is easy - map it
// to my arm's range". The throttle: armed, the motors idle until the fingers have lifted once; from then a fist is a FLOOR
// (a gentle descent), a full lift the range top, hover between, and the fist reference is locked (a mid-flight squeeze
// used to move hover under his fingers). Roll = the hand's own tilt against gravity. Trims at arming. A wrist-range step.
constexpr uint8_t  FLY_THR_FLOOR_PCT_DEFAULT = 30;    // a fist IN FLIGHT asks for this much (of full, at the reference voltage): a gentle descent, never a drop - live "thrfloor", NVS "fflo". Betaflight's minimum only before lift-off and after a cut. Set it from the flight summary's mean (about hover) - roughly 0.85 x hover
constexpr float    FLY_THR_FLOOR_MAX_OF_TOP  = 0.80f; // the floor never exceeds this fraction of the range top (a low range must still be able to descend)
constexpr float    FLY_THR_LIFT_X            = 0.12f; // armed: the motors idle until the fingers have lifted this far (of the lift span) once; then the floor applies and the fist reference is locked
constexpr float    FLY_THR_CUT_OVER          = 0.08f; // a deliberate CUT: a fist this much tighter than the reference ...
constexpr float    FLY_THR_CUT_MIN           = 0.85f; // ... and at least this closed ...
constexpr uint32_t FLY_THR_CUT_MS            = 1200;  // ... held this long = motors idle (landing); lifting the fingers makes the throttle live again. Unreachable when the arming fist is already the glove's limit - the thumb double disarms then
constexpr float    FLY_THR_FILTER_S          = 0.30f; // (0.25 until 2026-10-03: "it jumped a little up and down") // the throttle follows the fingers through this time constant (the sticks keep FLY_FILTER_S): "an average, still real time"
constexpr bool     FLY_ROLL_ABSOLUTE         = true;  // roll = the hand's own tilt against gravity (its page step-1 pose = level, imuSegAngles), not hand-vs-forearm twist: tilting the whole arm counts, and a palm-down wrist can hardly pronate further anyway ("left, I can only go slightly")
constexpr float    FLY_ROLL_HOLD_PITCH_DEG   = 70.0f; // the hand this near vertical: roll is ill-defined, the last value is held
constexpr bool     FLY_TRIM_AT_ARM           = true;  // the hand's roll and the wrist's flex at the moment of arming are neutral (as yaw already was) ...
constexpr float    FLY_TRIM_MAX_DEG          = 30.0f; // ... up to this much: the pose it arms in is neutral in full (12 until 2026-10-03: a recording showed his wrist 20-30 deg "fingers down" past step 1 in flight, a steady forward pitch of 15-35 % the trim could not absorb - "it goes straight in one direction"; the arming check still wants the wrist within FLY_ARM_FLEX_DEG)
constexpr uint32_t FLY_RANGE_CAPTURE_MS      = 6000;  // the page's step 7 (GET :8080/fly?range=1): the wrist's extremes over this window ...
constexpr float    FLY_RANGE_USE             = 0.80f; // ... full stick at this fraction of each extreme ...
constexpr float    FLY_RANGE_MIN_DEG         = 8.0f;  // ... never nearer than this ...
constexpr float    FLY_RANGE_MAX_DEG         = 40.0f; // ... nor further than this ...
constexpr float    FLY_RANGE_NEED_DEG        = 10.0f; // ... and a direction the wrist did not reach this far in keeps the default (FLY_STICK_FULL_DEG / FLY_YAW_FULL_DEG). range=0 forgets the capture
constexpr uint32_t FLY_RANGE_START_MS        = 400;   // the first part of the capture is the starting pose: each range is how far the wrist MOVED from there (a glove resting upside down read "tilt 173" as an extreme on the first try)
// Eighth round (2026-10-01, evening) - Toby: "two thumb clicks should instantly turn it off"; "the LiDAR: for now only on
// when the laser is on"; "the laser: my thumb has to be crunched down as well as my fingers - my pointer finger can't
// crunch that much, but there is a noticeable difference"; "yaw in relation to my forearm so I can move around".
constexpr float    LASER_THUMB_CURL      = 0.50f;  // the laser's fist also needs the thumb this far into its (fist-limited) swing ...
constexpr float    LASER_INDEX_CURL      = 0.30f;  // ... and the index at least this far closed (it cannot crunch far in the glove). The drone's arming fist keeps the thumb out, so the two no longer collide. An uncalibrated finger does not block
constexpr bool     LIDAR_WITH_LASER_ONLY = true;   // the LiDAR triggers measurements only while the laser is on (no bus-1 traffic beside the arm BNO085 otherwise, no ticker in the log); the boot probe still finds it, idle = mm 0
// Ninth round (2026-10-01, late) - Toby: "three slightly dramatic thumb presses instead of two to disarm"; "never go up
// super fast"; "sometimes it lands and flips - a flip protocol: I arm it again, it flips back over and turns off, then I
// arm it to fly"; "arming takes a couple of tries at first" (the records: the drone itself said '!' for 1-3 s).
constexpr float    DISARM_AMP_FRAC        = 0.25f;  // "slightly dramatic": each of the first two presses at least this fraction of the thumb's flat-to-fist swing ...
constexpr uint16_t DISARM_AMP_MIN_MV      = 60;     // ... and at least this many mV (30 until 2026-10-03: with a 35 mV thumb calibration two 30 mV twitches in rhythm disarmed the drone in flight; his real presses are 100-190 mV)
// ---- ported from HandBench's seventeenth to nineteenth rounds (2026-10-05 .. 07), verbatim (Toby, 2026-10-07 night: "the drone
// gestures must match HandBench EXACTLY; I trained on them") ----
constexpr float    DISARM_FIST_CURL      = 0.60f;  // the disarm pose: the MEDIAN of middle / ring / pinky past this (0.70 for each of the three until 2026-10-05 late - his middle-finger sensor reads open while he points) ...
constexpr float    DISARM_FIST_OPEN_CURL = 0.50f;  // ... and the pose holds until that median drops under this
constexpr uint32_t DISARM_CLOSED_MS      = 60;     // closed at least this long before the first thumb press began (150 until the same day: a quick close-then-two-presses needed a third press; the dragged thumb starts before the hand counts as closed anyway) (so the thumb dragged along by the closing is not a press)
constexpr uint32_t DISARM_WINDOW_MS      = 800;    // the second press starts within this of the first (and at least GESTURE_MIN_GAP_MS after it)
constexpr float    DISARM_INDEX_UP_CURL   = 0.35f;  // the disarm pose's index: at most this curled (up, pointing - his pointing read 0-27 %) ...
constexpr float    DISARM_INDEX_DOWN_CURL = 0.45f;  // ... and the pose ends when it curls past this. A full fist - the descent - is never the pose
constexpr uint32_t GESTURE_AFTER_FLIGHT_MS = 1500;  // (Ironman only, 2026-10-07 night) the triple-crunch waits this long after a flight ends and forgets the clicks before it: the disarm's two clicks plus one more must never complete a triple (a ring change or, arm raised, the thermal camera)
constexpr uint32_t FLY_FLEX_LOST_MS        = 300;   // (Ironman, 2026-10-08) armed, and fewer than three of the four fingers readable (faulted: broken, shorted, the ADS1115 silent) this long: disarmed - the throttle cannot be trusted
// Toby's gesture map (gesturemap.cpp, 2026-10-21 - it replaced the placeholders of 2026-10-07). The four fingers move as
// ONE measure for the fist / open / pulse gestures - the drone's own (fly.cpp: the mean of the middle two of the four) with
// the drone's own lines, so the plates, the camera and the arming feel the same: a fist at FLY_CLOSED_CURL, "out" at
// FLY_PULSE_OPEN_CURL or a quick FLY_PULSE_DROP down from the fist.
constexpr float    GMAP_THUMB_IN_CURL    = 0.50f;   // the thumb IN: its (fist-limited) curl at least this (the laser's LASER_THUMB_CURL) ...
constexpr float    GMAP_THUMB_OUT_CURL   = 0.30f;   // ... OUT: at most this; between = neither, nothing fires on it
constexpr uint32_t GMAP_THUMB_LOOK_MS    = 200;     // the thumb's state = its highest curl over the last this long (it tucks in late, it may lift a little as the fingers open)
constexpr float    GMAP_FINGER_OUT_CURL  = 0.40f;   // one finger OUT (the green laser's index + middle): at most this ...
constexpr float    GMAP_FINGER_IN_CURL   = 0.50f;   // ... IN (its ring + pinky): at least this
constexpr float    GMAP_OPEN_CURL        = 0.35f;   // the whole hand OPEN (the plates' "all the way back out"): the four at most this, the thumb OUT
constexpr uint32_t GMAP_FULL_RECENT_MS   = 300;     // the plates' and the camera's fist is a REAL one (the index and three of the four in - not a finger gun) within this before the fingers leave it
constexpr uint32_t GMAP_FIST_HOLD_MS     = 200;     // the plates' fist (thumb in) held at least this before the fingers leave it
constexpr uint32_t GMAP_QUICK_MS         = 500;     // open all: from leaving the fist to the whole hand open inside this ("quickly")
constexpr uint32_t GMAP_PULSE_MS         = 1000;    // a pulse (close all, the camera): out of the fist and back in it inside this
constexpr uint32_t GMAP_DOUBLE_GAP_MS    = 250;     // the camera: the second pulse leaves the fist inside this after the first came back - under the drone's
                                                     // FLY_ARM_OPEN_MS hold, so a camera double never arms the drone (its first pulse is cancelled by the second)
constexpr float    GMAP_ALIGN_FLEX_DEG   = FLY_ARM_FLEX_DEG;    // "aligned" = the wrist in line with the forearm - the drone's own arming check, one feel for all
constexpr float    GMAP_ALIGN_SIDE_DEG   = FLY_ARM_SIDE_DEG;
constexpr float    GMAP_ALIGN_TWIST_DEG  = FLY_ARM_TWIST_DEG;
constexpr uint16_t GMAP_BUZZ_MS          = 80;      // the answer in the hand: one buzz = done, two = refused (the battery, the mode, not aligned, busy)
constexpr uint16_t GMAP_BUZZ_GAP_MS      = 90;
constexpr uint32_t DISARM_POSE_GRACE_MS   = 150;    // the second click counts if the pose was seen within this (it may flicker between the clicks)
constexpr float    LASER_FIST_CURL       = 0.60f;  // the laser's fist: the MEDIAN of middle / ring / pinky past this (and it stays closed until under LASER_SHUT_LEAVE_CURL); the thumb and the index must be in at the moment it fires ...
constexpr float    LASER_OPEN_CURL       = 0.40f;  // ... and it goes off when that median falls under this for LASER_OPEN_MS (0.45 on the mean until 2026-10-05 late: his middle finger reading open turned the laser off the moment it fired - 8 shots he never saw)
constexpr float    LASER_SHUT_LEAVE_CURL  = 0.50f;  // the laser's closed hand ends when the median drops under this (enters at LASER_FIST_CURL)
constexpr uint32_t LASER_TRY_FROM_OPEN_MS = 1500;   // a closing of the hand within this of it reading open is a laser ATTEMPT: recorded with why it did not fire (log, feed laser.ltry / lcnt, the page's laser row)
constexpr float    FLY_PULSE_DROP        = 0.30f;  // (2026-10-07) ... or a quick flick: the curl dropped this much from the fist it left (a quick flick does not open the fingers all the way - "I have to do it fairly slow now")
constexpr float    FLY_THR_SLEW_UP        = 0.20f;  // (0.30 until 2026-10-03: "I like the speed, a little slower would be great")  // the throttle may rise at most this much of full per second ("never go up super fast") ...
constexpr float    FLY_THR_SLEW_DOWN      = 0.35f;  // (0.50 until 2026-10-03)  // ... and fall this much (FLY_THR_SLEW before, both ways)
constexpr float    FLY_THR_CLIMB_GAMMA    = 1.6f;   // above the hover estimate the lift maps to throttle with this power: gentle just above hover, the full climb only at a full lift
constexpr uint8_t  FLY_THR_HOVER_GUESS_PCT = 35;    // hover before any flight has taught it; then the mean throttle of every flight with the throttle live for FLY_HOVER_LEARN_MIN_S that lands inside the floor-top window (live "hover", NVS "fhv2")
constexpr float    FLY_HOVER_LEARN_MIN_S  = 10.0f;
constexpr uint32_t FLY_ARM_WAIT_READY_MS  = 3000;   // a good gesture while the drone says '!' (or its telemetry is not fresh) waits this long for '*' with the fist held, then arms by itself
constexpr uint8_t  FLY_FLIP_AUX           = 6;      // channel 6 = AUX3 = Betaflight's FLIP OVER AFTER CRASH on this whoop (1700-2100, Configurator 2026-09-06)
constexpr float    FLY_FLIP_INVERTED_DEG  = 60.0f;  // the drone's own attitude past this = it lies over: the arm gesture flips it instead of arming
constexpr float    FLY_FLIP_UPRIGHT_DEG   = 35.0f;  // ... within this again = upright: stop at once, disarm, flip mode off
constexpr uint32_t FLY_FLIP_MODE_LEAD_MS  = 400;    // flip mode on this long before the arm (Betaflight reads the switch when it arms)
constexpr uint32_t FLY_FLIP_ARM_LEAD_MS   = 500;    // armed with the sticks centred this long before the first push (the motors take their reversed direction)
constexpr float    FLY_FLIP_PUSH          = 1.00f;  // FULL stick for a push: Betaflight's crashflip_expo (35 %) makes 0.6 stick only ~47 % motor - the first try on the whoop "pushed it to the other side" instead of flipping it (2026-10-01 night)
constexpr uint32_t FLY_FLIP_PUSH_MS       = 350;    // ... a jab this long ...
constexpr uint32_t FLY_FLIP_PAUSE_MS      = 650;    // ... then centred this long - long enough for a fresh attitude frame over the slow ELRS telemetry - before the next push is chosen from what that one did (FLY_FLIP_GAIN_DEG)
constexpr uint32_t FLY_FLIP_MAX_MS        = 9000;   // the whole thing, or it gives up
constexpr uint32_t LASER_SNAP_OPEN_MS     = 1000;   // the heading snap (a big error after a power-up) needs the hand open and relaxed this long (2 s before; the first arms of a session were waiting on it)
// Tenth round (2026-10-01, night) - Toby: the flip-over "tried: two motors, two, two, two on each side, it never flipped,
// it pushed it to the other side ... pump up the power"; "make the thumb go back to two, just like big movements -
// three is a little too much and I need to do it quick". Also the move to the min_spiffs partition table over USB.
constexpr uint8_t  FLY_FLIP_MAX_PUSHES    = 6;      // pushes before it gives up
constexpr float    FLY_FLIP_GAIN_DEG      = 15.0f;  // a push that brought the drone's attitude this much nearer level is repeated the same way; otherwise the other way, then the other axis (the pause after each push is long enough for a fresh attitude frame)
// Eleventh round (2026-10-01, late night) - Toby: after a battery change "I tried to arm it again and failed about eight
// times, then gave up". The board had been power-cycled before I looked, so every record was gone. Two answers: the arm
// records now live in NVS, and while the drone says '!' the board asks Betaflight WHY over MSP-in-CRSF.
constexpr uint32_t CRSF_MSP_EVERY_MS      = 1500;   // ask the FC for MSP_STATUS (its arming-disable flags) this often while its mode string says '!'
constexpr uint32_t CRSF_MSP_FLAGS_FRESH_MS = 6000;  // flags older than this are not quoted
constexpr bool     LIDAR_SLEEP_IDLE       = true;   // LIDAR-Lite v3: POWER_CONTROL (0x65) bit 2 = sleep and bit 0 = receiver off between measurements instead of the factory always-on (the v4 had POWER_MODE 0xE2 for this) (2026-10-01 night, "it's still firing ... the whole point was to save battery"); with LIDAR_WITH_LASER_ONLY that is most of the time. Volatile in the sensor: written every time it is found
// Twelfth round (2026-10-03) - Toby changed the whoop's frame, mounted the FC the wrong way round ("my bad"), and "sometimes
// it literally just can't arm - it might be my finger sensors". A part-way fist is recorded as a refused try now.
constexpr float    FLY_WEAK_FIST_CURL     = 0.25f;  // fingers at least this closed count as a fist attempt ...
constexpr float    FLY_WEAK_FIST_DROP     = 0.12f;  // ... that came out by this much ...
constexpr float    FLY_WEAK_FIST_BACK     = 0.04f;  // ... and closed back to within this of where it was: "the fist only got to N %" is recorded (a real fist is FLY_CLOSED_CURL)
// The heading healer, 2026-10-03 (Toby on step 7: "when I yaw left it would just snap back to the middle"): the fine healer
// erased a held turn within a second. Now LASER_HEAL_GAIN only inside LASER_HEAL_FAST_MS of boot / a snap / step 1 (the first
// arms of a session need it); after that LASER_HEAL_GAIN_SLOW capped at LASER_HEAL_SLOW_MAX_DEG per 100 ms - drift speed - and
// nothing at all while step 7 runs.
constexpr uint32_t LASER_HEAL_FAST_MS     = 20000;
constexpr float    LASER_HEAL_GAIN_SLOW   = 0.01f;   // per 100 ms, of the angle still to take out ...
constexpr float    LASER_HEAL_SLOW_MAX_DEG = 0.05f;  // ... never more than this per 100 ms (0.5 deg/s; the two headings drift apart a few degrees a minute)
// Thirteenth round (2026-10-03) - Toby: "make it super easy to fly, anyone can fly it: somewhere in the middle is hovering,
// fingers crunched is lowering, fingers up is raising, all relatively slowly ... the only thing you need is the hover number;
// do a test where I hover it as best I can and you average it".
constexpr float    FLY_HOVER_BAND         = 0.15f;  // the middle of the lift, this much each side, is exactly the hover number: finger wander there does nothing
constexpr float    FLY_FIST_FACTOR        = 0.88f;  // a fist (the bottom of the lift) asks for this x hover (never above thrfloor): a slow descent
constexpr float    FLY_HOVER_SKIP_HEAD_S  = 3.0f;   // a flight's throttle is averaged for the hover number without its first ...
constexpr float    FLY_HOVER_SKIP_TAIL_S  = 2.0f;   // ... and last seconds (take-off, landing)
constexpr float    FLY_HOVER_BLEND        = 0.30f;  // an ordinary flight moves the hover number this fraction of the way to its middle average; a HOVER TEST (/fly?hovertest=1, the page's button) takes it outright
// 2026-10-03, after the hover test: "it was kind of tilting left the whole time" and "it randomly turned off at first".
constexpr bool     FLY_TRIM_AT_LIFT        = true;   // the neutral sticks are the hand's pose at LIFT-OFF (the flying pose), not at the arming fist: the fist drops the wrist, the open hand came up 45 deg in the test flight - a steady pitch command the whole time
constexpr uint16_t GESTURE_THUMB_SWING_WARN_MV = 80; // a thumb calibration with less swing than this between flat and closed is logged as wrong (his thumb moves ~190 when the sensor sits right)
// Fourteenth round (2026-10-03) - Toby: "every time I try to arm there is a different reason ... five things have to go
// perfectly right ... lower that threshold, and give a big reason on the simulator"; "give a warning before it's too low -
// super fast cutting in and out of the throttle, bup bup, and the light; then once it gets lower, lower the drone, keep going
// until it's touching the ground"; "can you tell the temperature of the drone, and display all the details you get".
// Arming (values above): the trims are taken at lift-off now, so the arming pose only has to be a hand held out - level
// within 60 deg, wrist within 35 / 35 / 60; a fist from 40 % curl, out to 35 %, back within 1.5 s; the drone within 35 deg.
constexpr float    FLY_WARN_DIP            = 0.06f;  // GETTING LOW (FLY_CRAFT_WARN_V held FLY_CRAFT_LOW_MS): two dips of this much throttle, straight on the output ...
constexpr uint32_t FLY_WARN_DIP_MS         = 120;    // ... each this long ...
constexpr uint32_t FLY_WARN_DIP_GAP_MS     = 220;    // ... this far apart ...
constexpr uint32_t FLY_WARN_EVERY_MS       = 10000;  // (4 s until 2026-10-05: the warning started 8 s into a flight on a tired pack and dipped through all of it - "it was kind of fighting me")   // ... every this often while it lasts - "bup bup" - and the ring blinks red slowly
constexpr uint16_t PALM_WARN_BLINK_MS      = 700;    // the slow red blink (LOW keeps the 300 ms one)
constexpr float    FLY_LOW_DESCEND_S       = 2.0f;   // LOW (FLY_CRAFT_LOW_V held): the throttle cap eases from hover to the fist throttle over this ...
constexpr float    FLY_LOW_HOLD_S          = 4.0f;   // (unused since 2026-10-03 evening: LOW holds the descent until he disarms - a timed cut to zero can drop it from height)
constexpr float    FLY_LOW_OFF_S           = 2.0f;   // (unused, see FLY_LOW_HOLD_S)
constexpr uint32_t CRSF_MSP_POLL_MS        = 2000;   // MSP_STATUS from the FC this often whenever it is linked (cycle time, load, the flags; every CRSF_MSP_EVERY_MS while it says '!')
// Fifteenth round (2026-10-03, evening) - Toby: "out of nowhere it was high up and fell out of the sky and slammed into the
// floor; after that it wiggles and wobbles". The board had one throttle CUT on record for that session: a tight fist held
// 1.2 s = motors idle, which is the same motion as "come down", and the looser arming fist of the fourteenth round made it
// reachable (the cut needed a fist 8 % tighter than the arming one).
constexpr bool     FLY_THR_CUT_ENABLED     = false;  // the in-flight throttle cut is gone: a fist is a slow descent, never a drop; land, then two thumb presses
constexpr float    DISARM_FINGERS_STILL    = 0.25f;  // two thumb presses disarm only if the four fingers' mean curl moved less than this DURING EACH PRESS (0.20 across the whole pair, gap included, until 2026-10-03 evening - throttle moves between the presses made it "a little more difficult" to disarm): closing the hand drags the thumb's sensor 200+ mV, which must never read as two presses
constexpr float    LASER_SNAP_MIN_GAIN_DEG = 20.0f;  // (unused since 2026-10-03 evening: a gain is not evidence - snaps of 90-165 deg with 110+ deg left over passed it while the glove lay around)
constexpr float    LASER_SNAP_GOOD_DEG     = 20.0f;  // ... or leaves less than this: +-25 deg snaps that improved 55 -> 49 alternated every few seconds - bias from the relaxed wrist's real twist, which no heading shift can fix
// Sixteenth round (2026-10-03, evening) - Toby: "it was actually the software that was wobbling it - make sure that doesn't
// happen again"; "a little more difficult to turn it off, which is scary, but it also must not turn off while flying".
constexpr bool     FLY_SOLVE_HEADING       = true;   // at arming and at lift-off the forearm-hand heading is re-solved so the wrist's side angle is zero (the hand is held out in line then): flight never depends on the drift healer
constexpr float    FLY_HEADING_SOLVE_MIN_GAIN = 0.5f; // ... unless the side angle moves less than this per degree of heading (forearm steep or rolled): then the old heading stays
constexpr float    LASER_HEAL_POSE_DEG     = 30.0f;  // the drift healer (fine and snap) works only with the forearm AND the hand within this of their step-1 tilt - arm out, hand flat, measured by gravity, so no heading error can fake it; anywhere else a relaxed wrist's twist or a glove lying on the desk was turned into heading errors of up to 165 deg

// =====================================================================================================
// HW_FAKE: the clock the modules read is the hardware layer's, so the self-test can move time forward
// in steps instead of waiting (hal.h). Real builds compile to Arduino's millis()/delay() untouched.
// =====================================================================================================
#if HW_FAKE
uint32_t halMillis();
void     halDelay(uint32_t ms);
#define millis() halMillis()
#define delay(ms) halDelay(ms)
#endif

// ---------------- the glove's own battery on the palm ring (palm.cpp's warning blink, from the HandBench LBO) ----------
// Not used for the pack since 2026-10-08 (LOW is shown on the puck and the OLED only); palmWarning() keeps the blink.
constexpr uint32_t LBO_BLINK_MS    = 1000;   // the ring blinks white 1 s on / 1 s off ...
constexpr uint8_t  LBO_BLINK_LEVEL = 5;      // ... at 5/10 of the cap (15 % of full, ~0.19 A): the warning must not deepen the sag

// ---------------- the ELRS module's TX power, locked (design/01-power.md section 4) ------------------------------------
// The Max Power field's options on the BetaFPV 2G4Nano run 10;25;50;100;250;500 mW (ELRS 3.x). CRSF_AUTO_DYNAMIC
// sets the cap with Dynamic power on; key Q (lowest, for flashing) and "+" (one notch, never past the cap) remain:
// 250 mW starts the fan and 0.5 A, and the module's RF next to the ESP32's antenna jams the bench WiFi.
constexpr uint16_t CRSF_POWER_MAX_MW = 100;
constexpr uint8_t  CRSF_POWER_MAX_IDX_FALLBACK = 3;   // if the option text cannot be read: index 3 of 10;25;50;100;...

// ---------------- mode switch fitted? -----------------------------------------------------------------------------------
// true: the three-position switch on 36/39 decides SAFE / SHOW / FULL (the middle / the GPIO 36 end / the GPIO 39 end). false (a bench without
// the switch): the firmware boots in SHOW and the key M / GET /mode?force= chooses. With the switch fitted but
// a pole reading garbage (floating pin), the boot report says so.
constexpr bool     MODE_SWITCH_FITTED = true;
constexpr Mode     MODE_DEFAULT       = Mode::Show;

// =====================================================================================================
// Compile-time checks on the tables above: a pin booked twice, a strap / flash / console pin, or an
// input-only pin declared as an output stops the build here, before design/check.ps1 ever runs.
// =====================================================================================================
constexpr bool pinForbidden(uint8_t g) {   // straps 0 2 5 12 15, flash 6-11, console 1 3, and the numbers the WROOM-32 does not bring out (20, 24, 28-31, 37, 38)
  return g == 0 || g == 1 || g == 2 || g == 3 || g == 5 || (g >= 6 && g <= 12) || g == 15 || g == 20 || g == 24 || (g >= 28 && g <= 31) || g == 37 || g == 38 || g > 39;
}
constexpr bool pinInputOnly(uint8_t g) { return g >= 34 && g <= 39; }
constexpr bool pinsValid() {
  for (uint8_t i = 0; i < PIN_COUNT; i++) {
    if (pinForbidden(PINS[i].gpio)) return false;
    if (pinInputOnly(PINS[i].gpio) && (PINS[i].dir == PinDir::Out || PINS[i].dir == PinDir::InOut)) return false;
    for (uint8_t j = i + 1; j < PIN_COUNT; j++) if (PINS[i].gpio == PINS[j].gpio) return false;
  }
  return true;
}
constexpr bool pinListed(uint8_t g) { for (uint8_t i = 0; i < PIN_COUNT; i++) if (PINS[i].gpio == g) return true; return false; }
static_assert(pinsValid(), "config.h PINS[]: a pin is double-booked, forbidden (strap / flash / console), or an input-only pin is an output");
static_assert(pinListed(PIN_I2C1_SDA) && pinListed(PIN_I2C1_SCL) && pinListed(PIN_I2C2_SDA) && pinListed(PIN_I2C2_SCL) && pinListed(PIN_PCA_OE) &&
              pinListed(PIN_CRSF) && pinListed(PIN_ARM_LED) && pinListed(PIN_PALM_LED) && pinListed(PIN_VBAT) && pinListed(PIN_FLEX_THUMB) &&
              pinListed(PIN_S3_TX) && pinListed(PIN_S3_RX) && pinListed(PIN_BUZZER) && pinListed(PIN_MODE_A) && pinListed(PIN_MODE_B) &&
              pinListed(PIN_ACCENT) && pinListed(PIN_IMU_INT_ARM) && pinListed(PIN_IMU_INT_HAND), "config.h: every PIN_ constant must appear in PINS[]");
static_assert(!pinListed(PIN_SPARE), "config.h: PIN_SPARE is in use somewhere");
constexpr bool i2cValid() {
  for (uint8_t i = 0; i < I2C_DEVICE_COUNT; i++)
    for (uint8_t j = i + 1; j < I2C_DEVICE_COUNT; j++)
      if (I2C_DEVICES[i].bus == I2C_DEVICES[j].bus && I2C_DEVICES[i].addr == I2C_DEVICES[j].addr) return false;
  return true;
}
static_assert(i2cValid(), "config.h I2C_DEVICES[]: two devices share an address on one bus");
constexpr bool pcaValid() {
  for (uint8_t i = 0; i < PCA_CHANNEL_COUNT; i++) {
    if (PCA_CHANNELS[i].ch > 15) return false;
    for (uint8_t j = i + 1; j < PCA_CHANNEL_COUNT; j++) if (PCA_CHANNELS[i].ch == PCA_CHANNELS[j].ch) return false;
  }
  for (uint8_t i = 0; i < PLATE_COUNT; i++) if (PLATES[i].ch >= SERVO_COUNT) return false;
  return true;
}
static_assert(pcaValid(), "config.h PCA_CHANNELS[] / PLATES[]: a channel is used twice or a servo is on a static channel");
constexpr bool pucksValid() {
  for (uint8_t g = 0; g < PUCK_GROUPS; g++) {
    bool listed = false;
    for (uint8_t i = 0; i < PCA_CHANNEL_COUNT; i++) if (PCA_CHANNELS[i].ch == PCA_CH_PUCK_GROUP[g] && !PCA_CHANNELS[i].servo) listed = true;
    if (!listed || PCA_CH_PUCK_GROUP[g] < SERVO_COUNT) return false;
    bool used = false;
    for (uint8_t p = 0; p < PUCK_COUNT; p++) if (PUCK_GROUP_OF[p] == g) used = true;
    if (!used) return false;
  }
  for (uint8_t p = 0; p < PUCK_COUNT; p++) if (PUCK_GROUP_OF[p] >= PUCK_GROUPS) return false;
  return true;
}
static_assert(pucksValid(), "config.h: a puck group channel is not a static PCA_CHANNELS[] row, a group has no puck, or a puck has no group");
static_assert(BATT_ARM_FLOOR_V > BATT_LAND_WARN_V && BATT_LAND_WARN_V > BATT_LOW_V && BATT_LOW_V > BATT_IDLE_V && BATT_IDLE_V > BATT_CRITICAL_V, "config.h: battery thresholds out of order");
static_assert(LED_BRIGHTNESS_CAP <= 80, "config.h: LED cap above 30 % - 119 pixels white at 100 % is 7 A on a 5 A rail");
static_assert(SERVO_US_MIN >= 600 && SERVO_US_MAX <= 2400, "config.h: servo envelope wider than the measured 600-2400 us");

// ---------------- BNO085 INT lines (design 03 section 3: optional, recommended) --------------------------------------
// true: the SparkFun driver reads a sensor only when its INT line is low (no SCL stretching, no 13 ms waits).
// A sensor that stays quiet with the INT pin in use is re-started in polling mode by itself (the line not wired).
constexpr bool IMU_USE_INT = true;
