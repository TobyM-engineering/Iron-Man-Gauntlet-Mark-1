// GauntletLink - the line protocol between the arm's main ESP32 (Ironman/) and the round display, "the puck"
// (Waveshare ESP32-S3-Touch-LCD-1.28), over UART at 115200 baud 8N1, both directions. THE ARM REPO OWNS THIS
// FILE; the puck copies it verbatim. It compiles for the ESP32, the ESP32-S3 and a PC (lib/GauntletLink/test).
//
// Wiring (design/04-wiring.md sections 3 and 6): arm GPIO 18 (UART2 TX) -> puck header pin 8 = GPIO 16 (the
// puck's RX); puck header pin 7 = GPIO 15 (the puck's TX) -> arm GPIO 19 (UART2 RX); a ground wire from the
// arm's ESP32 ground bus to puck header pin 1, twisted with the two. NOT GPIO 43/44: those are the puck's
// UART0, wired to its CH343P USB-serial chip (flashing and its console).
//
// A frame is one ASCII line:   $<type>,<field>,<field>,...*<CCCC>\n
//   <type>  one letter (below); <field>s are decimal numbers, RRGGBB hex colours or short strings with no
//           comma, star or dollar; <CCCC> is the CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) of everything
//           between '$' and the LAST '*', as four upper-case hex digits. A line with a wrong CRC, no '*', more than
//           MAX_LINE bytes, or cut short by the next '$' is dropped and counted (Parser).
//
// PROTOCOL VERSION: every arm -> puck frame carries VERSION (below) and the puck's heartbeat carries its own.
// A frame from another version is REJECTED and counted (Result::BadVersion), never read field by field: a
// mismatch shows as "protocol vN vs vM", never as wrong numbers. v1 = the 2026-10-05 frames ($S without a
// version); v2 = 2026-10-07: the version fields, $G, plates state 5, CRC-16 (four digits) instead of CRC8.
// Still v2, compatible additions: $S flags bit5 = the CRITICAL shutdown (2026-10-07 evening); bit6 = THERMAL and
// the $E event line (2026-10-07 night) - a puck built on an older copy ignores the bit and the unknown line type.
//
// arm -> puck
//   $S  every 100 ms (10 Hz): the status - see Status below, field by field
//   $G  every 50 ms (20 Hz): the live state for the puck's model of the arm - see Live below, field by field
//   $E  when it happens: one gesture or action event, DONE, BLOCKED (the mode switch; a laser in flight) or REFUSED - see Event below
// puck -> arm
//   $H,seq,uptime_s,page,mlx,oled,fps,heap,ver*CCCC every 1000 ms: its health (mlx / oled 0/1 present, the LCD's
//                                                   fps, free heap) and its protocol version; 3 s without one =
//                                                   "puck link down" on the arm
//   $T,seq,kind,x,y*CCCC                            a touch: kind = TAP DOUBLE LONG UP DOWN LEFT RIGHT PRESS
//                                                   RELEASE; x, y 0..239. Touch is view-only (2026-10-07): the
//                                                   arm only counts and logs it - sending it is optional
//   $L,text*CCCC                                    a log line for the arm's console (no commas in text)
//   $A,seq*CCCC                                     acknowledges a status frame (optional, ignored)
//
// Bandwidth at 115200 baud (11 520 bytes/s each way): $G is at most G_MAX_LEN bytes (the host test builds the
// longest one) x 20 Hz, $S at most S_MAX_LEN x 10 Hz - together under half of the line; the puck's direction
// carries about 60 bytes a second. A frame never waits for room: the arm drops it and counts the drop.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

namespace glink {

static constexpr uint8_t  VERSION    = 2;
static constexpr size_t   MAX_LINE   = 256;     // the longest line either side accepts, '$' to '\n'
static constexpr uint8_t  MAX_FIELDS = 64;      // fields() splits at most this many
static constexpr uint32_t BAUD       = 115200;
static constexpr uint32_t S_PERIOD_MS = 100;    // $S: 10 Hz
static constexpr uint32_t G_PERIOD_MS = 50;     // $G: 20 Hz
static constexpr uint32_t H_PERIOD_MS = 1000;   // $H: 1 Hz
static constexpr size_t   S_MAX_LEN  = 80;      // the longest $S line (host test: every field at its widest - 76)
static constexpr size_t   G_MAX_LEN  = 250;     // the longest $G line (host test: every field at its widest - 247)

// The frame check: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) - every 1, 2 and 3-bit error and every burst up to
// 16 bits in a line of MAX_LINE bytes is caught (v1's CRC8 missed some two-bit errors in a line this long).
inline uint16_t crc16(uint16_t crc, uint8_t a) {
  crc ^= (uint16_t)a << 8;
  for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}
inline uint16_t crcOf(const char* body, size_t n) { uint16_t c = 0xFFFF; for (size_t i = 0; i < n; i++) c = crc16(c, (uint8_t)body[i]); return c; }
// v1's check (CRC8 DVB-S2, two hex digits) - only to recognise a v1 peer and say so
inline uint8_t crc8(uint8_t crc, uint8_t a) { crc ^= a; for (uint8_t i = 0; i < 8; i++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1); return crc; }
inline bool upperHex(char c) { return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'); }

// Wraps a body ("S,1,2,...") into "$body*CCCC\n". Returns the length written (0 = did not fit in cap or in MAX_LINE).
inline size_t frame(char* out, size_t cap, const char* body) {
  size_t n = strlen(body);
  if (n + 8 > cap || n + 8 > MAX_LINE) return 0;
  uint16_t c = crcOf(body, n);
  int w = snprintf(out, cap, "$%s*%04X\n", body, (unsigned)c);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}

// Feeds bytes one at a time; when a whole frame with a good CRC has arrived, feed() returns true and body() is the
// text between '$' and '*' (type letter first); fields() splits it. Every byte is O(1): a flood of garbage costs
// a few microseconds a byte and can never block, grow memory or wedge the parser (the next '$' starts afresh).
class Parser {
 public:
  bool feed(char ch) {
    if (ch == '$') { if (open) truncated++; len = 0; starAt = 0; open = true; return false; }   // a frame cut short by the next one
    if (!open) { stray++; return false; }
    if (ch == '\n' || ch == '\r') {
      open = false;
      buf[len] = 0;                                        // the CRC digits are parsed from here; stale bytes of a longer earlier frame sit beyond len
      if (len < 3 || !starAt) { bad++; return false; }     // the LAST '*' (a field may hold one: the FC mode "ANGL*")
      size_t digits = len - starAt, bodyLen = starAt - 1;
      for (size_t i = starAt; i < len; i++) if (!upperHex(buf[i])) { bad++; return false; }
      if (digits == 2) {                                   // a v1 line: recognised and counted, never accepted
        uint8_t c8 = 0; for (size_t i = 0; i < bodyLen; i++) c8 = crc8(c8, (uint8_t)buf[i]);
        if (c8 == (uint8_t)strtoul(buf + starAt, nullptr, 16)) v1++; else bad++;
        return false;
      }
      if (digits != 4) { bad++; return false; }
      if (crcOf(buf, bodyLen) != (uint16_t)strtoul(buf + starAt, nullptr, 16)) { bad++; return false; }
      buf[bodyLen] = 0; good++;
      return true;
    }
    if (len >= MAX_LINE - 2) { open = false; tooLong++; return false; }   // longer than any frame: dropped, wait for the next '$'
    buf[len++] = ch;
    if (ch == '*') starAt = len;
    return false;
  }
  const char* body() const { return buf; }
  char type() const { return buf[0]; }
  // Splits the body in place into fields (the type letter is field 0). Returns the count (at most max).
  uint8_t fields(const char** out, uint8_t max) {
    uint8_t n = 0; char* p = buf;
    while (p && n < max) { out[n++] = p; char* c = strchr(p, ','); if (c) { *c = 0; p = c + 1; } else p = nullptr; }
    return n;
  }
  uint32_t dropped() const { return bad + truncated + tooLong + v1; }
  uint32_t good = 0, bad = 0, stray = 0, truncated = 0, tooLong = 0, v1 = 0;   // v1 = whole lines from a protocol-v1 peer (two-digit CRC8)
 private:
  char buf[MAX_LINE] = { 0 }; size_t len = 0, starAt = 0; bool open = false;
};

// What a parse helper says about a frame whose CRC was good.
enum class Result : uint8_t { Ok, NotThisType, BadVersion, BadFields };
inline const char* resultName(Result r) { switch (r) { case Result::Ok: return "ok"; case Result::NotThisType: return "other type"; case Result::BadVersion: return "wrong protocol version"; default: return "wrong field count or value"; } }

// Strict field readers: a field must be a whole number (or RRGGBB hex) in range, else the frame is BadFields.
inline bool toLong(const char* s, long lo, long hi, long& v) { if (!s || !*s) return false; char* e; long x = strtol(s, &e, 10); if (*e || x < lo || x > hi) return false; v = x; return true; }
inline bool toHex24(const char* s, uint32_t& v) { if (!s || strlen(s) != 6) return false; for (int i = 0; i < 6; i++) if (!isxdigit((unsigned char)s[i])) return false; v = (uint32_t)strtoul(s, nullptr, 16); return true; }

// Touch event kinds (the puck sends the word; the arm maps it back)
enum class Touch : uint8_t { None, Tap, Double, Long, Up, Down, Left, Right, Press, Release };
inline const char* touchName(Touch t) {
  switch (t) { case Touch::Tap: return "TAP"; case Touch::Double: return "DOUBLE"; case Touch::Long: return "LONG"; case Touch::Up: return "UP"; case Touch::Down: return "DOWN";
               case Touch::Left: return "LEFT"; case Touch::Right: return "RIGHT"; case Touch::Press: return "PRESS"; case Touch::Release: return "RELEASE"; default: return "NONE"; }
}
inline Touch touchFrom(const char* s) {
  for (uint8_t k = 1; k <= 9; k++) if (strcmp(s, touchName((Touch)k)) == 0) return (Touch)k;
  return Touch::None;
}

// ---- $S, the status, 10 Hz ------------------------------------------------------------------------------------
//   $S,seq,mode,mv,bstate,plates,ringi,ringlvl,armed,lq,cv100,fcmode,lidar,laser,pucks,flags,page,ver*CCCC
//      seq      frame counter 0..9999
//      mode     0 SAFE 1 SHOW 2 FULL (the switch's three positions; the arm's code calls 2 "FLIGHT" - show it as FULL)
//      mv       the glove pack, millivolts (0..9999)
//      bstate   0 ok 1 arm floor (< 7.5 V; 7.2 until 2026-10-08) 2 LOW (< 7.0) 3 IDLE (< 6.8, forced idle) 4 CRITICAL (< 6.6) 5 monitor FAULT 6 no pack (USB / 5 V bench)
//      plates   0 unknown 1 open 2 closed 3 opening 4 closing 5 mixed (some open, some closed - a plate moved alone)
//      ringi    the palm ring's colour index (PALETTE below); ringlvl 0..10 (0 = off)
//      armed    0 / 1 (the drone)
//      lq       the drone link's uplink quality 0..100 (0 = no link)
//      cv100    the drone's cell voltage x 100 (0 = none)
//      fcmode   the drone FC's mode string ("ANGL*" ready, "ANGL!" refusing, "ANGL" armed) or - when unseen
//      lidar    millimetres, 0 = none
//      laser    bit0 red on, bit1 green on
//      pucks    the LED drivers lit: bit i = LED driver i+1 (its group is on), bit7 = any group on
//      flags    bit0 low-battery warning, bit1 thermal page wanted, bit2 firmware update in progress, bit3 drone flipping, bit4 bench test running,
//               bit5 SHUTDOWN (2026-10-07): the pack is CRITICAL and everything is off - blank the screen (backlight off, then
//               sleep if the puck can); the arm goes into deep sleep about 1.5 s later and sends nothing more
//               bit6 THERMAL (2026-10-07 night): the thermal camera is deployed (the hand plate is up) - read the
//               MLX90640 and show its image full screen while set; clear = stop reading it, back to the 3D view.
//               (bit1 "thermal page wanted" is never set by this arm; bit6 replaces it.)
//      page     the page the arm asks the puck to show: 0 leave as is, 1 status, 2 thermal, 3 boot, 4 flight
//      ver      VERSION
struct Status {
  uint16_t seq = 0; uint8_t mode = 0; uint16_t mv = 0; uint8_t bstate = 0, plates = 0, ringi = 0, ringlvl = 0, armed = 0, lq = 0;
  uint16_t cv100 = 0; char fcmode[12] = "-"; uint16_t lidar = 0; uint8_t laser = 0, pucks = 0, flags = 0, page = 0;
};
static constexpr uint8_t S_FIELDS = 18;   // with the type letter
inline size_t statusBody(char* out, size_t cap, const Status& s) {
  char fc[12]; size_t k = 0; for (const char* p = s.fcmode; *p && k < sizeof fc - 1; p++) if (*p != ',' && *p != '$' && *p != '\n' && *p != '\r') fc[k++] = *p; fc[k] = 0;
  int w = snprintf(out, cap, "S,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%s,%u,%u,%u,%u,%u,%u", s.seq, s.mode, s.mv, s.bstate, s.plates, s.ringi, s.ringlvl, s.armed, s.lq, s.cv100, k ? fc : "-", s.lidar, s.laser, s.pucks, s.flags, s.page, VERSION);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}
inline Result statusParse(const char** f, uint8_t n, Status& s) {
  if (!n || f[0][0] != 'S' || f[0][1]) return Result::NotThisType;
  long v; if (n != S_FIELDS) return n < S_FIELDS ? Result::BadVersion : Result::BadFields;   // a v1 $S had no version field
  if (!toLong(f[17], 0, 255, v) || v != VERSION) return Result::BadVersion;
  Status r; long x[17];
  static const long hi[17] = { 0, 9999, 2, 65535, 6, 5, 255, 10, 1, 100, 65535, 0, 65535, 3, 255, 255, 4 };
  for (uint8_t i = 1; i <= 16; i++) { if (i == 11) continue; if (!toLong(f[i], 0, hi[i], x[i])) return Result::BadFields; }
  r.seq = (uint16_t)x[1]; r.mode = (uint8_t)x[2]; r.mv = (uint16_t)x[3]; r.bstate = (uint8_t)x[4]; r.plates = (uint8_t)x[5]; r.ringi = (uint8_t)x[6]; r.ringlvl = (uint8_t)x[7];
  r.armed = (uint8_t)x[8]; r.lq = (uint8_t)x[9]; r.cv100 = (uint16_t)x[10]; strncpy(r.fcmode, f[11], sizeof r.fcmode - 1); r.fcmode[sizeof r.fcmode - 1] = 0;
  r.lidar = (uint16_t)x[12]; r.laser = (uint8_t)x[13]; r.pucks = (uint8_t)x[14]; r.flags = (uint8_t)x[15]; r.page = (uint8_t)x[16];
  s = r; return Result::Ok;
}

// ---- $G, the live state, 20 Hz (v2) -----------------------------------------------------------------------------
// Everything the puck needs to draw the arm as it is right now, in one line. Field by field (index = field number
// after the type letter G; ranges are what the arm sends, the parser rejects anything outside them):
//   1  ver        VERSION
//   2  seq        frame counter 0..65535, wraps
//   3  t          the arm's clock, ms since its boot (0..4294967295): interpolate between frames with it
//   4-7  fq w,x,y,z  the FOREARM's orientation, quaternion x 10000 (-10000..10000)
//   8-11 hq w,x,y,z  the HAND's orientation, quaternion x 10000
//                 Both are the simulator's "segments": each sensor's rotation from the step-1 pose (arm straight
//                 out, palm down, wrist in line - 1,0,0,0 = that pose) in the arm's body frame: X forward along the
//                 arm, Y to the left, Z up. Both are sent, not the relative one: the puck draws the forearm AND the
//                 hand, and the wrist is one multiply away (wrist = conj(fq) * hq). The two raw sensor quaternions
//                 would be useless to the puck - each BNO085 has its own arbitrary heading - while these are
//                 already in one shared frame (the arm solves the zero poses and the heading between the sensors).
//   12 imu        bit0 forearm sensor live, bit1 hand sensor live, bit2 zeroed by the sim page's step 1 (without it
//                 the quaternions are only relative to the power-up pose)
//   13-17 curl    each finger's bend, thumb, index, middle, ring, pinky: 0 (flat) .. 1000 (closed), from the
//                 calibration on the arm; -1 = not calibrated or no sensor
//   18-22 mv      the same five fingers' raw sensor millivolts 0..3300 (0 = no sensor) - falls as a finger bends
//   23-28 servo   each servo's position, 0 (closed) .. 1000 (open), of its own travel (the last pulse it was
//                 given; a released servo stays where it was): SERVO_NAMES order - plate 1, plate 2, plate 3 A,
//                 plate 3 B, hand plate A, hand plate B; -1 = never driven since boot (position unknown)
//   29 drv        bitmask, bit i = servo i is being driven (getting pulses) right now
//   30 plates     0 unknown 1 open 2 closed 3 opening 4 closing 5 mixed (as $S)
//   31 ring       the palm ring (19 pixels, one colour) as it shows right now: RRGGBB of the drive each pixel gets
//                 (brightness and the 30 % cap included, so 4D4D4D is the brightest white it ever shows)
//   32 ringset    the colour that is set, full scale RRGGBB (PALETTE)
//   33 ringlvl    0..10 (0 = off), tenths of the cap
//   34 ringmode   0 normal, 1 held dark (a servo is moving), 2 glove battery low (blinking white), 3 drone battery
//                 low (blinking red), 4 chain test (one pixel walking)
//   35 strip      the forearm strip (100 pixels): RRGGBB of its first lit pixel, as set (before brightness)
//   36 striplvl   its brightness 0..255 (never above the cap, 77)
//   37 striplit   how many of its pixels are lit 0..100
//   38 strippos   the first lit pixel 0..99 (0 = the elbow end), -1 = none
//   39 groups     the LED drivers' three groups, bit0 group 1 (the three 20 mm palm LEDs), bit1 group 2 (the two
//                 square palm LEDs), bit2 group 3 (the index and middle finger LEDs) - on / off only
//   40 accent     the three small red accent LEDs in the palm, 0 / 1
//   41 lasers     bit0 red (back of the hand), bit1 green (forearm pod)
//   42 lidar      the LiDAR's distance in mm 0..40000, 0 = none (it measures only while the red laser is on)
//   43 mode       0 SAFE 1 SHOW 2 FULL (as $S)
//   44 gev        the last gesture or event (GEV_*): 0 none, 1 thumb triple-crunch (ring), 2 pinky dip (brighter),
//                 3 index dip (dimmer), 4 red laser on (fist + wrist jolt), 5 drone armed, 6 drone disarmed,
//                 7 cascade started, 8 plate moved alone, 9 LED group burst
//   45 gn         a counter 0..65535 that steps once per event: a new value = a new event (animate it once)
//   46 hand       0 unknown, 1 open, 2 fist (middle, ring and pinky closed)
//   47 mv         the glove pack, millivolts 0..9999
//   48 bstate     as $S
//   49 ma         the servo branch's current from the INA219, mA 0..9999, -1 = no sensor
//   50 outs       bit0 vibration motor, bit1 buzzer sounding, bit2 a cascade running, bit3 bench test running,
//                 bit4 drone armed, bit5 mode forced by a key (not the switch), bit6 firmware update in progress
static constexpr uint8_t G_FIELDS = 51;   // with the type letter
static constexpr uint8_t SERVOS = 6;
static constexpr const char* SERVO_NAMES[SERVOS] = { "plate 1", "plate 2", "plate 3A", "plate 3B", "hand A", "hand B" };
static constexpr const char* FINGER_NAMES[5] = { "thumb", "index", "middle", "ring", "pinky" };
enum Gev : uint8_t { GEV_NONE, GEV_TRIPLE, GEV_DIP_UP, GEV_DIP_DOWN, GEV_LASER, GEV_ARMED, GEV_DISARMED, GEV_CASCADE, GEV_PLATE, GEV_BURST, GEV_COUNT };
// The palm ring's colours, by $S ringi (the arm's palm.cpp list, in order)
static constexpr uint8_t PALETTE_N = 9;
static constexpr const char* PALETTE_NAMES[PALETTE_N] = { "white", "repulsor", "red", "orange", "amber", "green", "cyan", "blue", "magenta" };

struct Live {
  uint16_t seq = 0; uint32_t t = 0;
  int16_t  fq[4] = { 10000, 0, 0, 0 }, hq[4] = { 10000, 0, 0, 0 }; uint8_t imu = 0;
  int16_t  curl[5] = { -1, -1, -1, -1, -1 }; uint16_t fmv[5] = { 0, 0, 0, 0, 0 };
  int16_t  servo[SERVOS] = { -1, -1, -1, -1, -1, -1 }; uint8_t drv = 0, plates = 0;
  uint32_t ring = 0, ringset = 0; uint8_t ringlvl = 0, ringmode = 0;
  uint32_t strip = 0; uint8_t striplvl = 0; uint8_t striplit = 0; int8_t strippos = -1;
  uint8_t  groups = 0, accent = 0, lasers = 0; uint16_t lidar = 0; uint8_t mode = 0;
  uint8_t  gev = 0; uint16_t gn = 0; uint8_t hand = 0;
  uint16_t mv = 0; uint8_t bstate = 0; int16_t ma = -1; uint8_t outs = 0;
};
inline int16_t q10k(float v) { if (!(v == v)) return 0; float s = v * 10000.0f; if (s > 10000.0f) s = 10000.0f; if (s < -10000.0f) s = -10000.0f; return (int16_t)lroundf(s); }
template <typename T> inline T clampTo(T v, long lo, long hi) { return (long)v < lo ? (T)lo : (long)v > hi ? (T)hi : v; }
// The encoder clamps every field into the range liveParse accepts, so the arm can never send a frame the puck must reject.
inline size_t liveBody(char* out, size_t cap, const Live& in) {
  Live g = in;
  for (int i = 0; i < 4; i++) { g.fq[i] = clampTo<int16_t>(g.fq[i], -10000, 10000); g.hq[i] = clampTo<int16_t>(g.hq[i], -10000, 10000); }
  g.imu &= 7; for (int i = 0; i < 5; i++) { g.curl[i] = clampTo<int16_t>(g.curl[i], -1, 1000); g.fmv[i] = clampTo<uint16_t>(g.fmv[i], 0, 3300); }
  for (int i = 0; i < SERVOS; i++) g.servo[i] = clampTo<int16_t>(g.servo[i], -1, 1000);
  g.drv &= 63; g.plates = clampTo<uint8_t>(g.plates, 0, 5); g.ringlvl = clampTo<uint8_t>(g.ringlvl, 0, 10); g.ringmode = clampTo<uint8_t>(g.ringmode, 0, 4);
  g.striplit = clampTo<uint8_t>(g.striplit, 0, 100); g.strippos = clampTo<int8_t>(g.strippos, -1, 99); g.groups &= 7; g.accent = g.accent ? 1 : 0; g.lasers &= 3;
  g.lidar = clampTo<uint16_t>(g.lidar, 0, 40000); g.mode = clampTo<uint8_t>(g.mode, 0, 2); g.gev = clampTo<uint8_t>(g.gev, 0, GEV_COUNT - 1); g.hand = clampTo<uint8_t>(g.hand, 0, 2);
  g.mv = clampTo<uint16_t>(g.mv, 0, 9999); g.bstate = clampTo<uint8_t>(g.bstate, 0, 6); g.ma = clampTo<int16_t>(g.ma, -1, 9999); g.outs &= 127;
  int w = snprintf(out, cap,
    "G,%u,%u,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%d,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%u,%u,%06lX,%06lX,%u,%u,%06lX,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%d,%u",
    VERSION, g.seq, (unsigned long)g.t, g.fq[0], g.fq[1], g.fq[2], g.fq[3], g.hq[0], g.hq[1], g.hq[2], g.hq[3], g.imu,
    g.curl[0], g.curl[1], g.curl[2], g.curl[3], g.curl[4], g.fmv[0], g.fmv[1], g.fmv[2], g.fmv[3], g.fmv[4],
    g.servo[0], g.servo[1], g.servo[2], g.servo[3], g.servo[4], g.servo[5], g.drv, g.plates,
    (unsigned long)(g.ring & 0xFFFFFF), (unsigned long)(g.ringset & 0xFFFFFF), g.ringlvl, g.ringmode,
    (unsigned long)(g.strip & 0xFFFFFF), g.striplvl, g.striplit, g.strippos,
    g.groups, g.accent, g.lasers, g.lidar, g.mode, g.gev, g.gn, g.hand, g.mv, g.bstate, g.ma, g.outs);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}
inline Result liveParse(const char** f, uint8_t n, Live& g) {
  if (!n || f[0][0] != 'G' || f[0][1]) return Result::NotThisType;
  long v;
  if (n < 2 || !toLong(f[1], 0, 255, v)) return Result::BadFields;
  if (v != VERSION) return Result::BadVersion;                       // checked before the count: another version may have other fields
  if (n != G_FIELDS) return Result::BadFields;
  Live r; uint8_t k = 2;
  auto num = [&](long lo, long hi, long& out) { return toLong(f[k++], lo, hi, out); };
  auto hex = [&](uint32_t& out) { return toHex24(f[k++], out); };
  if (!num(0, 65535, v)) { return Result::BadFields; } r.seq = (uint16_t)v;
  { char* e; unsigned long t = strtoul(f[k], &e, 10); if (!*f[k] || *e || f[k][0] == '-') { return Result::BadFields; } r.t = (uint32_t)t; k++; }
  for (uint8_t i = 0; i < 4; i++) { if (!num(-10000, 10000, v)) { return Result::BadFields; } r.fq[i] = (int16_t)v; }
  for (uint8_t i = 0; i < 4; i++) { if (!num(-10000, 10000, v)) { return Result::BadFields; } r.hq[i] = (int16_t)v; }
  if (!num(0, 7, v)) { return Result::BadFields; } r.imu = (uint8_t)v;
  for (uint8_t i = 0; i < 5; i++) { if (!num(-1, 1000, v)) { return Result::BadFields; } r.curl[i] = (int16_t)v; }
  for (uint8_t i = 0; i < 5; i++) { if (!num(0, 3300, v)) { return Result::BadFields; } r.fmv[i] = (uint16_t)v; }
  for (uint8_t i = 0; i < SERVOS; i++) { if (!num(-1, 1000, v)) { return Result::BadFields; } r.servo[i] = (int16_t)v; }
  if (!num(0, 63, v)) { return Result::BadFields; } r.drv = (uint8_t)v;
  if (!num(0, 5, v)) { return Result::BadFields; } r.plates = (uint8_t)v;
  if (!hex(r.ring) || !hex(r.ringset)) return Result::BadFields;
  if (!num(0, 10, v)) { return Result::BadFields; } r.ringlvl = (uint8_t)v;
  if (!num(0, 4, v)) { return Result::BadFields; } r.ringmode = (uint8_t)v;
  if (!hex(r.strip)) return Result::BadFields;
  if (!num(0, 255, v)) { return Result::BadFields; } r.striplvl = (uint8_t)v;
  if (!num(0, 100, v)) { return Result::BadFields; } r.striplit = (uint8_t)v;
  if (!num(-1, 99, v)) { return Result::BadFields; } r.strippos = (int8_t)v;
  if (!num(0, 7, v)) { return Result::BadFields; } r.groups = (uint8_t)v;
  if (!num(0, 1, v)) { return Result::BadFields; } r.accent = (uint8_t)v;
  if (!num(0, 3, v)) { return Result::BadFields; } r.lasers = (uint8_t)v;
  if (!num(0, 40000, v)) { return Result::BadFields; } r.lidar = (uint16_t)v;
  if (!num(0, 2, v)) { return Result::BadFields; } r.mode = (uint8_t)v;
  if (!num(0, GEV_COUNT - 1, v)) { return Result::BadFields; } r.gev = (uint8_t)v;
  if (!num(0, 65535, v)) { return Result::BadFields; } r.gn = (uint16_t)v;
  if (!num(0, 2, v)) { return Result::BadFields; } r.hand = (uint8_t)v;
  if (!num(0, 9999, v)) { return Result::BadFields; } r.mv = (uint16_t)v;
  if (!num(0, 6, v)) { return Result::BadFields; } r.bstate = (uint8_t)v;
  if (!num(-1, 9999, v)) { return Result::BadFields; } r.ma = (int16_t)v;
  if (!num(0, 127, v)) { return Result::BadFields; } r.outs = (uint8_t)v;
  g = r; return Result::Ok;
}

// ---- $E, one gesture or action event, arm -> puck, when it happens (2026-10-07 night, a compatible addition) -------
//   $E,seq,act,res,mode,need,ver*CCCC
//      seq   0..65535, steps once per $E (a jump = lines lost; each line is sent once)
//      act   the action the gesture (or a key) asks for - ACT_* below; a number the puck does not know yet is still
//            a valid line (show it as "gesture N")
//      res   0 DONE, 1 BLOCKED (the gesture was seen - the puck may show it - and the arm did nothing on purpose): by
//            the mode switch when mode < need, or - a laser with mode = FULL - because the drone is armed or was
//            disarmed under 1.5 s ago; 2 REFUSED for another reason (a cascade running, a rest time, the heat model);
//            3 BATTERY (2026-10-21): refused because the battery is too low for it (the LED groups' scaled floor, LOW)
//      mode  the mode at that moment, 0 SAFE 1 SHOW 2 FULL
//      need  the lowest mode the action needs, 0..2 (SHOW for the plates, lights and the camera, FULL for the
//            lasers and the drone)
//      ver   VERSION
// A puck built on a header that predates $E gets NotThisType from every parser for it and ignores the line.
static constexpr uint8_t FLAG_SHUTDOWN = 5, FLAG_THERMAL = 6;   // $S flags bits
// (2026-10-21) ONE mark on the puck's battery bar (Toby: "a little red bar ... really small but noticeable ... as simple as
// possible"), against $S mv: under POWER_FLOOR_MV the drone does not arm and the big LEDs do not all start at once. Smaller
// light loads still start below it: the arm scales each start's floor with the LEDs that would then be lit, from 7.0 V (none)
// to POWER_FLOOR_MV (all seven) - their sag grows with the load - and answers RES_BATTERY when that stops one. A burst already
// running is cut only under 7.0 V. The arm checks its config.h uses this number.
static constexpr uint16_t POWER_FLOOR_MV = 7500;
// (2026-10-21) the battery bar's two ends against $S mv, the round screen and the OLED alike (Toby: "make 8.4 actually 100 %" -
// his charger stops at 4.2 V a cell, so the LiHV packs' 8.7 V is never reached and a full pack showed ~85 %): BATT_EMPTY_MV
// = 0 % (the arm's CRITICAL shutdown), BATT_FULL_MV = 100 %, a straight line between, clamped. The arm checks the empty end.
static constexpr uint16_t BATT_EMPTY_MV = 6600, BATT_FULL_MV = 8400;
enum Act : uint8_t { ACT_NONE, ACT_RING_ON, ACT_RING_OFF, ACT_RING_UP, ACT_RING_DOWN, ACT_CASCADE, ACT_PLATE, ACT_BURST, ACT_FINGERS,
                     ACT_VIBRATE, ACT_THERMAL_ON, ACT_THERMAL_OFF, ACT_LASER_RED, ACT_LASER_GREEN, ACT_ARM, ACT_DISARM, ACT_FLIP,
                     ACT_ALL_OFF, ACT_LIDAR, ACT_CAL, ACT_PUCK_UPDATE, ACT_COUNT };   // new numbers only ever go on the end (ACT_ALL_OFF 17, ACT_LIDAR 18: 2026-10-20; ACT_CAL 19, ACT_PUCK_UPDATE 20: 2026-10-21 - the puck's control panel)
static constexpr const char* ACT_NAMES[ACT_COUNT] = { "none", "ring on / colour", "ring off", "ring brighter", "ring dimmer", "plates cascade",
  "one plate", "palm burst", "finger LEDs", "vibrator", "thermal camera on", "thermal camera off", "red laser", "green laser", "drone arm",
  "drone disarm", "flip-over", "all off", "lidar", "calibrate", "puck update" };
inline const char* actName(uint8_t a) { return a < ACT_COUNT ? ACT_NAMES[a] : "gesture"; }
enum Res : uint8_t { RES_DONE, RES_BLOCKED, RES_REFUSED, RES_BATTERY };
struct Event { uint16_t seq = 0; uint8_t act = 0, res = 0, mode = 0, need = 0; };
static constexpr uint8_t E_FIELDS = 7;   // with the type letter
inline size_t eventBody(char* out, size_t cap, const Event& e) {
  int w = snprintf(out, cap, "E,%u,%u,%u,%u,%u,%u", e.seq, e.act, e.res > 3 ? 3 : e.res, e.mode > 2 ? 2 : e.mode, e.need > 2 ? 2 : e.need, VERSION);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}
inline Result eventParse(const char** f, uint8_t n, Event& e) {
  if (!n || f[0][0] != 'E' || f[0][1]) return Result::NotThisType;
  long v;
  if (n < E_FIELDS || !toLong(f[E_FIELDS - 1], 0, 255, v)) return Result::BadFields;
  if (v != VERSION) return Result::BadVersion;
  if (n != E_FIELDS) return Result::BadFields;
  Event r; long x[5]; static const long hi[5] = { 65535, 255, 3, 2, 2 };
  for (uint8_t i = 0; i < 5; i++) if (!toLong(f[i + 1], 0, hi[i], x[i])) return Result::BadFields;
  r.seq = (uint16_t)x[0]; r.act = (uint8_t)x[1]; r.res = (uint8_t)x[2]; r.mode = (uint8_t)x[3]; r.need = (uint8_t)x[4];
  e = r; return Result::Ok;
}

// ---- $C, a button on the puck's control panel, puck -> arm (2026-10-20, Toby: "a screen that I could just activate
// anything with ... if gestures are down"; a compatible addition: an arm built before it counts the line as an unknown
// type and ignores it) ---------------------------------------------------------------------------------------------------
//   $C,seq,act,a,b,ver*CCCC
//      seq   0..65535, steps once per press; the arm acts once per seq (the same line twice is one press)
//      act   ACT_* (the same numbers as $E); a, b 0..65535 per act (2 = toggle wherever there is an on / off):
//              ACT_CASCADE      a 1 open, 0 close, 2 toggle (open unless every plate is open)
//              ACT_PLATE        a plate 1..4 (4 = the hand plate), b 1 open, 0 close, 2 toggle
//                               (2026-10-21) opening 2 / 3 opens what they need first (the hand plate; + plate 2), closing
//                               plate 2 / the hand plate closes what needs it first (plate 3; + plate 2) - one press, one $E
//              ACT_BURST        a LED group 1 (palm 20 mm x3) or 2 (palm 10 mm x2), 0 = both; b 1 on, 0 off, 2 toggle
//              ACT_FINGERS      a 1 on, 0 off, 2 toggle (LED group 3)
//              ACT_LASER_RED, ACT_LASER_GREEN   a 1 on, 0 off, 2 toggle (each laser's plate opens first, as always)
//              ACT_LIDAR        a 1 on, 0 off, 2 toggle (2026-10-20): the hand plate lifts (the LiDAR sits under it), then it
//                               measures continuously with no laser - the distance in $G / $S lidar as always; off = it
//                               stops, then the hand plate closes if nothing else holds it. The lasers' rules: FULL, not
//                               while the drone flies, an off never blocked
//              ACT_VIBRATE      a milliseconds / 10 (0 = 300 ms)
//              ACT_RING_ON      a 0 = the gesture's own (off -> on, white <-> red); a = N + 1 = on in the palm ring's
//                               palette colour N at its level (off -> the level it had before; the default the first
//                               time): white 0, repulsor 1, red 2, orange 3, amber 4, green 5, cyan 6, blue 7, magenta 8
//                               (2026-10-20); an N past the palette is REFUSED
//              ACT_CAL          a = the calibration step (2026-10-21), one press per step: 1 fingers flat (all five),
//                               2 fingers closed (also the thumb inside a fist - its gestures), 3 the thumb closed on its own,
//                               fingers out (its full curl - the model), 4 the arm straight out palm down (the zero pose of
//                               both IMUs), 5 the arm hanging straight down, 6 straight up (5 / 6: each sensor's forward axis
//                               along its limb). The puck skips 3 while there is no thumb sensor. The arm waits 0.6 s
//                               (the tap jolts it), averages 2 s, saves the step at once (NVS) and answers ONE $E: DONE, or
//                               REFUSED (a finger's swing too small or its sensor faulted, an IMU absent, a pose not vertical
//                               enough, 5 / 6 before a step 4, 3 with no thumb sensor, the drone armed, a step already
//                               running). Any mode: it moves
//                               nothing. The thumb never refuses a step. Allow ~3 s for the answer
//              ACT_PUCK_UPDATE  no arguments (2026-10-21): the puck is about to restart into its WiFi updater - the arm
//                               stops its RC frames (the ELRS module beside the puck's antenna kept the updater off WiFi)
//                               and starts them again when the puck's $H comes back (after at least 8 s) or after 5 min.
//                               REFUSED, frames kept, while the drone is armed or flipping. The puck sends it three times
//                               with one seq (the arm acts once) and will not see the $E (it is restarting)
//              ACT_THERMAL_ON, ACT_THERMAL_OFF, ACT_RING_OFF, ACT_RING_UP, ACT_RING_DOWN, ACT_DISARM,
//              ACT_ALL_OFF (lasers, LED groups, vibrator; the plates stay)   no arguments
//      ver   VERSION
// The arm answers EVERY command with one $E for the same act: DONE, BLOCKED (the mode switch is below its need, or a
// laser while the drone flies) or REFUSED (the battery, the heat model, a move running, a rest time). The rules are the
// gestures' own - a button never gets round the mode switch. ACT_ARM and ACT_FLIP are never taken from a button.
struct Command { uint16_t seq = 0; uint8_t act = 0; uint16_t a = 0, b = 0; };
static constexpr uint8_t C_FIELDS = 6;   // with the type letter
inline size_t commandBody(char* out, size_t cap, const Command& c) {
  int w = snprintf(out, cap, "C,%u,%u,%u,%u,%u", c.seq, c.act, c.a, c.b, VERSION);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}
inline Result commandParse(const char** f, uint8_t n, Command& c) {
  if (!n || f[0][0] != 'C' || f[0][1]) return Result::NotThisType;
  long v;
  if (n < C_FIELDS || !toLong(f[C_FIELDS - 1], 0, 255, v)) return Result::BadFields;
  if (v != VERSION) return Result::BadVersion;
  if (n != C_FIELDS) return Result::BadFields;
  Command r; long x[4]; static const long hi[4] = { 65535, 255, 65535, 65535 };
  for (uint8_t i = 0; i < 4; i++) if (!toLong(f[i + 1], 0, hi[i], x[i])) return Result::BadFields;
  r.seq = (uint16_t)x[0]; r.act = (uint8_t)x[1]; r.a = (uint16_t)x[2]; r.b = (uint16_t)x[3];
  c = r; return Result::Ok;
}

// ---- $H, the puck's heartbeat, 1 Hz (v2 adds its version as the last field) ------------------------------------
struct Heartbeat { uint16_t seq = 0; uint32_t uptimeS = 0; uint8_t page = 0, mlx = 0, oled = 0, fps = 0; uint32_t heap = 0; };
static constexpr uint8_t H_FIELDS = 9;   // with the type letter
inline size_t heartbeatBody(char* out, size_t cap, const Heartbeat& h) {
  int w = snprintf(out, cap, "H,%u,%lu,%u,%u,%u,%u,%lu,%u", h.seq, (unsigned long)h.uptimeS, h.page, h.mlx, h.oled, h.fps, (unsigned long)h.heap, VERSION);
  return w > 0 && (size_t)w < cap ? (size_t)w : 0;
}
inline Result heartbeatParse(const char** f, uint8_t n, Heartbeat& h, long* theirVersion = nullptr) {
  if (!n || f[0][0] != 'H' || f[0][1]) return Result::NotThisType;
  long v;
  if (n < H_FIELDS) { if (theirVersion) *theirVersion = 1; return Result::BadVersion; }   // a v1 heartbeat had no version
  if (!toLong(f[8], 0, 255, v)) return Result::BadFields;
  if (theirVersion) *theirVersion = v;
  if (v != VERSION) return Result::BadVersion;
  if (n != H_FIELDS) return Result::BadFields;
  Heartbeat r; long x;
  if (!toLong(f[1], 0, 65535, x)) { return Result::BadFields; } r.seq = (uint16_t)x;
  { char* e; unsigned long u = strtoul(f[2], &e, 10); if (!*f[2] || *e) { return Result::BadFields; } r.uptimeS = (uint32_t)u; }
  if (!toLong(f[3], 0, 255, x)) { return Result::BadFields; } r.page = (uint8_t)x;
  if (!toLong(f[4], 0, 1, x)) { return Result::BadFields; } r.mlx = (uint8_t)x;
  if (!toLong(f[5], 0, 1, x)) { return Result::BadFields; } r.oled = (uint8_t)x;
  if (!toLong(f[6], 0, 255, x)) { return Result::BadFields; } r.fps = (uint8_t)x;
  { char* e; unsigned long u = strtoul(f[7], &e, 10); if (!*f[7] || *e) { return Result::BadFields; } r.heap = (uint32_t)u; }
  h = r; return Result::Ok;
}

}  // namespace glink
