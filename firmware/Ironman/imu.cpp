#include "config.h"
#include "hal.h"
#include "imu.h"
#include "config.h"
#include "buses.h"
#include "flex.h"
#include "palm.h"
#include "gesture.h"
#include "lidar.h"
#include "laser.h"
#include "fly.h"
#include <WiFi.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <Preferences.h>
#include <GauntletBenchIO.h>

// ---------------- quaternions ---------------------------------------------------
// (w, x, y, z), Hamilton product. The BNO reports q = sensor frame -> world
// frame; the game rotation vector's world is Z up with an arbitrary heading
// that restarts whenever the sensor (re)starts. These same helpers will do
// "hand relative to forearm" on the gauntlet: d = q_hand * conj(q_arm).
struct Quat { float w, x, y, z; };

static Quat qmul(const Quat& a, const Quat& b) {
  return { a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
           a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
           a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
           a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
}
static Quat qconj(const Quat& q) { return { q.w, -q.x, -q.y, -q.z }; }
static Quat qnorm(Quat q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n > 1e-6f) { q.w /= n; q.x /= n; q.y /= n; q.z /= n; } else q = { 1, 0, 0, 0 };
  return q;
}
static Quat qAboutZ(float rad) { return { cosf(rad * 0.5f), 0, 0, sinf(rad * 0.5f) }; }
static Quat qTilt(float rollRad, float pitchRad) {              // R_y(pitch) * R_x(roll)
  Quat qy = { cosf(pitchRad * 0.5f), 0, sinf(pitchRad * 0.5f), 0 };
  Quat qx = { cosf(rollRad * 0.5f), sinf(rollRad * 0.5f), 0, 0 };
  return qmul(qy, qx);
}

// ZYX Euler angles in degrees in the BNO's native Z-up frame:
// q = R_z(yaw) * R_y(pitch) * R_x(roll). Signs: pitch + = the X axis dipping,
// yaw + = turning left (anticlockwise from above), roll + = right side down.
static void quatToEuler(const Quat& q, float& roll, float& pitch, float& yaw) {
  roll  = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
  float s = 2.0f * (q.w * q.y - q.z * q.x);
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  pitch = asinf(s);
  yaw   = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
  roll *= RAD_TO_DEG; pitch *= RAD_TO_DEG; yaw *= RAD_TO_DEG;
}

// ---------------- sensors ---------------------------------------------------------
struct Imu {
  BNO080   dev;
  bool     present = false;
  bool     useInt  = IMU_USE_INT;                 // read only when the INT line says there is data; falls back to polling if the line is not wired
  uint8_t  bus = 0, addr = 0;
  Quat     q = { 1, 0, 0, 0 };                    // latest orientation, raw
  float    rawRoll = 0, rawPitch = 0, rawYaw = 0; // native angles
  float    roll = 0, pitch = 0, yaw = 0;          // what the stream prints
  Quat     d = { 1, 0, 0, 0 };                    // the zeroed orientation those angles come from
  // the simulator's way (2026-09-28 night): the page sends its step-1 zero quaternion and, once steps 2/3 are done,
  // the sensor's forward axis (the limb's direction in sensor coordinates); body frame B = the heading of that
  // axis at the zero pose (X forward, Z up); seg = conj(B) * q * conj(qz) * B. The laser compares the two seg -
  // physically the same frame for both sensors when the wrist was straight at step 1, whatever the mounts do.
  Quat     qz = { 1, 0, 0, 0 };                   // zero quaternion from the page (or the mean of a key-z window)
  bool     qzSet = false;
  bool     qzAuto = false;                        // qz is the fixed zero (config IMU_ZERO_DEG + the heading at the first sample), not a measured one - never saved
  float    fwd[3] = { 1, 0, 0 };                  // the limb's axis in sensor coordinates (page steps 2/3); X until then
  Quat     B = { 1, 0, 0, 0 };
  Quat     seg = { 1, 0, 0, 0 };                  // rotation from the zero pose in the body frame (= d until the page has zeroed)
  Quat     segPrev = { 1, 0, 0, 0 };              // the seg before this sample, and when that sample was read: the two sensors are read at different moments, so a comparison takes the newer one back to the older one's time (a fast turn of the whole arm read as a wrist angle otherwise - "the yaw turns when I turn my body", 2026-10-01)
  uint32_t prevSampleMs = 0;
  float    yawFixRad = 0;                         // the laser's drift healer turns the hand's world a little about Z
  float    zeroRoll = 0, zeroPitch = 0;           // the mounting tilt at the zero pose, degrees (NVS, else config)
  bool     zeroSaved = false;                     // true = came from NVS (key z), not the config default
  float    headRad = 0;                           // yaw origin: heading at the zero pose, or at the first sample after a (re)start
  bool     headSet = false;
  Quat     headFix = { 1, 0, 0, 0 };              // yaw correction keeping the world heading continuous across sensor restarts
  Quat     qBefore = { 1, 0, 0, 0 };              // last good orientation before a restart, to re-align to
  bool     healPending = false;
  uint8_t  lastWhy = 0;                           // the sensor's own reset cause: 1 power-on, 2 internal, 3 watchdog, 4 pin, 5 other
  uint32_t samples = 0, resets = 0, lastSampleMs = 0, lastStaleWarnMs = 0, beganMs = 0, lastEnableMs = 0, retries = 0;
};
// the sensor's own account of why it restarted (SH-2 product id response, byte 1)
static const char* bnoResetWhy(uint8_t w) {
  switch (w) {
    case 1:  return "power-on reset - its 3V3 or GND blinked";
    case 2:  return "internal reset";
    case 3:  return "watchdog - the sensor hung, likely an I2C upset";
    case 4:  return "external reset pin";
    case 5:  return "other";
    default: return "reason unknown";
  }
}
static Imu         imus[2];                       // [0] arm on bus 1, [1] hand on bus 2
static bool        streaming = false;   // off at boot since 2026-10-03: on, ten lines a second fill the 4 KB log ring within seconds and every boot report / fly / crsf line is gone before anyone reads it (key s turns it on; the pages take their angles from the feed)
static bool        zeroed    = true;              // false = raw mode (key u)
static uint32_t    lastPollMs = 0, lastPrintMs = 0;
static Preferences prefs;

static const char* imuName(uint8_t i) { return i == 0 ? "arm " : "hand"; }

// SH-2 Initialize command, P0 = 1: re-initialise the whole sensor hub. The sensor answers
// with an internal reset (reset cause 2) and starts reporting again.
static bool sh2Initialize(Imu& s) {
  uint8_t* c = s.dev.shtpData;
  c[0] = SHTP_REPORT_COMMAND_REQUEST; c[1] = s.dev.commandSequenceNumber++; c[2] = COMMAND_INITIALIZE; c[3] = 1;
  for (uint8_t k = 4; k < 12; k++) c[k] = 0;
  return s.dev.sendPacket(2, 12);
}

bool    imuStreaming() { return streaming; }
uint8_t imuCount()     { return (imus[0].present ? 1 : 0) + (imus[1].present ? 1 : 0); }
void    imuStream(bool on) {
  streaming = on;
  Bench.printf("imu: stream %s%s\n", on ? "on" : "off", (on && !imuCount()) ? " (no sensor - nothing to print; r to re-probe)" : "");
}

// ---------------- zero pose (NVS) ---------------------------------------------------
static void zeroKeys(uint8_t i, char* kr, char* kp) { snprintf(kr, 5, "zr%u", i); snprintf(kp, 5, "zp%u", i); }

static void loadZero(uint8_t i) {
  Imu& s = imus[i];
  s.zeroRoll = IMU_ZERO_DEG[i][0]; s.zeroPitch = IMU_ZERO_DEG[i][1]; s.zeroSaved = false;
  char kr[5], kp[5]; zeroKeys(i, kr, kp);
  if (prefs.begin("ironman", true)) {           // read-only; fails harmlessly until z has ever been pressed
    if (prefs.isKey(kr)) {
      s.zeroRoll  = prefs.getFloat(kr, s.zeroRoll);
      s.zeroPitch = prefs.getFloat(kp, s.zeroPitch);
      s.zeroSaved = true;
    }
    prefs.end();
  }
}

static void saveZero(uint8_t i) {
  char kr[5], kp[5]; zeroKeys(i, kr, kp);
  if (!prefs.begin("ironman", false)) { Bench.println("imu: NVS open failed - zero pose NOT saved, it will be lost at reboot"); return; }
  prefs.putFloat(kr, imus[i].zeroRoll);
  prefs.putFloat(kp, imus[i].zeroPitch);
  prefs.end();
  imus[i].zeroSaved = true;
}

// Raw angles always; zeroed angles = the rotation from the zero pose to now,
// expressed in a frame with Z up and X along the heading at zero time. The
// reference pose is R_z(head) * T with T = R_y(zeroPitch) * R_x(zeroRoll), so
//   d = conj(R_z(head)) * q * conj(T).
// Never subtract Euler angles - with a 29 deg roll and 22 deg pitch in the
// mount that would mix every axis into every other.
static void qrot(const Quat& q, const float* v, float* out) {   // rotate a vector by a quaternion
  Quat r = qmul(qmul(q, { 0, v[0], v[1], v[2] }), qconj(q));
  out[0] = r.x; out[1] = r.y; out[2] = r.z;
}
static Quat qFromAxes(const float* x, const float* y, const float* z) {   // rotation matrix with these columns -> quaternion (the page's Q.fromAxes)
  float m00 = x[0], m01 = y[0], m02 = z[0], m10 = x[1], m11 = y[1], m12 = z[1], m20 = x[2], m21 = y[2], m22 = z[2];
  float tr = m00 + m11 + m22; Quat q;
  if (tr > 0)                          { float s = sqrtf(tr + 1) * 2;             q = { s / 4, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s }; }
  else if (m00 > m11 && m00 > m22)     { float s = sqrtf(1 + m00 - m11 - m22) * 2; q = { (m21 - m12) / s, s / 4, (m01 + m10) / s, (m02 + m20) / s }; }
  else if (m11 > m22)                  { float s = sqrtf(1 + m11 - m00 - m22) * 2; q = { (m02 - m20) / s, (m01 + m10) / s, s / 4, (m12 + m21) / s }; }
  else                                 { float s = sqrtf(1 + m22 - m00 - m11) * 2; q = { (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, s / 4 }; }
  return qnorm(q);
}
static void bodyFrame(Imu& s) {                    // B: X = the limb's heading at the zero pose, Z up
  float f[3]; qrot(s.qz, s.fwd, f);
  float n = sqrtf(f[0] * f[0] + f[1] * f[1]);
  float X[3] = { 1, 0, 0 };
  if (n >= 0.15f) { X[0] = f[0] / n; X[1] = f[1] / n; X[2] = 0; }
  float Y[3] = { -X[1], X[0], 0 }, Z[3] = { 0, 0, 1 };  // Z x X
  s.B = qFromAxes(X, Y, Z);
}

static void compute(Imu& s) {
  quatToEuler(s.q, s.rawRoll, s.rawPitch, s.rawYaw);
  if (!s.headSet) { s.headRad = s.rawYaw * DEG_TO_RAD; s.headSet = true; }
  if (!zeroed) { s.roll = s.rawRoll; s.pitch = s.rawPitch; s.yaw = s.rawYaw; s.d = s.q; }
  else {
    s.d = qmul(qmul(qconj(qAboutZ(s.headRad)), s.q),
               qconj(qTilt(s.zeroRoll * DEG_TO_RAD, s.zeroPitch * DEG_TO_RAD)));
    quatToEuler(s.d, s.roll, s.pitch, s.yaw);
  }
  // No measured step-1 zero (the sim page's step 1 not done, no key z): the fixed zero stands in for it - qz = the
  // reference pose d is taken from (the heading at the first sample, the config tilt), forward axis X - so seg is
  // exactly d and the arming / lift-off heading solve (fly.cpp) has its frame from power-up, with no touch or WiFi
  // step (Toby, 2026-10-07). Not saved: the heading restarts with every power-up.
  if (!s.qzSet && zeroed && s.headSet) {
    s.qz = qmul(qAboutZ(s.headRad), qTilt(s.zeroRoll * DEG_TO_RAD, s.zeroPitch * DEG_TO_RAD));
    s.fwd[0] = 1; s.fwd[1] = 0; s.fwd[2] = 0; s.yawFixRad = 0; s.qzSet = true; s.qzAuto = true;
    bodyFrame(s);
  }
  if (s.qzSet) { Quat qe = qmul(qAboutZ(s.yawFixRad), s.q); s.seg = qmul(qmul(qconj(s.B), qmul(qe, qconj(s.qz))), s.B); }
  else s.seg = s.d;
}

// The hand relative to the forearm: conj(d_arm) * d_hand, as roll / pitch / yaw. With both zero poses taken
// in the same moment with the hand in line (key z, or the sim page's step 1 through /laser?zero=1) this is
// the wrist itself - flexion (pitch, + = toward the palm), side-to-side (yaw), twist (roll) - whatever the
// arm is doing, because the heading mismatch between the two game rotation vectors is removed by the two
// yaw origins taken together. It drifts as the two headings drift apart (no magnetometer); the laser heals
// the yaw slowly while the hand is flat and open.
static Quat qnlerp(const Quat& a, Quat b, float t) {   // between two orientations a few degrees apart a normalised lerp is as good as a slerp
  if (a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z < 0) b = { -b.w, -b.x, -b.y, -b.z };
  return qnorm({ a.w + (b.w - a.w) * t, a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t });
}
// One sensor's seg at time t, between its last two samples. The two BNO085s report on their own clocks and are read
// in turn, so their latest samples sit up to a report period (or a loop stall) apart; while the whole arm turns or
// swings fast that gap reads as a wrist angle - a yaw command when he turns his body. Comparing both sensors at the
// older sample's moment takes it out (2026-10-01).
static Quat segAt(const Imu& s, uint32_t t) {
  if (s.samples < 2 || (int32_t)(t - s.lastSampleMs) >= 0) return s.seg;
  if ((int32_t)(t - s.prevSampleMs) <= 0) return s.segPrev;
  uint32_t span = s.lastSampleMs - s.prevSampleMs;
  if (span == 0 || span > 200) return s.seg;
  return qnlerp(s.segPrev, s.seg, (float)(t - s.prevSampleMs) / (float)span);
}
bool imuRelative(float& roll, float& pitch, float& yaw, uint32_t& ageMs) {
  if (!imus[0].present || !imus[1].present || !imus[0].samples || !imus[1].samples || !zeroed) return false;
  uint32_t t = (int32_t)(imus[0].lastSampleMs - imus[1].lastSampleMs) < 0 ? imus[0].lastSampleMs : imus[1].lastSampleMs;   // the older of the two latest samples
  Quat rel = qmul(qconj(segAt(imus[0], t)), segAt(imus[1], t));
  quatToEuler(rel, roll, pitch, yaw);
  uint32_t now = millis(), a0 = now - imus[0].lastSampleMs, a1 = now - imus[1].lastSampleMs;
  ageMs = a0 > a1 ? a0 : a1;
  return true;
}
// Both segments as quaternions (w,x,y,z) for the puck link, at the older of the two latest samples when both are
// live (the same alignment as imuRelative). A sensor that is absent or quiet for IMU_LINK_LIVE_MS keeps its last
// segment (identity if it never reported) and loses its live bit. Returns bit0 forearm live, bit1 hand live,
// bit2 both zeroed by the page's step 1.
uint8_t imuSegPair(float fq[4], float hq[4]) {
  uint32_t now = millis();
  bool l0 = imus[0].present && imus[0].samples && now - imus[0].lastSampleMs <= IMU_LINK_LIVE_MS;
  bool l1 = imus[1].present && imus[1].samples && now - imus[1].lastSampleMs <= IMU_LINK_LIVE_MS;
  Quat a = imus[0].samples ? imus[0].seg : Quat{ 1, 0, 0, 0 }, b = imus[1].samples ? imus[1].seg : Quat{ 1, 0, 0, 0 };
  if (l0 && l1) {
    uint32_t t = (int32_t)(imus[0].lastSampleMs - imus[1].lastSampleMs) < 0 ? imus[0].lastSampleMs : imus[1].lastSampleMs;
    a = segAt(imus[0], t); b = segAt(imus[1], t);
  }
  fq[0] = a.w; fq[1] = a.x; fq[2] = a.y; fq[3] = a.z;
  hq[0] = b.w; hq[1] = b.x; hq[2] = b.y; hq[3] = b.z;
  return (l0 ? 1 : 0) | (l1 ? 2 : 0) | (imuRelZeroed() ? 4 : 0);
}
bool imuRelativeQuat(float* q4, uint32_t& ageMs) {   // the same hand-vs-forearm rotation as a quaternion (w, x, y, z): the change between two of these over a fraction of a second does not depend on the heading between the two sensors
  if (!imus[0].present || !imus[1].present || !imus[0].samples || !imus[1].samples || !zeroed) return false;
  uint32_t t = (int32_t)(imus[0].lastSampleMs - imus[1].lastSampleMs) < 0 ? imus[0].lastSampleMs : imus[1].lastSampleMs;
  Quat rel = qmul(qconj(segAt(imus[0], t)), segAt(imus[1], t));
  q4[0] = rel.w; q4[1] = rel.x; q4[2] = rel.y; q4[3] = rel.z;
  uint32_t now = millis(), a0 = now - imus[0].lastSampleMs, a1 = now - imus[1].lastSampleMs;
  ageMs = a0 > a1 ? a0 : a1;
  return true;
}
bool imuSegAngles(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs) {
  if (i > 1 || !imus[i].present || !imus[i].samples) return false;
  quatToEuler(imus[i].seg, roll, pitch, yaw);    // seg = the page-frame rotation from step 1 when the page has zeroed it, else the console's zeroed d
  ageMs = millis() - imus[i].lastSampleMs;
  return true;
}
bool imuRelZeroed() { return imus[0].qzSet && !imus[0].qzAuto && imus[1].qzSet && !imus[1].qzAuto; }   // a MEASURED step-1 zero on both (key z, or the sim page when WiFi is on); the fixed zero does not count
float imuRelAngleDeg(float extraHandYawRad) {    // the whole angle between forearm and hand if the hand's heading fix were shifted by extraHandYawRad - the laser's healer probes with this in any arm pose
  if (!imus[0].present || !imus[1].present || !imus[0].samples || !imus[1].samples) return NAN;
  const Imu& h = imus[1];
  Quat segH;
  if (h.qzSet) { Quat qe = qmul(qAboutZ(h.yawFixRad + extraHandYawRad), h.q); segH = qmul(qmul(qconj(h.B), qmul(qe, qconj(h.qz))), h.B); }
  else segH = h.seg;
  Quat rel = qmul(qconj(imus[0].seg), segH);
  float w = fabsf(rel.w); if (w > 1) w = 1;
  return 2 * acosf(w) * RAD_TO_DEG;
}
float imuHandYawFixDeg() { return imus[1].qzSet ? imus[1].yawFixRad * RAD_TO_DEG : 0; }
void imuNudgeHandYaw(float deg) {                 // take deg out of the hand's side angle (the laser's drift healer)
  Imu& s = imus[1];
  if (!s.present || !s.samples) return;
  if (s.qzSet) { s.yawFixRad -= deg * DEG_TO_RAD; s.segPrev = qmul(qAboutZ(-deg * DEG_TO_RAD), s.segPrev); } else s.headRad += deg * DEG_TO_RAD;   /* segPrev too: the time-aligned comparison must not blend the old and the new heading for a sample */
  compute(s);
}
void imuShiftHandYaw(float deg) {                 // turn the hand's heading fix by deg (fly.cpp's solve at arming / lift-off); page-zeroed sensors only
  Imu& s = imus[1];
  if (!s.present || !s.samples || !s.qzSet) return;
  s.yawFixRad += deg * DEG_TO_RAD; s.segPrev = qmul(qAboutZ(deg * DEG_TO_RAD), s.segPrev);
  compute(s);
}
bool imuRelativeShifted(float extraHandYawRad, float& roll, float& pitch, float& yaw) {   // the wrist angles if the hand's heading were turned by extra - nothing applied
  if (!imus[0].present || !imus[1].present || !imus[0].samples || !imus[1].samples || !zeroed || !imus[1].qzSet) return false;
  Quat segH = qmul(qAboutZ(extraHandYawRad), imus[1].seg);   // seg = Rz(fix) * (...), so a heading turn is a left Z rotation of seg
  quatToEuler(qmul(qconj(imus[0].seg), segH), roll, pitch, yaw);
  return true;
}
bool imuIsZeroed() { return zeroed; }

// The page's zero for one sensor: its step-1 quaternion (the 2 s mean the model uses) and, when it has them, the
// limb's axis in sensor coordinates from steps 2/3. Kept in NVS as one blob per sensor ("rz0", "rz1").
struct RelZeroBlob { float w, x, y, z, fx, fy, fz; };
static void saveRelZero(uint8_t i) {
  RelZeroBlob b = { imus[i].qz.w, imus[i].qz.x, imus[i].qz.y, imus[i].qz.z, imus[i].fwd[0], imus[i].fwd[1], imus[i].fwd[2] };
  char k[4]; snprintf(k, sizeof k, "rz%u", i);
  if (prefs.begin("ironman", false)) { prefs.putBytes(k, &b, sizeof b); prefs.end(); }
}
static void loadRelZero(uint8_t i) {
  char k[4]; snprintf(k, sizeof k, "rz%u", i);
  RelZeroBlob b;
  if (prefs.begin("ironman", true)) {
    if (prefs.getBytesLength(k) == sizeof b && prefs.getBytes(k, &b, sizeof b) == sizeof b) {
      imus[i].qz = qnorm({ b.w, b.x, b.y, b.z }); imus[i].fwd[0] = b.fx; imus[i].fwd[1] = b.fy; imus[i].fwd[2] = b.fz; imus[i].qzSet = true; imus[i].qzAuto = false;
      bodyFrame(imus[i]);
    }
    prefs.end();
  }
}
bool imuSetRelZero(uint8_t i, const float* q4, const float* fwd3) {
  if (i > 1) return false;
  Imu& s = imus[i];
  if (q4) { s.qz = qnorm({ q4[0], q4[1], q4[2], q4[3] }); s.qzSet = true; s.qzAuto = false; s.yawFixRad = 0; }
  if (fwd3) { float n = sqrtf(fwd3[0] * fwd3[0] + fwd3[1] * fwd3[1] + fwd3[2] * fwd3[2]); if (n > 0.5f) { s.fwd[0] = fwd3[0] / n; s.fwd[1] = fwd3[1] / n; s.fwd[2] = fwd3[2] / n; } }
  if (!s.qzSet) return false;
  bodyFrame(s); if (!s.qzAuto) saveRelZero(i);
  if (s.present && s.samples) compute(s);
  Bench.printf("imu: %s zero from the page: q %+.3f %+.3f %+.3f %+.3f, forward axis %+.2f %+.2f %+.2f%s - the hand-vs-forearm angles use it (saved)\n",
               imuName(i), s.qz.w, s.qz.x, s.qz.y, s.qz.z, s.fwd[0], s.fwd[1], s.fwd[2], fwd3 ? "" : " (X, no swing yet)");
  return true;
}

static bool readOne(uint8_t i, uint32_t now);   // below

// Zeroing: hold still for a window; the zero pose is the average of every sample in it (an outstretched arm
// wobbles a few degrees). Roll and pitch average directly, yaw as a vector so +179/-179 does not average
// to 0. Runs from imuPoll() without blocking anything (before 2026-09-28 key z sat in its own 3 s loop and
// the feed stopped) - key z uses IMU_ZERO_AVG_MS, the sim page's step 1 (/laser?zero=1) LASER_ZERO_MS.
struct ZeroAcc { double r = 0, p = 0, sy = 0, cy = 0; float rmin = 1e9f, rmax = -1e9f, pmin = 1e9f, pmax = -1e9f; uint32_t n = 0; double qw = 0, qx = 0, qy = 0, qz = 0; Quat ref = { 1, 0, 0, 0 }; };
static ZeroAcc  zacc[2];
static bool     zActive = false;
static uint32_t zStart = 0, zDur = 0;

void imuZeroAsync(uint32_t ms) {
  if (!imuCount()) { Bench.println("imu: nothing to zero - no sensor (r to re-probe)"); return; }
  zacc[0] = ZeroAcc(); zacc[1] = ZeroAcc();
  zActive = true; zStart = millis(); zDur = ms;
  Bench.printf("imu: HOLD STILL - averaging the zero pose over %u ms...\n", (unsigned)ms);
}
static void zeroAccumulate(uint8_t i) {
  const Imu& s = imus[i]; ZeroAcc& a = zacc[i];
  a.r += s.rawRoll; a.p += s.rawPitch;
  a.sy += sin(s.rawYaw * DEG_TO_RAD); a.cy += cos(s.rawYaw * DEG_TO_RAD);
  if (s.rawRoll  < a.rmin) a.rmin = s.rawRoll;
  if (s.rawRoll  > a.rmax) a.rmax = s.rawRoll;
  if (s.rawPitch < a.pmin) a.pmin = s.rawPitch;
  if (s.rawPitch > a.pmax) a.pmax = s.rawPitch;
  if (!a.n) a.ref = s.q;                                                  // the mean quaternion, signs aligned to the first sample
  float sg = (s.q.w * a.ref.w + s.q.x * a.ref.x + s.q.y * a.ref.y + s.q.z * a.ref.z) < 0 ? -1.0f : 1.0f;
  a.qw += sg * s.q.w; a.qx += sg * s.q.x; a.qy += sg * s.q.y; a.qz += sg * s.q.z;
  a.n++;
}
static uint8_t  zLastCount = 0;   // (2026-10-21) sensors the last zero window zeroed (the puck's calibration step 3 reads it)
static void zeroFinish() {
  zActive = false;
  zeroed = true;
  zLastCount = 0;
  for (uint8_t i = 0; i < 2; i++) {
    Imu& s = imus[i]; ZeroAcc& a = zacc[i];
    if (!s.present) continue;
    if (a.n < 3) { Bench.printf("imu: %s - no samples during the hold, not zeroed\n", imuName(i)); continue; }
    s.zeroRoll  = (float)(a.r / a.n);
    s.zeroPitch = (float)(a.p / a.n);
    s.headRad   = (float)atan2(a.sy, a.cy); s.headSet = true;
    saveZero(i);
    float mq[4] = { (float)a.qw, (float)a.qx, (float)a.qy, (float)a.qz };
    imuSetRelZero(i, mq, nullptr);                                        // the hand-vs-forearm frame from the same window (forward axis kept)
    compute(s); zLastCount++;
    Bench.printf("imu: %s zeroed to the average of %u samples: raw roll %+.2f pitch %+.2f yaw %+.1f (wobble roll %.1f, pitch %.1f); tilt %s\n",
                 imuName(i), (unsigned)a.n, s.zeroRoll, s.zeroPitch, s.headRad * RAD_TO_DEG, a.rmax - a.rmin, a.pmax - a.pmin,
                 s.zeroSaved ? "saved to NVS (survives reboots)" : "NOT saved");
  }
  Bench.println("imu: this pose is now 0/0/0 for both sensors, and the hand-vs-forearm angles are 0 here. pitch + = fingers down, yaw + = turning left, roll + = right side down");
}
void imuZeroHere() { imuZeroAsync(IMU_ZERO_AVG_MS); }   // key z
bool    imuZeroRunning()   { return zActive; }
uint8_t imuLastZeroCount() { return zLastCount; }

// (2026-10-21) the puck's CALIBRATE steps 4 / 5 - HandBench's sim page steps 2 / 3, now on the arm: with the arm hanging
// straight down (or held straight up) each limb points along gravity, so the limb's axis in the sensor's own frame is
// world down (up) turned back through that pose's mean quaternion - a fixed property of the mount, whatever the twist.
// Both poses are kept (NVS "fd0" "fd1" "fu0" "fu1"); the axis used is their average, or the hanging one alone when they
// disagree by more than IMU_AXIS_AGREE_DEG (gravity straightens wrist and elbow there). A pose under IMU_AXIS_MIN_DEG
// from the measured zero (step 3) is refused, and so is a step without that zero. Both sensors or neither.
static ZeroAcc aacc[2];
static bool    aActive = false, aUp = false;
static uint32_t aStart = 0, aDur = 0;
static int8_t  aState = 0;            // -1 running, 1 done, 0 refused / idle
static char    aWhy[110] = "";
static float   axDown[2][3], axUp[2][3];
static bool    hasDown[2] = { false, false }, hasUp[2] = { false, false };
static void axisSaveNvs(uint8_t i) {
  if (!prefs.begin("ironman", false)) return;
  char k[4] = { 'f', 'd', (char)('0' + i), 0 };
  if (hasDown[i]) prefs.putBytes(k, axDown[i], sizeof axDown[i]);
  k[1] = 'u'; if (hasUp[i]) prefs.putBytes(k, axUp[i], sizeof axUp[i]);
  prefs.end();
}
static void axisLoadNvs(uint8_t i) {
  if (!prefs.begin("ironman", true)) return;
  char k[4] = { 'f', 'd', (char)('0' + i), 0 };
  hasDown[i] = prefs.getBytesLength(k) == sizeof axDown[i] && prefs.getBytes(k, axDown[i], sizeof axDown[i]) == sizeof axDown[i];
  k[1] = 'u'; hasUp[i] = prefs.getBytesLength(k) == sizeof axUp[i] && prefs.getBytes(k, axUp[i], sizeof axUp[i]) == sizeof axUp[i];
  prefs.end();
}
static void quatAccumulate(ZeroAcc& a, const Quat& q) {
  if (!a.n) a.ref = q;
  float sg = (q.w * a.ref.w + q.x * a.ref.x + q.y * a.ref.y + q.z * a.ref.z) < 0 ? -1.0f : 1.0f;
  a.qw += sg * q.w; a.qx += sg * q.x; a.qy += sg * q.y; a.qz += sg * q.z; a.n++;
}
bool imuAxisStart(bool up, uint32_t ms) {
  aWhy[0] = 0;
  for (uint8_t i = 0; i < 2; i++) {
    if (!imus[i].present || !imus[i].samples) { snprintf(aWhy, sizeof aWhy, "the %s IMU is absent", imuName(i)); aState = 0; return false; }
    if (!imus[i].qzSet || imus[i].qzAuto) { snprintf(aWhy, sizeof aWhy, "no measured zero yet - do step 4 (arm out) first"); aState = 0; return false; }
  }
  aacc[0] = ZeroAcc(); aacc[1] = ZeroAcc(); aUp = up; aStart = millis(); aDur = ms; aActive = true; aState = -1;
  Bench.printf("imu: HOLD STILL - the arm %s, averaging %u ms for each sensor's forward axis\n", up ? "straight UP" : "hanging straight DOWN", (unsigned)ms);
  return true;
}
int8_t imuAxisState(const char** why) { if (why) *why = aWhy; return aState; }
static void axisFinish() {
  aActive = false;
  float ax[2][3];
  for (uint8_t i = 0; i < 2; i++) {
    const Imu& s = imus[i]; const ZeroAcc& a = aacc[i];
    if (a.n < 3) { snprintf(aWhy, sizeof aWhy, "the %s IMU sent no samples during the hold", imuName(i)); aState = 0; Bench.printf("imu: axis - %s\n", aWhy); return; }
    Quat m = qnorm({ (float)a.qw, (float)a.qx, (float)a.qy, (float)a.qz });
    float dot = fabsf(m.w * s.qz.w + m.x * s.qz.x + m.y * s.qz.y + m.z * s.qz.z); if (dot > 1) dot = 1;
    float fromZero = 2.0f * acosf(dot) * RAD_TO_DEG;
    if (fromZero < IMU_AXIS_MIN_DEG) {
      snprintf(aWhy, sizeof aWhy, "the %s is only %.0f deg from straight out - %s", imuName(i), fromZero, aUp ? "hold the arm straight up" : "let the arm hang straight down");
      aState = 0; Bench.printf("imu: axis refused - %s\n", aWhy); return;
    }
    const float g[3] = { 0, 0, aUp ? 1.0f : -1.0f };
    qrot(qconj(m), g, ax[i]);                                             // world down / up in the sensor's frame = the limb's axis
  }
  for (uint8_t i = 0; i < 2; i++) {
    float* dst = aUp ? axUp[i] : axDown[i]; for (uint8_t k = 0; k < 3; k++) dst[k] = ax[i][k];
    (aUp ? hasUp : hasDown)[i] = true;
    float f[3]; const char* how;
    if (hasDown[i] && hasUp[i]) {
      float c = axDown[i][0] * axUp[i][0] + axDown[i][1] * axUp[i][1] + axDown[i][2] * axUp[i][2]; if (c > 1) c = 1; if (c < -1) c = -1;
      float apart = acosf(c) * RAD_TO_DEG;
      if (apart > IMU_AXIS_AGREE_DEG) { for (uint8_t k = 0; k < 3; k++) f[k] = axDown[i][k]; how = "the hanging pose alone (the two disagree)"; }
      else { for (uint8_t k = 0; k < 3; k++) f[k] = axDown[i][k] + axUp[i][k]; how = "down and up averaged"; }
      Bench.printf("imu: %s forward axis - down and up %.1f deg apart, %s\n", imuName(i), apart, how);
    } else { for (uint8_t k = 0; k < 3; k++) f[k] = ax[i][k]; how = aUp ? "the raised pose (no hanging one yet)" : "the hanging pose (no raised one yet)"; Bench.printf("imu: %s forward axis from %s\n", imuName(i), how); }
    axisSaveNvs(i);
    imuSetRelZero(i, nullptr, f);                                         // the zero kept, the axis replaced; saved
  }
  aState = 1;
}

// Key y: heading only - yaw drifts (no magnetometer) and restarts with the
// sensor, the tilt does neither.
void imuZeroYaw() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 2; i++) {
    Imu& s = imus[i];
    if (!s.present || !s.samples) continue;
    s.headRad = s.rawYaw * DEG_TO_RAD; s.headSet = true;
    if (s.qzAuto) s.qzSet = false;              // the fixed zero follows the new heading (a measured one keeps its own)
    compute(s);
    Bench.printf("imu: %s yaw zeroed here (heading %+.1f), tilt untouched\n", imuName(i), s.rawYaw);
    n++;
  }
  if (!n) { Bench.println("imu: no sample to zero yaw on"); return; }
  zeroed = true;
}

// Key k: drop the NVS zero poses so the config.h defaults apply (those hold the
// last bench-measured values).
void imuZeroForget() {
  if (prefs.begin("ironman", false)) { prefs.clear(); prefs.end(); }
  for (uint8_t i = 0; i < 2; i++) {
    loadZero(i); imus[i].qzSet = false; imus[i].qzAuto = false; imus[i].yawFixRad = 0;   // the page zero went with prefs.clear() too (compute re-takes the fixed one)
    if (imus[i].present && imus[i].samples) compute(imus[i]);
  }
  zeroed = true;
  Bench.printf("imu: saved zero poses forgotten - config defaults in effect: arm roll %+.2f pitch %+.2f, hand roll %+.2f pitch %+.2f (yaw origins unchanged)\n",
               imus[0].zeroRoll, imus[0].zeroPitch, imus[1].zeroRoll, imus[1].zeroPitch);
}

void imuRaw() {
  zeroed = false;
  Bench.println("imu: RAW angles (BNO native, no zero). z, y or k go back to zeroed");
}

// ---------------- bring-up -------------------------------------------------------------
static bool startOne(uint8_t i) {
  Imu& s = imus[i];
  uint8_t bus = i + 1;
  s.present = false;
#if HW_FAKE
  s.present = true; s.bus = bus; s.addr = 0x4A;
  s.samples = 0; s.resets = 0; s.lastSampleMs = millis(); s.lastStaleWarnMs = 0; s.beganMs = millis(); s.lastEnableMs = millis();
  s.headSet = false; s.headFix = { 1, 0, 0, 0 }; s.healPending = false; s.lastWhy = 0;
  loadZero(i); loadRelZero(i); axisLoadNvs(i);
  Bench.printf("imu: %s = FAKE BNO08x on %s (samples come from imuFakeInject)\n", imuName(i), busName(bus));
  return true;
#endif
  for (uint8_t addr : BNO_ADDRS) {
    if (!busAck(bus, addr)) continue;
    for (uint8_t t = 1; t <= IMU_BEGIN_TRIES; t++) {
      if (s.dev.begin(addr, busWire(bus), s.useInt ? (i == 0 ? PIN_IMU_INT_ARM : PIN_IMU_INT_HAND) : 255)) {
        s.present = true; s.bus = bus; s.addr = addr;
        s.samples = 0; s.resets = 0; s.lastSampleMs = millis(); s.lastStaleWarnMs = 0; s.beganMs = millis(); s.lastEnableMs = millis();
        s.headSet = false;                        // the sensor just restarted: its heading did too
        s.headFix = { 1, 0, 0, 0 }; s.healPending = false; s.lastWhy = 0;
        s.dev.enableGameRotationVector(IMU_REPORT_MS);
        s.dev.hasReset();                         // clear the reset we just caused
        loadZero(i); loadRelZero(i); axisLoadNvs(i);            // the page's wrist zero, if it ever sent one
        Bench.printf("imu: %s = BNO08x at 0x%02X on %s, game rotation vector every %u ms, %s\n",
                     imuName(i), addr, busName(bus), IMU_REPORT_MS, s.useInt ? "read on INT" : "polled");
        Bench.printf("imu: %s zero pose: tilt roll %+.1f pitch %+.1f (%s), yaw origin = first sample. z re-zeroes here, u shows raw\n",
                     imuName(i), s.zeroRoll, s.zeroPitch, s.zeroSaved ? "saved by z" : "config default");
        return true;
      }
      Bench.printf("imu: 0x%02X on %s acks but did not answer the product-id request (try %u/%u)\n",
                   addr, busName(bus), t, IMU_BEGIN_TRIES);
      delay(250);
    }
  }
  return false;
}

void imuBegin() {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 2; i++) if (startOne(i)) n++;
  if (n) return;
  Bench.println("imu: NO BNO085 FOUND on either bus. check:");
  Bench.println("  - module VIN to ESP32 3V3 (not 5 V, not VIN), module GND to an ESP32 GND pin");
  Bench.printf ("  - SDA/SCL: bus 1 (forearm) SDA %u SCL %u, bus 2 (palm, across the wrist) SDA %u SCL %u\n",
                PIN_I2C1_SDA, PIN_I2C1_SCL, PIN_I2C2_SDA, PIN_I2C2_SCL);
  Bench.println("  - PS0 and PS1 pads both open = I2C mode. either one bridged puts the module in UART or SPI and it vanishes from the bus");
  Bench.println("  - ADR pad open = 0x4A, bridged = 0x4B - both are probed, so the scan above should show one of them");
  Bench.println("  - address shows in the scan but init fails: power-cycle the module (or pull its RST low and release), then r");
  Bench.println("  keys: i rescan, r re-probe");
}

// ---------------- run ---------------------------------------------------------------------
static bool flexInStream = false;
void imuStreamFlex(bool on) { flexInStream = on; Bench.printf("imu: flex mV %s the stream\n", on ? "added to" : "removed from"); }

static void printLine() {
  char line[200]; size_t n = 0;
  for (uint8_t i = 0; i < 2; i++) {
    const Imu& s = imus[i];
    if (!s.present) continue;
    n += snprintf(line + n, sizeof line - n, "%s%s%s roll %+7.1f  pitch %+7.1f  yaw %+7.1f",
                  n ? "   |   " : "", imuName(i), zeroed ? "" : " raw", s.roll, s.pitch, s.yaw);
  }
  if (flexInStream)
    n += snprintf(line + n, sizeof line - n, "%sflex %4u %4u %4u %4u %4u", n ? "   |   " : "", flexMv(0), flexMv(1), flexMv(2), flexMv(3), flexMv(4));
  Bench.println(line);
}

// One sensor: reset bookkeeping, then take a report if one is waiting. True = fresh sample.
#if HW_FAKE
static Quat fakeQ[2] = { { 1, 0, 0, 0 }, { 1, 0, 0, 0 } }; static bool fakePending[2] = { false, false };
void imuFakeInject(uint8_t i, float w, float x, float y, float z) { if (i > 1) return; fakeQ[i] = { w, x, y, z }; fakePending[i] = true; }
#endif
static bool readOne(uint8_t i, uint32_t now) {
  Imu& s = imus[i];
  if (!s.present) return false;
#if HW_FAKE
  if (!fakePending[i]) return false;
  fakePending[i] = false;
  Quat raw = qnorm(fakeQ[i]);
  s.segPrev = s.seg; s.prevSampleMs = s.lastSampleMs;
  s.q = qnorm(qmul(s.headFix, raw));
  compute(s);
  s.samples++; s.lastSampleMs = now;
  return true;
#else
  if (s.dev.hasReset()) {
    if (now - s.beganMs < 3000) {              // the sensor announcing the reset begin() gave it: not an event
      s.dev.enableGameRotationVector(IMU_REPORT_MS); s.lastEnableMs = now;
    } else {
      s.resets++;
      s.lastWhy = s.dev.resetReason();         // one round trip to the sensor
      s.qBefore = s.q; s.healPending = s.samples > 0;   // its heading restarted; re-align to the pose we last saw
      Bench.printf("!! imu: %s BNO08x RESET #%u: %s. report re-enabled; heading will be re-aligned to the last pose\n", imuName(i), (unsigned)s.resets, bnoResetWhy(s.lastWhy));
      flyNote("%s sensor reset (heading re-aligned)", imuName(i));
      s.dev.enableGameRotationVector(IMU_REPORT_MS); s.lastEnableMs = now;
    }
  }
  if (s.dev.dataAvailable()) {
    Quat raw = qnorm({ s.dev.getQuatReal(), s.dev.getQuatI(), s.dev.getQuatJ(), s.dev.getQuatK() });
    if (s.healPending) {                       // first sample after a restart: tilt is gravity's, only the heading jumped
      s.healPending = false;
      float r0, p0, y0, r1, p1, y1;
      quatToEuler(s.qBefore, r0, p0, y0);
      quatToEuler(raw, r1, p1, y1);
      s.headFix = qAboutZ((y0 - y1) * DEG_TO_RAD);
      Bench.printf("imu: %s heading re-aligned by %+.1f deg after the restart (assumes the arm held still for it)\n", imuName(i), y0 - y1);
    }
    s.segPrev = s.seg; s.prevSampleMs = s.lastSampleMs;
    s.q = qnorm(qmul(s.headFix, raw));
    compute(s);
    s.samples++; s.lastSampleMs = now;
    return true;
  }
  // Quiet sensor: climb a recovery ladder once a second until reports flow. 1: re-send the
  // enable (dropped if sent while the sensor was still booting). 2: executable-channel "on"
  // in case it is asleep. 3: SH-2 "initialize the whole hub" - the move that revived the arm
  // module on 2026-09-23 after a supply glitch left its host interface answering but its
  // fusion core dead, which the library's soft reset could not clear. Then round again.
  if (now - s.lastSampleMs > 1000 && now - s.lastEnableMs > 1000) {
    s.retries++;
    if (s.useInt && s.samples == 0 && s.retries >= 2) {   // nothing ever came with the INT pin in use: the line is probably not wired (its pull-up holds it high) - poll instead
      s.useInt = false;
      Bench.printf("imu: %s quiet with the INT line - treating it as not wired, polling the sensor instead\n", imuName(i));
      startOne(i);
      return false;
    }
    if (s.retries % 3 == 2) { s.dev.modeOn(); delay(20); }
    if (s.retries % 3 == 0) { sh2Initialize(s); Bench.printf("imu: %s still quiet - sent SH-2 initialize (hub restart)\n", imuName(i)); }
    s.dev.enableGameRotationVector(IMU_REPORT_MS); s.lastEnableMs = now;
    if (now - s.lastSampleMs > IMU_STALE_MS && now - s.lastStaleWarnMs > 5000) {
      s.lastStaleWarnMs = now;
      Bench.printf("imu: %s - no report for %u s, asking for it again (unplugged? bus stuck? r re-probes)\n", imuName(i), (unsigned)((now - s.lastSampleMs) / 1000));
    }
  }
  return false;
#endif
}

void imuPoll() {
  uint32_t now = millis();
  if (now - lastPollMs < 5) return;            // the sensor reports every IMU_REPORT_MS; asking every 5 ms is plenty
  lastPollMs = now;
  bool fresh = false;
  for (uint8_t i = 0; i < 2; i++) if (readOne(i, now)) { fresh = true; if (zActive) zeroAccumulate(i); if (aActive) quatAccumulate(aacc[i], imus[i].q); }
  if (zActive && now - zStart >= zDur) zeroFinish();
  if (aActive && now - aStart >= aDur) axisFinish();
  if (fresh && streaming && now - lastPrintMs >= IMU_PRINT_MS - 10) { lastPrintMs = now; printLine(); }
}

// GET /imu: the raw game rotation vectors, fresh at the sensor rate. The arm
// simulator (sim/arm.html) polls this ~25x a second and does its own
// calibration, so nothing here depends on the zero pose.
bool imuZeroed(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs) {
  if (i > 1 || !imus[i].present || !imus[i].samples) return false;
  const Imu& s = imus[i];
  roll = s.roll; pitch = s.pitch; yaw = s.yaw; ageMs = millis() - s.lastSampleMs;
  return true;
}
bool imuRawAngles(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs) {
  if (i > 1 || !imus[i].present || !imus[i].samples) return false;
  const Imu& s = imus[i];
  roll = s.rawRoll; pitch = s.rawPitch; yaw = s.rawYaw; ageMs = millis() - s.lastSampleMs;
  return true;
}

String imuJson() {                                   // the feed frame. A String, not a fixed buffer: the fly object grows with the arm-attempt records and a 1500-byte buffer truncated it mid-JSON (the pages "crashed", 2026-09-30 night)
  char b[400];
  String s; s.reserve(3000);
  snprintf(b, sizeof b, "{\"t\":%u", (unsigned)millis()); s += b;
  for (uint8_t i = 0; i < 2; i++) {
    const Imu& x = imus[i];
    snprintf(b, sizeof b, ",\"%s\":{\"ok\":%d,\"q\":[%.5f,%.5f,%.5f,%.5f],\"n\":%u,\"age\":%u,\"resets\":%u,\"why\":%u}",
             i == 0 ? "arm" : "hand", x.present ? 1 : 0, x.q.w, x.q.x, x.q.y, x.q.z,
             (unsigned)x.samples, (unsigned)(x.present ? millis() - x.lastSampleMs : 0), (unsigned)x.resets, (unsigned)x.lastWhy);
    s += b;
  }
  snprintf(b, sizeof b, ",\"flex\":[%u,%u,%u,%u,%u],\"rssi\":%d,\"dbl\":%u,\"dip\":%u,\"ring\":",   // thumb..pinky mV; WiFi at the board; thumb crunches; finger dips; then the palm ring, LiDAR, laser, drone link + flight
           flexMv(0), flexMv(1), flexMv(2), flexMv(3), flexMv(4), (int)WiFi.RSSI(), (unsigned)gestureCount(), (unsigned)gestureDipCount());
  s += b; s += palmJson(); s += ",\"lidar\":"; s += lidarJson(); s += ",\"laser\":"; s += laserJson(); s += ",\"fly\":"; s += flyJson(); s += "}";
  return s;
}

static void productId(Imu& s, const char* label);   // below

// Key j: hard evidence from a quiet sensor. Sends the executable-channel "on" (wake) and
// the set-feature command by hand and reports whether each I2C write was ACKed, then dumps
// every raw SHTP packet the sensor sends for ~3 s, then tries the plain rotation vector.
void imuDiagnose() {
#if HW_FAKE
  Bench.println("imu: fake build - no sensor to diagnose");
  return;
#endif
  uint8_t i = 0;
  for (uint8_t k = 0; k < 2; k++) if (imus[k].present && millis() - imus[k].lastSampleMs > 1000) { i = k; break; }
  Imu& s = imus[i];
  if (!s.present) { Bench.println("imu: nothing to diagnose - no sensor present"); return; }
  Bench.printf("imu: diagnosing %s (0x%02X on %s): %u samples, last %u ms ago, %u resets\n",
               imuName(i), s.addr, busName(s.bus), (unsigned)s.samples, (unsigned)(millis() - s.lastSampleMs), (unsigned)s.resets);
  s.dev.shtpData[0] = 2;                                                     // executable channel: 2 = on
  Bench.printf("  wake ('on') write, 1 byte: %s\n", s.dev.sendPacket(1, 1) ? "ACKed" : "FAILED - NACK or timeout");
  delay(50);
  uint32_t us = IMU_REPORT_MS * 1000UL; uint8_t* d = s.dev.shtpData;
  d[0] = SHTP_REPORT_SET_FEATURE_COMMAND; d[1] = SENSOR_REPORTID_GAME_ROTATION_VECTOR; d[2] = 0; d[3] = 0; d[4] = 0;
  d[5] = us & 0xFF; d[6] = (us >> 8) & 0xFF; d[7] = (us >> 16) & 0xFF; d[8] = (us >> 24) & 0xFF;
  for (uint8_t k = 9; k < 17; k++) d[k] = 0;
  Bench.printf("  set-feature write, 17 bytes: %s\n", s.dev.sendPacket(2, 17) ? "ACKed" : "FAILED - NACK or timeout: the bus drops long writes (wiring / pull-ups)");
  uint32_t t0 = millis(); uint16_t n = 0;
  while (millis() - t0 < 3000 && n < 15) {
    if (s.dev.receivePacket()) {
      uint16_t len = (((uint16_t)s.dev.shtpHeader[1] << 8) | s.dev.shtpHeader[0]) & 0x7FFF;
      Bench.printf("  pkt ch%u len %u:", s.dev.shtpHeader[2], len);
      for (uint8_t k = 0; k < 8 && k + 4 < len; k++) Bench.printf(" %02X", s.dev.shtpData[k]);
      Bench.println();
      n++;
    }
    Bench.poll(); halWdtFeed(); delay(3);
  }
  Bench.printf("  %u packets in %u ms  (ch3 FB.. = sensor reports = good; ch2 F1/FC.. = command replies; ch1 01 = reset complete; none = not running)\n", n, (unsigned)(millis() - t0));
  s.dev.enableRotationVector(IMU_REPORT_MS);                                 // does a different report work?
  t0 = millis(); n = 0;
  while (millis() - t0 < 1500) { if (s.dev.receivePacket()) n++; Bench.poll(); halWdtFeed(); delay(3); }
  Bench.printf("  plain rotation vector instead: %u packets in 1.5 s\n", n);
  // Who is answering? The product-id reply names the running software: the sensor-hub
  // application and the bootloader have different part numbers. Compare both sensors.
  for (uint8_t k = 0; k < 2; k++) if (imus[k].present) productId(imus[k], imuName(k));
  // Last software resort: SH-2 "initialize" with P0 = 1 (re-initialize the whole hub).
  Bench.printf("  reinitialize command: %s\n", sh2Initialize(s) ? "ACKed" : "FAILED");
  t0 = millis(); n = 0;
  while (millis() - t0 < 1200) { if (s.dev.receivePacket()) n++; Bench.poll(); halWdtFeed(); delay(3); }
  Bench.printf("  %u packets in the 1.2 s after it\n", n);
  s.dev.enableGameRotationVector(IMU_REPORT_MS);
  Bench.println("  done. writes ACKed + no packets at all = the sensor core is not running (supply glitch wedged it, or BOOT pad low = bootloader mode): power-cycle, then check the module's BOOT/PS pads and 3V3/GND");
}

// SH-2 product id: [1] reset cause, [2].[3] version, [4..7] part number, [8..11] build, [12..13] patch.
// A healthy BNO085 answers with (at least) the application, part 10003608; the bootloader is a different part.
static void productId(Imu& s, const char* label) {
  s.dev.shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; s.dev.shtpData[1] = 0;
  bool w = s.dev.sendPacket(2, 2);
  uint32_t t0 = millis(); uint8_t got = 0;
  while (millis() - t0 < 500 && got < 3) {
    if (!s.dev.receivePacket()) continue;
    if (s.dev.shtpData[0] != SHTP_REPORT_PRODUCT_ID_RESPONSE) continue;
    const uint8_t* d = s.dev.shtpData;
    uint32_t part = ((uint32_t)d[7] << 24) | ((uint32_t)d[6] << 16) | ((uint32_t)d[5] << 8) | d[4];
    uint32_t build = ((uint32_t)d[11] << 24) | ((uint32_t)d[10] << 16) | ((uint32_t)d[9] << 8) | d[8];
    Bench.printf("  %s product id: reset cause %u, SW %u.%u.%u, part %lu, build %lu\n", label, d[1], d[2], d[3], d[12] | (d[13] << 8), (unsigned long)part, (unsigned long)build);
    got++;
  }
  if (!got) Bench.printf("  %s product id: request %s, no response in 500 ms\n", label, w ? "ACKed" : "FAILED");
}

void imuStatus() {
  for (uint8_t i = 0; i < 2; i++) {
    const Imu& s = imus[i];
    if (!s.present) { Bench.printf("imu: %s - none on %s\n", imuName(i), busName(i + 1)); continue; }
    Bench.printf("imu: %s - BNO08x 0x%02X on %s, %u samples, %u resets%s%s, last %u ms ago\n",
                 imuName(i), s.addr, busName(s.bus), (unsigned)s.samples, (unsigned)s.resets,
                 s.resets ? " (last: " : "", s.resets ? (String(bnoResetWhy(s.lastWhy)) + ")").c_str() : "", (unsigned)(millis() - s.lastSampleMs));
    Bench.printf("      zero pose tilt roll %+.1f pitch %+.1f (%s), yaw origin %+.1f | zeroed roll %+.1f pitch %+.1f yaw %+.1f | raw roll %+.1f pitch %+.1f yaw %+.1f\n",
                 s.zeroRoll, s.zeroPitch, s.zeroSaved ? "saved" : "config default", s.headRad * RAD_TO_DEG,
                 zeroed ? s.roll : 0.0f, zeroed ? s.pitch : 0.0f, zeroed ? s.yaw : 0.0f, s.rawRoll, s.rawPitch, s.rawYaw);
  }
  Bench.printf("imu: stream %s, showing %s angles\n", streaming ? "on" : "off", zeroed ? "zeroed" : "raw");
}


// The CRITICAL shutdown: both BNO08X into the SH-2 sleep state over the executable channel (milliamps saved each).
void imuSleep() {
#if !HW_FAKE
  for (uint8_t i = 0; i < 2; i++) if (imus[i].present) imus[i].dev.modeSleep();
#endif
}
