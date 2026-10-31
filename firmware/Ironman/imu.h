// BNO085 orientation from the game rotation vector (no magnetometer: yaw is
// relative to the heading at power-up and drifts slowly). Bus 1 = arm, bus 2
// = hand; none, one or both may be wired - nothing here crashes without a
// sensor.
//
// The stream shows angles relative to a "zero pose" (key z), because the
// module sits on the glove at an angle: the tilt at that pose is stored in NVS
// and the heading at that moment is the yaw origin. Quaternion maths, not
// Euler subtraction - see compute() in imu.cpp.
#pragma once
#include <Arduino.h>

void    imuBegin();            // probe 0x4A/0x4B on both buses, start the report on each sensor found. Re-callable (key r).
void    imuPoll();             // every loop(): read reports, print roll/pitch/yaw while streaming
void    imuStream(bool on);
bool    imuStreaming();
uint8_t imuCount();            // sensors found, 0-2
void    imuStatus();           // per sensor: zero pose, zeroed and raw angles
void    imuZeroHere();         // key z: hold still ~3 s; the average of that window = 0/0/0, tilt saved to NVS, yaw origin taken (non-blocking, from imuPoll)
void    imuZeroAsync(uint32_t ms);   // the same over any window - the sim page's step 1 uses it through /laser?zero=1
bool    imuZeroRunning();      // a zero window is averaging now
uint8_t imuLastZeroCount();    // sensors the last zero window zeroed (2 = both)
bool    imuAxisStart(bool up, uint32_t ms);   // (2026-10-21) the puck's calibration steps 4 / 5: each sensor's forward axis from the arm hanging down / held up; false = refused at once (imuAxisState says why)
int8_t  imuAxisState(const char** why);       // -1 averaging, 1 done (saved), 0 refused (why)
bool    imuIsZeroed();
void    imuZeroYaw();          // key y: yaw origin = current heading, tilt untouched
void    imuZeroForget();       // key k: drop the NVS zero poses, config.h defaults apply
void    imuRaw();              // key u: show the BNO's native angles instead
String  imuJson();             // GET /imu body: both raw quaternions (sensor -> world) + the five flex mV, for the arm simulator
bool    imuZeroed(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs);   // the zeroed angles the stream prints (0 arm, 1 hand); false if that sensor is absent or has never reported
bool    imuRawAngles(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs); // the BNO's native angles (gravity's pitch and roll, arbitrary heading) - no zero pose involved
bool    imuRelative(float& roll, float& pitch, float& yaw, uint32_t& ageMs);   // the hand relative to the forearm (conj(seg_arm) * seg_hand): the wrist - flexion (pitch), side (yaw), twist (roll); false without both sensors
bool    imuSegAngles(uint8_t i, float& roll, float& pitch, float& yaw, uint32_t& ageMs);   // one sensor's rotation from its page step-1 pose in the page's body frame (the simulator's segment): 0/0/0 = as at step 1, i.e. level palm down for the hand. Falls back to the console zero when the page has not zeroed it
bool    imuSetRelZero(uint8_t i, const float* q4, const float* fwd3);           // the page's step-1 zero quaternion (w,x,y,z) and optional forward axis for sensor i (0 arm, 1 hand); saved. Either may be null to keep the old one
uint8_t imuSegPair(float fq[4], float hq[4]);   // both segments (the simulator's, w x y z) time-aligned, for the puck link; returns bit0 forearm live, bit1 hand live, bit2 zeroed by the page
bool    imuRelZeroed();              // both sensors have a MEASURED wrist zero (key z, or the page with WiFi on); without one the fixed zero in config.h stands in
void    imuNudgeHandYaw(float deg);  // take deg out of the hand's side angle: the laser heals the slow heading drift between the two sensors with it
float   imuRelAngleDeg(float extraHandYawRad);   // the whole forearm-to-hand angle if the hand's heading fix were shifted by this: the healer probes +/- a degree and steps downhill, in any arm pose. NAN without both sensors
bool    imuRelativeQuat(float* q4, uint32_t& ageMs);   // the wrist as a quaternion (w, x, y, z) - the laser's jolt, which must not depend on the heading
void    imuShiftHandYaw(float deg);  // turn the hand's heading fix by deg (the solve at arming / lift-off)
bool    imuRelativeShifted(float extraHandYawRad, float& roll, float& pitch, float& yaw);   // the wrist angles with the hand's heading turned by extra, not applied
float   imuHandYawFixDeg();                       // the hand's current heading fix, degrees (feed "yf": the simulator applies the same turn to its hand, so the model self-rights without a step 1)
void    imuDiagnose();         // key j: raw evidence from a quiet sensor (I2C write results, raw packets)
void    imuSleep();            // the CRITICAL shutdown: both sensors into the SH-2 sleep state
void    imuStreamFlex(bool on);   // key g: append the flex millivolts to every stream line
#if HW_FAKE
void    imuFakeInject(uint8_t i, float w, float x, float y, float z);   // the next sample of sensor i (0 arm, 1 hand) as the raw game rotation vector; consumed by imuPoll()
#endif
