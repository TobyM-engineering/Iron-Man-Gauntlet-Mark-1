// The hand as the sticks (2026-09-30, seventh round 2026-10-01). Throttle = the mean curl of index,
// middle, ring and pinky against the fist it armed with: the motors idle until the fingers have lifted
// once, then a fist is the FLOOR (a gentle descent, never a drop), a full lift the range top, hover
// between; a tight fist held 1.2 s is a deliberate cut. Roll = the hand's own tilt against gravity,
// pitch = wrist flexion, yaw = the hand turned on the forearm, each from the pose it armed in (trims),
// full stick at the wrist's own range (the page's step 7) - the arm can be anywhere. Arm = a fist, a
// quick open, a fist again, hand level, wrist in line, while the module hears the drone and the drone
// says it is ready and level. Disarm = a thumb double-crunch, the page, key D, or a fail-safe (hand
// sensor quiet, link or telemetry lost, the drone flipped, a firmware update). Armed switches the ring
// and laser gestures off: a fist is throttle here, not "ring off". Numbers in config.h (FLY_*).
#pragma once
#include <Arduino.h>

void   flyBegin();
void   flyPoll();                              // every loop(), after imuPoll / flexPoll / gesturePoll / crsfPoll: writes the sticks every CRSF_PERIOD_MS
void   flyEnable(bool on, const char* why);    // (no flight mode any more: off = disarm, on = a note)
bool   flyEnabled();                           // = armed: the ring and laser gestures stand down
bool   flyArmed();
float  flyFingerCurl();                        // the four fingers as one (the mean of the middle two), 0 open .. 1 a fist; NAN uncalibrated
bool   flyRangeCapturing();                    // the page's step 7 (wrist range) is running
void   flyDisarm(const char* why);             // safe at once: throttle min, ARM low; prints the flight summary
String flyBenchArm(uint32_t seconds);          // key A / GET :8080/fly?arm=N: ARM through the drone's own checks (linked, ready, level) and let the glove fly it for N s. Props off. Returns the verdict
String flySet(const String& query);            // GET :8080/fly?arm=N &disarm=1 &bind=1|0 &quiet=1 &power=up &frames=0|1 &thrmax= &thrsens= &thrfloor= &stickmax= &yawmax= &range=1|0 &raw=1 -> flyJson + "msg"
String flyJson();                              // the feed's "fly": flight + link + craft in one object
String flyStatusLine();
void   flyNote(const char* fmt, ...);          // one line (45 chars) into the flight log - NVS, the page's "flight log": heading snaps, sensor resets, refused disarms                        // key m / e
