// The red aiming laser on the back of the hand: the TTL #1056 on PCA9685 channel
// PCA_CH_LASER_RED (outputs.cpp drives the channel; this file decides WHEN from the wrist and the fingers).


// Fired by the wrist, judged from the HAND RELATIVE TO THE FOREARM (imuRelative -
// both sensors zeroed together at the sim page's step 1), never from where the
// arm points, so it reads the same palm up, palm down, anywhere: start with the
// wrist flat (no flex, no side, no twist), then a fist (middle, ring, pinky - the
// thumb gesture's rule; the thumb crunched in and the index at least partly closed as well since 2026-10-01, so the drone's thumb-out fist cannot fire it) with a small, quick flex either
// way switches it on; from then on the arm may do anything - only letting the
// fingers go past half switches it off. Key p / GET :8080/laser?force=on|off|auto
// override it for bench checks; /laser?zero=1 zeroes both sensors (step 1).
// Fail-safes: off when either IMU goes quiet, after LASER_MAX_ON_MS (5 min), in
// the first seconds after boot, and the moment an OTA starts.
#pragma once
#include <Arduino.h>

enum class LaserForce : uint8_t { Auto, On, Off };

void       laserBegin();                  // pin low before anything else in setup()
void       laserPoll();                   // every loop(), after imuPoll(), flexPoll(), gesturePoll()
bool       laserOn();
LaserForce laserForce();
void       laserSetForce(LaserForce f);   // key p cycles auto -> on -> off -> auto; the page's buttons set one
void       laserKeyCycle();               // key p
void       laserOff(const char* why);     // OTA start, the shutdown: off and forced off
void       laserStop(const char* why);    // a mode change, goSafe: off now, the wrist gesture still works later (a forced-on is cancelled)
float      laserWristDeg();               // wrist flexion, hand relative to forearm, + = toward the palm; NAN without both sensors zeroed
String     laserJson();                   // the feed's "laser":{...}
String     laserSet(const String& query); // GET :8080/laser?force=on|off|auto -> JSON state + msg; ?zero=1 = the wrist is flat now (the page's step 1), zero both sensors
String     laserStatusLine();             // key m
