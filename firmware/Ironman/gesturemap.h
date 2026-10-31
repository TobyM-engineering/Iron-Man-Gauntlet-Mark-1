// The gesture map - Toby's (2026-10-21; it replaced the placeholders of 2026-10-07): the table is at the top of
// gesturemap.cpp, the lines in config.h GMAP_*. Thumb clicks in a finger pose (gesture.cpp hands them over), the four
// fingers' fist / open / pulse moves (polled here), the wrist aligned for every one, and the answer in the hand (one buzz
// done, two refused). The drone's arm / disarm and the red laser stay HandBench's (fly.cpp, laser.cpp) and use the same
// alignment and buzz from here.
#pragma once
#include <Arduino.h>

void        gmapPoll();                    // every loop(), after flyPoll(): the fist / open / pulse gestures
void        gmapTriple(bool closedHand);   // gesture.cpp: a triple-crunch was detected (closedHand = HandBench's closed hand)
bool        gmapThumbDouble();             // gesture.cpp: a thumb double - true = it was used (the green laser off), the clicks are forgotten
bool        gmapAligned(char* why, size_t n);   // the wrist in line with the forearm (the drone's arming check); false = why
void        gmapBuzz(bool ok);             // one buzz = done, two = refused
bool        gmapThumbIn();                 // the thumb tucked in (its highest curl over the last GMAP_THUMB_LOOK_MS) - fly.cpp: a thumb-in pulse is the plates', not the drone's
bool        gmapTwoFingers();              // the green laser's pose now (index + middle out, ring + pinky in) - fly.cpp: its four-finger measure reads half a fist, never an arm
uint32_t    gmapCameraDoubleMs();          // when the camera's two-pulse gesture last completed - fly.cpp: never an arm
const char* gmapPoseText();                // the hand right now, in words (the log, key m)
