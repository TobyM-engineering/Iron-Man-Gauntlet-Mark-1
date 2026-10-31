// calib.h - the puck's CALIBRATE screen (2026-10-21): $C ACT_CAL with a = the step (GauntletLink.h). One step at a time:
// wait CAL_SETTLE_MS (the tap on the puck jolts the arm), average CAL_AVG_MS, save it to NVS at once, answer ONE $E.
//   1 fingers flat       2 fingers closed      (gestureCalPose: the other pose kept, the thumb never refuses; step 2 is also
//                                               the thumb's closed-in-a-fist, which its gestures use)
//   3 the thumb closed on its own, fingers out (gestureCalThumbAlone: its full curl, the model's thumb)
//   4 the arm straight out, palm down: the zero pose of both IMUs (imuZeroAsync - key z's own)
//   5 the arm hanging straight down   6 straight up: each sensor's forward axis along its limb (imuAxisStart)
// Any mode (it moves nothing); refused while the drone is armed, and while another step is running.
#pragma once
#include <Arduino.h>

void calibStart(uint8_t step);   // panel.cpp, from a $C ACT_CAL
void calibPoll();                // every loop()
bool calibBusy();
