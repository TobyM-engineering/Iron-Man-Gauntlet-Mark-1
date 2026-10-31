// Thumb triple-crunch and finger dips, detected on the board at the flex sample
// rate (no WiFi in the loop). A crunch counts only when it is fast: the thumb's
// millivolts fall by GESTURE_FLICK_FRAC of the flat-to-fist swing within
// GESTURE_FLICK_MS, then come back up a little or pause. Three such flicks alike
// in rhythm and size fire the action: hand open -> ring off -> on (at the level it
// had), white -> red, red -> white (other colours left alone); middle, ring and
// pinky closed (index ignored) -> ring off, whatever it showed. A pinky dip (down
// and back, the other fingers still) = one level brighter, an index dip = one
// level dimmer, while the ring is on. The numbers come from the sim page's finger
// calibration (GET :8080/gesture?tf=N&tc=N&fl=a,b,c,d&fx=a,b,c,d): thumb flat mV,
// thumb closed as far as a fist allows, and index..pinky flat / closed mV (mV
// falls as a finger bends; "closed" = GESTURE_FIST_FRAC of the way). NVS.
#pragma once
#include <Arduino.h>

void     gestureBegin();                    // thresholds from NVS
void     gesturePoll();                     // every loop(), after flexPoll()
void     gestureFake();                     // key n: run the triple-crunch action now, fingers as they are
String   gestureSet(const String& query);   // GET /gesture[?tf=N&tc=N&fl=a,b,c,d&fx=a,b,c,d | f=a,b,c,d | fake=crunch|up|down | trace=1] -> JSON state (+ "msg")
String   gestureJson();                     // the same state JSON, no message - the telemetry page's /tele carries it
uint32_t gestureCount();                    // triple-crunches so far - the feed's "dbl"
uint32_t gestureDipCount();                 // finger dips so far (both fingers) - the feed's "dip"
uint32_t gestureDipUpCount();               // pinky dips (brighter) so far - the puck link's gesture events
uint32_t gestureDipDownCount();             // index dips (dimmer) so far
void     gestureFlightClicksReset();         // at arming: this flight's thumb clicks start from zero
bool     gestureFlightClicks(char* b, size_t n);   // at disarm: the line for the flight log (false = no clicks seen)
uint32_t gestureDisarmCount();              // the drone's disarm: POINTING (index up, middle, ring and pinky closed), then two thumb presses (the second counted as it starts), only while armed or flipping: fly.cpp disarms on any change
uint32_t gestureDoubleCount();              // thumb DOUBLE crunches so far (two flicks in rhythm and alike, judged on every flick): the drone's disarm (fly.cpp). A triple counts one on its second flick
bool     gestureFist();                     // the closed hand right now: middle, ring and pinky below their thresholds (the laser uses the same rule)
float    gestureHandOpenMv();               // how far those three read ABOVE their closed thresholds, averaged (mV; 0 = at the threshold, + = more open); NAN without thresholds
float    gestureFingerCurl(uint8_t flexIdx); // 0 thumb .. 4 pinky: curl 0 (flat) .. 1 (closed) as a straight line in sensor RESISTANCE between the page's flat and closed (the model's curve); NAN if not calibrated. The thumb's "closed" is the fist-limited one. fly.cpp's throttle
String   gestureStatusLine();               // key m
void     gestureCaptureStart(bool fist);    // keys K (flat) / J (fist): average every finger for GESTURE_CAPTURE_MS, then save it like the page's steps 4 / 5 (no computer needed; the sim page does the same over WiFi)
bool     gestureCapturing();
const char* gestureCalPose(bool fist, const uint16_t mv[5]);
const char* gestureCalThumbAlone(uint16_t mv);   // (2026-10-21) calibration step 3: the thumb closed on its own (its full curl); "" = saved
float       gestureThumbCurlFull();              // the thumb's curl against its closed-alone value (the model), else the fist-limited one   // (2026-10-21) the puck's calibration steps 1 / 2: one pose (thumb..pinky mV) saved, the other kept; "" = saved, else why it was refused
