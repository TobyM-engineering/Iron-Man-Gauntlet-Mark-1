// The puck (round display, Waveshare ESP32-S3-Touch-LCD-1.28) on UART2, protocol GauntletLink v2
// (lib/GauntletLink/src/GauntletLink.h has every field): $S status every S3_STATUS_MS, $G live state every
// S3_LIVE_MS, the puck's heartbeat ($H, with its protocol version) and touch events ($T) back. Never blocks:
// a frame the TX ring has no room for is dropped and counted, RX is parsed at most S3_RX_MAX_PER_POLL bytes a
// loop, and a missing / dead / wrong-version puck changes nothing but the counters and one log line. Touch is
// VIEW-ONLY (Toby, 2026-10-07): the puck uses every touch for its own 3D view (drag orbits, hold zooms out, double
// tap returns to the hand); the arm logs and counts each event, puts it in the feed, and does nothing else
// (Ironman.ino onTouch).
#pragma once
#include <Arduino.h>
#include <GauntletLink.h>

void        s3linkBegin();
void        s3linkPoll();                       // every loop(): RX (bounded), events, $G and $S on time
bool        s3linkUp();                         // a v2 heartbeat inside S3_HEARTBEAT_LOST_MS
uint32_t    s3linkHeartbeats();
uint32_t    s3linkStatusSent();                 // $S frames put on the wire (or counted, in the self-test)
uint32_t    s3linkLiveSent();                   // $G frames
uint32_t    s3linkTouches();
uint32_t    s3linkTxDrops();                    // frames dropped because the TX ring had no room
uint32_t    s3linkPollMaxUs();                  // the slowest s3linkPoll() since boot, microseconds (self-test excluded)
uint32_t    s3linkWriteMaxUs();                 // the slowest single UART write
long        s3linkTheirVersion();               // the puck's protocol version from its heartbeat, -1 = never heard
uint32_t    s3linkVersionRejects();             // lines ignored for a wrong protocol version (v1 lines included)
const glink::Parser& s3linkParser();
glink::Touch s3linkLastTouch(uint16_t* x, uint16_t* y, uint32_t* ageMs);
void        s3linkRequestPage(uint8_t page);    // 1 status 2 thermal 3 boot 4 flight (sent in the next $S)
void        s3linkSetFlag(uint8_t bit, bool on);
const char* s3linkLastStatusLine();             // the last $S as sent (the self-test parses it back)
const char* s3linkLastLiveLine();               // the last $G as sent
void        s3linkAct(uint8_t act, uint8_t res, uint8_t need);   // a gesture / action event (glink::ACT_*, RES_DONE / RES_BLOCKED / RES_REFUSED, the lowest mode it needs 0..2) -> one $E line, sent once; a BLOCKED one is logged
uint32_t    s3linkEventsSent();
uint32_t    s3linkEventsBlocked();             // gestures the mode switch blocked since boot
const char* s3linkLastEventLine();             // the last $E as sent (the self-test parses it back)
String      s3linkJson();                       // the feed's "display": counters, the puck's heartbeat, timing
String      s3linkStatusLine();
void        s3linkOnTouch(void (*fn)(glink::Touch t, uint16_t x, uint16_t y));   // Ironman.ino's handler (log only: touch is view-only)
void        s3linkOnCommand(void (*fn)(const glink::Command& c));   // a $C (a control-panel button, 2026-10-20): panel.cpp, once per seq
uint32_t    s3linkCommands();                   // $C presses taken (repeats of the same seq not counted)
bool        s3linkUartOn();                     // UART2 was started (false only for the fake build on the HandBench devkit)
void        s3linkTestMode(bool on);            // the self-test: nothing to the wire, real RX discarded (its fake clock outruns the line)
void        s3linkFloodTest(uint16_t frames, uint32_t& maxUs, uint32_t& drops, uint32_t& accepted);   // widest $G frames back to back through the real UART
#if HW_FAKE
void        s3linkInject(const char* line);     // a line as the puck would send it (with $ ... *CCCC\n)
#endif
