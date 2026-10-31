// panel.h - the puck's control panel: a button press on the round screen arrives as a $C line (GauntletLink.h) and is
// done here by the gestures' own functions under the gestures' own rules (2026-10-20, Toby: "a whole screen that I could
// just activate anything with ... if gestures are down"). The 3D view's touches stay view-only; only $C acts.
#pragma once
#include <Arduino.h>
#include <GauntletLink.h>

void     panelCommand(const glink::Command& c);   // s3link.cpp calls it once per new seq; answers with exactly one $E
uint32_t panelPresses();                          // commands taken since boot
uint32_t panelRefused();                          // ... of them BLOCKED or REFUSED
String   panelStatusLine();
void     panelPoll();                             // every loop(): the RC frames back on after a puck update ($C ACT_PUCK_UPDATE)
bool     panelPuckUpdating();                     // the RC frames are off for a puck update now
