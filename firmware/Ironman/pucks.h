// The BuckPuck bank: seven 3023-D-N-1000 (1 A fixed, no CTRL wire) in three groups, each group's Vin- cut
// low-side by one D4184 MOSFET module whose TRIG hangs on a PCA9685 channel (Toby, 2026-10-06 final plan):
//   group 1 = the three 20 mm palm LEDs (XM-L2, pucks 1-3)      PCA_CH_PUCK_GROUP[0]
//   group 2 = the two square palm LEDs (10 mm, pucks 6-7)      PCA_CH_PUCK_GROUP[1]
//   group 3 = the two finger LEDs (index + middle, pucks 4-5)   PCA_CH_PUCK_GROUP[2]
// On / off only (the channel's full-ON / full-OFF bits) - nothing here ever dims. Bursts only - the LEDs
// sit on aluminium that absorbs heat but does not shed it; the heat model (pucks.cpp) ends a burst, these are backstops:
//   groups 1-2 (palm): on <= PUCK_ON_MAX_MS_PALM, then off >= PUCK_OFF_MIN_MS_PALM
//   group 3 (fingers): on <= PUCK_ON_MAX_MS_FINGER, then off >= PUCK_OFF_MIN_MS_FINGER
//   every group <= PUCK_DUTY_MAX of the last PUCK_DUTY_WINDOW_MS
// Refused: pack under PUCK_SHOW_FLOOR_V, a cascade running, the drone armed or flipping, mode SAFE,
// the PCA absent. Every group goes off on an OTA start and on goSafe().
#pragma once
#include <Arduino.h>
#include "config.h"

void        pucksBegin();
void        pucksPoll();                               // every loop(): the burst limits
bool        puckGroupOn(uint8_t g, const char* why);   // 0..2; false = refused (pucksLastRefusal() says why)
bool        pucksRefusedForBattery();                  // (2026-10-21) the last puckGroupOn refusal was the battery (LOW, or the load-scaled start floor)
void        puckGroupOff(uint8_t g);
void        pucksAllOff(const char* why);
bool        puckGroupIsOn(uint8_t g);
bool        puckIsOn(uint8_t idx);                     // 0..6: lit = its group is on
uint8_t     pucksOnCount();                            // pucks lit, 0..7
bool        pucksBranchOn();                           // the bank is live = at least one group on (no pin of its own)
const char* pucksBlockText();                          // why nothing may light right now ("" = allowed)
const char* pucksLastRefusal();
bool        pucksShow(const char* name);               // "repulsor" = groups 1 + 2 (the palm), "fingers" = group 3, "all"; false = refused
uint32_t    pucksBursts();
String      pucksJson();                               // {"branch":0,"on":[g1,g2,g3],"ch":[..],"lit":[7 pucks],"cool":[ms],"duty":[s],...}
String      pucksStatusLine();
float       pucksPlateC();                             // the palm 20 mm LEDs' aluminium, modelled (config PLATE_*: start refused at 48 C, a burst ends at 55 C)
float       pucksPalm10C();                            // the palm 10 mm LEDs' aluminium (2026-10-20: a body of its own)
float       pucksTabC();                               // a finger's aluminium stack
uint32_t    pucksHeatCuts();                           // bursts the cap ended
void        pucksThermalBoot(bool powerOn);            // at boot (pucksBegin): a real power-up cold, any other reset from RTC memory (else at the cap)
float       pucksBankC();                              // the LED-driver bank's (2026-10-08: DRIVER_HEAT_W per lit driver)
void        pucksThermalSet(float palmC, float tabC, float bankC = PLATE_AMBIENT_C);   // the self-test: put the model at a temperature (PLATE_AMBIENT_C = cold)
