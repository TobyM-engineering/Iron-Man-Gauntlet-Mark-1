#include "haptic.h"
#include "config.h"
#include "hal.h"
#include <GauntletBenchIO.h>
#if !HW_FAKE
#include <Wire.h>
#include "buses.h"
#endif

static bool present = false, on = false;

#if !HW_FAKE
// DRV2605L registers (TI datasheet): 0x00 STATUS, 0x01 MODE, 0x02 RTP_INPUT, 0x03 LIBRARY_SEL,
// 0x1A FEEDBACK_CONTROL (bit 7: 0 = ERM), 0x1D CONTROL3 (bit 5: ERM open loop).
static bool wr(uint8_t reg, uint8_t v) {
  TwoWire& w = busWire(2);
  w.beginTransmission(DRV2605_ADDR); w.write(reg); w.write(v);
  return w.endTransmission() == 0;
}
static int rd(uint8_t reg) {
  TwoWire& w = busWire(2);
  w.beginTransmission(DRV2605_ADDR); w.write(reg);
  if (w.endTransmission() != 0) return -1;
  if (w.requestFrom((int)DRV2605_ADDR, 1) != 1) return -1;
  return w.read();
}
#endif

void hapticBegin() {
  on = false;
#if HW_FAKE
  present = true;
#else
  int st = rd(0x00);                                              // STATUS: bits 7:5 = device id (DRV2605L reads 7)
  present = st >= 0;
  if (present) {
    wr(0x01, 0x00);                                               // MODE: out of standby, internal trigger
    int fb = rd(0x1A); if (fb >= 0) wr(0x1A, (uint8_t)(fb & 0x7F));          // ERM
    int c3 = rd(0x1D); if (c3 >= 0) wr(0x1D, (uint8_t)(c3 | 0x20));          // ERM open loop: RTP amplitude = a fraction of VDD
    wr(0x03, 0x01);                                               // ERM library A (unused in RTP, set for completeness)
    wr(0x02, 0x00);                                               // amplitude 0
    wr(0x01, 0x40);                                               // standby until a tick
  }
#endif
  Bench.printf("haptic: DRV2605L at 0x%02X on bus 2 %s - the #1201 motor on its OUT pins, open-loop ERM, tick level %u/127\n",
               DRV2605_ADDR, present ? "found" : "ABSENT (haptic ticks will be skipped)", HAPTIC_RTP_LEVEL);
}

void hapticStart() {
  if (!present || on) return;
  on = true;
#if !HW_FAKE
  wr(0x01, 0x05);                                                 // RTP mode (also leaves standby)
  wr(0x02, HAPTIC_RTP_LEVEL);
#endif
}

void hapticStop() {
  if (!on) return;
  on = false;
#if !HW_FAKE
  wr(0x02, 0x00);
  wr(0x01, 0x40);                                                 // standby
#endif
}

bool hapticPresent() { return present; }
bool hapticOn()      { return on; }
