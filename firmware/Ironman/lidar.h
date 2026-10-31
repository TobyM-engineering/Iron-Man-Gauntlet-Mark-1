// Garmin LIDAR-Lite v3 (Adafruit #4058) at LIDAR_ADDR on I2C bus 2 (the palm bus, with the hand
// BNO085 and the ADS1115), 5 V with Garmin's 680 uF at its leads, ground = the LiDAR's own return.
// v3 register map: ACQ_COMMAND 0x00 (0x04 = measure with bias correction), STATUS 0x01 bit 0 busy,
// distance = two bytes from 0x8f (0x0f high, 0x10 low, auto-increment), POWER_CONTROL 0x65 (bit 2
// sleep, bit 0 receiver off). The v3 refuses a repeated START, so every read is STOP then START.
// A measurement is triggered every LIDAR_PERIOD_MS while the laser is on (LIDAR_WITH_LASER_ONLY)
// and collected when the busy flag clears; absent, unpowered or unplugged mid-run it is reported
// absent and re-probed every LIDAR_PROBE_MS - nothing blocks or crashes.
#pragma once
#include <Arduino.h>

void     lidarBegin();          // probe once; logs found / absent
void     lidarPoll();           // every loop()
bool     lidarPresent();
uint16_t lidarMm();             // last good distance in mm, 0 = no valid reading yet (or out of range / idle)
uint32_t lidarAgeMs();          // since the last good reading
uint32_t lidarCount();          // good readings since boot
void     lidarLog(bool on);     // "lidar: NNNN mm" in the console at the measurement rate (key q toggles)
bool     lidarLogging();
bool     lidarAsleep();         // the sleep bits are set (idle)
String   lidarJson();           // the feed's "lidar":{...}
String   lidarStatusLine();     // key m
#if HW_FAKE
void     lidarFakeSetCm(uint16_t cm);        // what the fake sensor measures
void     lidarFakePresent(bool p);
uint8_t  lidarFakeLastReadReg();             // the register the driver last read the distance from (the test wants 0x8f)
uint8_t  lidarFakeLastPower();               // the last value written to POWER_CONTROL
#endif
