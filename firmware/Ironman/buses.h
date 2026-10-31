// The two I2C buses. Wire = bus 1 (arm BNO085), Wire1 = bus 2 (hand BNO085).
#pragma once
#include <Arduino.h>
#include <Wire.h>

void        busesBegin();                        // both buses up at I2C_HZ
void        busesScan();                         // every address on both buses, annotated - the boot report and key i
TwoWire&    busWire(uint8_t bus);                // 1 or 2
const char* busName(uint8_t bus);
bool        busAck(uint8_t bus, uint8_t addr);   // does one address answer?
