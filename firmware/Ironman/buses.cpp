#include "buses.h"
#include "config.h"
#include <GauntletBenchIO.h>

TwoWire&    busWire(uint8_t bus) { return bus == 2 ? Wire1 : Wire; }
const char* busName(uint8_t bus) { return bus == 2 ? "bus 2 (palm)" : "bus 1 (forearm)"; }

bool busAck(uint8_t bus, uint8_t addr) {
#if HW_FAKE
  (void)bus; (void)addr; return false;                 // nothing is on a bus in a fake build; every module fakes its own device
#else
  TwoWire& w = busWire(bus);
  w.beginTransmission(addr);
  return w.endTransmission() == 0;
#endif
}

void busesBegin() {
#if HW_FAKE
  Bench.println("i2c: fake build - no bus is started");
#else
  bool ok1 = Wire.begin(PIN_I2C1_SDA, PIN_I2C1_SCL, I2C1_HZ);
  bool ok2 = Wire1.begin(PIN_I2C2_SDA, PIN_I2C2_SCL, I2C2_HZ);
  Wire.setTimeOut(50); Wire1.setTimeOut(50);           // a stretched or stuck line costs 50 ms, not a second
  Bench.printf("i2c: bus 1 (forearm) SDA %u SCL %u %s at %u kHz | bus 2 (palm) SDA %u SCL %u %s at %u kHz\n",
               PIN_I2C1_SDA, PIN_I2C1_SCL, ok1 ? "up" : "FAILED to init", (unsigned)(I2C1_HZ / 1000),
               PIN_I2C2_SDA, PIN_I2C2_SCL, ok2 ? "up" : "FAILED to init", (unsigned)(I2C2_HZ / 1000));
#endif
}

static const char* whatIs(uint8_t bus, uint8_t a) {
  for (uint8_t i = 0; i < I2C_DEVICE_COUNT; i++) if (I2C_DEVICES[i].bus == bus && I2C_DEVICES[i].addr == a) return I2C_DEVICES[i].name;
  switch (a) {
    case 0x4B: return "BNO08x with AD0 high - the gauntlet's two are strapped to 0x4A (AD0, PS0, PS1 to GND)";
    case 0x3C: case 0x3D: return "SSD1306 display - it belongs on the round display's bus, not here";
    case 0x33: return "MLX90640 - it belongs on the round display's bus, not here";
    case 0x68: case 0x69: return "MPU6050 / DS3231";
    default:   return "? (not in config.h's I2C_DEVICES)";
  }
}

static void scanOne(uint8_t bus) {
#if HW_FAKE
  Bench.printf("%s: (fake build, no bus)\n", busName(bus));
#else
  TwoWire& w = busWire(bus);
  Bench.printf("%s:\n", busName(bus));
  uint8_t found = 0, timeouts = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    w.beginTransmission(a);
    uint8_t err = w.endTransmission();       // 0 ack, 2 nack (nothing there), 5 timeout (a line held low)
    if (err == 0)      { found++; Bench.printf("  0x%02X  %s\n", a, whatIs(bus, a)); }
    else if (err == 5) timeouts++;
  }
  for (uint8_t i = 0; i < I2C_DEVICE_COUNT; i++) {   // what the design expects and did not answer
    if (I2C_DEVICES[i].bus != bus) continue;
    w.beginTransmission(I2C_DEVICES[i].addr);
    if (w.endTransmission() != 0) Bench.printf("  0x%02X  %s - %s\n", I2C_DEVICES[i].addr, I2C_DEVICES[i].name, I2C_DEVICES[i].optional ? "absent (optional)" : "ABSENT");
  }
  if (found) return;
  if (timeouts) Bench.printf("  nothing answered and %u addresses timed out: SDA or SCL is being held low (short, unpowered module, SDA/SCL swapped)\n", timeouts);
  else          Bench.println("  nothing (bus idle, no device answered)");
#endif
}

void busesScan() { scanOne(1); scanOne(2); }
