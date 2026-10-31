// show_bus.h - the puck's "show bus" (header pins 9 SDA / 10 SCL, Wire1): the MLX90640 thermal camera and the
// SSD1306 OLED, run by ONE task pinned to core 0. Included by app.cpp only (hardware).
//
//   camera  read ONLY while the thermal page is on (want): chess mode, 18 bit, 8 Hz; getFrame() reads both
//           sub-pages (~250 ms, mostly waiting on the bus) and computes 768 temperatures; the newest frame, its range,
//           centre and hottest pixel are handed to the renderer (thermal_snapshot). While the 3D view is up the camera
//           is configured but never read, so the 3D view never shares core 0 or the bus with it.
//   OLED    oled_view.h, redrawn every OLED_PERIOD_MS from the status the loop hands over (show_status)
//   absent  a missing device is probed again every SHOW_PROBE_MS; nothing blocks without it
//   sim     "mlxsim 1" (no camera wired): a made-up, moving thermal scene, PLUS the camera's per-pixel arithmetic on
//           made-up data at the camera's rate, so the page and its load can be tested and measured
//
// Core 0 also rasterizes the bottom half of every strip while the 3D view is up (app.cpp); this task runs at a lower
// priority than that helper and mostly waits on the bus.
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include "config.h"
#include "oled_view.h"
#include "badpix.h"

namespace show {

struct State {
  volatile bool mlx = false, oled = false;
  volatile bool want = false;    // the thermal page is on: read the camera
  volatile bool sim = false;
  volatile uint32_t frames = 0, readMs = 0, calcUs = 0;
  float px[768];                 // the newest thermal frame, row-major 32 x 24, deg C
  float lo = 20, hi = 30, centre = 25; int hx = 16, hy = 12;
  oledv::Status st;
  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t loops = 0;
  volatile uint32_t probes = 0, probeUs = 0, probeMaxUs = 0;   // probe passes for an absent device: count, last and longest time
  // (2026-10-16, the arm session, Toby's OK) the camera's reads, reported to the arm as $L lines while the page is up:
  // the library waits for 'frame ready' with no timeout, so the task waits for it here first (MLX_READY_MS)
  volatile int lastErr = 0, lastStatus = -2;     // the last getFrame() error (-100 = never ready); the status register then
  volatile uint32_t fails = 0, consec = 0, notReady = 0, rebegins = 0;
  volatile uint8_t slow = 0;                      // 0 400 kHz 8 Hz, 1 400 kHz 4 Hz, 2 100 kHz 2 Hz - one step down per MLX_SLOW_AFTER failures in a row
};
inline State& S() { static State s; return s; }

inline void show_status(const oledv::Status& st) { State& s = S(); portENTER_CRITICAL(&s.lock); s.st = st; portEXIT_CRITICAL(&s.lock); }
inline void show_sim(bool on) { S().sim = on; }
inline void show_want(bool on) { S().want = on; }

// the renderer's copy of the newest frame (3 KB under the lock - about 10 us)
inline uint32_t thermal_snapshot(float* out, float& lo, float& hi, float& centre, int& hx, int& hy) {
  State& s = S();
  portENTER_CRITICAL(&s.lock);
  memcpy(out, s.px, sizeof s.px); lo = s.lo; hi = s.hi; centre = s.centre; hx = s.hx; hy = s.hy;
  uint32_t n = s.frames;
  portEXIT_CRITICAL(&s.lock);
  return n;
}

static Adafruit_MLX90640* mlxDev = nullptr;
static Adafruit_SSD1306* oledDev = nullptr;

inline bool probe(uint8_t a) { Wire1.beginTransmission(a); return Wire1.endTransmission() == 0; }
static constexpr uint32_t MLX_READY_MS = 1500;    // a sub-page comes every 125 ms at 8 Hz, 500 ms at 2 Hz
static constexpr uint32_t MLX_SLOW_AFTER = 6;     // failed reads in a row before the bus drops to 100 kHz (the long wrist wires)
inline int mlx_status() {                         // the status register 0x8000 (bit 3 = a new sub-page is ready); -1 = no answer
  Wire1.beginTransmission(MLX_ADDR); Wire1.write(0x80); Wire1.write(0x00);
  if (Wire1.endTransmission(false) != 0) return -1;
  if (Wire1.requestFrom((uint8_t)MLX_ADDR, (uint8_t)2) != 2) return -1;
  int hi = Wire1.read(), lo = Wire1.read(); return (hi << 8) | lo;
}
inline bool mlx_wait_ready(uint32_t ms) {
  uint32_t t0 = millis(); int st = -1;
  for (;;) { st = mlx_status(); if (st >= 0 && (st & 0x0008)) { S().lastStatus = st; return true; } if (millis() - t0 > ms) { S().lastStatus = st; return false; } vTaskDelay(pdMS_TO_TICKS(5)); }
}

// the camera's own list of faulty pixels: EEPROM words 0x2440 + pixel, 0 = broken, bit 0 = outlier (Melexis' driver,
// ExtractDeviatingPixels) - read here because the Adafruit library never corrects them (badpix.h blends them in)
inline bool mlx_read_words(uint16_t addr, int n, uint16_t* out) {
  Wire1.beginTransmission(MLX_ADDR); Wire1.write(addr >> 8); Wire1.write(addr & 0xFF);
  if (Wire1.endTransmission(false) != 0) return false;
  if (Wire1.requestFrom((uint8_t)MLX_ADDR, (uint8_t)(2 * n)) != 2 * n) return false;
  for (int k = 0; k < n; k++) { int hi = Wire1.read(), lo = Wire1.read(); out[k] = (uint16_t)((hi << 8) | lo); }
  return true;
}
inline void mlx_load_bad() {
  uint16_t w[32]; int broken = 0, outlier = 0;
  for (int i = 0; i < 768; i += 32) {
    if (!mlx_read_words(0x2440 + i, 32, w)) { Serial.println("show bus: MLX90640 pixel list - read failed"); return; }
    for (int k = 0; k < 32; k++) {
      if (w[k] == 0) { badpix::mark_ee(i + k); broken++; Serial.printf("show bus: MLX90640 pixel (%d,%d) broken (the camera's own list)\n", (i + k) % 32, (i + k) / 32); }
      else if (w[k] & 1) { badpix::mark_ee(i + k); outlier++; Serial.printf("show bus: MLX90640 pixel (%d,%d) an outlier (the camera's own list)\n", (i + k) % 32, (i + k) / 32); }
    }
  }
  Serial.printf("show bus: MLX90640 pixel list - %d broken, %d outliers, blended in\n", broken, outlier);
}
// the pixels found while running (badpix.h), kept across power-ups
static Preferences camPrefs;
inline void bad_load() {
  camPrefs.begin("puckcam", false);
  uint16_t l[badpix::MAX_RUN]; size_t n = camPrefs.getBytesLength("bad") / 2;
  if (n > (size_t)badpix::MAX_RUN) n = badpix::MAX_RUN;
  if (n) camPrefs.getBytes("bad", l, n * 2);
  for (size_t k = 0; k < n; k++) { badpix::mark_saved(l[k]); Serial.printf("show bus: pixel (%d,%d) masked (found before)\n", l[k] % 32, l[k] / 32); }
}
inline void bad_save() {
  uint16_t l[badpix::MAX_RUN]; int n = badpix::found_list(l, badpix::MAX_RUN);
  if (n) camPrefs.putBytes("bad", l, n * 2); else camPrefs.remove("bad");
}

inline bool mlx_begin() {
  if (!probe(MLX_ADDR)) return false;
  if (!mlxDev->begin(MLX_ADDR, &Wire1)) return false;
  mlx_load_bad();
  mlxDev->setMode(MLX90640_CHESS);
  mlxDev->setResolution(MLX90640_ADC_18BIT);
  mlxDev->setRefreshRate(MLX90640_8_HZ);
  Serial.printf("show bus: MLX90640 at 0x%02X, serial %04X%04X%04X, chess, 18 bit, 8 Hz (read only while the thermal page is on)\n", MLX_ADDR,
                mlxDev->serialNumber[0], mlxDev->serialNumber[1], mlxDev->serialNumber[2]);
  return true;
}

inline bool oled_begin() {
  if (!probe(OLED_ADDR)) return false;
  if (!oledDev->begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, true, false)) return false;   // no Wire.begin: the task owns Wire1
  oledDev->setTextWrap(false);
  oledDev->clearDisplay(); oledDev->display();
  Serial.printf("show bus: SSD1306 at 0x%02X\n", OLED_ADDR);
  return true;
}

inline void publish(float* f) {
  State& s = S();
  badpix::Event e = badpix::fix(f);   // the faulty pixels blended into their neighbours, before the range and the hottest spot
  if (e.marked >= 0) Serial.printf("show bus: pixel (%d,%d) stays off its neighbours - masked\n", e.marked % 32, e.marked / 32);
  if (e.healed >= 0) Serial.printf("show bus: pixel (%d,%d) agrees with its neighbours again - unmasked\n", e.healed % 32, e.healed / 32);
  if (e.marked >= 0 || e.healed >= 0) bad_save();
  float lo = 1e9f, hi = -1e9f; int hi_i = 0;
  for (int i = 0; i < 768; i++) { if (f[i] < lo) lo = f[i]; if (f[i] > hi) { hi = f[i]; hi_i = i; } }
  float c = 0.25f * (f[11 * 32 + 15] + f[11 * 32 + 16] + f[12 * 32 + 15] + f[12 * 32 + 16]);
  portENTER_CRITICAL(&s.lock);
  memcpy(s.px, f, sizeof s.px); s.lo = lo; s.hi = hi; s.centre = c; s.hx = hi_i % 32; s.hy = hi_i / 32; s.frames++;
  portEXIT_CRITICAL(&s.lock);
}

// the camera's per-pixel arithmetic (MLX90640_CalculateTo) on made-up data, one sub-page (384 px): only its LOAD
inline void sim_calc(int page) {
  static uint32_t seed = 1; static volatile float sink = 0;
  float ta = 24.5f, tr = ta - 8, taTr = powf(tr + 273.15f, 4) - (powf(tr + 273.15f, 4) - powf(ta + 273.15f, 4)) / 0.95f;
  for (int p = 0; p < 768; p++) {
    if (((p / 32) + p) % 2 != page) continue;
    seed = seed * 1664525u + 1013904223u;
    float irData = (float)(int16_t)(seed >> 16) * 0.01f;
    irData = irData * 1.02f - 23.0f * (1 + 0.0039f * (ta - 25)) * (1 + 0.0015f * 0.0f);
    irData = (irData - 0.32f * (1 - 0.0031f * (ta - 25))) / 0.95f;
    float alpha = 1.2e-7f * (1 + 0.004f * (ta - 25));
    float Sx = 0.0005f * sqrtf(sqrtf(alpha * alpha * alpha * (fabsf(irData) + alpha * taTr)));
    float To = sqrtf(sqrtf(fabsf(irData / (alpha * (1 - 0.0011f * 273.15f) + Sx) + taTr))) - 273.15f;
    int range = To < -40 ? 0 : (To < 0 ? 1 : (To < 100 ? 2 : 3));
    To = sqrtf(sqrtf(fabsf(irData / (alpha * 0.98f * (1 + 0.001f * (To - range * 10))) + taTr))) - 273.15f;
    sink += To;
  }
}
// a made-up, moving scene for the page: a warm hand that sways, a hot mug, a 22 C room
inline void sim_scene(float* f, float t) {
  float hx = 12 + 3 * sinf(t * 0.7f), hy = 15 + 1.5f * sinf(t * 0.5f);
  for (int y = 0; y < 24; y++)
    for (int x = 0; x < 32; x++) {
      float v = 21.5f + 0.8f * y / 24 + 0.3f * sinf(x / 5.0f);
      float hand = expf(-(((x - hx) / 5.5f) * ((x - hx) / 5.5f) + ((y - hy) / 6.5f) * ((y - hy) / 6.5f)));
      for (int k = 0; k < 4; k++) { float fx = hx - 5 + 3 * k; if (y < hy - 3 && y > hy - 12) hand += 0.8f * expf(-((x - fx) / 1.1f) * ((x - fx) / 1.1f)); }
      v += 11.5f * fminf(1.0f, 1.3f * hand);
      float mug = expf(-(((x - 25) / 2.6f) * ((x - 25) / 2.6f) + ((y - 13) / 3.4f) * ((y - 13) / 3.4f)));
      v += 36 * fminf(1.0f, mug * 1.4f);
      f[y * 32 + x] = v;
    }
}

inline void task(void*) {
  Wire1.begin(SHOW_SDA, SHOW_SCL, SHOW_BUS_HZ);   // its interrupt is installed here, on core 0
  Wire1.setTimeOut(50);
  bad_load();
  State& s = S();
  static float frame[768];
  uint32_t probeT = 0, oledT = 0, simT = 0;
  int simPage = 0;
  for (;;) {
    s.loops++;
    uint32_t now = millis();
    if ((!s.mlx || !s.oled) && now - probeT >= SHOW_PROBE_MS) {
      probeT = now;
      uint32_t u0 = micros();
      if (!s.mlx) s.mlx = mlx_begin();
      if (!s.oled) s.oled = oled_begin();
      uint32_t du = micros() - u0;
      s.probes++; s.probeUs = du; if (du > s.probeMaxUs) s.probeMaxUs = du;
    }
    if (s.want && s.mlx) {
      uint32_t t0 = millis();
      int r = mlx_wait_ready(MLX_READY_MS) ? mlxDev->getFrame(frame) : -100;   // both sub-pages; blocks on the bus (this task only), ~250 ms at 8 Hz
      if (r == 0) { s.readMs = millis() - t0; s.consec = 0; publish(frame); }
      else {
        s.lastErr = r; s.fails++; s.consec++; if (r == -100) s.notReady++;
        if (!probe(MLX_ADDR)) { s.mlx = false; Serial.println("show bus: MLX90640 lost"); }
        else if (s.consec >= MLX_SLOW_AFTER && s.slow < 2) {   // too slow a read for the frame rate (-8): a slower camera, then a slower bus
          s.slow++; s.consec = 0;
          if (s.slow == 1) mlxDev->setRefreshRate(MLX90640_4_HZ); else { Wire1.setClock(100000); mlxDev->setRefreshRate(MLX90640_2_HZ); }
          Serial.printf("show bus: MLX90640 - %u failed reads (last %d): now %s\n", (unsigned)MLX_SLOW_AFTER, r, s.slow == 1 ? "400 kHz 4 Hz" : "100 kHz 2 Hz");
        } else if (s.consec >= 2 * MLX_SLOW_AFTER) {           // still nothing: start the camera again (its settings, its calibration)
          s.consec = 0; s.rebegins++; s.mlx = mlx_begin(); if (s.slow == 1) mlxDev->setRefreshRate(MLX90640_4_HZ); else if (s.slow == 2) mlxDev->setRefreshRate(MLX90640_2_HZ);
          Serial.printf("show bus: MLX90640 started again (%lu) after %d\n", (unsigned long)s.rebegins, r);
        }
      }
    } else if (s.want && s.sim && now - simT >= 125) {   // the camera's rate: a sub-page every 125 ms
      simT = now;
      uint32_t u0 = micros();
      sim_calc(simPage);
      s.calcUs = micros() - u0;
      if ((simPage ^= 1) == 0) { sim_scene(frame, now / 1000.0f); publish(frame); }
    }
    if (s.oled && now - oledT >= OLED_PERIOD_MS) {
      oledT = now;
      oledv::Status st;
      portENTER_CRITICAL(&s.lock); st = s.st; portEXIT_CRITICAL(&s.lock);
      oledv::draw(*oledDev, st, now);
      oledDev->display();
      if (!probe(OLED_ADDR)) { s.oled = false; Serial.println("show bus: SSD1306 lost"); }
    }
    vTaskDelay(pdMS_TO_TICKS(s.want && s.mlx ? 1 : 10));
  }
}

inline void begin() {
  mlxDev = new Adafruit_MLX90640();                    // ~5 KB of calibration tables: taken before the triangle buffer
  oledDev = new Adafruit_SSD1306(128, 64, &Wire1, -1); // (its 1 KB frame buffer comes with its begin())
  xTaskCreatePinnedToCore(task, "showbus", 8192, nullptr, 1, nullptr, 0);   // core 0, below the raster helper
}

}  // namespace show
