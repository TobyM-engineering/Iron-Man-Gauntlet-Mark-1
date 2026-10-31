// app.cpp - the puck: Waveshare ESP32-S3-Touch-LCD-1.28 showing the R3 gauntlet.
//
//   LIVE  : the arm's GauntletLink v2 $G frames (20 Hz, UART RX on GPIO16 = header pin 8): the wrist from its two
//           BNO085s, the fingers, each plate from its servos, the lights, the battery (arm_link.h). $S (10 Hz) too.
//           The puck answers with its $H heartbeat once a second on GPIO15 (header pin 7). Typed on the USB serial,
//           a $G / $S line goes through the same parser; other lines are test commands ("help").
//   ARM   : only $S frames: the cascade replayed from the plates state.
//   STILL : no arm yet: the hand still in its neutral pose, nothing lit, the home view (no demo animation).
//   UPDATE: a finger put on the screen within 10 s of power-up and held still UPDATE_HOLD_MS, BOOT held as long while
//           running (or "update"
//           on the USB console) restarts
//           into the updater in the second program slot (firmware/PuckUpdater: WiFi, over-the-air update, back).
//   Show bus (header 9 / 10, show_bus.h): the MLX90640 and the SSD1306, run by a task on core 0.
//   Touch : drag = orbit, double-tap = back to the default view, long press = the link-health view; a sideways drag
//           from the left / right rim = the control panel (panel.h: buttons that send $C to the arm), sideways back.
//   Blank : the arm reports the glove pack CRITICAL -> backlight and panel off until it says otherwise for 5 s.
// The renderer (render.h, view.h, kin.h are all inline and compiled here) is built at -O2: the Arduino ESP32
// default is -Os, which leaves the float loops of the transform, lighting and rasterizer much slower.
#pragma GCC optimize("O2")
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <esp_timer.h>
#include <esp_heap_caps.h>
#include <esp_ota_ops.h>
#define PUCK_NOW_US() esp_timer_get_time()
#define PUCK_CYCLES() ((uint32_t)esp_cpu_get_cycle_count())
#define PUCK_HOT static IRAM_ATTR __attribute__((noinline))   // renderer hot loops in IRAM, not run from the flash cache
#include "config.h"
#include "puck_core.h"
#include "show_bus.h"

// ------------------------------------------------------------------ display
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX() {
    {
      auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = SPI_MHZ * 1000000;
      cfg.freq_read = 16000000;
      cfg.spi_3wire = false;
      cfg.use_lock = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk = 10;
      cfg.pin_mosi = 11;
      cfg.pin_miso = -1;
      cfg.pin_dc = 8;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs = 9;
      cfg.pin_rst = 14;
      cfg.pin_busy = -1;
      cfg.panel_width = 240;
      cfg.panel_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.offset_rotation = 0;
      cfg.readable = false;
      cfg.invert = PANEL_INVERT;
      cfg.rgb_order = PANEL_RGB_ORDER;
      cfg.dlen_16bit = false;
      cfg.bus_shared = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};

static LGFX tft;

static app::Core core;           // all non-hardware state + logic (shared with the browser preview)
static rnd::Frame frame, frame2;   // the two cores' halves of the visible triangles, in ONE buffer (frame's)
static int triTotal = 0;           // that buffer's size; frame2 takes its upper part, re-split every frame
// Split the shared triangle buffer between the two cores from what each needed last frame (dropped ones included), the
// spare shared equally - so neither runs out while the other has room. A sudden jump can drop a few for one frame.
static void split_tri_buffer() {
  int a = frame.n + frame.dropped, b = frame2.n + frame2.dropped, spare = triTotal - a - b;
  int capA = spare > 0 ? a + spare / 2 : triTotal * (a + 1) / (a + b + 2);
  if (capA < 200) capA = 200;
  if (capA > triTotal - 200) capA = triTotal - 200;
  frame.cap = capA;
  frame2.tris = frame.tris + capA; frame2.cap = triTotal - capA;
}
static rnd::Pipe pipe_;
static Preferences prefs;

static hud::Data hudData;        // this frame's HUD values (app::core_hud)
static hud::Look hudLook;
static app::PanelView panelView;   // this frame's control panel (app::panel_view)
// the calibration's arm steps: its words over the small live arm, the battery and the mode symbol, the page dots
static void cal_overlay(uint16_t* fb, uint16_t*, int y0, int rows) {
  app::draw_cal(fb, y0, rows, hudLook, panelView);
  hud::panel_overlay(fb, y0, rows, hudLook, hudData);
  hud::page_dots(fb, y0, rows, hudLook, 1);
}
static void overlay(uint16_t* fb, uint16_t* zb, int y0, int rows) {
  hud::draw(fb, zb, y0, rows, hudLook, hudData);   // behind the model: zb says where the arm is
  hud::page_dots(fb, y0, rows, hudLook, 0);           // the page dots, over everything
}

// ------------------------------------------------------------------ the second core draws half of every strip
// The 3D view's strips are split in two: their top rows drawn here (core 1, Arduino's loop), their bottom rows by
// this helper on core 0 at the same time (rnd::render_frame_split) - the picture is identical, the drawing takes
// about half as long. (The thermal camera is read only while the thermal page is up, when there is no 3D view.)
static TaskHandle_t helperTask = nullptr, loopTask = nullptr;
static void (*volatile helperFn)(void*) = nullptr;
static void* volatile helperCtx = nullptr;
static uint32_t helperJobs = 0;
static void helper_loop(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    helperFn(helperCtx);
    helperJobs++;
    xTaskNotifyGive(loopTask);
  }
}
static void helper_split(void (*fn)(void*), void* ctx) { helperFn = fn; helperCtx = ctx; xTaskNotifyGive(helperTask); }
static void helper_join() { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); }

static uint32_t tWaitUs = 0;   // time spent waiting for the display DMA (profiling, status line)
static void push_strip(int y0, uint16_t* buf, int rows) {
  int64_t tw = esp_timer_get_time();
  tft.waitDMA();                                                        // previous strip done
  tWaitUs += (uint32_t)(esp_timer_get_time() - tw);
  tft.pushImageDMA(0, y0, rnd::W, rows, (const lgfx::swap565_t*)buf);  // returns immediately
}

// ------------------------------------------------------------------ I2C helpers
static bool i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool i2c_read(uint8_t addr, uint8_t reg, uint8_t* buf, int n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// ------------------------------------------------------------------ touch (CST816S @ 0x15)
constexpr uint8_t TOUCH_ADDR = 0x15;
constexpr int TOUCH_INT_PIN = 5;   // the board's TP_INT (Waveshare ESP32-S3-Touch-LCD-1.28)

static void touch_init() {
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW); delay(10);
  digitalWrite(TOUCH_RST, HIGH); delay(60);
  uint8_t id = 0;
  bool ok = i2c_read(TOUCH_ADDR, 0xA7, &id, 1);   // chip id register
  i2c_write(TOUCH_ADDR, 0xFE, 0x01);              // disable auto-sleep (else it stops answering)
  // MotionMask 0: the chip's own double-click mode OFF (round 9 switched it on and the chip then held back quick taps:
  // 6 of 20 double taps; round 7, which worked, never touched it). The puck finds the double taps itself.
  i2c_write(TOUCH_ADDR, 0xEC, 0x00);
  // the interrupt (round 10b): the chip pulls TP_INT low on a touch's start, every report while touching and its end
  // (IrqCtl 0xFA = EnTouch | EnChange); each pulse wakes touch_task, which reads the chip at that moment
  i2c_write(TOUCH_ADDR, 0xFA, 0x60);
  uint8_t mm = 0xFF, as = 0, irq = 0; i2c_read(TOUCH_ADDR, 0xEC, &mm, 1); i2c_read(TOUCH_ADDR, 0xFE, &as, 1); i2c_read(TOUCH_ADDR, 0xFA, &irq, 1);
  Serial.printf("touch CST816S: %s (id 0x%02X), MotionMask 0x%02X (chip double click off), auto-sleep %s (0xFE = 0x%02X), IrqCtl 0x%02X on GPIO %d\n",
                ok ? "found" : "no answer (still polled)", id, mm, as ? "off" : "ON", as, irq, TOUCH_INT_PIN);
}

// gesture = the chip's own gesture register (0x05 click, 0x0B double click), read in the same transfer.
// Returns 1 = a finger, 0 = none, -1 = the read failed - which must never count as "finger up": with the interrupt's
// reader and the loop on one bus a failed read had split one tap into two (round 10b, a double tap toggled twice)
static int touch_read(int& x, int& y, uint8_t& gesture) {
  uint8_t b[6];
  gesture = 0;
  if (!i2c_read(TOUCH_ADDR, 0x01, b, 6) && !i2c_read(TOUCH_ADDR, 0x01, b, 6)) return -1;  // gesture, finger count, XH, XL, YH, YL
  gesture = b[0];
  // the point is read even with no finger: the chip keeps the last one, so its click report (on the release) says where
  x = ((b[2] & 0x0F) << 8) | b[3];
  y = ((b[4] & 0x0F) << 8) | b[5];
  if (TOUCH_SWAP_XY) { int t = x; x = y; y = t; }
  if (TOUCH_FLIP_X) x = 239 - x;
  if (TOUCH_FLIP_Y) y = 239 - y;
  return (b[1] & 0x0F) ? 1 : 0;
}

// ------------------------------------------------------------------ touch by interrupt (round 10b)
// The loop reads the chip once a frame (~50 ms) and Toby's taps last ~50 ms: 3 of his 20 double taps had one tap fall
// between two reads, unseen by anything. Now every pulse on TP_INT wakes touch_task, which reads the chip right then
// and queues the reading with its time; the loop feeds the queue to core_touch in order, then its own frame read.
// Nothing runs while nobody touches the screen. (Wire's own lock keeps this read and the loop's apart.)
struct TouchSample { uint32_t ms; int16_t x, y; uint8_t down, gesture; };
constexpr int TQ_N = 32;   // ~30 frames' worth of pulses; drained every frame
static TouchSample tq[TQ_N];
static uint16_t tqHead = 0, tqTail = 0;
static portMUX_TYPE tqMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t touchTask = nullptr;
static volatile uint32_t touchIrqs = 0;
static uint32_t touchReads = 0, touchDropped = 0, touchErrors = 0, touchPolls = 0, touchFed = 0, touchStale = 0;
static void IRAM_ATTR touch_isr() {
  touchIrqs++;
  BaseType_t hp = pdFALSE;
  if (touchTask) vTaskNotifyGiveFromISR(touchTask, &hp);
  if (hp) portYIELD_FROM_ISR();
}
// The task is the chip's ONLY reader once the loop runs (2026-10-21, on the arm): the loop used to read it too (a fallback,
// then every frame on the control panel) and after a few reads side by side the task stopped reading for good - its
// "reads" stuck at 10 while the interrupts went on to 982 - so touch only worked where the loop read it (the panel), and
// the thermal page ignored every tap. Now the task also covers what the loop's reads were for: while a finger is on it
// reads every 12 ms (a lost release pulse cannot leave a finger "down"), and before the first interrupt it reads every
// 150 ms (the INT line unproven), every 12 ms if a finger shows up with no interrupt (the line broken).
static volatile bool touchFingerDown = false, touchLineBroken = false;
static volatile uint32_t touchTaskMs = 0, touchIrqDone = 0;   // the task's last read, and the interrupts it had seen by then
// Round 5 (2026-10-21): the task still stopped once with the loop no longer reading the chip (reads 173, then none for
// minutes; the loop's safety net kept touch working). Where it stops is now reported (touchWhere, its FreeRTOS state, its
// stack's low-water mark) and the loop wakes it if it hangs blocked inside a read (xTaskAbortDelay: the blocking call
// returns a failure, the read is skipped, the task goes on).
static volatile uint8_t touchWhere = 0;   // 1 waiting for a pulse, 2 reading the chip, 3 queuing the sample
static uint32_t touchWoken = 0;
static void touch_task(void*) {
  for (;;) {
    TickType_t wait = portMAX_DELAY;
    if (touchFingerDown) wait = pdMS_TO_TICKS(12);
    else if (touchIrqs == 0) wait = pdMS_TO_TICKS(touchLineBroken ? 12 : 150);
    touchWhere = 1;
    ulTaskNotifyTake(pdTRUE, wait);   // pulses that came while reading fold into this one read
    uint32_t irq = touchIrqs;
    int x = 0, y = 0; uint8_t g = 0;
    touchWhere = 2;
    int d = touch_read(x, y, g);
    touchWhere = 3;
    touchTaskMs = millis(); touchIrqDone = irq;
    if (d < 0) { touchErrors++; continue; }
    if (d == 1 && touchIrqs == 0) touchLineBroken = true;
    touchFingerDown = d == 1;
    TouchSample smp{(uint32_t)millis(), (int16_t)x, (int16_t)y, (uint8_t)d, g};
    portENTER_CRITICAL(&tqMux);
    uint16_t n = (uint16_t)((tqHead + 1) % TQ_N);
    if (n != tqTail) { tq[tqHead] = smp; tqHead = n; } else touchDropped++;
    portEXIT_CRITICAL(&tqMux);
    touchReads++;
  }
}
static bool touch_pop(TouchSample& smp) {
  bool ok = false;
  portENTER_CRITICAL(&tqMux);
  if (tqTail != tqHead) { smp = tq[tqTail]; tqTail = (uint16_t)((tqTail + 1) % TQ_N); ok = true; }
  portEXIT_CRITICAL(&tqMux);
  return ok;
}
static void touch_irq_start() {
  xTaskCreatePinnedToCore(touch_task, "touch", 6144, nullptr, 3, &touchTask, 1);   // 6 KB (was 3): Wire's error logging formats on this stack   // core 1, above the loop: a read is ~0.2 ms
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), touch_isr, FALLING);
}

// ------------------------------------------------------------------ update mode (2026-10-08)
// The updater is its own program in the second slot (ota_1 of the 16 MB layout), so this firmware never carries WiFi
// (~80 KB of internal RAM the triangle buffer needs). Entering it = pointing the boot there and restarting.
// A finger put on the screen AFTER power-up: the CST816S takes its untouched baseline when it powers up, so a finger
// already on the glass then is part of the baseline and never reported (three tries on 2026-10-11: no touch at all
// until it was lifted). Switch on, then touch and hold.
// the act that tells the arm the puck is going into update mode (the arm's glink::ACT_PUCK_UPDATE, 20, 2026-10-21)
constexpr uint8_t ACT_PUCK_UPDATE = glink::ACT_PUCK_UPDATE;
static_assert(ACT_PUCK_UPDATE == 20, "the update-mode act moved: GauntletLink.h numbers only ever grow at the end");
constexpr uint32_t UPDATE_START_WINDOW_MS = 10000;  // the hold must begin this soon after power-up ...
constexpr uint32_t UPDATE_HOLD_MS = 3000;           // ... and last this long, the finger still
static void enter_update_mode(const char* why) {
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
  esp_app_desc_t d;
  if (!p || esp_ota_get_partition_description(p, &d) != ESP_OK) {
    Serial.printf("update mode (%s): no updater in the second program slot - flash firmware/PuckUpdater once over USB\n", why);
    return;
  }
  Serial.printf("update mode (%s): restarting into the updater (%s, %s)\n", why, d.project_name, d.date);
  Serial.flush();
  tft.waitDMA(); tft.endWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(middle_center); tft.setTextColor(TFT_CYAN, TFT_BLACK); tft.setTextSize(2);
  tft.drawString("UPDATE MODE", 120, 120);
  // tell the arm first (2026-10-21, Toby: "why would it be transmitting when I'm putting it into update mode"): its ELRS
  // module transmitting beside the puck keeps the updater off WiFi, so the arm stops its RC frames until the puck's
  // heartbeat comes back (never while the drone is armed - the arm decides). Sent three times over ~0.6 s with one seq:
  // at a power-up hold the arm may still be booting; it acts once per seq. An arm without the act just refuses it.
  {
    glink::Command cmd; cmd.seq = core.panel.seq++; cmd.act = ACT_PUCK_UPDATE;
    char body[40], line[64];
    size_t n = glink::commandBody(body, sizeof body, cmd) ? glink::frame(line, sizeof line, body) : 0;
    for (int k = 0; k < 3 && n; k++) { Serial1.write((const uint8_t*)line, n); Serial1.flush(); delay(200); }
  }
  esp_ota_set_boot_partition(p);
  delay(300);
  esp_restart();
}

// ------------------------------------------------------------------ IMU (QMI8658, accelerometer only)
static uint8_t imu_addr = 0;

static void imu_init() {
  const uint8_t addrs[2] = {0x6B, 0x6A};
  for (uint8_t a : addrs) {
    uint8_t who = 0;
    if (i2c_read(a, 0x00, &who, 1) && who == 0x05) { imu_addr = a; break; }
  }
  if (imu_addr) {
    // Soft reset, then accelerometer AND gyro on. With the accelerometer enabled alone (CTRL7 = 0x01) this
    // board's QMI8658 never starts sampling (TIMESTAMP stays 0, STATUS 0) and the data registers hold whatever
    // the last firmware left there - one frozen sample, so the demo hand sat at a fixed angle and ignored the
    // puck. The reset matters too: the chip keeps its state across an ESP32 reset (flash, EN, brownout).
    i2c_write(imu_addr, 0x60, 0xB0);  // RESET
    uint8_t done = 0;
    for (int i = 0; i < 40 && done != 0x80; i++) { delay(5); i2c_read(imu_addr, 0x4D, &done, 1); }  // reset done
    i2c_write(imu_addr, 0x02, 0x40);  // CTRL1: register auto-increment, little-endian
    i2c_write(imu_addr, 0x08, 0x00);  // CTRL7: sensors off while configuring
    i2c_write(imu_addr, 0x03, 0x15);  // CTRL2: accel +-4 g, ~250 Hz
    i2c_write(imu_addr, 0x04, 0x54);  // CTRL3: gyro +-512 dps, ~450 Hz (not read; it keeps the engine running)
    i2c_write(imu_addr, 0x08, 0x03);  // CTRL7: accelerometer + gyro on
    delay(80);
  }
  core.imuPresent = imu_addr != 0;
  uint8_t ts[3] = {0};
  if (imu_addr) i2c_read(imu_addr, 0x30, ts, 3);
  Serial.printf("imu QMI8658: %s, sampling %s\n", imu_addr ? (imu_addr == 0x6B ? "found at 0x6B" : "found at 0x6A") : "NOT FOUND (demo uses a slow wobble)",
                !imu_addr ? "-" : (ts[0] | ts[1] | ts[2]) ? "yes" : "NO (timestamp 0)");
}

static int16_t imuRaw[3];
static float imuScreen[3] = {0, 0, 1};   // the same reading in the screen's frame (x right, y up, z out of the glass)
static uint32_t imuReads = 0, imuFails = 0, imuTs = 0;

static void imu_update() {
  if (!imu_addr) return;
  uint8_t b[6], ts[3];
  if (!i2c_read(imu_addr, 0x30, ts, 3) || !i2c_read(imu_addr, 0x35, b, 6)) { imuFails++; return; }
  imuReads++;
  imuTs = ts[0] | (ts[1] << 8) | ((uint32_t)ts[2] << 16);   // the chip's sample counter: must keep moving
  float ax = (int16_t)(b[0] | (b[1] << 8)), ay = (int16_t)(b[2] | (b[3] << 8)), az = (int16_t)(b[4] | (b[5] << 8));
  imuRaw[0] = (int16_t)ax; imuRaw[1] = (int16_t)ay; imuRaw[2] = (int16_t)az;
  if (fabsf(ax) + fabsf(ay) + fabsf(az) < 2000) return;   // no sample yet (all zero after the reset)
  float gx = IMU_SWAP_XY ? ay : ax, gy = IMU_SWAP_XY ? ax : ay;
  float g[3] = {IMU_SIGN_X * gx, IMU_SIGN_Y * gy, IMU_SIGN_Z * az};
  memcpy(imuScreen, g, sizeof g);
  app::core_imu_sample(core, g);
}

static void fatal(const char* msg) {
  Serial.println(msg);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setCursor(40, 115);
  tft.print(msg);
  while (true) delay(1000);
}

// ------------------------------------------------------------------ the header pin-identification mode
// Waveshare's photo and schematic number the 12-pin header differently, so before any wire goes to the arm Toby
// meters the cable (DC volts, black probe on the USB-C plug's shell). Every header GPIO gets its own PWM duty at
// 20 kHz - a DC meter reads its average - so each wire says which GPIO it is. Entered by typing "pinid" (the puck
// reboots into it so nothing else holds those pins), left by "exit" or unplugging. Nothing else runs meanwhile.
struct PinId { uint8_t gpio, pct, pin; };
static const PinId kPinId[] = {{15, 10, 7}, {16, 25, 8}, {17, 40, 9}, {18, 55, 10}, {21, 70, 11}, {33, 85, 12}};
static void pinid_mode() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(lgfx::top_center);
  tft.drawString("PIN ID - METER THE CABLE", 120, 34);
  tft.drawString("black probe: USB-C shell", 120, 48);
  char b[40];
  int y = 66;
  tft.drawString("GND (pins 1, 5)  0.00 V", 120, y); y += 12;
  tft.drawString("3V3 (6) 3.30 V, holds 1k", 120, y); y += 12;
  for (const PinId& p : kPinId) {
    snprintf(b, sizeof b, "GPIO%-2u (pin %u)  %.2f V", p.gpio, p.pin, 3.3f * p.pct / 100);
    tft.drawString(b, 120, y); y += 12;
  }
  tft.drawString("VSYS (2) ~4.7 V: NEVER", 120, y); y += 12;
  tft.drawString("RESET 3 / BOOT 4: 3.3 V", 120, y); y += 12;
  tft.drawString("\"exit\" / unplug: leave", 120, y + 6);
  Serial.println("\npin-identification mode: every header GPIO drives its own PWM duty at 20 kHz; a DC meter reads (black probe on the USB-C shell):");
  for (const PinId& p : kPinId) {
    ledcAttach(p.gpio, 20000, 8);
    ledcWrite(p.gpio, (uint32_t)(255 * p.pct / 100));
    Serial.printf("  GPIO%-2u = header pin %-2u (Waveshare's schematic): %3u %% -> %.2f V\n", p.gpio, p.pin, p.pct, 3.3f * p.pct / 100);
  }
  Serial.println("  GND pins 1 and 5: 0.00 V (and ~0 ohm to the USB-C shell); pin 6 3V3: 3.30 V and still ~3.3 V with 1 kohm from it to GND;");
  Serial.println("  pin 2 VSYS: ~4.7 V with USB in - NEVER connect it; pins 3 RESET / 4 BOOT: 3.3 V, but a 1 kohm to GND pulls them down (RESET restarts the puck).");
  Serial.println("type \"exit\" (or unplug) to leave");
  String in;
  for (;;) {
    while (Serial.available()) { char ch = (char)Serial.read(); if (ch == '\n' || ch == '\r') { if (in == "exit") ESP.restart(); in = ""; } else in += ch; }
    delay(20);
  }
}

// ------------------------------------------------------------------ the orbit fps sweep (USB "sweep")
// The camera visits 62 directions round the hand (every 30 deg of yaw at five pitches, and straight over and under)
// at the hand view, then at the whole-arm view; ~1 s each after a 0.3 s settle; the fps of each, the worst per zoom.
// Whatever the arm (or garm) is doing keeps going. The camera is put back where it was afterwards.
struct SweepStep { float yaw, pitch; uint8_t zoom; };
static SweepStep sweepList[124]; static int sweepN = 0, sweepI = -1, sweepFrames = 0;
static uint32_t sweepT0 = 0;
static float sweepWorst[2], sweepSum[2]; static int sweepWorstI[2], sweepCnt[2], sweepWorstTris[2];
static float sweepSavedR[9], sweepSavedZoom;
static void sweep_apply(const SweepStep& st) {
  float a = st.yaw * 0.017453293f, b = st.pitch * 0.017453293f;
  float ca = cosf(a), sa = sinf(a), cb = cosf(b), sb = sinf(b);
  float Ry[9] = {ca, 0, sa, 0, 1, 0, -sa, 0, ca}, Rx[9] = {1, 0, 0, 0, cb, -sb, 0, sb, cb};
  kin::mat3_mul(Ry, Rx, core.cam.R);
  core.cam.zoom = core.cam.zoomTarget = st.zoom;
}
static void sweep_tick(uint32_t ms) {
  if (core.arm.test.sweep == 1) {
    core.arm.test.sweep = -1;
    sweepN = 0;
    for (int z = 0; z < 2; z++) {
      for (int p = -60; p <= 60; p += 30)
        for (int y = 0; y < 360; y += 30) sweepList[sweepN++] = {(float)y, (float)p, (uint8_t)z};
      sweepList[sweepN++] = {0, 90, (uint8_t)z}; sweepList[sweepN++] = {0, -90, (uint8_t)z};
    }
    memcpy(sweepSavedR, core.cam.R, sizeof sweepSavedR); sweepSavedZoom = core.cam.zoomTarget;
    for (int z = 0; z < 2; z++) { sweepWorst[z] = 1e9f; sweepSum[z] = 0; sweepCnt[z] = 0; }
    sweepI = 0; sweep_apply(sweepList[0]); sweepT0 = ms + 300; sweepFrames = 0; core.cam.hold = true;   // no action moves the camera meanwhile
    Serial.printf("sweep: %d camera directions, hand view then whole arm\n", sweepN);
    return;
  }
  if (sweepI < 0) return;
  if ((int32_t)(ms - sweepT0) < 0) return;
  sweepFrames++;
  if (ms - sweepT0 < 1000) return;
  const SweepStep& st = sweepList[sweepI];
  float f = (sweepFrames - 1) * 1000.0f / (ms - sweepT0);
  int z = st.zoom;
  sweepSum[z] += f; sweepCnt[z]++;
  if (f < sweepWorst[z]) { sweepWorst[z] = f; sweepWorstI[z] = sweepI; sweepWorstTris[z] = frame.n + frame2.n; }
  Serial.printf("sweep %s yaw %3.0f pitch %3.0f: %.1f fps, %d tris\n", z ? "ARM " : "HAND", st.yaw, st.pitch, f, frame.n + frame2.n);
  if (++sweepI >= sweepN) {
    for (int k = 0; k < 2; k++) {
      const SweepStep& w = sweepList[sweepWorstI[k]];
      Serial.printf("sweep RESULT %s view: worst %.1f fps at yaw %.0f pitch %.0f (%d tris), mean %.1f fps over %d directions\n", k ? "whole-arm" : "hand",
                    sweepWorst[k], w.yaw, w.pitch, sweepWorstTris[k], sweepSum[k] / sweepCnt[k], sweepCnt[k]);
    }
    memcpy(core.cam.R, sweepSavedR, sizeof sweepSavedR); core.cam.zoomTarget = sweepSavedZoom;
    sweepI = -1; core.cam.hold = false;
    return;
  }
  sweep_apply(sweepList[sweepI]); sweepT0 = ms + 300; sweepFrames = 0;
}

// ------------------------------------------------------------------ setup / loop
void app_setup() {
  Serial.setTxBufferSize(2048);   // log lines queue instead of stalling a frame while 115200 baud drains them
  Serial.begin(115200);
  delay(300);
  Serial.println("\ngauntlet puck starting");
  Serial.printf("arm link: GauntletLink v%u, RX GPIO%d (header pin 8), TX GPIO%d (pin 7), %d baud; type $G / $S lines or \"help\" here\n",
                (unsigned)glink::VERSION, PUCK_RX_PIN, PUCK_TX_PIN, LINK_BAUD);
  pinMode(2, OUTPUT); digitalWrite(2, HIGH);  // backlight
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  app::core_init(core);
  core.panel.seq = (uint16_t)esp_random();   // the arm drops a $C repeating its last seq: a restarted puck must not start where it did
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  touch_init();
  pinMode(0, INPUT_PULLUP);   // the BOOT button (header pin 4, nothing on it in the arm): held 3 s while running = update mode
  imu_init();

  prefs.begin("puck", false);
  if (prefs.getBytesLength("qzero") == sizeof core.qZero) prefs.getBytes("qzero", core.qZero, sizeof core.qZero);
  if (prefs.getBool("pinid", false)) { prefs.putBool("pinid", false); pinid_mode(); }   // never returns

  Serial1.setRxBufferSize(2048);   // ~170 ms of the arm's $G + $S at full rate: a long frame never overflows it
  Serial1.setTxBufferSize(512);    // the heartbeat queues; a write never waits (checked against availableForWrite)
  Serial1.begin(LINK_BAUD, SERIAL_8N1, PUCK_RX_PIN, PUCK_TX_PIN);
  show::begin();                   // the show bus on core 0 (its buffers are taken before the triangle buffer)

  size_t sb = (size_t)rnd::W * STRIP_H * 2;
  pipe_.fb[0] = (uint16_t*)heap_caps_malloc(sb, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  pipe_.fb[1] = (uint16_t*)heap_caps_malloc(sb, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  pipe_.zb = (uint16_t*)heap_caps_malloc(sb, MALLOC_CAP_INTERNAL);
  pipe_.stripH = STRIP_H;
  if (!pipe_.fb[0] || !pipe_.fb[1] || !pipe_.zb) fatal("out of memory (strips)");

  loopTask = xTaskGetCurrentTaskHandle();
  xTaskCreatePinnedToCore(helper_loop, "raster1", 6144, nullptr, 5, &helperTask, 0);   // core 0, above the show bus
  { hud::Data d; static uint16_t one[rnd::W * 2]; for (int k = 1; k <= 3; k++) { d.dir = k; hud::draw(one, nullptr, 0, 2, app::core_look(), d); } }   // build the HUD's tables once, on this core

  // triangle buffer last: as many visible triangles per frame as TRI_CAP asks, but always leaving RAM_RESERVE
  // free for everything else (Wire, Serial, Preferences); if a frame ever needs more, the forearm loses
  // triangles first, never the hand.
  app::hull();   // the zoomed-out framing's outline, at its exact size, before the big buffers take the heap
  // two frames, one per core, each half of the parts; ONE triangle buffer between them (frame's, split every frame -
  // split_tri_buffer), each its own vertex scratch and half the edge lines. frame2's own one-triangle array is never used.
  bool ok = false;
#ifdef MODEL_HAS_EDGES
  ok = app::style_has_lines() ? rnd::frame_alloc(frame2, 1, model::MAX_PART_VERTS, LINE_CAP / 2, model::MAX_PART_TRIS) : rnd::frame_alloc(frame2, 1, model::MAX_PART_VERTS);
#else
  ok = rnd::frame_alloc(frame2, 1, model::MAX_PART_VERTS);
#endif
  if (!ok) fatal("out of memory (frame 2)");
  ok = false;
  for (int cap = TRI_CAP; cap >= 1200 && !ok; cap -= 100) {
#ifdef MODEL_HAS_EDGES
    ok = app::style_has_lines() ? rnd::frame_alloc(frame, cap, model::MAX_PART_VERTS, LINE_CAP / 2, model::MAX_PART_TRIS) : rnd::frame_alloc(frame, cap, model::MAX_PART_VERTS);
#else
    ok = rnd::frame_alloc(frame, cap, model::MAX_PART_VERTS);
#endif
    if (ok && ESP.getFreeHeap() < RAM_RESERVE) { rnd::frame_free(frame); ok = false; }
    if (ok) triTotal = cap;
  }
  if (!ok) fatal("out of memory (frame)");
  split_tri_buffer();
  touch_irq_start();   // after the triangle buffer: its task's stack must not split the one big free block
  { app::FrameSetup f = app::core_begin_frame(core); app::draw_scene(frame, f.cam, core.world, f.lod, f.shut); app::draw_scene(frame, f.cam, core.world, 1, f.shut); }   // the lighting tables and visibility unions, built once here

  Serial.printf("model: %d parts, %d triangles, %d vertices, triangle buffer %d shared by the two cores (%u bytes) | free heap %u, largest block %u\n", model::NPART,
                model::NTRI, model::NVERT, triTotal, (unsigned)(triTotal * sizeof(rnd::Tri)), (unsigned)ESP.getFreeHeap(),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  tft.startWrite();  // keep the SPI bus for the DMA pipeline from here on
}

void app_loop() {
  static int64_t tPrev = esp_timer_get_time();
  static float fps = 0;
  static int frames = 0;
  static uint32_t fpsT0 = millis(), dbgT0 = millis();

  int64_t tNow = esp_timer_get_time();
  float dt = (tNow - tPrev) * 1e-6f;
  if (dt > 0.2f) dt = 0.2f;
  tPrev = tNow;
  uint32_t ms = millis();

  // GauntletLink $G / $S, taken from the UART in blocks: one driver call per block instead of one per byte (the arm's
  // ~6 KB/s read byte by byte cost ~1.3 fps, 2026-10-10); at most the whole RX buffer per frame. "uart 0" still drains
  // the bytes but ignores them (bench tests)
  static uint8_t rx[512];
  for (int k = 0; k < 4; k++) {
    int avail = Serial1.available();
    if (avail <= 0) break;
    int got = (int)Serial1.read(rx, avail < (int)sizeof rx ? (size_t)avail : sizeof rx);
    if (core.arm.test.uartOff != 1)
      for (int i = 0; i < got; i++) app::arm_uart_byte(core.arm, (char)rx[i], ms);
  }
  int n = Serial.available();
  while (n-- > 0) {
    app::arm_usb_byte(core.arm, (char)Serial.read(), ms);
    if (core.arm.reply[0]) { Serial.println(core.arm.reply); core.arm.reply[0] = 0; }
  }
  app::core_arm_update(core, ms);
  if (core.arm.test.pinid == 1) { Serial.flush(); prefs.putBool("pinid", true); delay(50); ESP.restart(); }
  if (core.arm.test.mlxsim >= 0) { show::show_sim(core.arm.test.mlxsim == 1); core.arm.test.mlxsim = -1; }

  // the puck's heartbeat to the arm, once a second: never waits for room (a full TX buffer drops it)
  static uint32_t hbT0 = 0; static uint16_t hbSeq = 0;
  if (ms - hbT0 >= glink::H_PERIOD_MS) {
    hbT0 = ms;
    char hb[96];
    size_t len = app::arm_heartbeat(hb, sizeof hb, hbSeq++, ms, 1, show::S().mlx, show::S().oled, fps, ESP.getFreeHeap());
    if (len && Serial1.availableForWrite() >= (int)len) Serial1.write((const uint8_t*)hb, len);
  }
  // (2026-10-21) touch health to the arm's log every 2 s ("puck: touch health ..."): interrupts, the chip read by them and by
  // the loop, samples fed to the touch logic, failed reads, queue drops, the page, the touch state - over WiFi, no cable
  static uint32_t thT0 = 0;
  if (ms - thT0 >= 2000) {
    thT0 = ms;
    char body[230], out[256];
    snprintf(body, sizeof body, "L,touch health: page %u; irq %lu; task reads %lu (%lu ms ago; at %u; state %d; stack free %u; woken %lu); loop reads %lu; fed %lu; failed %lu; stale %lu; noise %lu; down %d; at %d/%d; presses %lu",
             (unsigned)core.panel.page, (unsigned long)touchIrqs, (unsigned long)touchReads, (unsigned long)(millis() - touchTaskMs), (unsigned)touchWhere,
             (int)eTaskGetState(touchTask), (unsigned)uxTaskGetStackHighWaterMark(touchTask), (unsigned long)touchWoken, (unsigned long)touchPolls,
             (unsigned long)touchFed, (unsigned long)touchErrors, (unsigned long)touchStale, (unsigned long)core.ts.noise, (int)core.ts.down, core.ts.lx, core.ts.ly,
             (unsigned long)core.panel.presses);
    size_t n = glink::frame(out, sizeof out, body);
    if (n && Serial1.availableForWrite() >= (int)n) Serial1.write((const uint8_t*)out, n);
  }
  // (2026-10-16) the thermal camera's reads, every 2 s while its page is up: a $L line the arm prints as "puck: ..."
  static uint32_t camT0 = 0;
  if (show::S().want && ms - camT0 >= 2000) {
    camT0 = ms;
    const show::State& c = show::S();
    char body[150], out[180];
    snprintf(body, sizeof body, "L,camera: %s; %lu frames; %lu failed reads (%lu never ready); last error %d; status 0x%04X; bus %s; restarts %lu",
             c.mlx ? "found" : "absent", (unsigned long)c.frames, (unsigned long)c.fails, (unsigned long)c.notReady, c.lastErr, (unsigned)(c.lastStatus & 0xFFFF),
             c.slow == 2 ? "100 kHz 2 Hz" : (c.slow == 1 ? "400 kHz 4 Hz" : "400 kHz 8 Hz"), (unsigned long)c.rebegins);
    size_t n = glink::frame(out, sizeof out, body);
    if (n && Serial1.availableForWrite() >= (int)n) Serial1.write((const uint8_t*)out, n);
    Serial.println(body + 2);
    // the faulty pixels (badpix.h) and the hottest spot - a hottest spot that never moves is a fault still to find
    char mk[110];
    badpix::describe(mk, sizeof mk);
    snprintf(body, sizeof body, "L,camera %s; hottest %d/%d %.1f C", mk, c.hx, c.hy, c.hi);
    n = glink::frame(out, sizeof out, body);
    if (n && Serial1.availableForWrite() >= (int)n) Serial1.write((const uint8_t*)out, n);
    Serial.println(body + 2);
  }

  // the low-battery shutdown: the arm's $S says SHUTDOWN (flags bit 5) -> the screen and the OLED go dark at once
  static bool blanked = false;
  if (app::core_blank_update(core, ms) != blanked) {
    blanked = core.blank;
    tft.waitDMA();
    if (blanked) { digitalWrite(2, LOW); tft.endWrite(); tft.sleep(); Serial.println("blank: the arm says SHUTDOWN (the glove pack is CRITICAL) - screen and OLED off"); }
    else { tft.wakeup(); tft.startWrite(); digitalWrite(2, HIGH); Serial.println("blank: the arm is back without SHUTDOWN - screen on"); }
  }
  static uint32_t oledT0 = 0;
  if (ms - oledT0 >= 200) {   // what the OLED shows (core 0 draws it, oled_view.h)
    oledT0 = ms;
    const app::ArmLink& L = core.arm;
    hud::Data hd = app::core_hud(core, ms);
    oledv::Status st;
    bool any = app::core_is_live(core) || L.have;
    st.link = hd.mismatch ? 3 : (!any ? 0 : (hd.link == hud::LINK_UP ? 1 : 2));
    st.rate = app::core_is_live(core) ? L.liveRate : L.goodRate;
    st.mode = hd.mode; st.mv = hd.mv; st.floorMv = glink::POWER_FLOOR_MV;
    st.bstate = app::core_is_live(core) ? L.live.bstate : (L.have ? L.st.bstate : 0);
    if (L.up(ms)) { st.lq = L.st.lq; st.armed = L.st.armed; st.cv100 = L.st.cv100; }
    else if (app::core_is_live(core)) st.armed = (L.live.outs >> 4) & 1;
    st.plates = hd.plates > 5 ? 5 : hd.plates; st.lidar = hd.lidar;
    st.ring = hd.ringOn; st.leds = hd.pucks; st.red = hd.red; st.green = hd.green; st.strip = hd.strip;
    st.thermal = core.thermal; st.hot = show::S().hi;
    strncpy(st.event, hd.event, sizeof st.event - 1); st.eventAge = hd.eventAge;
    if (hd.blocked[0]) {
      if (hd.blockedWhy[0]) snprintf(st.event, sizeof st.event, "%s: %s", hd.blocked, hd.blockedWhy);
      else snprintf(st.event, sizeof st.event, "%s %s", hd.blocked, hd.blockedWarn ? "REFUSED" : "BLOCKED");
      st.eventAge = hd.blockedAge;
    }
    st.shutdown = core.blank; st.fps = fps;
    show::show_status(st);
  }
  if (core.blank) { delay(20); return; }   // nothing is drawn; the link, the heartbeat and the show bus keep going

  // touch: every reading the touch task queued since the last frame, in order (the task is the chip's only reader - it
  // covers a lost release pulse and a broken INT line itself); a failed read is skipped, never taken as "finger up"
  {
    static uint32_t lastT = 0;
    static uint8_t lastGesture = 0;
    auto feed_touch = [&](uint32_t t, bool touching, int tx, int ty, uint8_t g) {
      // a read older than the last one fed is dropped (2026-10-21): with the loop reading beside a stuck task, a task read
      // queued before the loop's "finger up" came after it, re-opened the touch and closed it on the next read - a
      // one-sample phantom tap. (Clamping its time to the last one, as before, is exactly that.)
      if ((int32_t)(t - lastT) < 0) { touchStale++; return; }
      lastT = t; touchFed++;
      if (g != lastGesture) { Serial.printf("touch: chip gesture 0x%02X\n", g); lastGesture = g; }
      if (sweepI >= 0) touching = false;   // the sweep measures the 3D view: no touch meanwhile
      app::core_event(core, app::core_touch(core, touching, tx, ty, t, g), ms);
      if (core.ts.trace[0]) {   // every touch, also to the arm's log ("puck: touch ..."): how the chip saw it, readable over WiFi
        Serial.println(core.ts.trace);
        static uint32_t lT0 = 0; static int lN = 0;
        if (ms - lT0 >= 1000) { lT0 = ms; lN = 0; }
        if (lN++ < 6) {
          char body[150], out[180];
          snprintf(body, sizeof body, "L,%s", core.ts.trace);
          for (char* q = body + 2; *q; q++) if (*q == ',' || *q == '$' || *q == '*') *q = ';';
          size_t n = glink::frame(out, sizeof out, body);
          if (n && Serial1.availableForWrite() >= (int)n) Serial1.write((const uint8_t*)out, n);
        }
        core.ts.trace[0] = 0;
      }
      if (core.ts.log[0]) {
        Serial.printf("%s | taps %u, chip clicks %u, doubles by taps %u / clicks %u / chip %u, toggles %u, junk %u | irq %u, reads %u\n", core.ts.log,
                      (unsigned)core.ts.taps, (unsigned)core.ts.clicks, (unsigned)core.ts.doublesOwn, (unsigned)core.ts.doublesClicks,
                      (unsigned)core.ts.doublesChip, (unsigned)core.ts.toggles, (unsigned)core.ts.junk, (unsigned)touchIrqs, (unsigned)touchReads);
        if (touchErrors) Serial.printf("touch: %u failed reads skipped\n", (unsigned)touchErrors);
        core.ts.log[0] = 0;
      }
    };
    TouchSample q;
    while (touch_pop(q)) feed_touch(q.ms, q.down, q.x, q.y, q.gesture);
    // the loop reads the chip itself only if the task has stopped answering its interrupts (never seen since the task
    // became the only reader; kept as a safety net, and counted in the touch-health line)
    uint32_t nowMs = (uint32_t)millis();
    static uint32_t lastPollMs = 0; static bool stuckSaid = false;
    const bool stuck = touchIrqs != touchIrqDone && nowMs - touchTaskMs > 500;
    static uint32_t lastWakeMs = 0;
    if (stuck && touchWhere == 2 && nowMs - touchTaskMs > 1000 && nowMs - lastWakeMs > 1000 && eTaskGetState(touchTask) == eBlocked) {
      lastWakeMs = nowMs; touchWoken++; xTaskAbortDelay(touchTask);   // hung inside a read: make its blocking call give up
    }
    if (stuck && nowMs - lastPollMs >= 15) {
      lastPollMs = nowMs; touchPolls++;
      if (!stuckSaid) { stuckSaid = true; Serial.println("touch: the touch task stopped reading - the loop reads the chip itself"); }
      int tx = 0, ty = 0; uint8_t g = 0;
      int r = touch_read(tx, ty, g);
      if (r >= 0) feed_touch(nowMs, r == 1, tx, ty, g); else touchErrors++;
    }
    if (sweepI >= 0) { core.thermal = false; core.panel.page = app::PAGE_MAIN; }
  }

  // update mode: a finger put on within UPDATE_START_WINDOW_MS of power-up and held still UPDATE_HOLD_MS, BOOT, or "update"
  {
    static bool decided = false;
    const app::TouchState& ts = core.ts;
    if (!decided) {
      if (ts.down && !ts.moved && ts.tDown < UPDATE_START_WINDOW_MS && millis() - ts.tDown >= UPDATE_HOLD_MS) { decided = true; enter_update_mode("held at power-up"); }
      if (ts.moved || (!ts.down && millis() > UPDATE_START_WINDOW_MS)) decided = true;
    }
    if (core.arm.test.update == 1) { core.arm.test.update = -1; enter_update_mode("console"); }
    // the second way in (Toby, 2026-10-11): BOOT held for UPDATE_HOLD_MS while the puck runs (held at power-up it
    // starts the chip's own USB flashing mode instead - that is the ROM, not this)
    static uint32_t bootDown = 0;
    if (digitalRead(0) == LOW) { if (!bootDown) bootDown = millis(); else if (millis() - bootDown >= UPDATE_HOLD_MS) { bootDown = 0; enter_update_mode("BOOT held"); } }
    else bootDown = 0;
  }

  if (!app::core_is_live(core) && !core.arm.have) imu_update();
  app::core_step(core, dt, ms);
  const bool panel = core.panel.page == app::PAGE_PANEL;
  // a control-panel press: its $C line to the arm (never waits: a full TX buffer drops it, and says so)
  if (core.panel.txLen) {
    if (Serial1.availableForWrite() >= core.panel.txLen) Serial1.write((const uint8_t*)core.panel.tx, core.panel.txLen);
    else Serial.println("panel: the UART's TX buffer is full - the press was not sent");
    core.panel.txLen = 0;
  }
  if (core.panel.log[0]) { Serial.println(core.panel.log); core.panel.log[0] = 0; }
  show::show_want(core.thermal && !panel);   // the camera is read only while its page is up
  static float armRate = 0;
  hudData = app::core_hud(core, ms);
  hudLook = app::core_look();
  sweep_tick(ms);
  int64_t tb0 = esp_timer_get_time(), tb1 = tb0;
  const bool calArm = app::core_cal_arm_view(core);   // the calibration's arm steps: the 3D view, small, under its words
  if (panel || calArm) panelView = app::panel_view(core, ms);
  if (panel && !calArm) {   // the control panel: buttons, no 3D view (the model keeps following the arm underneath)
    app::PanelView& pv = panelView;
    for (int y0 = 0; y0 < rnd::H; y0 += pipe_.stripH) {
      uint16_t* fb = pipe_.fb[pipe_.b];
      app::draw_panel(fb, y0, pipe_.stripH, hudLook, pv);
      hud::panel_overlay(fb, y0, pipe_.stripH, hudLook, hudData);   // the main page's battery and mode symbol
      push_strip(y0, fb, pipe_.stripH);
      pipe_.b ^= 1;
    }
  } else if (core.thermal && !calArm) {   // the thermal page: the whole glass is the camera's image; no 3D view, nothing on core 0's raster helper
    float* tpx = frame2.sp;   // 768 floats in core 0's vertex scratch (3 x MAX_PART_VERTS): no 3D view is built while this page is up
    float lo, hi, c; int hx, hy;
    uint32_t n = show::thermal_snapshot(tpx, lo, hi, c, hx, hy);
    hud::Thermal T{n ? tpx : nullptr, lo, hi, c, hx, hy, show::S().sim && !show::S().mlx, show::S().mlx || show::S().sim};
    for (int y0 = 0; y0 < rnd::H; y0 += pipe_.stripH) {
      uint16_t* fb = pipe_.fb[pipe_.b];
      hud::draw_thermal(fb, y0, pipe_.stripH, hudLook, T, ms);
      hud::thermal_overlay(fb, y0, pipe_.stripH, hudLook, hudData);   // the mode symbol and the battery bar stay on top
      hud::page_dots(fb, y0, pipe_.stripH, hudLook, 0);
      push_strip(y0, fb, pipe_.stripH);
      pipe_.b ^= 1;
    }
  } else {
    // build: every other part on core 0 (into frame2) while this core builds the rest (into frame)
    static app::FrameSetup fs; fs = calArm ? app::core_begin_cal_frame(core) : app::core_begin_frame(core);
    split_tri_buffer();
    static auto build_half = [](void*) { app::draw_scene(frame2, fs.cam, core.world, fs.lod, fs.shut, 1, 2, fs.tone); };
    helper_split(build_half, nullptr);
    app::draw_scene(frame, fs.cam, core.world, fs.lod, fs.shut, 0, 2, fs.tone);
    helper_join();
    app::core_end_frame(core, fs);
    tb1 = esp_timer_get_time();
    rnd::render_frame_split(frame, &frame2, pipe_, calArm ? cal_overlay : overlay, push_strip, &core.fx, helper_split, helper_join);
  }
  static uint32_t tBuildUs = 0, tRasterUs = 0, tFrames = 0;
  tBuildUs += (uint32_t)(tb1 - tb0);
  tRasterUs += (uint32_t)(esp_timer_get_time() - tb1);
  tFrames++;
  static int lineMax = 0; static uint32_t lineDrops = 0;   // edge lines: the most in one frame, and any the buffers had no room for
  if (!core.thermal && !panel) { lineMax = max(lineMax, frame.ln + frame2.ln); lineDrops += (uint32_t)(frame.ldropped + frame2.ldropped); }

  frames++;
  ms = millis();
  if (ms - fpsT0 >= 500) {
    fps = frames * 1000.0f / (ms - fpsT0);
    frames = 0; fpsT0 = ms;
  }
  // accelerometer 4x a second while in DEMO, to confirm it reads: raw counts (sensor axes, 8192 = 1 g) and roll /
  // pitch of the puck in the screen's frame (0 / 0 lying face up; roll + = right edge down, pitch + = top edge down)
  static uint32_t imuT0 = 0;
  if (imu_addr && !app::core_is_live(core) && !core.arm.have && ms - imuT0 >= 250) {
    imuT0 = ms;
    float x = imuScreen[0], y = imuScreen[1], z = imuScreen[2];
    float roll = atan2f(-x, z) * 57.29578f, pitch = atan2f(-y, sqrtf(x * x + z * z)) * 57.29578f;
    Serial.printf("imu raw %6d %6d %6d  roll %6.1f  pitch %6.1f  sample# %u  reads %u fails %u\n", imuRaw[0], imuRaw[1], imuRaw[2], roll,
                  pitch, (unsigned)imuTs, (unsigned)imuReads, (unsigned)imuFails);
  }
  // link health once a second (the same numbers as the debug view): good and CRC-failed $S frames per second, the
  // last good frame's age, totals per stream, then the last frame's fields
  static uint32_t linkT0 = 0;
  if (ms - linkT0 >= 1000) {
    linkT0 = ms;
    const app::ArmLink& L = core.arm;
    armRate = L.goodRate;
    const glink::Live& g = L.live;
    char age[24];
    if (L.haveLive) snprintf(age, sizeof age, "%lu ms", (unsigned long)(ms - L.liveMs)); else snprintf(age, sizeof age, "never");
    Serial.printf("link %s: $G %.1f/s (longest gap %lu ms, last %s ago), all good %.1f/s, dropped %.1f/s | totals $G %u $S %u, older %u, wrong version %u (theirs v%ld, v1 lines %u), bad fields %u | uart good %u bad %u stray %u too-long %u | usb good %u bad %u\n",
                  app::core_is_live(core) ? (L.liveFresh(ms) ? "LIVE" : (L.liveUp(ms) ? "LIVE-FROZEN" : "LIVE-LOST")) : (L.up(ms) ? "ARM" : (L.have ? "ARM-LOST" : "none")),
                  L.liveRate, (unsigned long)L.liveGapMax, age, L.goodRate, L.badRate, (unsigned)L.liveFrames, (unsigned)L.frames, (unsigned)L.liveOld,
                  (unsigned)L.badVersion, L.theirVersion, (unsigned)(L.uart.v1 + L.usb.v1), (unsigned)L.badFields,
                  (unsigned)L.uart.good, (unsigned)L.uart.bad, (unsigned)L.uart.stray, (unsigned)L.uart.tooLong, (unsigned)L.usb.good, (unsigned)L.usb.bad);
    if (L.haveLive)
      Serial.printf("  $G seq %u t %lu imu %u fq %d %d %d %d hq %d %d %d %d | curl %d %d %d %d %d | servo %d %d %d %d %d %d drv %u plates %u | ring %06lX lvl %u mode %u | strip %06lX %u lit %u at %d | groups %u acc %u lasers %u lidar %u | mode %u gev %u/%u hand %u | %u mV bstate %u %d mA outs %u\n",
                    g.seq, (unsigned long)g.t, g.imu, g.fq[0], g.fq[1], g.fq[2], g.fq[3], g.hq[0], g.hq[1], g.hq[2], g.hq[3],
                    g.curl[0], g.curl[1], g.curl[2], g.curl[3], g.curl[4], g.servo[0], g.servo[1], g.servo[2], g.servo[3], g.servo[4], g.servo[5],
                    g.drv, g.plates, (unsigned long)g.ring, g.ringlvl, g.ringmode, (unsigned long)g.strip, g.striplvl, g.striplit, g.strippos,
                    g.groups, g.accent, g.lasers, g.lidar, g.mode, g.gev, g.gn, g.hand, g.mv, g.bstate, g.ma, g.outs);
    show::State& sb = show::S();
    Serial.printf("  show bus: MLX90640 %s%s, read %s, %u frames (read %u ms, sim maths %u us), hot %.1f / centre %.1f C | SSD1306 %s | core-0 loops %u | raster helper %u half-strips | probes %u (last %.1f ms, longest %.1f ms)\n",
                  sb.mlx ? "ok" : "absent", sb.sim ? " (simulation ON)" : "", sb.want ? "ON (thermal page)" : "off", (unsigned)sb.frames, (unsigned)sb.readMs, (unsigned)sb.calcUs,
                  sb.hi, sb.centre, sb.oled ? "ok" : "absent", (unsigned)sb.loops, (unsigned)helperJobs, (unsigned)sb.probes, sb.probeUs / 1000.0f, sb.probeMaxUs / 1000.0f);
    Serial.printf("  touch: interrupts %u, reads %u, failed reads %u, queue drops %u, INT line %s, taps %u, toggles %u | panel: %s, opened %u, presses %u, answered %u, unanswered %u\n", (unsigned)touchIrqs,
                  (unsigned)touchReads, (unsigned)touchErrors, (unsigned)touchDropped, digitalRead(TOUCH_INT_PIN) ? "high" : "LOW",
                  (unsigned)core.ts.taps, (unsigned)core.ts.toggles, core.panel.page == app::PAGE_PANEL ? "UP" : "closed", (unsigned)core.panel.opens,
                  (unsigned)core.panel.presses, (unsigned)core.panel.answered, (unsigned)core.panel.unanswered);
  }
  if (ms - dbgT0 >= 2000) {
    dbgT0 = ms;
    Serial.printf("%s%s%s  %.1f fps  tris %d  glows %d beams %d  $G %u  dropped %u  plates %s  zoom %.2f  hud %u | ms/frame: build %.1f, draw %.1f (of which display wait %.1f)\n",
                  app::core_is_live(core) ? "LIVE" : (core.arm.have ? "ARM" : "NO ARM"), core.thermal ? " THERMAL" : "", panel ? " PANEL" : "", fps, frame.n + frame2.n, core.fx.ng, core.fx.nb, (unsigned)core.arm.liveFrames, (unsigned)core.arm.bad(),
                  core.cam.platesOpen ? "open" : "closed", core.cam.zoom, (unsigned)core.hudDir, tFrames ? tBuildUs / 1000.0f / tFrames : 0,
                  tFrames ? tRasterUs / 1000.0f / tFrames : 0, tFrames ? tWaitUs / 1000.0f / tFrames : 0);
    rnd::Profile& P = rnd::prof();
    if (tFrames)
      Serial.printf("  profile per frame: vertices %u in %.1f ms, triangles %u in %.1f ms (lighting %u vertices, %.1f ms) | raster: %u setups, %u rows, %u px  | cpu %u MHz | heap free %u, lowest %u, dropped tris %d | edge lines max %d, dropped %u\n",
                    (unsigned)(P.verts / tFrames), P.vtxUs / 1000.0f / tFrames, (unsigned)(P.tris / tFrames), P.triUs / 1000.0f / tFrames, (unsigned)(P.lit / tFrames), P.litCycles / 240000.0f / tFrames,
                    (unsigned)(P.setups / tFrames), (unsigned)(P.rows / tFrames), (unsigned)(P.pix / tFrames), (unsigned)getCpuFrequencyMhz(),
                    (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), frame.dropped + frame2.dropped, lineMax, (unsigned)lineDrops);
    lineMax = 0; lineDrops = 0;
    P = rnd::Profile();
    tBuildUs = tRasterUs = tWaitUs = tFrames = 0;
  }
}
