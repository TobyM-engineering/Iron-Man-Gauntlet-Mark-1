// oled_view.h - what the SSD1306 shows: Toby's display, the September bench's layout (ironman-gauntlet
// Gauntlet/display.cpp, 2026-09-07) kept row for row, with only what is new since filled in. Hardware-free: drawn
// through the Adafruit_GFX calls, so the puck draws it on the real panel and the PC tools (puck_oled.cpp) draw the
// very same code into a picture.
//
// The panel is two-colour: rows 0-15 yellow, 16-63 blue. Nothing straddles row 16. Data only (Toby, round 10: no
// ARMOR line, no mode word, no "ARM 20/s"; the freed row spread over the rest):
//   yellow  ARM   the glove pack: bar with a nub, volts, percent (glink::BATT_FULL_MV = 100 %); the row blinks on CRITICAL
//   18-28   DRN   the drone's battery (its cell, 3.3 .. 4.2 V), from the arm's $S - as tall as the ARM bar's text
//   31-42   LINK  the drone link: five bars + LQ % (nothing else on the row)
//   45-52   ARMED (a solid block) or DISARMED on the left; LIDAR in feet on the right (only while the red laser fires)
//   54      a rule
//   56-63   the ticker: warnings first (pack CRITICAL / LOW / FAULT / USB power, protocol, link lost), else the newest
//           event for a moment (done, BLOCKED, REFUSED), else what is lit
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "GauntletLink.h"   // glink::BATT_EMPTY_MV / BATT_FULL_MV: the pack's 0 % and 100 %

namespace oledv {

struct Status {
  uint8_t link = 0;            // the puck's link to the arm: 0 never, 1 up, 2 lost, 3 protocol mismatch
  float rate = 0;              // the arm's $G per second
  int8_t mode = -1;            // 0 SAFE 1 SHOW 2 FLIGHT
  uint16_t mv = 0; uint8_t bstate = 0;   // the glove pack (bstate as GauntletLink)
  uint16_t floorMv = 0;                  // the arm's POWER_FLOOR_MV: one notch on the ARM bar (0 = none)
  uint8_t lq = 0, armed = 0; uint16_t cv100 = 0;   // the drone ($S)
  uint8_t plates = 0;          // as $G (0 unknown 1 open 2 closed 3 opening 4 closing 5 mixed)
  uint16_t lidar = 0;          // mm
  bool ring = false, leds = false, red = false, green = false, strip = false;
  bool thermal = false; float hot = 0;
  char event[22] = ""; uint32_t eventAge = 1u << 30;
  bool shutdown = false;
  float fps = 0;
};

// the glove pack's percent: the same straight line as the round screen's (arm_link.h battery_frac), BATT_EMPTY_MV .. BATT_FULL_MV
inline int pack_pct(uint16_t mv) {
  int p = (int)(((float)mv - glink::BATT_EMPTY_MV) * 100.0f / (float)(glink::BATT_FULL_MV - glink::BATT_EMPTY_MV));
  return p < 0 ? 0 : (p > 100 ? 100 : p);
}
inline int cell_pct(uint16_t cv100) { int p = (int)((cv100 - 330) * 100 / 90); return p < 0 ? 0 : (p > 100 ? 100 : p); }   // 3.3 .. 4.2 V

// A battery gauge: label, bar with a nub, voltage, percent. h = bar height. (display.cpp batteryRow, unchanged)
template <class G> void battery_row(G& o, uint32_t ms, int y, int h, const char* label, float volts, int pct, bool known, bool warn, int markPct = -1) {
  o.setTextSize(1);
  int ty = y + (h - 7) / 2;
  o.setCursor(0, ty); o.print(label);
  const int bx = 22, bw = 46;
  o.drawRect(bx, y, bw, h, 1);
  o.fillRect(bx + bw, y + 3, 2, h - 6, 1);
  if (known) {
    int fill = (int)((bw - 4) * (pct < 0 ? 0 : (pct > 100 ? 100 : pct)) / 100.0f);
    if (fill > 0) o.fillRect(bx + 2, y + 2, fill, h - 4, 1);
    char v[8]; snprintf(v, sizeof v, "%.2fV", volts);
    o.setCursor(74, ty); o.print(v);
    char p[6]; snprintf(p, sizeof p, "%3d%%", pct < 0 ? 0 : (pct > 100 ? 100 : pct));
    o.setCursor(106, ty); o.print(p);
  } else {
    o.setCursor(74, ty); o.print("--.--V");
    o.setCursor(106, ty); o.print(" --%");
  }
  if (markPct >= 0) {   // the one mark (a two-colour OLED: a notch through the bar, a tick above and below it)
    int mx = bx + 2 + (int)((bw - 4) * markPct / 100.0f);
    o.fillRect(mx, y + 1, 1, h - 2, 2);
    if (y > 0) o.fillRect(mx, y - 1, 1, 1, 1);
    o.fillRect(mx, y + h, 1, 1, 1);
  }
  if (warn && (ms / 400) % 2) o.fillRect(0, y, 128, h, 2);   // blink the row (INVERSE)
}

template <class G> void draw(G& o, const Status& s, uint32_t ms) {
  o.clearDisplay();
  if (s.shutdown) return;   // the low-battery shutdown: dark
  o.setTextColor(1);
  // yellow band: the glove pack, taller bar; blinks on CRITICAL and on a monitor FAULT
  bool packKnown = s.link && s.bstate != 5 && s.bstate != 6 && s.mv;
  battery_row(o, ms, 1, 13, "ARM", s.mv / 1000.0f, pack_pct(s.mv), packKnown, s.bstate == 4 || s.bstate == 5, s.floorMv ? pack_pct(s.floorMv) : -1);
  // the drone's battery
  bool droneKnown = s.lq && s.cv100;
  battery_row(o, ms, 18, 11, "DRN", s.cv100 / 100.0f, cell_pct(s.cv100), droneKnown, droneKnown && cell_pct(s.cv100) <= 15);
  // LINK: five bars + LQ % (display.cpp linkRow), the row to itself
  int y = 31;
  o.setTextSize(1);
  o.setCursor(0, y + 3); o.print("LINK");
  for (int i = 0; i < 5; i++) {
    int h = 4 + 2 * i, x = 27 + i * 7;
    if (s.lq > i * 20) o.fillRect(x, y + 12 - h, 5, h, 1); else o.drawRect(x, y + 12 - h, 5, h, 1);
  }
  char b[24];
  if (s.lq) snprintf(b, sizeof b, "LQ %u%%", s.lq); else snprintf(b, sizeof b, "LQ --");
  o.setCursor(128 - 6 * (int)strlen(b), y + 3); o.print(b);
  // ARMED / DISARMED on the left, the LiDAR in feet on the right
  y = 45;
  if (s.armed) { o.fillRect(0, y - 1, 33, 10, 1); o.setTextColor(0); o.setCursor(2, y); o.print("ARMED"); o.setTextColor(1); }
  else { o.setCursor(0, y); o.print("DISARMED"); }
  if (s.lidar) { unsigned t = (unsigned)(s.lidar / 30.48f + 0.5f); snprintf(b, sizeof b, "LIDAR %u.%uft", t / 10, t % 10); }
  else snprintf(b, sizeof b, "LIDAR --ft");
  o.setCursor(128 - 6 * (int)strlen(b), y); o.print(b);
  o.drawFastHLine(0, 54, 128, 1);
  // the ticker, 21 characters: warnings first, then the newest event, then what is lit
  const char* msg = nullptr; bool inv = false;
  if (s.bstate == 4) { msg = "ARM BATTERY CRITICAL"; inv = true; }
  else if (s.bstate == 5) { msg = "BATTERY MONITOR FAULT"; inv = true; }
  else if (s.link == 3) { msg = "PROTOCOL MISMATCH"; inv = true; }
  else if (s.link == 2) { msg = "ARM LINK LOST"; inv = true; }
  else if (s.bstate == 2 || s.bstate == 3) msg = "arm battery low";
  else if (s.bstate == 6) msg = "USB power - no pack";
  else if (s.event[0] && s.eventAge < 2500) msg = s.event;
  else if (s.thermal) { snprintf(b, sizeof b, "THERMAL  HOT %.1fC", s.hot); msg = b; }
  else if (s.ring || s.leds || s.red || s.green || s.strip) {
    snprintf(b, sizeof b, "%s%s%s%s%s", s.ring ? "RING " : "", s.leds ? "LEDS " : "", s.red ? "RED " : "", s.green ? "GRN " : "", s.strip ? "STRIP" : "");
    msg = b;
  } else if (!s.link) msg = "waiting for the arm";
  else { snprintf(b, sizeof b, "screen %.0f fps", s.fps); msg = b; }
  if (inv && (ms / 500) % 2) { o.fillRect(0, 56, 128, 8, 1); o.setTextColor(0); }
  o.setCursor(0, 56); o.print(msg);
  o.setTextColor(1);
}

}  // namespace oledv
