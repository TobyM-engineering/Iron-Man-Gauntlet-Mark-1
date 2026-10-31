// hud.h - the round screen's 2D layer, drawn into each render strip after the model. Hardware-free, so the PC tools
// render exactly what the puck shows. Everything is added light (a glow over the dark scene).
//
// Round 7 (Toby, 2026-10-07: "fewer words, much cooler ... shapes, icons and colour first ... nothing over the hand
// ... things that change show briefly and fade ... a better battery"):
//   - every HUD pixel lands only where the model is NOT (the strip's depth buffer, zb == 0): nothing ever covers the
//     arm; the hand view leaves a band inside the glass for it (config.h HAND_FILL_PX)
//   - the lights are icons in their own colour, drawn only while they are on - no words
//   - events, a mode change, the link coming and going: one short word at the top that fades in ~2.5 s
//   - three directions to choose from (Data::dir), all style A's red on black:
//       1 BEZEL    an instrument bezel: the battery is a segmented arc down the right rim, the lights an icon arc
//                  down the left, the mode a glyph at eleven o'clock, a pulse dot for the link at one
//       2 RETICLE  film-HUD targeting brackets left and right, a scanning tick segment that sweeps the rim, the
//                  battery filling the left bracket with its number, the link the right one, the lights above them
//       3 ORBIT    three small round complications on the rim (battery ring gauge with its number, mode, link) and
//                  the lights as small icons down the right rim
// References (looked at, nothing copied): Jayse Hansen's notes on the Iron Man HUD (every element has a function,
// reticles that come in from depth), round smartwatch faces (information around the bezel, the centre free, curved
// layouts, balance, black base).
#pragma once
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hud_font.h"
#include "render.h"
#include "GauntletLink.h"   // glink::VERSION, printed on a protocol mismatch

namespace hud {

enum Link : uint8_t { LINK_NONE = 0, LINK_UP, LINK_LOST, LINK_PACKET };
struct Data {
  uint8_t dir = 1;             // the direction: 0 none, 1 BEZEL, 2 RETICLE, 3 ORBIT
  uint32_t t = 0;              // ms, the animation clock
  uint8_t link = LINK_NONE;    // NONE = no arm yet (DEMO), UP, LOST
  int8_t mode = -1;            // 0 SAFE 1 SHOW 2 FLIGHT, -1 unknown
  float battery = -1;          // 0..1, < 0 = unknown
  uint16_t mv = 0;
  bool batWarn = false, batCrit = false;
  uint8_t plates = 0;          // 0 unknown 1 open 2 closed 3 opening 4 closing 5 mixed 6 plate 1 open alone 7 plate 1 moving
  float rate = 0;              // the arm's live frames per second
  bool ringOn = false; uint8_t ringRGB[3] = {0, 0, 0}; uint8_t ringLvl = 0; const char* ringName = "";
  uint8_t pucks = 0; bool red = false, green = false, strip = false, accent = false;
  uint8_t stripRGB[3] = {255, 0, 0};
  uint16_t lidar = 0;          // mm, 0 = not measuring
  const char* event = "";      // the newest event / change, shown at the top for a moment
  uint32_t eventAge = 1u << 30;   // ms since it happened
  bool eventWarn = false;      // drawn in the warning colour
  bool debug = false;          // the link-health view (USB "debug" only)
  float goodRate = 0, badRate = 0;
  uint32_t ageMs = 0, uartGood = 0, uartBad = 0, usbGood = 0, usbBad = 0;
  uint8_t mismatch = 0;        // frames of another GauntletLink version arriving: their version (0 = none)
  const char* blocked = "";    // a gesture the arm saw but did not do ($E BLOCKED / REFUSED): shown "seen, not done"
  uint32_t blockedAge = 1u << 30;
  bool blockedWarn = false;    // REFUSED (the battery, a cascade, ...): red; BLOCKED by the mode switch: grey
  const char* blockedWhy = ""; // BLOCKED for another reason than the mode switch: "DRONE" (flying, or just disarmed)
  bool armed = false;          // the drone is armed: the mode symbol spins
  float batMark = -1;          // where glink::POWER_FLOOR_MV sits on the battery bar (0..1, the bar's own mapping); < 0 = none
};

struct Look { uint8_t level; uint8_t hud[3], accent[3]; };

constexpr int CX = 120, CY = 120;
constexpr float TEXT_R = 114.0f;   // every HUD text pixel within this radius of the centre: 6 px inside the 120 px glass
constexpr float GLASS_R = 118.5f;  // every HUD pixel at all
constexpr int MODE_X = 148, MODE_Y = 170;   // (old sprite place, kept for puck_core.h's HUD_X / HUD_Y)

struct Pt { uint8_t x, y, v; };

// ---- strip-clipped primitives; zb != null = only where the model is not
struct Strip { uint16_t* fb; uint16_t* zb; int y0, rows; bool solid = false; };   // solid: overwrite (a dark backing), not add
inline void plot(const Strip& s, int x, int y, const uint8_t c[3], int v) {   // v 0..255
  if (y < s.y0 || y >= s.y0 + s.rows || x < 0 || x >= rnd::W) return;
  int o = (y - s.y0) * rnd::W + x;
  if (s.zb && s.zb[o]) return;   // the model is here: the HUD stays behind it
  if (s.solid) { s.fb[o] = rnd::pack565_swapped(c[0] * v >> 8, c[1] * v >> 8, c[2] * v >> 8); return; }
  rnd::add565(s.fb[o], c[0] * v >> 8, c[1] * v >> 8, c[2] * v >> 8);
}
inline void line(const Strip& s, float x0, float y0, float x1, float y1, const uint8_t c[3], int v) {
  float dx = x1 - x0, dy = y1 - y0;
  int n = (int)fmaxf(fabsf(dx), fabsf(dy)) + 1;
  for (int i = 0; i <= n; i++) plot(s, (int)floorf(x0 + dx * i / n), (int)floorf(y0 + dy * i / n), c, v);
}
inline void text(const Strip& s, int x, int y, const char* str, const uint8_t c[3], int v) {
  if (y > s.y0 + s.rows || y + 8 < s.y0 || v <= 0) return;
  for (; *str; str++, x += 6) {
    const unsigned char* g = kHudFont + 5 * (unsigned char)*str;
    for (int cx = 0; cx < 5; cx++)
      for (int cy = 0; cy < 7; cy++)
        if (g[cx] >> cy & 1) plot(s, x + cx, y + cy, c, v);
  }
}
inline int text_w(const char* str) { int n = 0; while (str[n]) n++; return 6 * n - 1; }
inline void text2(const Strip& s, int x, int y, const char* str, const uint8_t c[3], int v) {   // double size
  if (y > s.y0 + s.rows || y + 15 < s.y0 || v <= 0) return;
  for (; *str; str++, x += 12) {
    const unsigned char* g = kHudFont + 5 * (unsigned char)*str;
    for (int cx = 0; cx < 5; cx++)
      for (int cy = 0; cy < 7; cy++)
        if (g[cx] >> cy & 1) { plot(s, x + 2 * cx, y + 2 * cy, c, v); plot(s, x + 2 * cx + 1, y + 2 * cy, c, v);
                               plot(s, x + 2 * cx, y + 2 * cy + 1, c, v); plot(s, x + 2 * cx + 1, y + 2 * cy + 1, c, v); }
  }
}
inline void centred(const Strip& s, int y, const char* str, const uint8_t c[3], int v) { text(s, CX - text_w(str) / 2, y, str, c, v); }
inline void centred2(const Strip& s, int y, const char* str, const uint8_t c[3], int v) { text2(s, CX - (2 * text_w(str) + 1) / 2, y, str, c, v); }
// darken a box to a quarter (the link-health panel, a debug tool)
inline void dim_box(const Strip& s, int x0, int y0, int x1, int y1) {
  int ya = y0 < s.y0 ? s.y0 : y0, yb = y1 >= s.y0 + s.rows ? s.y0 + s.rows - 1 : y1;
  x0 = x0 < 0 ? 0 : x0; x1 = x1 >= rnd::W ? rnd::W - 1 : x1;
  for (int y = ya; y <= yb; y++)
    for (int x = x0; x <= x1; x++) {
      uint16_t& p = s.fb[(y - s.y0) * rnd::W + x];
      uint16_t c = (uint16_t)((p >> 8) | (p << 8));
      c = (uint16_t)((c >> 2) & 0x39E7);
      p = (uint16_t)((c >> 8) | (c << 8));
    }
}

// angle of (x, y) clockwise from 12 o'clock, degrees 0..360 (a fast atan2: ~0.3 deg)
inline float clock_deg(float dx, float dy) {
  float ax = fabsf(dx), ay = fabsf(dy), mn = fminf(ax, ay), mx = fmaxf(ax, ay);
  if (mx < 1e-6f) return 0;
  float a = mn / mx, s2 = a * a;
  float r = ((-0.0464964749f * s2 + 0.15931422f) * s2 - 0.327622764f) * s2 * a + a;   // atan(a), 0..pi/4
  if (ay > ax) r = 1.57079637f - r;                  // atan2(|dy|, |dx|)
  if (dx < 0) r = 3.14159274f - r;
  if (dy < 0) r = -r;                                 // atan2(dy, dx), y down
  float deg = r * 57.2957795f + 90.0f;               // 12 o'clock = 0, clockwise
  return deg < 0 ? deg + 360 : (deg >= 360 ? deg - 360 : deg);
}
inline bool in_span(float a, float a0, float a1) {   // clockwise from a0 to a1 (may wrap through 0)
  return a0 <= a1 ? (a >= a0 && a <= a1) : (a >= a0 || a <= a1);
}
inline void ring_xy(float deg, float r, float& x, float& y);
// an annulus sector r0..r1, clockwise a0..a1 (degrees from 12 o'clock); f(a10) -> brightness 0..255 for a pixel at
// angle a10 (tenths of a degree, 0..3599 - integer rules, no fmodf); colour c. Only the sector's bounding box is
// scanned (its two ends and any 3 / 6 / 9 / 12 o'clock point inside it).
template <class F>
inline void arc(const Strip& s, float r0, float r1, float a0, float a1, const uint8_t c[3], F f) {
  float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  auto grow = [&](float deg) { for (float r : {r0, r1}) { float x, y; ring_xy(deg, r, x, y); xmin = fminf(xmin, x); xmax = fmaxf(xmax, x); ymin = fminf(ymin, y); ymax = fmaxf(ymax, y); } };
  grow(a0); grow(a1);
  for (float q : {0.0f, 90.0f, 180.0f, 270.0f}) if (in_span(q, a0, a1)) grow(q);
  int ya = (int)floorf(ymin) - 1, yb = (int)ceilf(ymax) + 1, xlo = (int)floorf(xmin) - 1, xhi = (int)ceilf(xmax) + 1;
  if (ya < s.y0) ya = s.y0;
  if (yb > s.y0 + s.rows - 1) yb = s.y0 + s.rows - 1;
  const float r0s = r0 * r0, r1s = r1 * r1;
  for (int y = ya; y <= yb; y++) {
    float dy = y + 0.5f - CY, dy2 = dy * dy;
    if (dy2 > r1s) continue;
    float xo = sqrtf(r1s - dy2), xi = dy2 < r0s ? sqrtf(r0s - dy2) : 0;
    for (int side = 0; side < 2; side++) {
      int xa = side ? (int)(CX + xi) : (int)(CX - xo), xb = side ? (int)(CX + xo) : (int)(CX - xi);
      if (xa < xlo) xa = xlo;
      if (xb > xhi) xb = xhi;
      for (int x = xa; x <= xb; x++) {
        float dx = x + 0.5f - CX, r2 = dx * dx + dy2;
        if (r2 < r0s || r2 > r1s) continue;
        float a = clock_deg(dx, dy);
        if (!in_span(a, a0, a1)) continue;
        int v = f((int)(a * 10));
        if (v > 0) plot(s, x, y, c, v > 255 ? 255 : v);
      }
    }
  }
}
// A solid annulus sector about (cx, cy): r0..r1, clockwise a0..a1 degrees from 12 o'clock, colour c at v. No angle per
// pixel: a pixel is inside when it lies clockwise of the a0 ray and anticlockwise of the a1 ray (two cross products);
// a sector of 180 degrees or more is everything but its complement. Only the rows of this strip inside its box are scanned.
inline void sector(const Strip& s, float cx, float cy, float r0, float r1, float a0, float a1, const uint8_t c[3], int v) {
  if (v <= 0) return;
  float span = a1 - a0; if (span < 0) span += 360;
  bool invert = span >= 180;
  float b0 = invert ? a1 : a0, b1 = invert ? a0 : a1;   // the small sector to test against
  float u0x = sinf(b0 * 0.017453293f), u0y = -cosf(b0 * 0.017453293f), u1x = sinf(b1 * 0.017453293f), u1y = -cosf(b1 * 0.017453293f);
  // the box: the two ends at both radii and any 12 / 3 / 6 / 9 o'clock point the sector passes
  float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  auto grow = [&](float deg) { float sx = sinf(deg * 0.017453293f), sy = -cosf(deg * 0.017453293f);
    for (float r : {r0, r1}) { float x = cx + r * sx, y = cy + r * sy; xmin = fminf(xmin, x); xmax = fmaxf(xmax, x); ymin = fminf(ymin, y); ymax = fmaxf(ymax, y); } };
  grow(a0); grow(a1);
  for (float q : {0.0f, 90.0f, 180.0f, 270.0f}) if (in_span(q, a0, a1)) grow(q);
  int ya = (int)floorf(ymin) - 1, yb = (int)ceilf(ymax) + 1, xlo = (int)floorf(xmin) - 1, xhi = (int)ceilf(xmax) + 1;
  if (ya < s.y0) ya = s.y0;
  if (yb > s.y0 + s.rows - 1) yb = s.y0 + s.rows - 1;
  const float r0s = r0 * r0, r1s = r1 * r1;
  for (int y = ya; y <= yb; y++) {
    float dy = y + 0.5f - cy, dy2 = dy * dy;
    if (dy2 > r1s) continue;
    float xo = sqrtf(r1s - dy2), xi = dy2 < r0s ? sqrtf(r0s - dy2) : 0;
    for (int side = 0; side < 2; side++) {
      int xa = side ? (int)(cx + xi) : (int)floorf(cx - xo), xb = side ? (int)ceilf(cx + xo) : (int)(cx - xi);
      if (xa < xlo) xa = xlo;
      if (xb > xhi) xb = xhi;
      for (int x = xa; x <= xb; x++) {
        float dx = x + 0.5f - cx, r2 = dx * dx + dy2;
        if (r2 < r0s || r2 > r1s) continue;
        bool in = (u0x * dy - u0y * dx) >= 0 && (dx * u1y - dy * u1x) >= 0;
        if (in != invert) plot(s, x, y, c, v);
      }
    }
  }
}
inline void arc_solid(const Strip& s, float r0, float r1, float a0, float a1, const uint8_t c[3], int v) { sector(s, CX, CY, r0, r1, a0, a1, c, v); }
// radial tick marks: n ticks from angle a0 every step degrees, r0..r1; v(i) brightness of tick i (0 = skip)
static inline const float* sin_table() { static float t[360]; static bool ok = false; if (!ok) { for (int i = 0; i < 360; i++) t[i] = sinf(i * 0.017453293f); ok = true; } return t; }
template <class V>
inline void ticks(const Strip& s, int n, int a0, int step, float r0, float r1, const uint8_t c[3], V v) {
  if (s.y0 > CY + r1 || s.y0 + s.rows < CY - r1) return;
  const float* sn = sin_table();
  for (int i = 0; i < n; i++) {
    int b = v(i);
    if (b <= 0) continue;
    int a = ((a0 + i * step) % 360 + 360) % 360;
    float sx = sn[a], cy = sn[(a + 90) % 360];
    float y0 = CY - r0 * cy, y1 = CY - r1 * cy;
    if (fmaxf(y0, y1) < s.y0 - 1 || fminf(y0, y1) > s.y0 + s.rows + 1) continue;
    line(s, CX + r0 * sx, y0, CX + r1 * sx, y1, c, b > 255 ? 255 : b);
  }
}
inline void ring_xy(float deg, float r, float& x, float& y) { float a = deg * 0.017453293f; x = CX + r * sinf(a); y = CY - r * cosf(a); }

// ---- the icons: 9 x 9, '#' lit, drawn at 1x or 2x
struct Icon { const char* row[9]; };
static const Icon ICON_RING = {{"..#####..", ".#.....#.", "#.......#", "#..###..#", "#..#.#..#", "#..###..#", "#.......#", ".#.....#.", "..#####.."}};
static const Icon ICON_LASER = {{"........#", ".......#.", "......#..", ".....#...", "....#....", "...#.....", "..#......", ".###.....", "#####...."}};
static const Icon ICON_LINK = {{"..#####..", ".#.....#.", "#..###..#", ".#.....#.", "...###...", "..#...#..", "....#....", "...###...", "....#...."}};
static const Icon ICON_NOLINK = {{"#.#####..", ".##....#.", "#.##....#", ".#..#..#.", "...#.#...", "..#...#..", "....#.#..", "...###.#.", "....#...#"}};
static const Icon ICON_SAFE = {{"...###...", "..#...#..", "..#...#..", ".#######.", ".#######.", ".###.###.", ".###.###.", ".#######.", "........."}};
static const Icon ICON_SHOW = {{"....#....", "....#....", "...###...", "#########", ".#######.", "...###...", "..##.##..", ".##...##.", "........."}};
static const Icon ICON_FLIGHT = {{"##.....##", "##.....##", "..#...#..", "...###...", "...###...", "...###...", "..#...#..", "##.....##", "##....###"}};
static const Icon ICON_PLATES = {{"....#....", "...#.#...", "..#...#..", ".#..#..#.", "...#.#...", "..#...#..", ".#..#..#.", "...#.#...", "..#...#.."}};
inline void icon(const Strip& s, int x, int y, const Icon& ic, const uint8_t c[3], int v, int scale = 1) {
  if (v <= 0 || y > s.y0 + s.rows || y + 9 * scale < s.y0) return;
  for (int r = 0; r < 9; r++)
    for (int k = 0; k < 9; k++)
      if (ic.row[r][k] == '#')
        for (int a = 0; a < scale; a++)
          for (int b = 0; b < scale; b++) plot(s, x + k * scale + b, y + r * scale + a, c, v);
}
inline const Icon& mode_icon(int m) { return m == 0 ? ICON_SAFE : m == 2 ? ICON_FLIGHT : ICON_SHOW; }

inline float clampd(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
// The arm's three-position switch: 1 SAFE, 2 SHOW, 3 FULL ($G mode 0 / 1 / 2 - GauntletLink names the third FLIGHT).
// A safety thing when strangers hold the arm: each mode has its own colour, used for the glyph, its word and the ring.
inline const char* mode_name(int m) { return m == 0 ? "SAFE" : m == 1 ? "SHOW" : m == 2 ? "FULL" : nullptr; }
inline const uint8_t* mode_rgb(int m) {
  static const uint8_t kSafe[3] = {90, 255, 130}, kShow[3] = {255, 220, 40}, kFull[3] = {255, 40, 45}, kNone[3] = {120, 120, 130};
  return m == 0 ? kSafe : m == 1 ? kShow : m == 2 ? kFull : kNone;
}
inline const char* plates_name(int p) {
  return p == 1 ? "PLATES OPEN" : p == 2 ? "PLATES CLOSED" : p == 3 ? "PLATES OPENING" : p == 4 ? "PLATES CLOSING"
       : p == 5 ? "PLATES MIXED" : p == 6 ? "PLATE 1 OPEN" : p == 7 ? "PLATE 1 MOVING" : "PLATES ?";
}

// the battery bar's one mark (2026-10-21, Toby: "a little red bar ... really small but noticeable"): where the arm's
// POWER_FLOOR_MV sits - under it the drone does not arm and the big LEDs do not all start. Drawn solid (it stays red over
// a lit segment): a short radial tick across an arc bar at angle deg, r0..r1 about (cx, cy)
static const uint8_t kBatMark[3] = {255, 25, 30};
inline void bat_mark_tick(const Strip& s, float cx, float cy, float r0, float r1, float deg, float halfDeg) {
  Strip o = s; o.solid = true; o.zb = nullptr;
  sector(o, cx, cy, r0, r1, deg - halfDeg, deg + halfDeg, kBatMark, 255);
}

// how much of a brief thing is left: full for hold ms, then gone over fade ms
inline int fade(uint32_t age, uint32_t hold = 1600, uint32_t fadeMs = 900) {
  if (age < hold) return 255;
  if (age >= hold + fadeMs) return 0;
  return (int)(255 * (1.0f - (float)(age - hold) / fadeMs));
}
inline bool blink(uint32_t t, uint32_t ms = 350) { return (t / ms) & 1; }

// the lights that are on, in their own colours (shared by the three directions): the palm ring and the lasers only,
// as on the model (Toby, 2026-10-10 - no LED-driver, strip or accent icons)
struct LightIcon { const Icon* ic; uint8_t c[3]; };
inline int lights_on(const Data& d, LightIcon out[6]) {
  int n = 0;
  if (d.ringOn) { out[n] = {&ICON_RING, {d.ringRGB[0], d.ringRGB[1], d.ringRGB[2]}}; n++; }
  if (d.red) { out[n] = {&ICON_LASER, {255, 40, 48}}; n++; }
  if (d.green) { out[n] = {&ICON_LASER, {60, 255, 90}}; n++; }
  return n;
}

// the event / change word at the top, fading; a protocol mismatch stays until it is gone
inline void lidar_feet(uint16_t mm, char* b, size_t n) { unsigned t = (unsigned)(mm / 30.48f + 0.5f); snprintf(b, n, "%u.%u ft", t / 10, t % 10); }
inline void top_word(const Strip& s, const Look& L, const Data& d, int y) {
  if (d.mismatch) {
    char b[24]; snprintf(b, sizeof b, "ARM V%u  PUCK V%u", (unsigned)d.mismatch, (unsigned)glink::VERSION);
    centred(s, y, "PROTOCOL", L.hud, blink(d.t, 600) ? 255 : 140);
    centred(s, y + 10, b, L.accent, 230);
    return;
  }
  if (d.blocked[0]) {   // seen, not done: struck through - grey when the mode switch blocked it, red when refused
    int vb = fade(d.blockedAge, 1800, 700);
    if (vb) {
      static const uint8_t kGrey[3] = {150, 150, 160};
      const uint8_t* col = d.blockedWarn ? L.hud : kGrey;
      int w = text_w(d.blocked), x0 = CX - w / 2;
      text(s, x0, y, d.blocked, col, vb);
      line(s, x0 - 2, y + 3, x0 + w + 1, y + 3, col, vb);   // the strike: seen, not done
      if (d.blockedWhy[0]) centred(s, y + 10, d.blockedWhy, col, vb);   // why, when it was not the mode switch
      return;
    }
  }
  if (d.lidar) {   // the range while the red laser fires, in feet with one decimal (Toby, round 10) - before any word
    char b[16]; lidar_feet(d.lidar, b, sizeof b);
    centred2(s, y - 8, b, L.accent, 255);
    return;
  }
  int v = fade(d.eventAge);
  if (v && d.event[0]) centred(s, y, d.event, d.eventWarn ? L.hud : L.accent, v);
}

// the LiDAR range while the red laser measures (a live value, so it stays while it is there)
inline void range_text(const Strip& s, const Look& L, const Data& d, int y) {
  if (!d.lidar) return;
  char b[16]; lidar_feet(d.lidar, b, sizeof b);
  centred2(s, y, b, L.accent, 255);
}

inline uint8_t bat_level(const Data& d) { return d.battery < 0 ? 0 : (uint8_t)(clampd(d.battery, 0, 1) * 100 + 0.5f); }

// ---------------------------------------------------------------- the mode: one repulsor symbol, no words
// A ring round a glowing core, original artwork: three segments round the core, 1 lit = SAFE (green), 2 = SHOW (yellow),
// 3 = FULL (red) - it reads by shape as well as by colour; a thin outer ring; the core glowing in the mode's colour.
// While the drone is armed the three segments spin (a turn a second) and the core pulses.
inline void mode_repulsor(const Strip& s, float cx, float cy, int mode, bool armed = false, uint32_t t = 0) {
  if (cy + 15 < s.y0 || cy - 15 > s.y0 + s.rows) return;
  const uint8_t* c = mode_rgb(mode);
  sector(s, cx, cy, 12.2f, 13.4f, 0, 359.9f, c, 110);                     // the outer ring
  int lit = mode < 0 ? 0 : mode + 1;
  float spin = armed ? (float)(t % 1000) * 0.36f : 0.0f;
  for (int i = 0; i < 3; i++) {                                           // three segments, 100 deg each, from the top
    float a0 = fmodf(i * 120.0f + 10 + spin, 360.0f), a1 = fmodf(a0 + 100, 360.0f);
    sector(s, cx, cy, 8.0f, 10.8f, a0, a1, c, i < lit ? 245 : 40);
  }
  if (armed) {   // the pulse: a halo that swells and fades twice a second
    float ph = (t % 500) / 500.0f;
    sector(s, cx, cy, 13.6f + 3.0f * ph, 14.8f + 3.0f * ph, 0, 359.9f, c, (int)(200 * (1 - ph)));
  }
  for (int y = (int)cy - 7; y <= (int)cy + 7; y++)                        // the core and its glow
    for (int x = (int)cx - 7; x <= (int)cx + 7; x++) {
      float dx = x + 0.5f - cx, dy = y + 0.5f - cy, r = sqrtf(dx * dx + dy * dy);
      if (r > 6.5f) continue;
      if (r <= 3.2f) { static const uint8_t kW[3] = {255, 255, 255}; plot(s, x, y, c, 255); plot(s, x, y, kW, mode < 0 ? 40 : 140); }
      else plot(s, x, y, c, (int)(200 * (6.5f - r) / 3.3f));
    }
}

// ---------------------------------------------------------------- 1 BEZEL
inline void draw_bezel(const Strip& s, const Look& L, const Data& d, bool word = true) {   // word false: the control panel (its own word)
  // the battery: 20 segments down the right rim (150 deg at the bottom up to 40), lit up to the level; low = blinks red
  const int seg = 20; float lvl = d.battery < 0 ? 0 : clampd(d.battery, 0, 1);
  int lit = (int)(lvl * seg + 0.5f);
  bool warn = d.batWarn || (d.battery >= 0 && lvl < 0.2f);
  const uint8_t* bc = warn ? L.hud : L.accent;
  int on = warn && blink(d.t, 500) ? 90 : 235;
  for (int i = 0; i < seg; i++)   // segment i from the bottom (150 deg) up to 40 deg; 5.5 deg each with a 1.3 deg gap
    arc_solid(s, 104.0f, 112.0f, 150 - (i + 0.92f) * 5.5f, 150 - (i + 0.18f) * 5.5f, bc, i < lit ? on : 34);
  if (d.battery >= 0) { char b[8]; snprintf(b, sizeof b, "%u", bat_level(d)); text(s, 196 - text_w(b), 182, b, bc, 230); }
  if (d.batMark >= 0) bat_mark_tick(s, CX, CY, 101.5f, 115.0f, 150 - d.batMark * seg * 5.5f, 0.7f);
  // the mode: the repulsor symbol at nine o'clock
  { float x, y; ring_xy(270, 101, x, y); mode_repulsor(s, x, y, d.mode, d.armed, d.t); }
  if (word) top_word(s, L, d, 20);
}
// the two page dots at the bottom of every page (Toby, 2026-10-21: "the two dots at the bottom at all times"): page 0 the
// 3D view / thermal page, 1 the control panel; drawn over everything
inline void page_dots(uint16_t* fb, int y0, int rows, const Look& L, int page) {
  if (y0 > 234 || y0 + rows < 226) return;
  Strip s{fb, nullptr, y0, rows};
  static const uint8_t kDot[3] = {200, 200, 210};
  sector(s, 113.5f, 230.5f, 0, 2.6f, 0, 359.9f, page == 0 ? L.accent : kDot, page == 0 ? 230 : 70);
  sector(s, 126.5f, 230.5f, 0, 2.6f, 0, 359.9f, page == 1 ? L.accent : kDot, page == 1 ? 230 : 70);
}
// the control panel's battery and mode symbol: the BEZEL's own, whatever direction the main page uses (panel.h)
inline void panel_overlay(uint16_t* fb, int y0, int rows, const Look& L, const Data& d) { Strip s{fb, nullptr, y0, rows}; draw_bezel(s, L, d, false); }

// ---------------------------------------------------------------- 2 RETICLE
inline void draw_reticle(const Strip& s, const Look& L, const Data& d) {
  static const uint8_t kDim[3] = {110, 20, 26};
  // the two brackets: double thin arcs left (215 .. 325) and right (35 .. 145), with end caps
  arc_solid(s, 103.0f, 104.2f, 215, 325, L.hud, 150); arc_solid(s, 113.0f, 114.2f, 215, 325, L.hud, 150);
  arc_solid(s, 103.0f, 104.2f, 35, 145, L.hud, 150); arc_solid(s, 113.0f, 114.2f, 35, 145, L.hud, 150);
  for (float a : {215.0f, 325.0f, 35.0f, 145.0f}) { float x0, y0, x1, y1; ring_xy(a, 101, x0, y0); ring_xy(a, 115, x1, y1); line(s, x0, y0, x1, y1, L.hud, 230); }
  // a scanning segment of fine ticks sweeping the outer rim once every 6 s - the screen is alive
  int sweep = (int)((d.t / 50) % 120) * 3;   // one 3-degree step every 50 ms: once round in 6 s
  ticks(s, 120, 0, 3, 115.0f, 117.3f, kDim, [&](int i) {
    int rel = ((sweep - i * 3) % 360 + 360) % 360;
    if (rel > 60) return i % 10 == 0 ? 110 : 0;
    return (int)(255 * (60 - rel) / 60);
  });
  // the battery fills the left bracket from the bottom; its number at nine o'clock
  float lvl = d.battery < 0 ? 0 : clampd(d.battery, 0, 1);
  bool warn = d.batWarn || (d.battery >= 0 && lvl < 0.2f);
  const uint8_t* bc = warn ? L.hud : L.accent;
  float top = 215 + 110 * lvl;   // 215 = the bracket's bottom end (seven o'clock), 325 its top
  int on = warn && blink(d.t, 500) ? 70 : 220;
  arc_solid(s, 105.5f, 111.7f, 215, top, bc, on); arc_solid(s, 105.5f, 111.7f, top, 325, bc, 22);
  if (d.battery >= 0) { char b[8]; snprintf(b, sizeof b, "%u", bat_level(d)); text2(s, 26, 112, b, bc, 235); }
  // the link fills the right bracket (frames per second against the arm's 20), the mode glyph at three o'clock
  float q = d.link == LINK_UP ? clampd(d.rate / 20.0f, 0, 1) : 0;
  arc_solid(s, 105.5f, 111.7f, 35, 35 + 110 * q, L.hud, 200); arc_solid(s, 105.5f, 111.7f, 35 + 110 * q, 145, L.hud, 22);
  if (d.mode >= 0) { icon(s, 190, 111, mode_icon(d.mode), mode_rgb(d.mode), 240, 2); const char* w = mode_name(d.mode); text(s, 199 - text_w(w) / 2, 132, w, mode_rgb(d.mode), 240); }
  if (d.link == LINK_LOST) icon(s, 196, 140, ICON_NOLINK, L.hud, blink(d.t) ? 255 : 70);
  // the lights: in the two gaps below the brackets' lower ends (right 149 .. 163 deg, left 211 .. 197), clear of the
  // top line and of the forearm at six o'clock in the hand view; only those that are on
  LightIcon li[6]; int n = lights_on(d, li);
  static const float kAng[6] = {151, 209, 159, 201, 167, 193};
  for (int i = 0; i < n; i++) { float x, y; ring_xy(kAng[i], 108, x, y); icon(s, (int)x - 4, (int)y - 4, *li[i].ic, li[i].c, 255); }
  top_word(s, L, d, 26);
}

// ---------------------------------------------------------------- 3 ORBIT
inline void complication(const Strip& s, float deg, const uint8_t ring[3], int v, float frac, const Icon* ic, const uint8_t ic_c[3], const char* num) {
  float cx, cy; ring_xy(deg, 101, cx, cy);
  const float r0 = 11.0f, r1 = 13.0f;
  // a small ring gauge: its fill (frac) clockwise from the top, the rest faint
  if (cy + r1 + 1 < s.y0 || cy - r1 - 1 > s.y0 + s.rows) { if (num || ic) {} } else {
    float f = frac < 0 ? 0 : (frac > 1 ? 1 : frac);
    if (f >= 0.999f) sector(s, cx, cy, r0, r1, 0, 359.9f, ring, v);
    else if (f <= 0.001f) sector(s, cx, cy, r0, r1, 0, 359.9f, ring, 40);
    else { sector(s, cx, cy, r0, r1, 0, f * 360, ring, v); sector(s, cx, cy, r0, r1, f * 360, 359.9f, ring, 40); }
  }
  if (ic) icon(s, (int)cx - 4, (int)cy - 4, *ic, ic_c, 240);
  if (num) text(s, (int)cx - text_w(num) / 2, (int)cy - 3, num, ic_c, 240);
}
inline void draw_orbit(const Strip& s, const Look& L, const Data& d) {
  // three round complications on the rim: battery (10:30), mode (1:30), link (6)
  float lvl = d.battery < 0 ? 0 : clampd(d.battery, 0, 1);
  bool warn = d.batWarn || (d.battery >= 0 && lvl < 0.2f);
  char b[8]; snprintf(b, sizeof b, "%u", bat_level(d));
  const uint8_t* bc = warn ? L.hud : L.accent;
  complication(s, 315, bc, warn && blink(d.t, 500) ? 70 : 230, lvl, nullptr, bc, d.battery >= 0 ? b : "--");
  if (d.mode >= 0) complication(s, 45, mode_rgb(d.mode), 220, 1.0f, &mode_icon(d.mode), mode_rgb(d.mode), nullptr);
  bool up = d.link == LINK_UP;
  float q = up ? clampd(d.rate / 20.0f, 0, 1) : (blink(d.t) ? 1.0f : 0.0f);
  complication(s, 270, L.hud, 200, q, up ? &ICON_LINK : &ICON_NOLINK, up ? L.accent : L.hud, nullptr);
  // the lights: small icons down the right rim from half past three, only those that are on, each in its own colour
  LightIcon li[6]; int n = lights_on(d, li);
  for (int i = 0; i < n; i++) {
    float deg = 100.0f + i * 10.0f, x, y;
    ring_xy(deg, 106, x, y);
    icon(s, (int)x - 4, (int)y - 4, *li[i].ic, li[i].c, 255);
  }
  // a faint orbit line joining them
  static const uint8_t kDim[3] = {90, 18, 22};
  ticks(s, 120, 0, 3, 116.0f, 117.3f, kDim, [](int) { return 90; });
  top_word(s, L, d, 20);
}

// the link-health panel (USB "debug"): frames per second, the last good frame's age, totals per stream
inline void draw_debug(const Strip& s, const Look& L, const Data& d) {
  if (!(s.y0 < 162 && s.y0 + s.rows > 110)) return;
  static const uint8_t kGood[3] = {120, 255, 140}, kGrey[3] = {150, 150, 160};
  Strip o = s; o.zb = nullptr;   // a debug tool: drawn over everything
  char buf[44];
  dim_box(o, 22, 110, 217, 161);
  centred(o, 115, "LINK HEALTH", L.hud, 255);
  snprintf(buf, sizeof buf, "GOOD %.0f/S BAD %.0f/S", clampd(d.goodRate, 0, 999), clampd(d.badRate, 0, 999));
  centred(o, 127, buf, d.badRate > 0 ? L.accent : kGood, 255);
  if (d.link == LINK_NONE) snprintf(buf, sizeof buf, "NO FRAME YET");
  else if (d.ageMs < 10000) snprintf(buf, sizeof buf, "LAST GOOD %lu MS AGO", (unsigned long)d.ageMs);
  else if (d.ageMs < 1000000) snprintf(buf, sizeof buf, "LAST GOOD %lu S AGO", (unsigned long)(d.ageMs / 1000));
  else snprintf(buf, sizeof buf, "LAST GOOD >999 S AGO");
  centred(o, 139, buf, d.ageMs > 1000 ? L.accent : L.hud, 255);
  auto cap = [](uint32_t v) { return (unsigned long)(v > 99999 ? 99999 : v); };
  snprintf(buf, sizeof buf, "UART %lu/%lu USB %lu/%lu", cap(d.uartGood), cap(d.uartBad), cap(d.usbGood), cap(d.usbBad));
  centred(o, 151, buf, kGrey, 230);
}

// Draw the HUD into one strip (zb = the strip's depth buffer, or null to draw everywhere).
inline void draw(uint16_t* fb, uint16_t* zb, int y0, int rows, const Look& L, const Data& d) {
  Strip s{fb, zb, y0, rows};
  if (d.dir == 1) draw_bezel(s, L, d);
  else if (d.dir == 2) draw_reticle(s, L, d);
  else if (d.dir == 3) draw_orbit(s, L, d);
  if (d.debug) draw_debug(s, L, d);
}
inline void draw(uint16_t* fb, int y0, int rows, const Look& L, const Data& d) { draw(fb, nullptr, y0, rows, L, d); }

// the mode symbol and the battery bar over the thermal page (Toby, round 10), each on a dark backing
inline void thermal_overlay(uint16_t* fb, int y0, int rows, const Look& L, const Data& d) {
  // The thermal page's frame (2026-10-16, Toby: "an actual simplistic design, not an awkward rectangle in a circle ...
  // battery at the top, the Fahrenheit at the bottom, the mode on the left, not cutting off the screen"): the image
  // outlined, the glass's edge drawn where the image leaves room, nothing over the image.
  // Upright since 2026-10-16 (Toby: "putting my hand down from the top makes it come from the right"): the image is
  // 144 x 192 now, so the room is down the two sides - the mode on the left, the battery and the scale on the right.
  Strip s{fb, nullptr, y0, rows};
  const uint8_t* ac = L.accent;
  for (int k = 0; k < 3; k++) {                                           // the outline: 3 px, 3 px out from the 144 x 192
    line(s, 42, 18 + k, 197, 18 + k, ac, 200); line(s, 42, 221 - k, 197, 221 - k, ac, 200);
    line(s, 42 + k, 18, 42 + k, 221, ac, 200); line(s, 197 - k, 18, 197 - k, 221, ac, 200);
  }
  arc_solid(s, 114, 117, 226, 246, ac, 110); arc_solid(s, 114, 117, 294, 314, ac, 110);   // the edge round the mode bar
  // the mode: a bar down the left crescent in the mode's own colour (Toby: "just a bar - yellow, red or green")
  arc_solid(s, 109, 117, 252, 288, mode_rgb(d.mode), d.mode >= 0 ? 255 : 90);
  // the battery, right: its percentage, large, and ten segments under it (warning colour, blinking, when low)
  float lvl = d.battery < 0 ? 0 : clampd(d.battery, 0, 1);
  bool warn = d.batWarn || (d.battery >= 0 && lvl < 0.2f);
  const uint8_t* bc = warn ? L.hud : ac;
  int on = warn && blink(d.t, 500) ? 90 : 235;
  char b[8];
  if (d.battery >= 0) snprintf(b, sizeof b, "%u%%", bat_level(d)); else snprintf(b, sizeof b, "--%%");
  if (2 * text_w(b) + 1 <= 36) text2(s, 219 - (2 * text_w(b) + 1) / 2, 92, b, bc, on);
  else text(s, 219 - text_w(b) / 2, 96, b, bc, on);                     // "100%" in the small size
  int lit = (int)(lvl * 10 + 0.5f);
  for (int i = 0; i < 10; i++) for (int y = 110; y < 114; y++) for (int x = 200 + i * 4; x < 203 + i * 4; x++) plot(s, x, y, bc, i < lit ? on : 40);
  if (d.batMark >= 0) { Strip o = s; o.solid = true; int mx = 200 + (int)(40 * d.batMark + 0.5f); for (int y = 107; y < 117; y++) plot(o, mx, y, kBatMark, 255); }
}

// ---------------------------------------------------------------- the thermal page
// The MLX90640's 32 x 24 image, upright (144 x 192: the camera lies on its side), bilinear, the "iron" palette, auto
// range; the hottest spot ringed with its temperature; the scale and the battery on the right, the mode bar on the left.
struct Thermal { const float* px; float lo, hi, centre; int hx, hy; bool sim; bool cam; };   // px null = no frame yet
inline void iron(float v, uint8_t c[3]) {
  static const float st[6] = {0, 0.2f, 0.4f, 0.6f, 0.8f, 1.0f};
  static const uint8_t col[6][3] = {{0, 0, 10}, {60, 0, 110}, {170, 0, 120}, {240, 60, 20}, {255, 180, 0}, {255, 255, 230}};
  v = clampd(v, 0, 1);
  int i = 0; while (i < 4 && v > st[i + 1]) i++;
  float f = (v - st[i]) / (st[i + 1] - st[i]);
  for (int k = 0; k < 3; k++) c[k] = (uint8_t)(col[i][k] + (col[i + 1][k] - col[i][k]) * f);
}
static constexpr float TH_PX = 6.0f;   // screen pixels per camera pixel: 24 x 6 = 144 wide by 32 x 6 = 192 tall, the diagonal 240 = the glass
// The camera lies on its side on the hand: its column 31 is the scene's top (Toby, 2026-10-16), so the screen's rows
// run down its columns. TH_MIRROR: the screen's left = the camera's last row instead of its first.
static constexpr bool TH_MIRROR = true;   // seen on the arm (Toby, 2026-10-16): left and right were swapped without it
inline void th_to_screen(float col, float row, float& x, float& y) {   // a camera pixel's centre on the screen
  float v = TH_MIRROR ? 23 - row : row, u = 31 - col;
  x = (v - 11.5f) * TH_PX + CX; y = (u - 15.5f) * TH_PX + CY;
}
inline float toF(float c) { return c * 1.8f + 32.0f; }   // the numbers in Fahrenheit (Toby, 2026-10-16)
inline void draw_thermal(uint16_t* fb, int y0, int rows, const Look& L, const Thermal& T, uint32_t t) {
  static uint16_t lut[64];   // the palette in 64 steps, packed (built once)
  static bool ready = false;
  if (!ready) { for (int i = 0; i < 64; i++) { uint8_t c[3]; iron(i / 63.0f, c); lut[i] = rnd::pack565_swapped(c[0], c[1], c[2]); } ready = true; }
  if (!T.px) {   // no frame yet: a black glass and why
    memset(fb, 0, (size_t)rows * rnd::W * 2);
    Strip s{fb, nullptr, y0, rows};
    centred(s, 106, "THERMAL", L.hud, 255);
    centred(s, 120, T.cam ? "STARTING" : "NO CAMERA", L.accent, blink(t, 700) ? 255 : 120);
    if (!T.cam) centred(s, 134, "TAP TO GO BACK", L.accent, 160);
    return;
  }
  const float span = T.hi - T.lo > 0.5f ? T.hi - T.lo : 0.5f, k = 63.0f / span;
  for (int y = y0; y < y0 + rows; y++) {
    uint16_t* row = fb + (y - y0) * rnd::W;
    float dy = y + 0.5f - CY;
    // down the screen = down the scene = the camera's columns from 31 to 0
    float fc = 31.0f - ((y + 0.5f - CY) / TH_PX + 15.5f); int ic = (int)floorf(fc); float wc = fc - ic;
    int ca = ic < 0 ? 0 : (ic > 31 ? 31 : ic), cb = ic + 1 > 31 ? 31 : (ic + 1 < 0 ? 0 : ic + 1);
    for (int x = 0; x < rnd::W; x++) {
      float dx = x + 0.5f - CX;
      if (dx * dx + dy * dy > 120.0f * 120.0f || fabsf(dx) > 12.0f * TH_PX || fabsf(dy) > 16.0f * TH_PX) { row[x] = 0; continue; }
      // 24 x 32 at TH_PX screen pixels each, centred: the whole image, upright, inside the glass
      float fv = (x + 0.5f - CX) / TH_PX + 11.5f, fr = TH_MIRROR ? 23.0f - fv : fv; int ir = (int)floorf(fr); float wr = fr - ir;
      int ra = ir < 0 ? 0 : (ir > 23 ? 23 : ir), rb = ir + 1 > 23 ? 23 : (ir + 1 < 0 ? 0 : ir + 1);
      const float* p = T.px;
      float v = (p[ra * 32 + ca] * (1 - wc) + p[ra * 32 + cb] * wc) * (1 - wr) + (p[rb * 32 + ca] * (1 - wc) + p[rb * 32 + cb] * wc) * wr;
      int i = (int)((v - T.lo) * k); i = i < 0 ? 0 : (i > 63 ? 63 : i);
      row[x] = lut[i];
    }
  }
  Strip s{fb, nullptr, y0, rows};
  float hx, hy; th_to_screen((float)T.hx, (float)T.hy, hx, hy);
  // the hottest spot: a ring and its temperature right beside it (Toby: "it was kind of cool when the max was on the
  // screen showing me exactly"), a dark shadow under both so they read on any colour
  static const uint8_t kW[3] = {255, 255, 255}, kK[3] = {0, 0, 0};
  for (int a = 0; a < 360; a += 10) {
    float sx = sinf(a * 0.01745f), sy = -cosf(a * 0.01745f);
    plot(s, (int)(hx + 7 * sx) + 1, (int)(hy + 7 * sy) + 1, kK, 200);
    plot(s, (int)(hx + 7 * sx), (int)(hy + 7 * sy), kW, 255);
  }
  char b[16];
  snprintf(b, sizeof b, "%.0fF", toF(T.hi));
  int tx = hx > CX + 10 ? (int)hx - 11 - text_w(b) : (int)hx + 11, ty = (int)hy - 3;
  if (ty < 28) ty = 28; if (ty > 204) ty = 204;
  text(s, tx + 1, ty + 1, b, kK, 220); text(s, tx, ty, b, kW, 255);
  // the scale, right, under the battery: hot at the top, each end's temperature beside it
  for (int y = 122; y < 165; y++) { uint8_t c[3]; iron((164 - y) / 42.0f, c); for (int x = 203; x < 208; x++) plot(s, x, y, c, 255); }
  snprintf(b, sizeof b, "%.0fF", toF(T.hi)); text(s, 211, 122, b, L.accent, 230);
  snprintf(b, sizeof b, "%.0fF", toF(T.lo)); text(s, 211, 158, b, L.accent, 230);
  if (T.sim) text(s, 6, 186, "SIM", L.hud, blink(t, 800) ? 255 : 120);
}

}  // namespace hud
