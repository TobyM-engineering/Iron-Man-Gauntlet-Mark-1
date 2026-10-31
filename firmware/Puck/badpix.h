// badpix.h - the thermal camera's faulty pixels, blended in (2026-10-16, made in the arm's session with Toby's OK:
// "a weird white square ... is it possible for you to fix that pixel, blend it in").
// Two sources:
//   the camera's own list   its EEPROM marks a pixel word 0 (broken) or bit 0 (outlier); Melexis' driver has a
//                           correction for them that the Adafruit library never calls. show_bus.h reads the list at
//                           start-up and marks them here (mark_ee) - for good.
//   found while running     a pixel that stays off its neighbours while they agree with each other (a flat patch, not
//                           an edge) AND keeps doing so as they change - the scene moving under it - is a fixed fault,
//                           not an object: a real hot spot moves off the pixel when the camera moves, and a still
//                           camera never counts at all. A found pixel keeps being judged; once it agrees with its
//                           neighbours again it is let go (healed). show_bus.h keeps the found ones in NVS.
// A marked pixel (or a read outside -40..300 C) is drawn as the median of its good neighbours, so it blends in.
// Pure maths, no hardware: the PC runs it too (a host test).
#pragma once
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace badpix {
constexpr int W = 32, H = 24, N = W * H;
constexpr float DEV_C = 1.8f;        // marked when this far off its neighbours on average (x1.5 on the border, noisier)
constexpr float SPREAD_C = 1.5f;     // judged only while the neighbours agree within this ...
constexpr float CHANGE_C = 0.7f;     // ... and only once they have changed this much since the pixel was last judged
constexpr float CLAMP_C = 6.0f;      // one judgement counts at most this much
constexpr float ALPHA = 0.2f;        // the running average's weight per judgement
constexpr uint8_t MIN_SEEN = 5;      // judgements before a pixel can be marked
constexpr uint8_t HEAL_SEEN = 8;     // judgements after marking before it can be let go
constexpr int MAX_RUN = 4;           // found pixels at most (the camera's own list comes on top)
constexpr float LO_C = -40.0f, HI_C = 300.0f;

struct State {               // ~8 KB, all of it static (the triangle buffer is sized from what is left at boot)
  bool bad[N], ee[N], haveRef[N];
  float dev[N], ref[N];
  uint8_t seen[N];
  uint32_t frames, junk, judged;
  int runCount, eeCount;
};
inline State& S() { static State s = {}; return s; }
struct Event { int marked = -1, healed = -1; };

inline bool border(int i) { int x = i % W, y = i / W; return x == 0 || y == 0 || x == W - 1 || y == H - 1; }
inline float thr(int i) { return border(i) ? 1.5f * DEV_C : DEV_C; }
inline bool junk(float v) { return !(v > LO_C && v < HI_C); }   // NaN reads as junk too

inline void mark_ee(int i) {
  State& s = S();
  if (i < 0 || i >= N || s.ee[i]) return;
  if (s.bad[i]) s.runCount--;
  s.bad[i] = s.ee[i] = true; s.eeCount++;
}
inline void mark_saved(int i) {
  State& s = S();
  if (i < 0 || i >= N || s.bad[i] || s.runCount >= MAX_RUN) return;
  s.bad[i] = true; s.seen[i] = 0; s.runCount++;
}
// the found pixels (not the camera's own) for NVS; returns how many
inline int found_list(uint16_t* out, int max) {
  State& s = S(); int n = 0;
  for (int i = 0; i < N && n < max; i++) if (s.bad[i] && !s.ee[i]) out[n++] = (uint16_t)i;
  return n;
}
// "camera's own 1 (17,9); found 0" - for the $L report
inline void describe(char* b, size_t len) {
  State& s = S(); size_t k = 0;
  auto put = [&](bool ee) {
    int n = 0;
    for (int i = 0; i < N; i++) if (s.bad[i] && s.ee[i] == ee) {
      if (n < 4 && k < len) k += snprintf(b + k, len - k, " %d/%d", i % W, i / W);
      n++;
    }
    if (n > 4 && k < len) k += snprintf(b + k, len - k, " +%d", n - 4);
  };
  k += snprintf(b + k, len - k, "masked: camera's own %d", s.eeCount); put(true);
  if (k < len) k += snprintf(b + k, len - k, "; found %d", s.runCount);
  if (k < len) put(false);
  if (k < len) snprintf(b + k, len - k, "; bad reads %lu", (unsigned long)s.junk);
}

// One frame in degrees C, row-major 32 x 24, corrected in place: only marked or junk pixels are rewritten, and those
// are never anyone's neighbour (a junk one, once rewritten, may be - it then holds a median, harmless).
inline Event fix(float* f) {
  State& s = S();
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      const int i = y * W + x;
      const bool jk = junk(f[i]);
      if (jk) s.junk++;
      float nb[8]; int n = 0;
      for (int dy = -1; dy <= 1; dy++)
        for (int dx = -1; dx <= 1; dx++) {
          if (!dx && !dy) continue;
          const int xx = x + dx, yy = y + dy;
          if (xx < 0 || xx >= W || yy < 0 || yy >= H) continue;
          const int j = yy * W + xx;
          if (!s.bad[j] && !junk(f[j])) nb[n++] = f[j];
        }
      if (n == 0) { if (jk) f[i] = 22.0f; continue; }
      for (int a = 1; a < n; a++) { float v = nb[a]; int b = a - 1; while (b >= 0 && nb[b] > v) { nb[b + 1] = nb[b]; b--; } nb[b + 1] = v; }
      const float med = (n & 1) ? nb[n / 2] : 0.5f * (nb[n / 2 - 1] + nb[n / 2]);
      if (!jk && n >= 3 && nb[n - 1] - nb[0] < SPREAD_C) {
        if (!s.haveRef[i]) { s.haveRef[i] = true; s.ref[i] = med; }
        else if (fabsf(med - s.ref[i]) > CHANGE_C) {   // the scene has moved under this pixel: judge it
          s.ref[i] = med;
          float d = f[i] - med;
          if (d > CLAMP_C) d = CLAMP_C; else if (d < -CLAMP_C) d = -CLAMP_C;
          s.dev[i] += ALPHA * (d - s.dev[i]);
          if (s.seen[i] < 255) s.seen[i]++;
          s.judged++;
        }
      }
      if (s.bad[i] || jk) f[i] = med;
    }
  s.frames++;
  Event e;
  float worst = 0;
  for (int i = 0; i < N; i++) {
    if (s.ee[i]) continue;
    const float a = fabsf(s.dev[i]), t = thr(i);
    if (!s.bad[i] && s.seen[i] >= MIN_SEEN && a > t && a - t > worst && s.runCount < MAX_RUN) { worst = a - t; e.marked = i; }
    if (s.bad[i] && e.healed < 0 && s.seen[i] >= HEAL_SEEN && a < t / 3) e.healed = i;
  }
  if (e.marked >= 0) { s.bad[e.marked] = true; s.seen[e.marked] = 0; s.runCount++; }
  if (e.healed >= 0) { s.bad[e.healed] = false; s.seen[e.healed] = 0; s.runCount--; }
  return e;
}
}  // namespace badpix
