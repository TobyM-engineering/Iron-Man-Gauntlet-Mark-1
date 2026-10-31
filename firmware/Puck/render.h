// render.h - software 3D renderer for the puck (no hardware deps; also built by the host test).
//
// Smooth (Gouraud) shading: every vertex is lit once (baked normals, split at hard CAD edges), the three
// colours are interpolated across the triangle and ordered-dithered into RGB565 so dark gradients do not
// band. 16-bit inverse-depth z-buffer, back-face culling, horizontal STRIPS so the display DMA sends strip
// N while the CPU draws strip N+1. Pixels are RGB565 already byte-swapped (what the GC9A01 wants over SPI).
//
// The look copies the HandBench arm simulator (ironman-gauntlet sim/arm.html): a white key light, a red rim
// light from behind, a cool blue fill, a cool sky / dark ground hemisphere, a room reflection on the metals
// (Fresnel-weighted, brighter at grazing angles = the bright edges), a clear coat on the black armour, ACES
// tone mapping at exposure 1.1, and its near-black blue background.
//
// Speed (measured on the puck, see the status line's profile): vertices are projected once each, not once
// per triangle; back faces are culled in screen space; depth and colour come from plane equations evaluated
// directly per row (no per-row edge-function products, no ceilf), so a strip renders bit-identical to a
// full frame; whole parts outside the view are skipped by their baked bounding sphere; the hot functions
// live in IRAM on the ESP32 (PUCK_HOT), out of the flash cache's way.
#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kin.h"

#ifndef SHADING_SMOOTH
#define SHADING_SMOOTH 1   // 0 = flat (one colour per triangle), for comparing the frame rate (config.h)
#endif
#ifndef PUCK_NOW_US
#define PUCK_NOW_US() 0    // app.cpp maps it to esp_timer_get_time() for the profile in the status line
#endif
#ifndef PUCK_CYCLES
#define PUCK_CYCLES() 0u   // app.cpp: the CPU cycle counter (profile)
#endif
#ifndef PUCK_HOT
#define PUCK_HOT inline    // app.cpp: static IRAM_ATTR noinline (static: an inline function's literals would land outside IRAM)
#endif
#define PUCK_FORCE_INLINE inline __attribute__((always_inline))

namespace rnd {

constexpr int W = 240, H = 240;

struct Profile { uint32_t vtxUs = 0, triUs = 0, rows = 0, pix = 0, setups = 0, verts = 0, tris = 0, partsSkipped = 0, lit = 0, litCycles = 0,
                 edgeUs = 0, linePx = 0; };
inline Profile& prof() { static Profile p; return p; }

// Screen-space triangle, ready to rasterize: 30 bytes (it was 52), so the per-frame buffer holds ~1.7x as many
// triangles in the same RAM. Coordinates are 1/8 px fixed point (+-4095 px), depth 16 bits like the z-buffer.
constexpr float XY_FIX = 8.0f;
constexpr float XY_LIMIT = 4000.0f;   // a vertex projected further out than this (it is near the camera plane) drops its triangle
struct Tri {
  int16_t x[3], y[3];    // screen position x 8
  uint16_t z[3];         // normalized inverse depth x 65535, 0 = far
  uint8_t ymin, ymax;    // first/last pixel row whose center is inside the bbox
  uint8_t c[3][3];       // per-vertex colour, 0..255 display values (after tone mapping)
};

struct Material {        // three.js MeshPhysicalMaterial parameters, as HandBench sets them
  float albedo[3];       // LINEAR 0..1 (convert sRGB hex with srgb_to_linear)
  float metal, rough, clearcoat;
  int shine;             // key-light highlight exponent = 2^shine (from the roughness, rounded)
  float envk = 1.0f;     // room-reflection strength (candy metal: more, for a deep-to-bright gradient)
};

struct Camera {
  kin::M34 view;         // world -> view (x right, y up, camera looks down -z)
  float f = 260, cx = 120, cy = 120, nearD = 20, farD = 2000;
};

// A glowing edge line (styles A / B): screen ends on the 1/8 px grid like Tri, depth 16 bits. 12 bytes (16 until
// 2026-10-10: the rows it touches are worked out when it is drawn, its kind - 0 crease, 1 silhouette - is z0's lowest
// bit), so 1600 fit where 1200 did: with an edge line on every armour part the hand view selects ~1500.
struct Line {
  int16_t x0, y0, x1, y1;
  uint16_t z0, z1;
};
PUCK_FORCE_INLINE int line_kind(const Line& L) { return L.z0 & 1; }

struct Frame {
  Tri* tris = nullptr;
  int cap = 0, n = 0, dropped = 0;
  Line* lines = nullptr;     // edge lines; none when lcap = 0
  int lcap = 0, ln = 0, ldropped = 0;
  uint8_t lineRGB[2][3] = {{0, 0, 0}, {0, 0, 0}};   // core colour of a crease / a silhouette (additive)
  float lineMinPx = 3.0f;    // shorter edges are skipped (clutter, cost)
  uint8_t lineHalo = 2;      // glow pixels beside the core: 0 none, 1 silhouettes only, 2 every line
  bool lineOutlinesOnly = false;   // only the silhouettes, no inner creases (a cheaper look, view.h line_mode)
  float* sp = nullptr;       // per-part scratch: screen x, y and normalized inverse depth
  uint8_t* svok = nullptr;   // bit 0: in front of the near plane, bit 1: lit (svc valid)
  uint8_t* svc = nullptr;    // per-part scratch: lit colour of each vertex (rgb)
  uint8_t* tfront = nullptr; // per-part scratch: triangle faces the camera (for the edge lines)
};
// A face within this of edge-on (|screen area| / the sum of its squared screen edges; ~0.14 face-on for an even
// triangle, so ~8 deg) keeps the facing it had last frame for the edge lines (add_part faceBits). Without it the
// outline of a low-poly finger or plate jumped between neighbouring edges and back with every sub-pixel tremor of the
// arm (2026-10-10: Toby's flickering fingers and plate 3) - silhouette selection, not the drawing, was most of it.
constexpr float FACE_HYST = 0.02f;

inline void frame_free(Frame& fr) {
  free(fr.tris); free(fr.sp); free(fr.svok); free(fr.svc); free(fr.lines); free(fr.tfront);
  fr.tris = nullptr; fr.sp = nullptr; fr.svok = nullptr; fr.svc = nullptr; fr.lines = nullptr; fr.tfront = nullptr;
  fr.cap = 0; fr.lcap = 0;
}
inline bool frame_alloc(Frame& fr, int triCap, int maxPartVerts, int lineCap = 0, int maxPartTris = 0) {
  frame_free(fr);
  fr.cap = triCap;
  fr.lcap = lineCap;
  fr.sp = (float*)malloc(sizeof(float) * 3 * maxPartVerts);
  fr.svok = (uint8_t*)malloc(maxPartVerts);
  fr.svc = (uint8_t*)malloc(3 * maxPartVerts);
  if (lineCap) { fr.lines = (Line*)malloc(sizeof(Line) * lineCap); fr.tfront = (uint8_t*)malloc(maxPartTris); }
  fr.tris = (Tri*)malloc(sizeof(Tri) * triCap);
  bool ok = fr.tris && fr.sp && fr.svok && fr.svc && (!lineCap || (fr.lines && fr.tfront));
  if (!ok) frame_free(fr);
  return ok;
}

inline uint16_t pack565_swapped(int r, int g, int b) {
  r = r < 0 ? 0 : (r > 255 ? 255 : r);
  g = g < 0 ? 0 : (g > 255 ? 255 : g);
  b = b < 0 ? 0 : (b > 255 ? 255 : b);
  uint16_t c = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
  return (uint16_t)((c >> 8) | (c << 8));
}

inline void begin(Frame& fr) { fr.n = 0; fr.dropped = 0; fr.ln = 0; fr.ldropped = 0; }

inline float srgb_to_linear(float c255) {
  float c = c255 / 255.0f;
  return c <= 0.04045f ? c / 12.92f : powf((c + 0.055f) / 1.055f, 2.4f);
}

// 1/sqrt(x), one Newton step (~0.2 %): plenty for lighting, a fraction of sqrtf + a division on the ESP32
PUCK_FORCE_INLINE float rsqrt_fast(float x) {
  union { float f; uint32_t i; } u{x};
  u.i = 0x5f3759dfu - (u.i >> 1);
  return u.f * (1.5f - 0.5f * x * u.f * u.f);
}
// floor / ceil for |t| < 4096 without the libm call (truncation of a positive number is floor)
PUCK_FORCE_INLINE int floor_fast(float t) { return (int)(t + 4096.0f) - 4096; }
PUCK_FORCE_INLINE int ceil_fast(float t) { return 4096 - (int)(4096.0f - t); }

// ---------------------------------------------------------------- lights (view space, unit, toward the light)
// HandBench's world lights seen from its default camera, carried into the puck's camera frame.
static const float kKey[3] = {0.30f, 0.55f, 0.78f};     // white 1.5, upper right front
static const float kRim[3] = {-0.30f, 0.62f, -0.73f};   // red 0xff3646 x 0.55, behind upper left
static const float kFill[3] = {-0.75f, -0.08f, 0.66f};  // blue 0x9fb4ff x 0.25, front left low
static const float kKeyRGB[3] = {2.2f, 2.2f, 2.2f};   // HandBench 1.5; brighter on the puck so the shapes read (2026-10-07)
static const float kRimRGB[3] = {0.55f * 1.0f, 0.55f * 0.0368f, 0.55f * 0.0595f};
static const float kFillRGB[3] = {0.25f * 0.347f, 0.25f * 0.456f, 0.25f * 1.0f};
static const float kSky[3] = {0.479f, 0.539f, 0.716f}, kGround[3] = {0.0060f, 0.0065f, 0.0103f};   // hemisphere 0xb8c2dc / 0x121318
constexpr float kHemi = 0.35f;
// room reflection: what a mirror sees in direction r (view space): dark below, a soft bright "ceiling" above,
// a brighter band of lights high up. Tuned by eye against a HandBench screenshot.
constexpr float kEnvLow = 0.2f, kEnvHigh = 1.0f, kEnvLights = 12.0f;
constexpr float kExposure = 1.1f;

// tone map (ACES filmic, exposure) + sRGB encode, one table: index = linear radiance * 256, 0..4
constexpr int TONE_N = 1024;
inline uint8_t* tone_table() { static uint8_t lut[TONE_N]; return lut; }
inline void tone_init() {   // once at start-up (core_init); the host tools call it too
  uint8_t* lut = tone_table();
  for (int i = 0; i < TONE_N; i++) {
    float x = (i + 0.5f) / 256.0f * kExposure;
    float t = (x * (2.51f * x + 0.03f)) / (x * (2.43f * x + 0.59f) + 0.14f);
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    float s = t <= 0.0031308f ? 12.92f * t : 1.055f * powf(t, 1 / 2.4f) - 0.055f;
    lut[i] = (uint8_t)(s * 255.0f + 0.5f);
  }
}
PUCK_FORCE_INLINE uint8_t tone(const uint8_t* lut, float x) {
  int i = (int)(x * 256.0f);
  return lut[i < 0 ? 0 : (i >= TONE_N ? TONE_N - 1 : i)];
}

PUCK_FORCE_INLINE float sat(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
PUCK_FORCE_INLINE float pow2n(float x, int n) { for (int k = 0; k < n; k++) x *= x; return x; }

// One vertex: n = unit normal, v = unit vector toward the eye (view space).
inline void shade(const float n[3], const float v[3], const Material& m, const uint8_t* lut, uint8_t out[3]) {
  float ndv = sat(n[0] * v[0] + n[1] * v[1] + n[2] * v[2]);
  float ndk = sat(n[0] * kKey[0] + n[1] * kKey[1] + n[2] * kKey[2]);
  float ndr = sat(n[0] * kRim[0] + n[1] * kRim[1] + n[2] * kRim[2]);
  float ndf = sat(n[0] * kFill[0] + n[1] * kFill[1] + n[2] * kFill[2]);
  float hy = 0.5f + 0.5f * n[1];
  // key-light half vector highlight (normalized Blinn-Phong)
  float hx = kKey[0] + v[0], hyv = kKey[1] + v[1], hz = kKey[2] + v[2];
  float ndh = sat((n[0] * hx + n[1] * hyv + n[2] * hz) * rsqrt_fast(hx * hx + hyv * hyv + hz * hz));
  float sp = pow2n(ndh, m.shine) * (float)((1 << m.shine) + 2) * 0.125f * ndk;
  float spc = m.clearcoat > 0 ? pow2n(ndh, 5) * 4.25f * ndk : 0;   // clear coat: a broader white lobe
  // Fresnel: (1 - n.v)^5 - the grazing edges catch the room and the red rim light
  float fr5 = 1 - ndv; fr5 *= fr5; fr5 *= fr5 * (1 - ndv);
  // room reflection along r = 2 (n.v) n - v
  float ry = 2 * ndv * n[1] - v[1];
  float env = kEnvLow + (kEnvHigh - kEnvLow) * sat(0.5f + 0.7f * ry) + kEnvLights * pow2n(sat(ry), 3);
  float gloss = 1 - m.rough;
  for (int c = 0; c < 3; c++) {
    float a = m.albedo[c];
    float f0 = 0.04f + (a - 0.04f) * m.metal;
    float F = f0 + (fmaxf(gloss, f0) - f0) * fr5;
    float diff = a * (1 - m.metal) * 0.3183f *
                 (kHemi * (kGround[c] + (kSky[c] - kGround[c]) * hy) * 3.1416f + kKeyRGB[c] * ndk + kFillRGB[c] * ndf + kRimRGB[c] * ndr);
    float spec = F * (kKeyRGB[c] * sp + env * m.envk * (0.35f + 0.65f * gloss)) + kRimRGB[c] * ndr * F * 6.0f;   // envk: the tinted reflection only
    float coat = m.clearcoat * (0.04f + 0.96f * fr5) * (kKeyRGB[c] * spc + env * 0.6f);
    out[c] = tone(lut, diff + spec + coat);
  }
}

// ---------------------------------------------------------------- matcap: the lighting as a table
// The lights are fixed to the camera, so with the view direction taken as straight ahead (0, 0, 1) a vertex's
// lit colour depends only on its view-space normal. Each material's colours are computed once with shade()
// on a MC_N x MC_N grid over the normal's (x, y) and looked up bilinearly per vertex: ~10x cheaper than
// shading every vertex (measured 10.4 ms -> see the status line). Off-axis the true view direction differs
// by up to ~25 deg at the screen edge; only the Fresnel edge brightening shifts a little with it.
constexpr int MC_N = 32;
struct MatCap { uint8_t rgb[MC_N * MC_N][3]; };

inline void matcap_build(MatCap& mc, const Material& m) {
  const uint8_t* lut = tone_table();
  const float v[3] = {0, 0, 1};
  for (int i = 0; i < MC_N; i++)
    for (int j = 0; j < MC_N; j++) {
      float nx = (j + 0.5f) / MC_N * 2 - 1, ny = 1 - (i + 0.5f) / MC_N * 2, nz;
      float r2 = nx * nx + ny * ny;
      if (r2 >= 1) { float s = 1 / sqrtf(r2); nx *= s; ny *= s; nz = 0; }   // outside the disc: the rim
      else nz = sqrtf(1 - r2);
      const float n[3] = {nx, ny, nz};
      shade(n, v, m, lut, mc.rgb[i * MC_N + j]);
    }
}

// n = unit view-space normal. A vertex normal turned away from the camera (on a silhouette) takes the rim.
PUCK_FORCE_INLINE void matcap_lookup(const MatCap& mc, float nx, float ny, float nz, uint8_t out[3]) {
  if (nz < 0) { float s = rsqrt_fast(nx * nx + ny * ny + 1e-12f); nx *= s; ny *= s; }
  float fx = (0.5f + 0.5f * nx) * MC_N - 0.5f, fy = (0.5f - 0.5f * ny) * MC_N - 0.5f;
  fx = fx < 0 ? 0 : (fx > MC_N - 1.001f ? MC_N - 1.001f : fx);
  fy = fy < 0 ? 0 : (fy > MC_N - 1.001f ? MC_N - 1.001f : fy);
  int x0 = (int)fx, y0 = (int)fy;
  int tx = (int)((fx - x0) * 256), ty = (int)((fy - y0) * 256);
  const uint8_t* a = mc.rgb[y0 * MC_N + x0];
  const uint8_t* b = a + 3;
  const uint8_t* c = a + 3 * MC_N;
  const uint8_t* d = c + 3;
  for (int k = 0; k < 3; k++) {
    int top = a[k] * 256 + (b[k] - a[k]) * tx, bot = c[k] * 256 + (d[k] - c[k]) * tx;
    out[k] = (uint8_t)((top * 256 + (bot - top) * ty + 32768) >> 16);
  }
}

// Is a sphere (model-space centre c, radius r) outside the view after M (model -> view)? Conservative.
inline bool sphere_outside(const Camera& cam, const kin::M34& M, const float c[3], float r) {
  const float* m = M.m;
  float x = m[0] * c[0] + m[1] * c[1] + m[2] * c[2] + m[3];
  float y = m[4] * c[0] + m[5] * c[1] + m[6] * c[2] + m[7];
  float z = m[8] * c[0] + m[9] * c[1] + m[10] * c[2] + m[11];
  if (-z + r < cam.nearD) return true;                        // wholly behind the near plane
  float tx = cam.cx / cam.f, ty = cam.cy / cam.f;             // half-width / half-height per unit depth
  float kx = rsqrt_fast(1 + tx * tx), ky = rsqrt_fast(1 + ty * ty);
  if ((x + tx * z) * kx > r || (-x + tx * z) * kx > r) return true;   // right / left of the view
  if ((y + ty * z) * ky > r || (-y + ty * z) * ky > r) return true;   // above / below
  return false;
}

// lids and caps (unlit parts): a flat near-black, a little darker than the background - the depth of an open bay
#ifndef PUCK_BACK_RGB
#define PUCK_BACK_RGB {5, 5, 7}
#endif
static const uint8_t kBackRGB[3] = PUCK_BACK_RGB;

// Precomputed visibility (round 5): puck_visibility.cpp records, per triangle, from which of 32 view directions
// (a Fibonacci sphere, world frame, "toward the eye") it ever won a pixel - with every plate shut (bits 0-31) and with a
// plate open (bits 32-63). The puck skips a triangle its current direction never sees: the inside skin behind the near
// wall, the far side of the hand, the dark backs of the openings when the opening faces away.
constexpr int VIS_BINS = 32;
inline void vis_bin_dir(int i, float d[3]) {
  float y = 1 - 2 * (i + 0.5f) / VIS_BINS, r = sqrtf(1 - y * y), ph = i * 2.39996323f;
  d[0] = r * cosf(ph); d[1] = y; d[2] = r * sinf(ph);
}
inline int vis_bin(const float z[3]) {   // the bin nearest to direction z (unit)
  int best = 0; float bd = -2;
  for (int i = 0; i < VIS_BINS; i++) {
    float d[3]; vis_bin_dir(i, d);
    float dot = d[0] * z[0] + d[1] * z[1] + d[2] * z[2];
    if (dot > bd) { bd = dot; best = i; }
  }
  return best;
}

// tint (round 10): {1, ..} = grey (the link lost), {3, ..} = a lighter grey (a body colour to pick, 2026-10-10),
// {2, r, g, b} = add a glow; applied to each vertex's lit colour
PUCK_FORCE_INLINE void apply_tint(uint8_t c[3], const uint8_t* tint) {
  if (tint[0] == 1) { uint8_t l = (uint8_t)((c[0] * 77 + c[1] * 150 + c[2] * 29) >> 8); c[0] = c[1] = c[2] = l; return; }
  if (tint[0] == 3) { int l = ((c[0] * 77 + c[1] * 150 + c[2] * 29) >> 8) * 13 / 10 + 22; c[0] = c[1] = c[2] = (uint8_t)(l > 255 ? 255 : l); return; }
  for (int k = 0; k < 3; k++) { int v = c[k] + tint[k + 1]; c[k] = (uint8_t)(v > 255 ? 255 : v); }
}

// Transform one part (model-space vertices + normals) by M (model -> view), cull, shade, project, append.
PUCK_HOT void add_part(Frame& fr, const Camera& cam, const kin::M34& M, const float (*verts)[3], const int8_t (*norms)[3],
                              int nv, const uint16_t (*tris)[3], int nt, const MatCap& mc,
                              const uint16_t (*edges)[4] = nullptr, int ne = 0, const uint8_t* flatRGB = nullptr,
                              const uint64_t* visDirs = nullptr, int t0 = 0, uint64_t visMask = ~0ull,
                              bool twoSided = false, const uint8_t* tint = nullptr,      // twoSided: flatRGB parts only (lids, caps)
                              uint8_t* faceBits = nullptr) {   // this part's facing memory (FACE_HYST), a bit per triangle
  const float* m = M.m;
  const float invNear = 1.0f / cam.nearD, invFar = 1.0f / cam.farD;
  const float zscale = 1.0f / (invNear - invFar);
  int64_t tp0 = PUCK_NOW_US();
  for (int i = 0; i < nv; i++) {   // transform + project every vertex once (screen x, y on the 1/8 px grid)
    const float* a = verts[i];
    float ox = m[0] * a[0] + m[1] * a[1] + m[2] * a[2] + m[3];
    float oy = m[4] * a[0] + m[5] * a[1] + m[6] * a[2] + m[7];
    float oz = m[8] * a[0] + m[9] * a[1] + m[10] * a[2] + m[11];
    fr.svok[i] = 0;
    if (-oz >= cam.nearD) {
      float iz = 1.0f / -oz;
      float sx = cam.cx + cam.f * ox * iz, sy = cam.cy - cam.f * oy * iz;
      if (fabsf(sx) < XY_LIMIT && fabsf(sy) < XY_LIMIT) {
        float* s = fr.sp + 3 * i;
        s[0] = (float)((int)(sx * XY_FIX + 32768.5f) - 32768) * (1.0f / XY_FIX);   // snapped: exactly what the rasterizer sees
        s[1] = (float)((int)(sy * XY_FIX + 32768.5f) - 32768) * (1.0f / XY_FIX);
        s[2] = (iz - invFar) * zscale;
        fr.svok[i] = 1;
      }
    }
  }
  int64_t tp1 = PUCK_NOW_US();
  prof().vtxUs += (uint32_t)(tp1 - tp0); prof().verts += nv; prof().tris += nt;
  const bool wantEdges = edges && ne && fr.lines;
  if (wantEdges) memset(fr.tfront, 0, nt);
  for (int t = 0; t < nt; t++) {
    if (fr.n >= fr.cap) { fr.dropped += nt - t; break; }
    const uint16_t* ix = tris[t];
    int i0 = ix[0], i1 = ix[1], i2 = ix[2];
    const bool vok = (fr.svok[i0] & fr.svok[i1] & fr.svok[i2] & 1) != 0;
    const bool skipped = visDirs && !(visDirs[t0 + t] & visMask);   // never seen from this direction
    if (!vok || (skipped && !wantEdges)) continue;
    const float* p0 = fr.sp + 3 * i0;
    const float* p1 = fr.sp + 3 * i1;
    const float* p2 = fr.sp + 3 * i2;
    // screen-space winding: front faces (CCW seen from the camera, y up) have area < 0 with y down
    float area = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p2[0] - p0[0]) * (p1[1] - p0[1]);
    if (wantEdges) {   // the facing for the edge lines - of a skipped face too (it counted as facing away, a false outline)
      bool f = area <= 0;   // a triangle snapped flat counts as facing: no false silhouette
      if (faceBits) {
        uint8_t& m = faceBits[t >> 3]; const uint8_t bit = (uint8_t)(1 << (t & 7));
        float ax = p1[0] - p0[0], ay = p1[1] - p0[1], bx = p2[0] - p0[0], by = p2[1] - p0[1], cx = p2[0] - p1[0], cy = p2[1] - p1[1];
        if (fabsf(area) < FACE_HYST * (ax * ax + ay * ay + bx * bx + by * by + cx * cx + cy * cy)) f = (m & bit) != 0;
        else if (f) m |= bit; else m &= (uint8_t)~bit;
      }
      fr.tfront[t] = f;
      if (skipped) continue;
    }
    if (area >= 0 && !(twoSided && area > 0)) continue;
    float minx = fminf(p0[0], fminf(p1[0], p2[0])), maxx = fmaxf(p0[0], fmaxf(p1[0], p2[0]));
    float miny = fminf(p0[1], fminf(p1[1], p2[1])), maxy = fmaxf(p0[1], fmaxf(p1[1], p2[1]));
    if (maxx < 0 || minx > W || maxy < 0 || miny > H) continue;
    int ylo = ceil_fast(fmaxf(miny, -1.0f) - 0.5f); if (ylo < 0) ylo = 0;
    int yhi = floor_fast(fminf(maxy, H + 1.0f) - 0.5f); if (yhi > H - 1) yhi = H - 1;
    if (ylo > yhi) continue;

    Tri& T = fr.tris[fr.n];
    if (flatRGB) {                           // a lid / cap: one flat colour, no lighting
      for (int k = 0; k < 3; k++) { T.c[k][0] = flatRGB[0]; T.c[k][1] = flatRGB[1]; T.c[k][2] = flatRGB[2]; }
      const float* q[3] = {p0, area < 0 ? p2 : p1, area < 0 ? p1 : p2};   // positive area for the rasterizer
      for (int k = 0; k < 3; k++) {
        T.x[k] = (int16_t)(int)(q[k][0] * XY_FIX + (q[k][0] >= 0 ? 0.5f : -0.5f));
        T.y[k] = (int16_t)(int)(q[k][1] * XY_FIX + (q[k][1] >= 0 ? 0.5f : -0.5f));
        float z = q[k][2] < 0 ? 0 : (q[k][2] > 1 ? 1 : q[k][2]);
        T.z[k] = (uint16_t)(z * 65535.0f + 0.5f);
      }
      T.ymin = (uint8_t)ylo; T.ymax = (uint8_t)yhi;
      fr.n++;
      continue;
    }
#if !SHADING_SMOOTH
    {   // flat: one colour per triangle from its face normal (model space, turned into view space; for comparison)
      const float* a = verts[i0]; const float* b = verts[i1]; const float* c = verts[i2];
      float e1x = b[0] - a[0], e1y = b[1] - a[1], e1z = b[2] - a[2], e2x = c[0] - a[0], e2y = c[1] - a[1], e2z = c[2] - a[2];
      float qx = e1y * e2z - e1z * e2y, qy = e1z * e2x - e1x * e2z, qz = e1x * e2y - e1y * e2x;
      float nx = m[0] * qx + m[1] * qy + m[2] * qz, ny = m[4] * qx + m[5] * qy + m[6] * qz, nz = m[8] * qx + m[9] * qy + m[10] * qz;
      float il = rsqrt_fast(nx * nx + ny * ny + nz * nz + 1e-24f);
      matcap_lookup(mc, nx * il, ny * il, nz * il, T.c[0]);
      if (tint) apply_tint(T.c[0], tint);
      memcpy(T.c[1], T.c[0], 3); memcpy(T.c[2], T.c[0], 3);
    }
#else
    uint32_t lc0 = PUCK_CYCLES();
    for (int k = 0; k < 3; k++) {   // light each vertex once per part
      int vi = ix[k];
      if (fr.svok[vi] & 2) continue;
      const int8_t* q = norms[vi];
      float qx = q[0], qy = q[1], qz = q[2];
      float wn[3] = {m[0] * qx + m[1] * qy + m[2] * qz, m[4] * qx + m[5] * qy + m[6] * qz, m[8] * qx + m[9] * qy + m[10] * qz};
      float il = rsqrt_fast(wn[0] * wn[0] + wn[1] * wn[1] + wn[2] * wn[2] + 1e-12f);
      matcap_lookup(mc, wn[0] * il, wn[1] * il, wn[2] * il, fr.svc + 3 * vi);
      if (tint) apply_tint(fr.svc + 3 * vi, tint);
      fr.svok[vi] |= 2;
      prof().lit++;
    }
    prof().litCycles += PUCK_CYCLES() - lc0;
    memcpy(T.c[0], fr.svc + 3 * i0, 3);
    memcpy(T.c[1], fr.svc + 3 * i2, 3);   // vertices 1 and 2 swapped: positive area for the rasterizer
    memcpy(T.c[2], fr.svc + 3 * i1, 3);
#endif
    const float* q[3] = {p0, p2, p1};   // vertices 1 and 2 swapped: positive area for the rasterizer
    for (int k = 0; k < 3; k++) {
      T.x[k] = (int16_t)(int)(q[k][0] * XY_FIX + (q[k][0] >= 0 ? 0.5f : -0.5f));
      T.y[k] = (int16_t)(int)(q[k][1] * XY_FIX + (q[k][1] >= 0 ? 0.5f : -0.5f));
      float z = q[k][2] < 0 ? 0 : (q[k][2] > 1 ? 1 : q[k][2]);
      T.z[k] = (uint16_t)(z * 65535.0f + 0.5f);
    }
    T.ymin = (uint8_t)ylo; T.ymax = (uint8_t)yhi;
    fr.n++;
  }
  prof().triUs += (uint32_t)(PUCK_NOW_US() - tp1);
  if (!wantEdges) return;

  // ---- edge lines: a sharp crease with a face toward the camera, or a silhouette (exactly one face toward it)
  int64_t tp2 = PUCK_NOW_US();
  const float minLen2 = fr.lineMinPx * fr.lineMinPx;
  for (int k = 0; k < ne; k++) {
    const uint16_t* e = edges[k];
    int a = e[0], b = e[1], f0 = e[2], f1 = e[3] & 0x7FFF;
    bool sharp = (e[3] & 0x8000) != 0;
    bool fr0 = fr.tfront[f0], fr1 = f1 != 0x7FFF ? fr.tfront[f1] : fr0;
    bool silhouette = f1 != 0x7FFF && fr0 != fr1;
    if (!(silhouette || (sharp && (fr0 || fr1) && !fr.lineOutlinesOnly))) continue;
    if (!(fr.svok[a] & fr.svok[b] & 1)) continue;
    const float* pa = fr.sp + 3 * a;
    const float* pb = fr.sp + 3 * b;
    float dx = pb[0] - pa[0], dy = pb[1] - pa[1];
    if (dx * dx + dy * dy < minLen2) continue;
    float miny = fminf(pa[1], pb[1]) - 1.5f, maxy = fmaxf(pa[1], pb[1]) + 1.5f;
    if (fmaxf(pa[0], pb[0]) < -2 || fminf(pa[0], pb[0]) > W + 2 || maxy < 0 || miny > H) continue;
    if (fr.ln >= fr.lcap) { fr.ldropped++; continue; }
    Line& L = fr.lines[fr.ln++];
    L.x0 = (int16_t)(int)(pa[0] * XY_FIX + (pa[0] >= 0 ? 0.5f : -0.5f));
    L.y0 = (int16_t)(int)(pa[1] * XY_FIX + (pa[1] >= 0 ? 0.5f : -0.5f));
    L.x1 = (int16_t)(int)(pb[0] * XY_FIX + (pb[0] >= 0 ? 0.5f : -0.5f));
    L.y1 = (int16_t)(int)(pb[1] * XY_FIX + (pb[1] >= 0 ? 0.5f : -0.5f));
    L.z0 = (uint16_t)(((int)(sat(pa[2]) * 65535.0f + 0.5f) & ~1) | (silhouette ? 1 : 0));
    L.z1 = (uint16_t)(sat(pb[2]) * 65535.0f + 0.5f);
  }
  prof().edgeUs += (uint32_t)(PUCK_NOW_US() - tp2);
}

// ---------------------------------------------------------------- background
// HandBench's background 0x0a0b0f (no ring at the glass edge since 2026-10-07)
inline uint16_t bg_color() { return pack565_swapped(10, 11, 15); }

PUCK_HOT void clear_strip(uint16_t* fb, uint16_t* zb, int y0, int rows) {
  (void)y0;
  const uint16_t bg = bg_color();
  const uint32_t bg2 = ((uint32_t)bg << 16) | bg;
  uint32_t* p = (uint32_t*)fb;
  for (int i = 0; i < rows * W / 2; i++) p[i] = bg2;
  memset(zb, 0, (size_t)rows * W * sizeof(uint16_t));
}

// ---------------------------------------------------------------- rasterizer
// 4x4 Bayer matrix: added before quantising to 5/6 bits so a smooth ramp dithers instead of banding
static const uint8_t kBayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};

PUCK_FORCE_INLINE uint16_t pack_dither(int r, int g, int b, int d) {   // d = Bayer value 0..15
  r += d >> 1; g += d >> 2; b += d >> 1;
  r = r < 0 ? 0 : (r > 255 ? 255 : r); g = g < 0 ? 0 : (g > 255 ? 255 : g); b = b < 0 ? 0 : (b > 255 ? 255 : b);
  uint16_t c = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
  return (uint16_t)((c >> 8) | (c << 8));
}

// value at (x, y) = base + dx * x + dy * y for one attribute across a triangle
struct Plane { float base, dx, dy; };
PUCK_FORCE_INLINE Plane plane_of(const float xs[3], const float ys[3], float v0, float v1, float v2, float inv) {
  float ex1 = xs[1] - xs[0], ey1 = ys[1] - ys[0], ex2 = xs[2] - xs[0], ey2 = ys[2] - ys[0];
  float dv1 = v1 - v0, dv2 = v2 - v0;
  Plane p;
  p.dx = (dv1 * ey2 - dv2 * ey1) * inv;
  p.dy = (dv2 * ex1 - dv1 * ex2) * inv;
  p.base = v0 - p.dx * xs[0] - p.dy * ys[0];
  return p;
}

PUCK_HOT void raster_strip(const Tri* tris, int n, int y0, int rows, uint16_t* fb, uint16_t* zb) {
  const int y1 = y0 + rows - 1;
  const float ZS = 65535.0f * 256.0f, CS = 65536.0f;
  Profile& P = prof();
  for (int k = 0; k < n; k++) {
    const Tri& T = tris[k];
    if (T.ymax < y0 || T.ymin > y1) continue;
    const float s = 1.0f / XY_FIX;
    const float xs[3] = {T.x[0] * s, T.x[1] * s, T.x[2] * s}, ys[3] = {T.y[0] * s, T.y[1] * s, T.y[2] * s};
    float A[3], Bc[3], slope[3];
    for (int i = 0; i < 3; i++) {
      int j = i == 2 ? 0 : i + 1;
      A[i] = ys[i] - ys[j];
      Bc[i] = xs[j] - xs[i];
      slope[i] = (fabsf(A[i]) > 1e-6f) ? -Bc[i] / A[i] : 0.0f;
    }
    float det = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0]);
    float inv = 1.0f / det;
    const float zs = 1.0f / 65535.0f;
    Plane pz = plane_of(xs, ys, T.z[0] * zs, T.z[1] * zs, T.z[2] * zs, inv);
    bool flat = !memcmp(T.c[0], T.c[1], 3) && !memcmp(T.c[0], T.c[2], 3);
    Plane pr, pg, pb;
    if (!flat) {
      pr = plane_of(xs, ys, T.c[0][0], T.c[1][0], T.c[2][0], inv);
      pg = plane_of(xs, ys, T.c[0][1], T.c[1][1], T.c[2][1], inv);
      pb = plane_of(xs, ys, T.c[0][2], T.c[1][2], T.c[2][2], inv);
    } else {
      pr = {(float)T.c[0][0], 0, 0}; pg = {(float)T.c[0][1], 0, 0}; pb = {(float)T.c[0][2], 0, 0};
    }
    const int32_t dz = (int32_t)(pz.dx * ZS), dcr = (int32_t)(pr.dx * CS), dcg = (int32_t)(pg.dx * CS), dcb = (int32_t)(pb.dx * CS);
    int ys0 = T.ymin > y0 ? T.ymin : y0;
    int ys1 = T.ymax < y1 ? T.ymax : y1;
    P.setups++; P.rows += ys1 - ys0 + 1;
    for (int y = ys0; y <= ys1; y++) {
      float py = y + 0.5f;
      float xlo = -2.0f, xhi = W + 2.0f;
      bool empty = false;
      for (int i = 0; i < 3; i++) {
        if (fabsf(A[i]) > 1e-6f) {
          float xb = xs[i] + slope[i] * (py - ys[i]);
          if (A[i] > 0) { if (xb > xlo) xlo = xb; }
          else          { if (xb < xhi) xhi = xb; }
        } else if (Bc[i] * (py - ys[i]) < 0) { empty = true; }
      }
      if (empty || xlo > xhi) continue;
      int ix0 = ceil_fast(xlo - 0.5f); if (ix0 < 0) ix0 = 0;
      int ix1 = floor_fast(xhi - 0.5f); if (ix1 > W - 1) ix1 = W - 1;
      if (ix0 > ix1) continue;
      P.pix += ix1 - ix0 + 1;
      float px = ix0 + 0.5f;
      int32_t acc = (int32_t)((pz.base + pz.dy * py + pz.dx * px) * ZS);
      int32_t cr = (int32_t)((pr.base + pr.dy * py + pr.dx * px) * CS);
      int32_t cg = (int32_t)((pg.base + pg.dy * py + pg.dx * px) * CS);
      int32_t cb = (int32_t)((pb.base + pb.dy * py + pb.dx * px) * CS);
      uint16_t* f = fb + (y - y0) * W;
      uint16_t* z = zb + (y - y0) * W;
      const uint8_t* bay = kBayer[y & 3];
      for (int x = ix0; x <= ix1; x++) {
        int zq = acc >> 8;
        if (zq > z[x]) {
          z[x] = (uint16_t)(zq > 65535 ? 65535 : zq);
          f[x] = pack_dither(cr >> 16, cg >> 16, cb >> 16, bay[x & 3]);
        }
        acc += dz; cr += dcr; cg += dcg; cb += dcb;
      }
    }
  }
}

// ---------------------------------------------------------------- edge lines (additive glow)
// add (r, g, b) to a byte-swapped RGB565 pixel, saturating
PUCK_FORCE_INLINE void add565(uint16_t& p, int r, int g, int b) {
  uint16_t c = (uint16_t)((p >> 8) | (p << 8));
  int R = (c >> 11) & 31, G = (c >> 5) & 63, B = c & 31;
  R = (R << 3 | R >> 2) + r; G = (G << 2 | G >> 4) + g; B = (B << 3 | B >> 2) + b;
  R = R > 255 ? 255 : R; G = G > 255 ? 255 : G; B = B > 255 ? 255 : B;
  c = (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
  p = (uint16_t)((c >> 8) | (c << 8));
}

// Anti-aliased (2026-10-10): in each column a shallow line crosses (row, for a steep one) the line lights the two
// pixels nearest its centre in proportion to how near it is to each, so a line that moves by a fraction of a pixel
// fades from one pixel into the next. The 1-px lines before it jumped a whole pixel at a time, and the arm is never
// still (the IMUs' jitter is a fraction of a pixel): every edge shimmered - Toby's "fingers flicker". Spans are
// half-open, so two lines meeting at a vertex never both light its column. A halo pixel each side of the pair at 3/8
// of the colour; every pixel depth-tested against the model with a small bias toward the camera (the line lies on
// the surface).
PUCK_HOT void raster_lines(const Frame& fr, int y0, int rows, uint16_t* fb, uint16_t* zb) {
  const int y1 = y0 + rows - 1;
  Profile& P = prof();
  for (int k = 0; k < fr.ln; k++) {
    const Line& L = fr.lines[k];
    const int ly0 = (L.y0 < L.y1 ? L.y0 : L.y1) / (int)XY_FIX - 2, ly1 = (L.y0 > L.y1 ? L.y0 : L.y1) / (int)XY_FIX + 2;   // rows touched, halo included
    if (ly1 < y0 || ly0 > y1) continue;
    const int kind = line_kind(L);
    const uint8_t* c = fr.lineRGB[kind];
    const int hw = fr.lineHalo == 2 || (fr.lineHalo == 1 && kind == 1) ? 1 : 0;
    const float xa = L.x0 * (1.0f / XY_FIX), ya = L.y0 * (1.0f / XY_FIX), xb = L.x1 * (1.0f / XY_FIX), yb = L.y1 * (1.0f / XY_FIX);
    const bool xmajor = fabsf(xb - xa) >= fabsf(yb - ya);
    float ua = xmajor ? xa : ya, ub = xmajor ? xb : yb, va = xmajor ? ya : xa, vb = xmajor ? yb : xb;
    float za = L.z0, zbb = L.z1;
    if (ua > ub) { float t = ua; ua = ub; ub = t; t = va; va = vb; vb = t; t = za; za = zbb; zbb = t; }
    const float du = ub - ua;
    if (du < 1e-3f) continue;
    const float slope = (vb - va) / du, zslope = (zbb - za) / du;
    int u0 = ceil_fast(ua - 0.5f), u1 = ceil_fast(ub - 0.5f) - 1;   // the pixels whose centres lie in [ua, ub)
    const int ulo = xmajor ? 0 : y0, uhi = xmajor ? W - 1 : y1;
    if (u0 < ulo) u0 = ulo;
    if (u1 > uhi) u1 = uhi;
    if (xmajor && fabsf(slope) > 1e-4f) {   // a shallow line: only the columns where it is within this band of rows
      float ta = ((float)y0 - 2.0f - va) / slope + ua - 0.5f, tb = ((float)y1 + 2.0f - va) / slope + ua - 0.5f;
      if (ta > tb) { float t = ta; ta = tb; tb = t; }
      const int ca = floor_fast(ta), cb = ceil_fast(tb);
      if (ca > u0) u0 = ca;
      if (cb < u1) u1 = cb;
    }
    if (u0 > u1) continue;
    // fixed-point stepping (16.16 across the line, depth x16): no float maths per pixel (2026-10-11, fps)
    const float t0 = u0 + 0.5f - ua;
    int32_t vf = (int32_t)((va + slope * t0 - 0.5f) * 65536.0f);
    const int32_t dv = (int32_t)(slope * 65536.0f);
    int32_t zf = (int32_t)((za + zslope * t0) * 16.0f);
    const int32_t dz = (int32_t)(zslope * 16.0f);
    const int c0 = c[0], c1 = c[1], c2 = c[2];
    for (int u = u0; u <= u1; u++, vf += dv, zf += dz) {
      const int v0 = vf >> 16;                                   // the line's centre, in pixel indices across it
      const int w1 = (vf >> 8) & 255, w0 = 256 - w1;             // the two nearest pixels' shares
      int zl = zf >> 4;
      zl += (zl >> 8) + 4;   // ~1 mm toward the camera at any zoom (zl >> 6 was 3-5 mm: hidden edges showed through)
      for (int h = -hw; h <= 1 + hw; h++) {
        const int vv = v0 + h;
        const int x = xmajor ? u : vv, y = xmajor ? vv : u;
        if (y < y0 || y > y1 || x < 0 || x >= W) continue;
        const int o = (y - y0) * W + x;
        if (zl < zb[o]) continue;
        const int w = h == 0 ? w0 : (h == 1 ? w1 : (h < 0 ? w0 : w1) * 3 / 8);
        if (w < 8) continue;
        add565(fb[o], c0 * w >> 8, c1 * w >> 8, c2 * w >> 8);
        P.linePx++;
      }
    }
  }
}

// ---------------------------------------------------------------- light effects (cheap 2D, depth-tested)
// A glow: a round additive spot at a 3D point, falling off as (1 - d^2/r^2)^2. Each pixel is tested against the model
// with the glow's own depth, so a light behind the hand stays hidden; a "halo" glow (bg) lands only on the background:
// the light a palm LED throws round the hand when the palm faces away.
struct Glow { int16_t x, y; uint16_t z; uint8_t r; uint8_t rgb[3]; uint8_t bg; };
// A beam: a depth-tested additive line with a soft width (core + falloff), e.g. a laser or the LiDAR.
struct Beam { int16_t x0, y0, x1, y1; uint16_t z0, z1; uint8_t halfw; uint8_t rgb[3]; };
constexpr int MAX_GLOWS = 40, MAX_BEAMS = 40;
struct Fx {
  Glow glows[MAX_GLOWS]; int ng = 0;
  Beam beams[MAX_BEAMS]; int nb = 0;
  void clear() { ng = 0; nb = 0; }
};

// world point -> screen (1/8 px) and normalized inverse depth; false when behind the near plane
inline bool project(const Camera& cam, const kin::M34& M, const float p[3], float& sx, float& sy, float& z, float& depth) {
  const float* m = M.m;
  float ox = m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3];
  float oy = m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7];
  float oz = m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11];
  if (-oz < cam.nearD) return false;
  float iz = 1.0f / -oz;
  sx = cam.cx + cam.f * ox * iz; sy = cam.cy - cam.f * oy * iz;
  const float invNear = 1.0f / cam.nearD, invFar = 1.0f / cam.farD;
  z = (iz - invFar) / (invNear - invFar);
  depth = -oz;
  return true;
}

// A glow of radius rmm (millimetres, scaled by perspective) at model point p under M; colour rgb x k.
inline void fx_glow(Fx& fx, const Camera& cam, const kin::M34& M, const float p[3], float rmm, const uint8_t rgb[3], float k, bool bg = false,
                    float minPx = 0) {   // minPx: never smaller than this on screen (a small light still shows zoomed out)
  if (fx.ng >= MAX_GLOWS || k <= 0.01f) return;
  float sx, sy, z, d;
  if (!project(cam, M, p, sx, sy, z, d)) return;
  float r = rmm * cam.f / d;
  if (r < minPx) r = minPx;
  if (r < 1.5f || sx < -r || sx > W + r || sy < -r || sy > H + r) return;
  Glow& g = fx.glows[fx.ng++];
  g.x = (int16_t)sx; g.y = (int16_t)sy; g.z = (uint16_t)(sat(z) * 65535.0f); g.r = (uint8_t)(r > 64 ? 64 : r); g.bg = bg;
  for (int c = 0; c < 3; c++) { float v = rgb[c] * k; g.rgb[c] = (uint8_t)(v > 255 ? 255 : v); }
}

// A beam from model point a to b (clipped to the near plane), half width in px.
inline void fx_beam(Fx& fx, const Camera& cam, const kin::M34& M, const float a[3], const float b[3], int halfw, const uint8_t rgb[3]) {
  if (fx.nb >= MAX_BEAMS) return;
  const float* m = M.m;
  float pa[3], pb[3];
  for (int i = 0; i < 3; i++) {
    pa[i] = m[i * 4] * a[0] + m[i * 4 + 1] * a[1] + m[i * 4 + 2] * a[2] + m[i * 4 + 3];
    pb[i] = m[i * 4] * b[0] + m[i * 4 + 1] * b[1] + m[i * 4 + 2] * b[2] + m[i * 4 + 3];
  }
  float da = -pa[2], db = -pb[2];
  if (da < cam.nearD && db < cam.nearD) return;
  if (da < cam.nearD || db < cam.nearD) {   // clip at the near plane
    float t = (cam.nearD - da) / (db - da);
    float q[3] = {pa[0] + (pb[0] - pa[0]) * t, pa[1] + (pb[1] - pa[1]) * t, pa[2] + (pb[2] - pa[2]) * t};
    if (da < cam.nearD) memcpy(pa, q, sizeof q); else memcpy(pb, q, sizeof q);
  }
  const float invNear = 1.0f / cam.nearD, invFar = 1.0f / cam.farD, zs = 1.0f / (invNear - invFar);
  float ia = 1.0f / -pa[2], ib = 1.0f / -pb[2];
  float x0 = cam.cx + cam.f * pa[0] * ia, y0 = cam.cy - cam.f * pa[1] * ia;
  float x1 = cam.cx + cam.f * pb[0] * ib, y1 = cam.cy - cam.f * pb[1] * ib;
  // clip the 2D segment to a generous box so the fixed-point ends cannot overflow
  const float lo = -200, hi = W + 200;
  for (int pass = 0; pass < 2; pass++) {
    float& ex = pass ? x1 : x0; float& ey = pass ? y1 : y0;
    float ox = pass ? x0 : x1, oy = pass ? y0 : y1;
    if (ex < lo || ex > hi || ey < lo || ey > hi) {
      float t = 1.0f;
      if (ex < lo) t = fminf(t, (ox - lo) / (ox - ex));
      if (ex > hi) t = fminf(t, (hi - ox) / (ex - ox));
      if (ey < lo) t = fminf(t, (oy - lo) / (oy - ey));
      if (ey > hi) t = fminf(t, (hi - oy) / (ey - oy));
      t = t < 0 ? 0 : t;
      ex = ox + (ex - ox) * t; ey = oy + (ey - oy) * t;
    }
  }
  Beam& B = fx.beams[fx.nb++];
  B.x0 = (int16_t)(x0 * XY_FIX); B.y0 = (int16_t)(y0 * XY_FIX); B.x1 = (int16_t)(x1 * XY_FIX); B.y1 = (int16_t)(y1 * XY_FIX);
  B.z0 = (uint16_t)(sat((ia - invFar) * zs) * 65535.0f); B.z1 = (uint16_t)(sat((ib - invFar) * zs) * 65535.0f);
  B.halfw = (uint8_t)halfw; memcpy(B.rgb, rgb, 3);
}

PUCK_HOT void raster_fx(const Fx& fx, int y0, int rows, uint16_t* fb, uint16_t* zb) {
  const int y1 = y0 + rows - 1;
  for (int k = 0; k < fx.ng; k++) {
    const Glow& g = fx.glows[k];
    int r = g.r;
    if (g.y + r < y0 || g.y - r > y1) continue;
    int ya = g.y - r < y0 ? y0 : g.y - r, yb = g.y + r > y1 ? y1 : g.y + r;
    int zg = g.z + (g.z >> 6) + 2;
    const float ir2 = 1.0f / (float)(r * r);
    for (int y = ya; y <= yb; y++) {
      int dy = y - g.y, span = (int)sqrtf((float)(r * r - dy * dy));
      int xa = g.x - span < 0 ? 0 : g.x - span, xb = g.x + span >= W ? W - 1 : g.x + span;
      uint16_t* zr = zb + (y - y0) * W;
      uint16_t* fr_ = fb + (y - y0) * W;
      for (int x = xa; x <= xb; x++) {
        if (g.bg ? zr[x] != 0 : zg < zr[x]) continue;
        int dx = x - g.x;
        float t = 1.0f - (float)(dx * dx + dy * dy) * ir2;
        if (t <= 0) continue;
        int f = (int)(t * t * 256.0f);
        add565(fr_[x], g.rgb[0] * f >> 8, g.rgb[1] * f >> 8, g.rgb[2] * f >> 8);
      }
    }
  }
  for (int k = 0; k < fx.nb; k++) {
    const Beam& B = fx.beams[k];
    int hw = B.halfw;
    int ymin = (B.y0 < B.y1 ? B.y0 : B.y1) / (int)XY_FIX - hw - 1, ymax = (B.y0 > B.y1 ? B.y0 : B.y1) / (int)XY_FIX + hw + 1;
    if (ymax < y0 || ymin > y1) continue;
    int ddx = B.x1 - B.x0, ddy = B.y1 - B.y0;
    int adx = ddx < 0 ? -ddx : ddx, ady = ddy < 0 ? -ddy : ddy;
    int steps = (adx > ady ? adx : ady) / (int)XY_FIX + 1;
    const int32_t sx = ((int32_t)ddx << 13) / steps, sy = ((int32_t)ddy << 13) / steps;
    const int32_t sz = (((int32_t)B.z1 - (int32_t)B.z0) << 14) / steps;
    // only the stretch of the beam that falls in this strip's rows (2026-10-10: walking every beam's whole length for
    // every strip cost the hand view ~1.5 fps with the forearm strip lit - long segments, mostly off this strip)
    int i0 = 0, i1 = steps;
    if (ddy != 0) {
      float ta = ((float)((y0 - hw - 1) * (int)XY_FIX) - B.y0) * steps / ddy, tb = ((float)((y1 + hw + 2) * (int)XY_FIX) - B.y0) * steps / ddy;
      if (ta > tb) { float t = ta; ta = tb; tb = t; }
      int a = (int)floorf(ta) - 1, b = (int)ceilf(tb) + 1;
      i0 = a > 0 ? a : 0; i1 = b < steps ? b : steps;
      if (i0 > i1) continue;
    }
    if (ddx != 0) {   // ... and inside the screen's width (a segment near the camera can run thousands of pixels off it)
      float ta = ((float)((-hw - 1) * (int)XY_FIX) - B.x0) * steps / ddx, tb = ((float)((W + hw + 1) * (int)XY_FIX) - B.x0) * steps / ddx;
      if (ta > tb) { float t = ta; ta = tb; tb = t; }
      int a = (int)floorf(ta) - 1, b = (int)ceilf(tb) + 1;
      if (a > i0) i0 = a;
      if (b < i1) i1 = b;
      if (i0 > i1) continue;
    }
    int32_t xf = ((int32_t)B.x0 << 13) + sx * i0, yf = ((int32_t)B.y0 << 13) + sy * i0, zf = ((int32_t)B.z0 << 14) + sz * i0;
    bool xmajor = adx >= ady;
    for (int i = i0; i <= i1; i++, xf += sx, yf += sy, zf += sz) {
      int px = xf >> 16, py = yf >> 16;
      if (py < y0 - hw - 1 || py > y1 + hw + 1) continue;
      int zl = zf >> 14; zl += (zl >> 6) + 2;
      for (int h = -hw; h <= hw; h++) {
        int qx = xmajor ? px : px + h, qy = xmajor ? py + h : py;
        if (qy < y0 || qy > y1 || qx < 0 || qx >= W) continue;
        int o = (qy - y0) * W + qx;
        if (zl < zb[o]) continue;
        int ah = h < 0 ? -h : h;
        int f = ah == 0 ? 256 : (512 * (hw + 1 - ah)) / (3 * (hw + 1));   // core full, halo falling off linearly
        add565(fb[o], B.rgb[0] * f >> 8, B.rgb[1] * f >> 8, B.rgb[2] * f >> 8);
      }
    }
  }
}

// One strip of the model: background, triangles, edge lines, light effects. fr2: a second triangle list (the half of the
// parts the other core built), drawn into the same depth buffer - the order does not matter.
inline void draw_strip(const Frame& fr, int y0, int rows, uint16_t* fb, uint16_t* zb, const Fx* fx = nullptr, const Frame* fr2 = nullptr) {
  clear_strip(fb, zb, y0, rows);
  raster_strip(fr.tris, fr.n, y0, rows, fb, zb);
  if (fr2) raster_strip(fr2->tris, fr2->n, y0, rows, fb, zb);
  if (fr.ln) raster_lines(fr, y0, rows, fb, zb);
  if (fr2 && fr2->ln) raster_lines(*fr2, y0, rows, fb, zb);
  if (fx && (fx->ng || fx->nb)) raster_fx(*fx, y0, rows, fb, zb);
}

// ---------------------------------------------------------------- strip pipeline
struct Pipe {
  uint16_t* fb[2];
  uint16_t* zb;
  int stripH;
  int b = 0;   // persists across frames (odd strip count: never reuse a buffer the DMA may still read)
};

// overlay(fb, zb, y0, rows) may draw 2D on top of each strip - zb tells it where the model is (0 = background), so
// the HUD can stay off the arm; push(y0, fb, rows) waits for the previous DMA, starts the new one and returns
// immediately.
template <class OverlayFn, class PushFn>
inline void render_frame(const Frame& fr, Pipe& p, OverlayFn overlay, PushFn push, const Fx* fx = nullptr) {
  for (int y0 = 0; y0 < H; y0 += p.stripH) {
    uint16_t* fb = p.fb[p.b];
    draw_strip(fr, y0, p.stripH, fb, p.zb, fx);
    overlay(fb, p.zb, y0, p.stripH);
    push(y0, fb, p.stripH);
    p.b ^= 1;
  }
}

// The same picture on two cores: each strip's top rows are drawn here while split(job) has the other core draw its
// bottom rows (disjoint rows of the same colour and depth buffers), then the strip goes out. split(fn, ctx) runs
// fn(ctx) on the helper and returns at once; join() waits for it. The result is identical to render_frame.
struct HalfJob { const Frame* fr; const Frame* fr2; const Fx* fx; uint16_t* fb; uint16_t* zb; int y0, rows; void (*overlay)(uint16_t*, uint16_t*, int, int); };
inline void half_job(void* p) {
  HalfJob& j = *(HalfJob*)p;
  draw_strip(*j.fr, j.y0, j.rows, j.fb, j.zb, j.fx, j.fr2);
  j.overlay(j.fb, j.zb, j.y0, j.rows);
}
template <class PushFn, class SplitFn, class JoinFn>
inline void render_frame_split(const Frame& fr, const Frame* fr2, Pipe& p, void (*overlay)(uint16_t*, uint16_t*, int, int), PushFn push, const Fx* fx,
                               SplitFn split, JoinFn join) {
  static HalfJob job;
  const int top = p.stripH / 2;
  for (int y0 = 0; y0 < H; y0 += p.stripH) {
    uint16_t* fb = p.fb[p.b];
    job = {&fr, fr2, fx, fb + top * W, p.zb + top * W, y0 + top, p.stripH - top, overlay};
    split(half_job, &job);
    draw_strip(fr, y0, top, fb, p.zb, fx, fr2);
    overlay(fb, p.zb, y0, top);
    join();
    push(y0, fb, p.stripH);
    p.b ^= 1;
  }
}

}  // namespace rnd
