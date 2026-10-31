// view.h - camera, demo animation, live-packet mapping, scene assembly. Hardware-free (host-tested).
#pragma once
#include <math.h>
#include "config.h"
#include "kin.h"
#include "render.h"
#include "gauntlet_packet.h"

namespace app {

// ---------------------------------------------------------------- styles (config.h PUCK_STYLE)
// Each style = the five materials (indexed by model::Material: armour, shell, mechanism, glove, red parts;
// albedo linear, metalness, roughness, clear coat, 2^shine, reflection strength), the edge-line colours and the HUD.
//   0  round 2 (flashed 2026-10-07): light gunmetal, solid metallic-red plates 1-2 and fingertips, mode / fps only
//   1  A "red HUD": dark gunmetal, no solid red; red glowing edge lines; full red HUD
//   2  B "arc reactor": as A in cyan / ice blue; red only as an accent
//   3  C "candy metal": round 2's gunmetal, deep candy-red plates 1-2 and fingertips, minimal HUD
constexpr int NMAT = 5;
struct StyleDef {
  rnd::Material mats[NMAT];
  uint8_t crease[3], silhouette[3];   // edge-line core colours, added to the pixel; {0,0,0} = no edge lines
  uint8_t hudLevel;                   // 0 = mode / fps, 1 = + tick ring, 2 = full HUD
  uint8_t hud[3], accent[3];
  uint8_t halo;                       // edge-line glow beside the core: 0 none, 1 silhouettes only, 2 every line
};
#define DARK_ARMOR {0.0423f, 0.0467f, 0.0578f}   // 0x3a3d44
#define DARK_SHELL {0.0296f, 0.0319f, 0.0395f}   // 0x303238
#define DARK_GUN {0.0137f, 0.0152f, 0.0185f}     // 0x1f2125
static const StyleDef kStyles[4] = {
    {{{MAT_ARMOR_RGB, 0.80f, 0.40f, 0.30f, 5},     // round 2
      {MAT_SHELL_RGB, 0.80f, 0.42f, 0.30f, 5},
      {MAT_GUN_RGB, 0.85f, 0.42f, 0.00f, 6},
      {MAT_UNDER_RGB, 0.55f, 0.72f, 0.00f, 3},
      {MAT_RED_RGB, 0.70f, 0.35f, 0.00f, 6}},      // HandBench matRedM 0xb5141e
     {0, 0, 0}, {0, 0, 0}, 0, {235, 235, 240}, {255, 40, 50}, 0},
    {{{DARK_ARMOR, 0.80f, 0.40f, 0.30f, 5},        // A red HUD
      {DARK_SHELL, 0.80f, 0.42f, 0.30f, 5},
      {DARK_GUN, 0.85f, 0.42f, 0.00f, 6},
      {MAT_UNDER_RGB, 0.55f, 0.72f, 0.00f, 3},
      {DARK_ARMOR, 0.80f, 0.40f, 0.30f, 5}},       // no solid red parts
     {120, 14, 20}, {255, 45, 50}, 2, {255, 45, 50}, {255, 215, 200}, 1},
    {{{DARK_ARMOR, 0.80f, 0.40f, 0.30f, 5},        // B arc reactor
      {DARK_SHELL, 0.80f, 0.42f, 0.30f, 5},
      {DARK_GUN, 0.85f, 0.42f, 0.00f, 6},
      {MAT_UNDER_RGB, 0.55f, 0.72f, 0.00f, 3},
      {DARK_ARMOR, 0.80f, 0.40f, 0.30f, 5}},
     {24, 96, 122}, {90, 215, 255}, 2, {90, 215, 255}, {255, 40, 50}, 1},
    {{{MAT_ARMOR_RGB, 0.80f, 0.40f, 0.30f, 5},     // C candy metal
      {MAT_SHELL_RGB, 0.80f, 0.42f, 0.30f, 5},
      {MAT_GUN_RGB, 0.85f, 0.42f, 0.00f, 6},
      {MAT_UNDER_RGB, 0.55f, 0.72f, 0.00f, 3},
      {{0.150f, 0.002f, 0.004f}, 0.95f, 0.18f, 0.55f, 8, 3.2f}},   // 0x6c070d deep candy red: strong red reflection, a clear coat for white glints
     {0, 0, 0}, {0, 0, 0}, 1, {150, 150, 160}, {255, 40, 50}, 0},
};
static const StyleDef& kStyle = kStyles[PUCK_STYLE];
inline bool style_has_lines() { return kStyle.silhouette[0] | kStyle.silhouette[1] | kStyle.silhouette[2]; }
// how the edge lines are drawn (2026-10-11, the console's "lines N" - to weigh their cost on the puck): 0 the style's
// (a glow beside every line), 1 the glow on outlines only, 2 no glow, 3 outlines only and no glow, 4 no lines,
// 5 as 0 but no lines on the fingers
inline uint8_t& line_mode() { static uint8_t m = 0; return m; }
// Where the inner creases go (Toby, 2026-10-11: "red edges everywhere including inner creases, except the fingers get
// outlines only"; if 15 fps is still missed, outlines only further up the arm, only as far as needed). Level: 0 creases
// everywhere, 1 the fingers outlines only, 2 + the hand plate, 3 + the rest of the hand, 4 + the forearm plates, 5 every
// part outlines only. The console's "creases N" sets it for a measurement; CREASE_LEVEL (config.h) is the default.
inline bool body_on_hand(int b);   // below
inline uint8_t& crease_level() { static uint8_t l = CREASE_LEVEL; return l; }
inline bool body_outlines_only(int body, int level) {
  if (level <= 0) return false;
  if (level >= 5) return true;
  const char* n = model::kBodyNames[body];
  size_t len = strlen(n);
  const bool finger = len > 3 && n[len - 3] == '_' && n[len - 2] == 'k';   // index_k1 .. thumb_k2
  if (finger) return true;
  if (level >= 2 && !strcmp(n, "hand_plate_coupler")) return true;
  if (level >= 3 && body_on_hand(body)) return true;
  if (level >= 4 && model::kBodies[body].kind == model::FOURBAR && model::kBodies[body].ref % 3 == 0) return true;
  return false;
}

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float smooth01(float x) { x = clampf(x, 0, 1); return x * x * (3 - 2 * x); }

// ---------------------------------------------------------------- camera (Toby, 2026-10-07: "I control it")
// Zoomed in, the hand is centred and fills most of the glass (the pivot is the hand's centre wherever the arm put it).
// Zoomed out, the pivot is the middle of the arm (halfway from the hand to the elbow end) and the whole arm fills most
// of the glass from whatever angle it is seen: the distance is fitted to the arm's outline in this view (frame_full).
// The orientation is Toby's alone: one-finger drag = a free trackball orbit, it stays where it is left. Zoom changes
// never touch the orientation: double tap toggles the zoom; hold = home (the hand, looking down at it from above);
// and the one automatic move - a plate opening zooms out (to show the plates), all plates closed again goes back to
// the zoom there was before. Nothing else: not events, not the link.
// Base orientation = looking down on the back of the hand: screen right = l, screen up = a (fingers up), toward the
// viewer = d (dorsal). R = the trackball rotation from there, in the camera's own axes.
struct CamState {
  float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float zoom = 0;               // 0 = hand view, 1 = the whole arm (eased toward zoomTarget)
  float zoomTarget = 0;
  bool platesOpen = false;      // any plate open (with hysteresis) - for the status lines only
  // the view follows the action (round 10): 1 = the hand (the hand plate lifting, the red laser), 2 = the forearm (its
  // plates, the green laser, a cascade); when it ends the view glides back to exactly the one before it
  uint8_t act = 0;
  float actIdle = 0;            // s since the action's last sign
  float savedR[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}, savedZoom = 0;   // the view before the action
  bool returning = false;       // gliding back to it (a drag stops the glide: the finger wins)
  bool hold = false;            // the fps sweep owns the camera: no action moves it
  float fullDist = 0;           // the whole-arm view's distance for this view (eased; 0 = not yet fitted)
  float fullOff[3] = {0, 0, 0}; // ... and the shift from the arm's middle to the centre of its outline on the screen
  float userYaw = 0, userPitch = 0;   // PC tools: an extra turn after R (the puck leaves them 0)
};

inline float half_fov_sin() { float h = atanf(120.0f / CAM_FOCAL_PX); return sinf(h); }

// how far the whole arm reaches from the hand's centre (rest pose, every part's bounding sphere), built once
inline float arm_reach() {
  static float r = 0;
  if (r == 0) {
    const model::MeshSet& ms = model::kMesh[0];
    for (int p = 0; p < ms.npart; p++) {
      const float* s = ms.sphere[p];
      float dx = s[0] - model::kHandCenter[0], dy = s[1] - model::kHandCenter[1], dz = s[2] - model::kHandCenter[2];
      r = fmaxf(r, sqrtf(dx * dx + dy * dy + dz * dz) + s[3]);
    }
  }
  return r;
}

// The arm's outline for framing: every part's extreme vertices in 64 directions spread evenly round the sphere, of the
// light model, with the body each moves with (built once) - the parts move rigidly, so these stay
// their extremes however the arm is posed; and the elbow end's centre (the forearm's vertices within 40 mm of its far end).
// Built once at boot, BEFORE the frame buffers (app.cpp calls hull() first), at its exact size on the heap, and only as
// indices into the light model's vertex table (in flash): RAM is what the shared triangle buffer gets - the first cut,
// a fixed 3600-point copy, cost 2,100 triangles and then would not boot.
constexpr int HULL_N = 1024;   // the most points it may have (this model: 739; the host test fails if it fills)
static_assert(model::NVERT_LO < 65536, "the hull keeps vertex indices in 16 bits");
struct Hull {
  uint16_t* vi = nullptr; uint8_t* body = nullptr; int n = 0; float elbow[3];
  const float* p(int i) const { return model::kMesh[1].verts[vi[i]]; }
};
inline const Hull& hull() {
  static Hull h;
  static bool built = false;
  if (built) return h;
  built = true;
  const model::MeshSet& ms = model::kMesh[1];
  constexpr int ND = 64;
  float dirs[ND][3];
  for (int i = 0; i < ND; i++) {   // a Fibonacci sphere
    float y = 1 - 2 * (i + 0.5f) / ND, r = sqrtf(1 - y * y), ph = i * 2.39996323f;
    dirs[i][0] = r * cosf(ph); dirs[i][1] = y; dirs[i][2] = r * sinf(ph);
  }
  // two passes over the same rule: count, then fill the exact-size arrays (a part's extremes are de-duplicated)
  for (int pass = 0; pass < 2; pass++) {
    int n = 0;
    for (int k = 0; k < ms.npart; k++) {
      const model::PartDef& pd = ms.parts[k];
      if (pd.flags & 2) continue;   // the dark backs, the lids and caps sit inside the shells
      int mine[ND]; int nm = 0;     // this part's chosen vertices so far
      for (int d = 0; d < ND; d++) {
        int best = -1; float bv = -1e9f;
        for (int i = 0; i < pd.nv; i++) {
          const float* v = ms.verts[pd.v0 + i];
          float t = v[0] * dirs[d][0] + v[1] * dirs[d][1] + v[2] * dirs[d][2];
          if (t > bv) { bv = t; best = i; }
        }
        if (best < 0) continue;
        bool dup = false;
        for (int j = 0; j < nm; j++) if (!memcmp(ms.verts[pd.v0 + mine[j]], ms.verts[pd.v0 + best], sizeof(float) * 3)) dup = true;
        if (dup || n >= HULL_N) continue;
        mine[nm++] = best;
        if (pass == 1) { h.vi[n] = (uint16_t)(pd.v0 + best); h.body[n] = pd.body; }
        n++;
      }
    }
    if (pass == 0) {
      h.vi = (uint16_t*)malloc(sizeof(uint16_t) * (n ? n : 1));
      h.body = (uint8_t*)malloc(n ? n : 1);
      if (!h.vi || !h.body) { h.n = 0; break; }
    } else h.n = n;
  }
  float aMin = 1e9f;
  for (int k = 0; k < ms.npart; k++) {
    const model::PartDef& pd = ms.parts[k];
    if (pd.body != model::BODY_ARM_ROOT) continue;
    for (int i = 0; i < pd.nv; i++) {
      const float* v = ms.verts[pd.v0 + i];
      float a = v[0] * model::kFrameA[0] + v[1] * model::kFrameA[1] + v[2] * model::kFrameA[2];
      if (a < aMin) aMin = a;
    }
  }
  float c[3] = {0, 0, 0}; int cn = 0;
  for (int k = 0; k < ms.npart; k++) {
    const model::PartDef& pd = ms.parts[k];
    if (pd.body != model::BODY_ARM_ROOT) continue;
    for (int i = 0; i < pd.nv; i++) {
      const float* v = ms.verts[pd.v0 + i];
      float a = v[0] * model::kFrameA[0] + v[1] * model::kFrameA[1] + v[2] * model::kFrameA[2];
      if (a < aMin + 40) { for (int j = 0; j < 3; j++) c[j] += v[j]; cn++; }
    }
  }
  for (int j = 0; j < 3; j++) h.elbow[j] = cn ? c[j] / cn : model::kFullCenter[j];
  return h;
}

inline void xform(const kin::M34& M, const float v[3], float o[3]) {
  const float* m = M.m;
  for (int i = 0; i < 3; i++) o[i] = m[i * 4] * v[0] + m[i * 4 + 1] * v[1] + m[i * 4 + 2] * v[2] + m[i * 4 + 3];
}
inline void hand_pivot(const kin::M34 world[], float p[3]) { xform(world[model::BODY_WRIST], model::kHandCenter, p); }
// the middle of the arm: halfway from the hand's centre to the elbow end's centre, as the arm is posed now
inline void arm_mid(const kin::M34 world[], float p[3]) {
  float hnd[3], elb[3];
  hand_pivot(world, hnd);
  xform(world[model::BODY_ARM_ROOT], hull().elbow, elb);
  for (int i = 0; i < 3; i++) p[i] = 0.5f * (hnd[i] + elb[i]);
}

// the camera's axes (columns of Rc: x, y, z in world) for this orientation
inline void cam_axes(const CamState& c, float Rc[9]) {
  float B[9] = {model::kFrameL[0], model::kFrameA[0], model::kFrameD[0],
                model::kFrameL[1], model::kFrameA[1], model::kFrameD[1],
                model::kFrameL[2], model::kFrameA[2], model::kFrameD[2]};
  float cy = cosf(c.userYaw), sy = sinf(c.userYaw), cp = cosf(c.userPitch), sp = sinf(c.userPitch);
  float Ry[9] = {cy, 0, sy, 0, 1, 0, -sy, 0, cy}, Rx[9] = {1, 0, 0, 0, cp, -sp, 0, sp, cp};
  float T[9], U[9];
  kin::mat3_mul(B, c.R, T);
  kin::mat3_mul(T, Ry, U);
  kin::mat3_mul(U, Rx, Rc);
}

// The whole-arm view: the distance that keeps every outline point within FULL_FIT of the glass's radius, and the shift
// (off, world) that centres the arm's outline on the screen - balanced top to bottom and side to side from any angle,
// not just round its middle. Two passes: the box in the view's plane, then once more in perspective at that distance.
inline float frame_full(const float Rc[9], const float mid[3], const kin::M34 world[], float off[3]) {
  const Hull& h = hull();
  const float fill = FULL_FIT * 120.0f;
  auto pt = [&](int i, float& x, float& y, float& z) {   // point i in the view's frame, from the arm's middle
    float q[3]; xform(world[h.body[i]], h.p(i), q);
    float r[3] = {q[0] - mid[0], q[1] - mid[1], q[2] - mid[2]};
    x = r[0] * Rc[0] + r[1] * Rc[3] + r[2] * Rc[6];
    y = r[0] * Rc[1] + r[1] * Rc[4] + r[2] * Rc[7];
    z = r[0] * Rc[2] + r[1] * Rc[5] + r[2] * Rc[8];   // toward the camera
  };
  float cx = 0, cy = 0, d = 0;
  for (int pass = 0; pass < 3; pass++) {
    // the distance for this centre
    d = 0;
    for (int i = 0; i < h.n; i++) {
      float x, y, z; pt(i, x, y, z);
      float dx = x - cx, dy = y - cy;
      float need = CAM_FOCAL_PX * sqrtf(dx * dx + dy * dy) / fill + z;   // the glass is round: the radial distance
      if (need > d) d = need;
    }
    if (pass == 2) break;
    // the outline's box on the screen at that distance -> the centre that balances it (back in the view's plane)
    float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
    for (int i = 0; i < h.n; i++) {
      float x, y, z; pt(i, x, y, z);
      float k = CAM_FOCAL_PX / fmaxf(d - z, 1.0f);
      float sx = (x - cx) * k, sy = (y - cy) * k;
      x0 = fminf(x0, sx); x1 = fmaxf(x1, sx); y0 = fminf(y0, sy); y1 = fmaxf(y1, sy);
    }
    cx += 0.5f * (x0 + x1) * d / CAM_FOCAL_PX; cy += 0.5f * (y0 + y1) * d / CAM_FOCAL_PX;
  }
  for (int i = 0; i < 3; i++) off[i] = cx * Rc[i * 3] + cy * Rc[i * 3 + 1];
  return d;
}

inline void mat_to_quat(const float R[9], float q[4]) {   // rotation matrix (row-major) -> unit quaternion w x y z
  q[0] = 0.5f * sqrtf(fmaxf(0.0f, 1 + R[0] + R[4] + R[8]));
  q[1] = copysignf(0.5f * sqrtf(fmaxf(0.0f, 1 + R[0] - R[4] - R[8])), R[7] - R[5]);
  q[2] = copysignf(0.5f * sqrtf(fmaxf(0.0f, 1 - R[0] + R[4] - R[8])), R[2] - R[6]);
  q[3] = copysignf(0.5f * sqrtf(fmaxf(0.0f, 1 - R[0] - R[4] + R[8])), R[3] - R[1]);
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  for (int i = 0; i < 4; i++) q[i] /= n;
}

// The camera, once per frame. The view follows the action (Toby, round 10):
//   the hand plate lifting or the red laser on        -> the hand view (zoom in)
//   a forearm plate opening (plate 1 for the green laser, a cascade) or the green laser on -> the whole arm (zoom out)
// (the forearm wins when both). A cascade counts as the forearm from its first moment (the arm's $G outs bit 2): the arm
// lifts the hand plate first and plate 2 only once it has landed (~350 ms), which must not zoom in and back out.
// A drag still orbits during the action, a double tap still toggles the zoom; when the
// action has been over for VIEW_ACT_END_S the view glides back to exactly the one it had before the action started -
// what the finger did meanwhile is forgotten. A new action during the glide keeps the same "before" view.
constexpr float VIEW_ACT_OPEN = 0.05f, VIEW_ACT_CLOSED = 0.02f;   // a plate this far open is moving / open
constexpr float VIEW_ACT_END_S = 0.40f, VIEW_RETURN_TAU_S = 0.22f;
inline void update_camera(CamState& c, const kin::Inputs& in, float dt, bool red = false, bool green = false, bool cascade = false) {
  float fore = 0, mx = 0;
  for (int j = 0; j < model::NFOURBAR; j++) {   // four-bars: 0 plate_1, 1 plate_3, 2 plate_2, 3 hand_plate
    float f = kin::plate_frac(in, j);
    mx = fmaxf(mx, f);
    if (j != 3) fore = fmaxf(fore, f);
  }
  const float hand = kin::plate_frac(in, 3);
  if (!c.platesOpen && mx > PLATES_OPEN_FRAC) c.platesOpen = true;
  if (c.platesOpen && mx < PLATES_CLOSED_FRAC) c.platesOpen = false;
  if (!c.hold) {
    bool armAct = green || cascade || fore > (c.act == 2 ? VIEW_ACT_CLOSED : VIEW_ACT_OPEN);
    bool handAct = red || hand > (c.act ? VIEW_ACT_CLOSED : VIEW_ACT_OPEN);
    uint8_t want = armAct ? 2 : (handAct ? 1 : 0);
    if (want) {
      if (!c.act && !c.returning) { memcpy(c.savedR, c.R, sizeof c.savedR); c.savedZoom = c.zoomTarget; }
      c.returning = false;
      if (want != c.act) c.zoomTarget = want == 2 ? 1.0f : 0.0f;
      c.act = want; c.actIdle = 0;
    } else if (c.act) {
      c.actIdle += dt;
      if (c.actIdle >= VIEW_ACT_END_S) { c.act = 0; c.zoomTarget = c.savedZoom; c.returning = true; }
    }
    if (c.returning) {   // the orientation glides back (the shortest way), then lands exactly on the saved one
      float q[4], t[4];
      mat_to_quat(c.R, q); mat_to_quat(c.savedR, t);
      float d = q[0] * t[0] + q[1] * t[1] + q[2] * t[2] + q[3] * t[3];
      if (fabsf(d) > 0.99999f) { memcpy(c.R, c.savedR, sizeof c.R); c.returning = false; }
      else {
        float k = 1 - expf(-dt / VIEW_RETURN_TAU_S), sg = d < 0 ? -1.0f : 1.0f;
        for (int i = 0; i < 4; i++) q[i] += (sg * t[i] - q[i]) * k;
        float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int i = 0; i < 4; i++) q[i] /= n;
        kin::quat_to_mat(q, c.R);
      }
    }
  }
  float k = 1 - expf(-dt / ZOOM_TIME_S);
  c.zoom += (c.zoomTarget - c.zoom) * k;
  if (fabsf(c.zoomTarget - c.zoom) < 1e-3f) c.zoom = c.zoomTarget;
}
inline void cam_toggle_zoom(CamState& c) { c.zoomTarget = c.zoomTarget > 0.5f ? 0.0f : 1.0f; }   // zoom only
inline void cam_home(CamState& c) {   // the hand view, looking down at the hand from above
  const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  memcpy(c.R, I, sizeof I); c.zoomTarget = 0; c.userYaw = c.userPitch = 0; c.returning = false;
}

inline void orthonormalize(float R[9]) {   // columns: Gram-Schmidt, so many small drags never skew the camera
  float x[3] = {R[0], R[3], R[6]}, y[3] = {R[1], R[4], R[7]};
  float n = sqrtf(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]); for (int i = 0; i < 3; i++) x[i] /= n;
  float d = x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; for (int i = 0; i < 3; i++) y[i] -= d * x[i];
  n = sqrtf(y[0] * y[0] + y[1] * y[1] + y[2] * y[2]); for (int i = 0; i < 3; i++) y[i] /= n;
  float z[3] = {x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]};
  for (int i = 0; i < 3; i++) { R[i * 3] = x[i]; R[i * 3 + 1] = y[i]; R[i * 3 + 2] = z[i]; }
}

// one finger's move: a turn about the camera's own up axis (left / right) and right axis (up / down), so it orbits
// freely in every direction - over the top, underneath, all the way round
inline void orbit(CamState& c, float dxPx, float dyPx) {
  c.returning = false;   // the finger wins over the glide back
  float a = -dxPx * ORBIT_RAD_PER_PX, b = -dyPx * ORBIT_RAD_PER_PX;
  float ca = cosf(a), sa = sinf(a), cb = cosf(b), sb = sinf(b);
  float Ry[9] = {ca, 0, sa, 0, 1, 0, -sa, 0, ca}, Rx[9] = {1, 0, 0, 0, cb, -sb, 0, sb, cb}, T[9], U[9];
  kin::mat3_mul(c.R, Ry, T);
  kin::mat3_mul(T, Rx, U);
  memcpy(c.R, U, sizeof U);
  orthonormalize(c.R);
}

// handPivot: the hand's centre (zoomed in); midPivot: the middle of the arm (zoomed out) - null = the rest pose's
inline rnd::Camera make_camera(const CamState& c, const float handPivot[3] = model::kHandCenter, const float* midPivot = nullptr) {
  float s = smooth01(c.zoom), sf = half_fov_sin();
  float dHand = CAM_FOCAL_PX * model::kHandRadius / HAND_FILL_PX + model::kHandFront;
  float reach = arm_reach();
  float dFull = c.fullDist > 0 ? c.fullDist : reach / (sf * FULL_FIT);
  float dist = dHand + (dFull - dHand) * s;
  float mid[3];
  if (midPivot) memcpy(mid, midPivot, sizeof mid);
  else for (int i = 0; i < 3; i++) mid[i] = 0.5f * (model::kHandCenter[i] + hull().elbow[i]);
  float pivot[3];
  for (int i = 0; i < 3; i++) pivot[i] = handPivot[i] + (mid[i] - handPivot[i]) * s;
  float Rc[9]; cam_axes(c, Rc);
  float eye[3];
  for (int i = 0; i < 3; i++) eye[i] = pivot[i] + Rc[i * 3 + 2] * dist;
  rnd::Camera cam;
  for (int r = 0; r < 3; r++) {
    float ax = Rc[0 * 3 + r], ay = Rc[1 * 3 + r], az = Rc[2 * 3 + r];
    cam.view.m[r * 4 + 0] = ax; cam.view.m[r * 4 + 1] = ay; cam.view.m[r * 4 + 2] = az;
    cam.view.m[r * 4 + 3] = -(ax * eye[0] + ay * eye[1] + az * eye[2]);
  }
  cam.f = CAM_FOCAL_PX;
  float span = reach * 1.15f;
  cam.nearD = fmaxf(20.0f, dist - span);
  cam.farD = dist + span;
  return cam;
}

// ---------------------------------------------------------------- scene
inline bool body_on_hand(int b) {
  for (; b >= 0; b = model::kBodies[b].parent)
    if (b == model::BODY_WRIST) return true;
  return false;
}

// hand-side parts first: if the triangle cap is ever hit, the forearm loses triangles, not the hand
// lod: 0 = the detailed set (close-up hand view), 1 = the light one (zoomed out, plates open; model::kMesh)
// the precomputed-visibility skip (render.h vis_bin); PC tools turn it off to check it hides nothing that shows
inline bool& vis_skip_enabled() { static bool on = true; return on; }

// platesShut: every plate closed (the back faces seen only through an open plate are skipped; false = draw them)
// part0 / step: this call builds every step-th part from part0 (the two cores each build half: 0 / 2 and 1 / 2)
// The scene's tone (round 10), applied per vertex after the lighting (no extra RAM, no extra triangles):
//   grey       the arm's link is lost - the model turns grey, its edge lines too (Toby: "no text")
//   bodyTone   the body's colour, the edge lines kept red (PUCK_BODY_TONE, Toby to pick): 0 the style's gunmetal,
//              1 the link-lost grey (looks the same: the gunmetal is nearly colourless), 2 a lighter grey
//   plateGlow  0..1 per plate (four-bar joint: 0 plate 1, 1 plate 3, 2 plate 2, 3 the hand plate), red while it moves
struct SceneTone { bool grey = false; uint8_t bodyTone = 0; float plateGlow[model::NFOURBAR] = {}; };
// the four-bar joint whose plate a body is (its coupler), else -1
inline int plate_joint_of(int body) {
  const model::BodyDef& bd = model::kBodies[body];
  return bd.kind == model::FOURBAR && bd.ref % 3 == 0 ? bd.ref / 3 : -1;
}
inline void draw_scene(rnd::Frame& fr, const rnd::Camera& cam, const kin::M34 world[model::NBODY], int lod = 0,
                       bool platesShut = false, int part0 = 0, int step = 1, const SceneTone& tone = SceneTone()) {
  static rnd::MatCap matcaps[NMAT];                   // the lighting per material, built on first use (~5 k shade calls)
  static bool ready = (rnd::tone_init(), [] { for (int i = 0; i < NMAT; i++) rnd::matcap_build(matcaps[i], kStyle.mats[i]); }(), true);
  (void)ready;
  rnd::begin(fr);
  memcpy(fr.lineRGB[0], kStyle.crease, 3);
  memcpy(fr.lineRGB[1], kStyle.silhouette, 3);
  if (tone.grey) {   // the edge lines grey too
    for (int k = 0; k < 2; k++) { int l = (fr.lineRGB[k][0] * 77 + fr.lineRGB[k][1] * 150 + fr.lineRGB[k][2] * 29) >> 8; fr.lineRGB[k][0] = fr.lineRGB[k][1] = fr.lineRGB[k][2] = (uint8_t)l; }
  }
  const uint8_t kGrey[4] = {1, 0, 0, 0}, kLight[4] = {3, 0, 0, 0};
  const uint8_t* bodyTint = tone.bodyTone == 1 ? kGrey : (tone.bodyTone == 2 ? kLight : nullptr);
  uint8_t kGlow[model::NFOURBAR][4];
  bool glowing[model::NFOURBAR];
  for (int j = 0; j < model::NFOURBAR; j++) {
    const float pg = tone.plateGlow[j] < 0 ? 0 : (tone.plateGlow[j] > 1 ? 1 : tone.plateGlow[j]);
    kGlow[j][0] = 2; kGlow[j][1] = (uint8_t)(150 * pg); kGlow[j][2] = (uint8_t)(14 * pg); kGlow[j][3] = (uint8_t)(20 * pg);
    glowing[j] = pg > 0.02f;
  }
  const uint8_t lm = line_mode();
  fr.lineHalo = lm == 0 ? kStyle.halo : (lm == 1 ? (uint8_t)(kStyle.halo ? 1 : 0) : 0);
  fr.lineOutlinesOnly = lm == 3;
  const bool lines = style_has_lines() && fr.lines && lm != 4;
  const model::MeshSet& ms = model::kMesh[lod ? 1 : 0];
  const float camZ[3] = {cam.view.m[8], cam.view.m[9], cam.view.m[10]};   // toward the eye, world frame
  // the bins were recorded with the arm in its rest frame: the direction is taken in the forearm's own frame for the
  // forearm's parts and in the hand's for the hand's, so a turned arm or a bent wrist never hides a triangle that shows
  auto local_bin = [&](int body) {
    const float* m = world[body].m;
    float d[3] = {m[0] * camZ[0] + m[4] * camZ[1] + m[8] * camZ[2], m[1] * camZ[0] + m[5] * camZ[1] + m[9] * camZ[2],
                  m[2] * camZ[0] + m[6] * camZ[1] + m[10] * camZ[2]};
    return 1ull << (rnd::vis_bin(d) + (platesShut ? 0 : 32));
  };
  const uint64_t maskArm = local_bin(model::BODY_ARM_ROOT), maskHand = local_bin(model::BODY_WRIST);
  // per part, the union of its triangles' bins (built once per detail level): a part none of whose triangles shows
  // from this direction is skipped whole, vertex transforms included
  // each part's facing memory for the edge lines (render.h FACE_HYST): a bit per triangle, each part from a byte of
  // its own (the two cores build different parts at once), per detail level - in .bss, ~1.5 KB
  static uint8_t faceMem[2][(model::NTRI > model::NTRI_LO ? model::NTRI : model::NTRI_LO) / 8 + (model::NPART > model::NPART_LO ? model::NPART : model::NPART_LO) + 1];
  static uint16_t faceOff[2][model::NPART > model::NPART_LO ? model::NPART : model::NPART_LO];
  static bool faceReady[2] = {false, false};
  if (!faceReady[lod ? 1 : 0]) {
    int o = 0;
    for (int p = 0; p < ms.npart; p++) { faceOff[lod ? 1 : 0][p] = (uint16_t)o; o += (ms.parts[p].nt + 7) / 8; }
    faceReady[lod ? 1 : 0] = true;
  }
  static uint64_t partVisAll[2][model::NPART > model::NPART_LO ? model::NPART : model::NPART_LO];
  static bool partVisReady[2] = {false, false};
  uint64_t* partVis = partVisAll[lod ? 1 : 0];
  if (!partVisReady[lod ? 1 : 0]) {
    for (int p = 0; p < ms.npart; p++) {
      uint64_t m = 0;
      for (int t = 0; t < ms.parts[p].nt; t++) m |= ms.visDirs[ms.parts[p].t0 + t];
      partVis[p] = m;
    }
    partVisReady[lod ? 1 : 0] = true;
  }
  for (int pass = 0; pass < 2; pass++)
    for (int p = part0; p < ms.npart; p += step) {
      const model::PartDef& pd = ms.parts[p];
      if (body_on_hand(pd.body) != (pass == 0)) continue;
      fr.lineOutlinesOnly = lm == 3 || body_outlines_only(pd.body, crease_level());
      const uint64_t visMask = pass == 0 ? maskHand : maskArm;
      if (vis_skip_enabled() && !(partVis[p] & visMask)) { rnd::prof().partsSkipped++; continue; }   // nothing of it shows from here
      kin::M34 M = kin::mul(cam.view, world[pd.body]);
      if (rnd::sphere_outside(cam, M, ms.sphere[p], ms.sphere[p][3])) { rnd::prof().partsSkipped++; continue; }
      int ne = lines && !(lm == 5 && body_outlines_only(pd.body, 1)) ? ms.partEdges[p][1] : 0;
      const int pj = plate_joint_of(pd.body);
      const uint8_t* tint = tone.grey ? kGrey : (pj >= 0 && glowing[pj] ? kGlow[pj] : bodyTint);
      rnd::add_part(fr, cam, M, &ms.verts[pd.v0], &ms.norms[pd.v0], pd.nv, &ms.tris[pd.t0], pd.nt, matcaps[pd.material],
                    ne ? &ms.edges[ms.partEdges[p][0]] : nullptr, ne, (pd.flags & 2) ? rnd::kBackRGB : nullptr,
                    vis_skip_enabled() ? ms.visDirs : nullptr, pd.t0, visMask, (pd.flags & 4) != 0, tint,
                    ne ? &faceMem[lod ? 1 : 0][faceOff[lod ? 1 : 0][p]] : nullptr);
    }
}

// which detail level the camera wants: the light set once the view has zoomed most of the way out
inline int lod_for(const CamState& c) { return c.zoom > LOD_ZOOM ? 1 : 0; }

// ---------------------------------------------------------------- demo
// The puck's own tilt (gravity, device frame) -> hand rotation. The hand copies the puck: tip the top edge
// of the puck away from you and the fingers tip away too. Device x = screen right, y = screen up, z = out.
inline void tilt_to_wrist(const float gNow[3], const float gRef[3], float R[9]) {
  float u[3] = {gNow[0], gNow[1], gNow[2]}, v[3] = {gRef[0], gRef[1], gRef[2]};
  float nu = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]), nv = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  if (nu < 1e-3f || nv < 1e-3f) { memcpy(R, I, sizeof I); return; }
  for (int i = 0; i < 3; i++) { u[i] /= nu; v[i] /= nv; }
  float ax[3] = {u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]};
  float sn = sqrtf(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]), cs = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  if (sn < 1e-5f) { memcpy(R, I, sizeof I); return; }
  for (int i = 0; i < 3; i++) ax[i] /= sn;
  float Rd[9];
  float deg = atan2f(sn, cs) * 57.29578f;
  if (deg > DEMO_TILT_MAX_DEG) deg = DEMO_TILT_MAX_DEG;   // never folds the hand over, however the puck is held
  kin::rot_axis(ax, deg, Rd);   // device frame: rotation taking gNow to gRef
  // C: device axes -> model axes of the default view (x -> l, y -> a, z -> d)
  float C[9] = {model::kFrameL[0], model::kFrameA[0], model::kFrameD[0],
                model::kFrameL[1], model::kFrameA[1], model::kFrameD[1],
                model::kFrameL[2], model::kFrameA[2], model::kFrameD[2]};
  float Ct[9], T[9];
  kin::mat3_transpose(C, Ct);
  kin::mat3_mul(C, Rd, T);
  kin::mat3_mul(T, Ct, R);
}

// t = seconds since demo start. Starts closed; plates open ~6 s in, then every DEMO_PLATE_PERIOD_S.
inline void demo_inputs(float t, kin::Inputs& in) {
  const float P = DEMO_PLATE_PERIOD_S;
  float tc = fmodf(t + P + 4.0f, 2 * P);
  bool opening = tc < P;
  float tt = opening ? tc : tc - P;
  for (int i = 0; i < model::NFOURBAR; i++) {
    int j = model::kOpenOrder[i];
    int stage = opening ? i : (model::NFOURBAR - 1 - i);  // close in reverse order
    float f = smooth01((tt - 0.7f * stage) / 1.2f);
    if (!opening) f = 1 - f;
    const model::FourbarDef& d = model::kFourbars[j];
    in.cmd[j] = d.cmd_closed + f * (d.cmd_open - d.cmd_closed);
  }
  for (int f = 0; f < 5; f++)
    in.flex[f] = 0.85f * (0.5f - 0.5f * cosf(6.2831853f * t / 4.0f - 0.55f * f));
}

// gentle wobble when the IMU is missing
inline void wobble_wrist(float t, float R[9]) {
  float g[3] = {0.35f * sinf(0.7f * t), 0.30f * sinf(0.5f * t + 1.0f), 1.0f};
  const float up[3] = {0, 0, 1};
  tilt_to_wrist(g, up, R);
}

// ---------------------------------------------------------------- live
// q_model = S * R(conj(q_zero) * q_rel) * S^T,  S = F * remap  (web/src/kinematics.js ImuJoint)
inline void live_wrist(const float qRel[4], const float qZero[4], float R[9]) {
  float zw = qZero[0], zx = -qZero[1], zy = -qZero[2], zz = -qZero[3];  // conj(q_zero)
  float w = qRel[0], x = qRel[1], y = qRel[2], z = qRel[3];
  float q[4] = {zw * w - zx * x - zy * y - zz * z, zw * x + zx * w + zy * z - zz * y,
                zw * y - zx * z + zy * w + zz * x, zw * z + zx * y - zy * x + zz * w};
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-6f) { const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; memcpy(R, I, sizeof I); return; }
  for (int i = 0; i < 4; i++) q[i] /= n;
  float Rq[9];
  kin::quat_to_mat(q, Rq);
  static const float remapCols[3][3] = IMU_REMAP;  // columns: sensor X, Y, Z in (a, d, l)
  float F[9] = {model::kFrameA[0], model::kFrameD[0], model::kFrameL[0],
                model::kFrameA[1], model::kFrameD[1], model::kFrameL[1],
                model::kFrameA[2], model::kFrameD[2], model::kFrameL[2]};
  float Rm[9];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) Rm[i * 3 + j] = remapCols[j][i];
  float S[9], St[9], T[9];
  kin::mat3_mul(F, Rm, S);
  kin::mat3_transpose(S, St);
  kin::mat3_mul(S, Rq, T);
  kin::mat3_mul(T, St, R);
}

inline void packet_quat(const gauntlet_packet_t& p, float q[4]) {
  for (int i = 0; i < 4; i++) q[i] = gauntlet_q15_to_float(p.q[i]);
}

// Apply one packet. Fields whose valid flag is clear keep their previous value.
inline void live_inputs(const gauntlet_packet_t& p, const float qZero[4], kin::Inputs& in) {
  if ((p.flags & GAUNTLET_FLAG_HAND_IMU) && (p.flags & GAUNTLET_FLAG_BODY_IMU)) {
    float q[4];
    packet_quat(p, q);
    live_wrist(q, qZero, in.wrist);
  }
  if (p.flags & GAUNTLET_FLAG_FLEX)
    for (int f = 0; f < 5; f++) {
      float lo = model::kFlexRaw[f][0], hi = model::kFlexRaw[f][1];
      in.flex[f] = clampf((p.flex[f] - lo) / (hi - lo), 0, 1);
    }
  for (int j = 0; j < model::NFOURBAR; j++) {
    uint8_t v = p.servo[model::kFourbars[j].channel];
    in.cmd[j] = (v == GAUNTLET_SERVO_ABSENT) ? model::kFourbars[j].cmd_closed : (float)v;
  }
}

}  // namespace app
