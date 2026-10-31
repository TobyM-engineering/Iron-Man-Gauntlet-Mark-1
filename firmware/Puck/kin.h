// kin.h - puck kinematics: servo commands + flex + wrist rotation -> world matrix of every body.
// Hardware-free (also built by puck/tools/puck_host_test.cpp against assembly/test_vectors.json).
//
// Four-bar plates: baked tables from tools/pose_r3.py, one entry per integer command 0..180 (exact for
// packet input; fractional commands from the demo animation interpolate between neighbours).
// Fingers: hinge rotation about the knuckle pivot (rig rule 5). Wrist: virtual body at frame.wrist that
// carries every hand-side body, so the forearm stays fixed and only the hand turns.
#pragma once
#include <math.h>
#include <string.h>
#include "model_data.h"

namespace kin {

struct M34 { float m[12]; };  // row-major 3x4, column-vector convention: p' = M * [p;1]

inline M34 ident() { return {{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0}}; }

inline M34 mul(const M34& a, const M34& b) {
  M34 r;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      r.m[i * 4 + j] = a.m[i * 4] * b.m[j] + a.m[i * 4 + 1] * b.m[4 + j] + a.m[i * 4 + 2] * b.m[8 + j];
    r.m[i * 4 + 3] = a.m[i * 4] * b.m[3] + a.m[i * 4 + 1] * b.m[7] + a.m[i * 4 + 2] * b.m[11] + a.m[i * 4 + 3];
  }
  return r;
}

// 3x3 rotation R (row-major) applied about `pivot`: p' = R (p - pivot) + pivot
inline M34 about_point(const float R[9], const float pivot[3]) {
  M34 r;
  for (int i = 0; i < 3; i++) {
    r.m[i * 4 + 0] = R[i * 3 + 0]; r.m[i * 4 + 1] = R[i * 3 + 1]; r.m[i * 4 + 2] = R[i * 3 + 2];
    r.m[i * 4 + 3] = pivot[i] - (R[i * 3] * pivot[0] + R[i * 3 + 1] * pivot[1] + R[i * 3 + 2] * pivot[2]);
  }
  return r;
}

// Rodrigues, unit axis, degrees
inline void rot_axis(const float ax[3], float deg, float R[9]) {
  float t = deg * 0.017453293f, c = cosf(t), s = sinf(t), k = 1 - c;
  float x = ax[0], y = ax[1], z = ax[2];
  R[0] = c + x * x * k;     R[1] = x * y * k - z * s; R[2] = x * z * k + y * s;
  R[3] = y * x * k + z * s; R[4] = c + y * y * k;     R[5] = y * z * k - x * s;
  R[6] = z * x * k - y * s; R[7] = z * y * k + x * s; R[8] = c + z * z * k;
}

// unit quaternion w x y z -> 3x3 row-major (same formula as tools/pose_r3.py)
inline void quat_to_mat(const float q[4], float R[9]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - z * w);     R[2] = 2 * (x * z + y * w);
  R[3] = 2 * (x * y + z * w);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - x * w);
  R[6] = 2 * (x * z - y * w);     R[7] = 2 * (y * z + x * w);     R[8] = 1 - 2 * (x * x + y * y);
}

inline void mat3_mul(const float A[9], const float B[9], float C[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
}
inline void mat3_transpose(const float A[9], float T[9]) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) T[j * 3 + i] = A[i * 3 + j];
}

struct Inputs {
  float cmd[model::NFOURBAR];  // commanded servo degrees per plate joint (kFourbars order)
  float flex[5];               // 0..1, thumb index middle ring pinky
  float wrist[9];              // hand rotation relative to the forearm, model frame; identity = rest
  float arm[9];                // the whole arm's rotation (the forearm's own orientation), model frame, about the
                               // wrist point; identity = the arm's step-1 pose (straight out, palm down)
};

inline void rest_inputs(Inputs& in) {
  for (int j = 0; j < model::NFOURBAR; j++) in.cmd[j] = model::kFourbars[j].cmd_closed;
  for (int f = 0; f < 5; f++) in.flex[f] = 0;
  const float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  memcpy(in.wrist, I, sizeof I);
  memcpy(in.arm, I, sizeof I);
}

inline float plate_frac(const Inputs& in, int j) {
  const model::FourbarDef& d = model::kFourbars[j];
  return (d.cmd_open == d.cmd_closed) ? 0.0f : (in.cmd[j] - d.cmd_closed) / (d.cmd_open - d.cmd_closed);
}

inline void solve(const Inputs& in, M34 world[model::NBODY]) {
  for (int b = 0; b < model::NBODY; b++) {
    const model::BodyDef& bd = model::kBodies[b];
    M34 local = ident();
    if (bd.kind == model::FOURBAR) {
      int j = bd.ref / 3, s = bd.ref % 3;
      float c = in.cmd[j];
      c = c < 0 ? 0 : (c > 180 ? 180 : c);
      int i0 = (int)c; if (i0 > 179) i0 = 179;
      float t = c - i0;
      const float* a = model::kFourbarTable[j][i0][s];
      const float* n = model::kFourbarTable[j][i0 + 1][s];
      for (int k = 0; k < 12; k++) local.m[k] = a[k] + (n[k] - a[k]) * t;
    } else if (bd.kind == model::HINGE) {
      const model::HingeDef& h = model::kHinges[bd.ref];
      float f = in.flex[h.finger];
      f = f < 0 ? 0 : (f > 1 ? 1 : f);
      bool thumb = h.finger == 0;
      float wgt = thumb ? model::kThumbWeights[h.knuckle] : model::kCurlWeights[h.knuckle];
      float total = thumb ? model::kThumbTotalDeg : model::kFistTotalDeg;
      float R[9];
      rot_axis(h.axis, model::kRestCurlDeg + f * total * wgt, R);
      local = about_point(R, h.pivot);
    } else if (bd.kind == model::WRIST) {
      local = about_point(in.wrist, model::kWrist);
    }
    world[b] = bd.parent < 0 ? mul(about_point(in.arm, model::kWrist), local) : mul(world[bd.parent], local);
  }
}

}  // namespace kin
