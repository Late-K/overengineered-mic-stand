#include "kinematics.h"

#include <math.h>

static constexpr float D2R = PI / 180.0f;
static constexpr float R2D = 180.0f / PI;

Pose poseOf(const float a[N_JOINTS]) {
  Pose p;
  for (int i = 0; i < N_JOINTS; i++) p.j[i] = a[i];
  return p;
}

Vec3 fk(const Pose& p, Point which) {
  float yaw = p.j[YAW] * D2R, sh = p.j[SH] * D2R, el = p.j[EL] * D2R, wr = p.j[WR] * D2R;
  float r = UPPER_ARM * cosf(sh), z = UPPER_ARM * sinf(sh);
  if (which != Point::Elbow) {
    r += FOREARM * cosf(el) + HEAD_OFFSET_R;
    z += FOREARM * sinf(el);
    float along = which == Point::Capsule ? MIC_DROP : which == Point::MicBottom ? MIC_HANG : 0;
    r -= along * sinf(wr);  // the flipping mic swings back over the arm
    z -= along * cosf(wr);
  }
  return {r * cosf(yaw), r * sinf(yaw), z};
}

bool ik(const Vec3& tip, Pose& out) {
  float r = hypotf(tip.x, tip.y) - HEAD_OFFSET_R;
  float d = hypotf(r, tip.z);
  if (d >= UPPER_ARM + FOREARM || d <= fabsf(UPPER_ARM - FOREARM) || d < 1) return false;
  float sh = atan2f(tip.z, r) + acosf((UPPER_ARM * UPPER_ARM + d * d - FOREARM * FOREARM) / (2 * UPPER_ARM * d));
  float el = atan2f(tip.z - UPPER_ARM * sinf(sh), r - UPPER_ARM * cosf(sh));
  out.j[YAW] = atan2f(tip.y, tip.x) * R2D;
  out.j[SH] = sh * R2D;
  out.j[EL] = el * R2D;
  out.j[WR] = 0;
  // Same directions, expressed inside each joint's range: yaw may run past 360.
  out.j[YAW] = fmodf(fmodf(out.j[YAW] - LIMIT_MIN[YAW], 360.0f) + 360.0f, 360.0f) + LIMIT_MIN[YAW];
  if (out.j[EL] > LIMIT_MAX[EL]) out.j[EL] -= 360;
  return true;
}

bool inLimits(const Pose& p, const char** why) {
  static const char* names[N_JOINTS] = {"yaw limit", "shoulder limit", "elbow limit", "wrist limit"};
  for (int i = 0; i < N_JOINTS; i++) {
    if (p.j[i] < LIMIT_MIN[i] || p.j[i] > LIMIT_MAX[i]) {
      if (why) *why = names[i];
      return false;
    }
  }
  float fold = p.j[SH] - p.j[EL];
  if (fold < FOLD_MIN || fold > FOLD_MAX) {
    if (why) *why = "elbow fold limit";
    return false;
  }
  return true;
}

float maxJointDelta(const Pose& a, const Pose& b) {
  float m = 0;
  for (int i = 0; i < N_JOINTS; i++) m = fmaxf(m, fabsf(b.j[i] - a.j[i]));
  return m;
}

Pose lerpPose(const Pose& a, const Pose& b, float t) {
  Pose p;
  for (int i = 0; i < N_JOINTS; i++) p.j[i] = a.j[i] + (b.j[i] - a.j[i]) * t;
  return p;
}

float dist3(const Vec3& a, const Vec3& b) { return sqrtf(sq(a.x - b.x) + sq(a.y - b.y) + sq(a.z - b.z)); }
