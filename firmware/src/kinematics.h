// Forward/inverse kinematics for the yaw + double-parallelogram arm + flip-up wrist.
// Same maths as sim/arm_sim.py.
#pragma once
#include "config.h"

struct Vec3 {
  float x, y, z;
};

// Joint angles in degrees: yaw, shoulder, elbow, wrist. The elbow angle is absolute (from horizontal)
// because its motor sits at the base and drives the forearm through a push-rod. The head stays level,
// and the wrist flips the mic from hanging (0) to pointing straight up (180).
struct Pose {
  float j[N_JOINTS];
};

enum class Point { Elbow, Tip, Capsule, MicBottom };

Pose poseOf(const float a[N_JOINTS]);
Vec3 fk(const Pose& p, Point which = Point::Tip);
bool ik(const Vec3& tip, Pose& out);  // elbow-up solution with the mic hanging; false if out of reach
bool inLimits(const Pose& p, const char** why = nullptr);
float maxJointDelta(const Pose& a, const Pose& b);
Pose lerpPose(const Pose& a, const Pose& b, float t);
float dist3(const Vec3& a, const Vec3& b);
