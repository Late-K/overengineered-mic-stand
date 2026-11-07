#include "safety.h"

const char* safetyCheck(const Pose& commanded, const Pose& actual, bool moving, float progress) {
  static const char* followNames[N_JOINTS] = {"yaw blocked or bumped", "shoulder blocked or bumped",
                                              "elbow blocked or bumped", "wrist blocked or bumped"};
  for (int j = 0; j < N_JOINTS; j++)
    if (fabsf(commanded.j[j] - actual.j[j]) > MAX_FOLLOW_ERR_DEG) return followNames[j];

  if (!inLimits(actual)) return "outside joint limits";

  if (SG_THRESHOLD && moving && progress > 0.15f && progress < 0.85f)
    for (int j = 0; j < N_JOINTS; j++)
      if (digitalRead(PIN_DIAG[j])) return "stall detected";
  return nullptr;
}
