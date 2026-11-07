// Checks run every SAFETY_PERIOD_MS. Anything returned here puts the arm into FAULT.
#pragma once
#include "kinematics.h"

// moving: a move is in progress (StallGuard is only trusted mid-move, at speed)
const char* safetyCheck(const Pose& commanded, const Pose& actual, bool moving, float progress);
