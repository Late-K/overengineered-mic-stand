// Settings that survive power-off, kept as JSON in the Pico's flash (LittleFS).
#pragma once
#include "kinematics.h"

struct Settings {
  bool calibrated = false;
  float calRaw[N_JOINTS] = {0, 0, 0, 0};  // encoder readings with the arm parked on its stops
  bool taught = false;
  Pose target = {{0, 0, 0, 0}};          // your taught mic position
  float targetDist = DEFAULT_TARGET_DIST_MM;
  float aimDeg = DEFAULT_AIM_DEG;
};

bool settingsLoad(Settings& s);
bool settingsSave(const Settings& s);
