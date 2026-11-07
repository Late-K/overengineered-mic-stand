// AS5600 absolute magnetic encoders on each joint's output, behind a TCA9548A I2C mux.
#pragma once
#include "kinematics.h"

void i2cMuxSelect(uint8_t ch);
bool encBegin(const char** why);            // false if a chip or magnet is missing
void encSetCalibration(const float rawAtStow[N_JOINTS]);
bool encCalibrate(float rawAtStow[N_JOINTS]);      // call with the arm resting at its stowed hard stops
bool encReadPose(Pose& out);                // false on I2C error or before calibration
