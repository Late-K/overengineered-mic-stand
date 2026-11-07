// VL53L1X on the mic head: keeps the mic a set distance from your face while LIVE.
#pragma once
#include "config.h"

bool tofBegin();
void trackingReset();     // forget readings and go back to the taught spot
int trackingDistance();   // latest filtered distance in mm, -1 if nobody / no reading
float trackingOffset();   // current offset from the taught spot along the mic axis, mm
// Poll the sensor (call only while the arm is still). Returns true when the arm should move,
// with the new offset from the taught spot in newOffset (positive = towards you).
bool trackingUpdate(uint32_t now, float targetDist, float& newOffset);
