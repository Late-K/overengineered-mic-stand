#include "tracking.h"

#include <VL53L1X.h>
#include <Wire.h>

#include "encoders.h"

static VL53L1X tof;
static bool tofOk = false;
static uint16_t window[5];
static uint8_t count = 0;
static int filtered = -1;
static float offset = 0;
static uint32_t outSince = 0;

bool tofBegin() {
  i2cMuxSelect(TOF_MUX_CH);
  tof.setBus(&Wire);
  tof.setTimeout(100);
  if (!tof.init()) return false;
  tof.setDistanceMode(VL53L1X::Short);  // best up to ~1.3 m and least affected by room light
  tof.setMeasurementTimingBudget(50000);
  tof.setROISize(6, 6);                 // narrow the field of view so it sees your face, not the wall
  tof.startContinuous(50);
  tofOk = true;
  return true;
}

void trackingReset() {
  count = 0;
  filtered = -1;
  offset = 0;
  outSince = 0;
}

int trackingDistance() { return filtered; }

float trackingOffset() { return offset; }

static int median5(const uint16_t* v) {
  uint16_t s[5];
  memcpy(s, v, sizeof(s));
  for (int i = 1; i < 5; i++)
    for (int k = i; k > 0 && s[k] < s[k - 1]; k--) {
      uint16_t t = s[k];
      s[k] = s[k - 1];
      s[k - 1] = t;
    }
  return s[2];
}

bool trackingUpdate(uint32_t now, float targetDist, float& newOffset) {
  if (!tofOk) return false;
  i2cMuxSelect(TOF_MUX_CH);
  if (!tof.dataReady()) return false;
  tof.read(false);
  uint16_t mm = tof.ranging_data.range_mm;
  if (tof.ranging_data.range_status != VL53L1X::RangeValid || mm > TOF_MAX_VALID_MM) {
    count = 0;  // nobody there: stay put
    filtered = -1;
    outSince = 0;
    return false;
  }
  window[count % 5] = mm;
  if (++count < 5) return false;
  filtered = median5(window);

  float err = filtered - targetDist;  // positive: you are further away than wanted
  bool tooClose = filtered < TRACK_MIN_DIST_MM;
  if (fabsf(err) < TRACK_DEADBAND_MM && !tooClose) {
    outSince = 0;
    return false;
  }
  if (!tooClose) {
    if (!outSince) outSince = now ? now : 1;
    if (now - outSince < TRACK_HOLD_MS) return false;
  }
  outSince = 0;
  float next = constrain(offset + err, -TRACK_RANGE_MM, TRACK_RANGE_MM);
  if (fabsf(next - offset) < 5) return false;  // already at the edge of the allowed range
  offset = next;
  newOffset = offset;
  count = 0;  // readings taken during the move would be stale
  return true;
}
