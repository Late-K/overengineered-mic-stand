#include "encoders.h"

#include <Wire.h>

static float calRaw[N_JOINTS];
static bool calibrated = false;

void i2cMuxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

static bool readReg(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(AS5600_ADDR, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool readRawDeg(uint8_t j, float& deg) {
  i2cMuxSelect(ENC_MUX_CH[j]);
  uint8_t b[2];
  if (!readReg(0x0C, b, 2)) return false;  // RAW ANGLE, 12 bits
  deg = (((b[0] & 0x0F) << 8) | b[1]) * (360.0f / 4096.0f);
  return true;
}

bool encBegin(const char** why) {
  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.begin();
  for (int j = 0; j < N_JOINTS; j++) {
    i2cMuxSelect(ENC_MUX_CH[j]);
    uint8_t status;
    if (!readReg(0x0B, &status, 1)) {
      *why = "encoder not answering";
      return false;
    }
    if (!(status & 0x20)) {  // MD bit: magnet detected
      *why = "encoder magnet missing";
      return false;
    }
  }
  return true;
}

void encSetCalibration(const float rawAtStow[N_JOINTS]) {
  for (int j = 0; j < N_JOINTS; j++) calRaw[j] = rawAtStow[j];
  calibrated = true;
}

bool encCalibrate(float rawAtStow[N_JOINTS]) {
  for (int j = 0; j < N_JOINTS; j++)
    if (!readRawDeg(j, rawAtStow[j])) return false;
  encSetCalibration(rawAtStow);
  return true;
}

// The magnet only knows 0-360, so each joint angle is placed in a 360-degree window centred on the
// joint's range. Works because every joint moves less than 360.
bool encReadPose(Pose& out) {
  if (!calibrated) return false;
  for (int j = 0; j < N_JOINTS; j++) {
    float raw;
    if (!readRawDeg(j, raw)) return false;
    float lo = LIMIT_MIN[j] - (360.0f - (LIMIT_MAX[j] - LIMIT_MIN[j])) / 2;
    float a = ENC_DIR[j] * (raw - calRaw[j]) + WAYPOINTS[0][j] - lo;
    out.j[j] = lo + fmodf(fmodf(a, 360.0f) + 360.0f, 360.0f);
  }
  return true;
}
