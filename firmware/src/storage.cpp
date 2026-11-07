#include "storage.h"

#include <ArduinoJson.h>
#include <LittleFS.h>

static const char* PATH = "/settings.json";

bool settingsLoad(Settings& s) {
  if (!LittleFS.begin()) return false;
  File f = LittleFS.open(PATH, "r");
  if (!f) return false;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) return false;
  s.calibrated = doc["calibrated"] | false;
  s.taught = doc["taught"] | false;
  for (int i = 0; i < N_JOINTS; i++) {
    s.calRaw[i] = doc["calRaw"][i] | 0.0f;
    s.target.j[i] = doc["target"][i] | 0.0f;
  }
  s.targetDist = doc["targetDist"] | DEFAULT_TARGET_DIST_MM;
  s.aimDeg = doc["aimDeg"] | DEFAULT_AIM_DEG;
  if ((doc["joints"] | 3) != N_JOINTS) {  // saved by firmware with a different arm: redo CAL and Teach
    s.calibrated = false;
    s.taught = false;
  }
  return true;
}

bool settingsSave(const Settings& s) {
  JsonDocument doc;
  doc["joints"] = N_JOINTS;
  doc["calibrated"] = s.calibrated;
  doc["taught"] = s.taught;
  for (int i = 0; i < N_JOINTS; i++) {
    doc["calRaw"][i] = s.calRaw[i];
    doc["target"][i] = s.target.j[i];
  }
  doc["targetDist"] = s.targetDist;
  doc["aimDeg"] = s.aimDeg;
  File f = LittleFS.open(PATH, "w");
  if (!f) return false;
  bool ok = serializeJson(doc, f) > 0;
  f.close();
  return ok;
}
