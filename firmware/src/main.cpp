// Motorised mic arm - state machine, routes, buttons and the USB serial protocol.
//
// Serial (USB, 115200, one command per line):
//   DEPLOY | STOW | TEACH | STOP | STATUS? | CAL | SET dist=<mm> | SET aim=<deg> | JOG j=<0..3> deg=<deg>
// Replies: OK, ERR <reason>, or a STATE line:
//   STATE <STOWED|DEPLOYING|LIVE|STOWING|TEACH|FAULT> yaw=<deg> sh=<deg> el=<deg> wr=<deg> dist=<mm> [reason=<text>]
// A STATE line is also sent unprompted on every state change.

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "axes.h"
#include "config.h"
#include "encoders.h"
#include "kinematics.h"
#include "safety.h"
#include "storage.h"
#include "tracking.h"

enum class State { STOWED, DEPLOYING, LIVE, STOWING, TEACH, FAULT };
static const char* const STATE_NAMES[] = {"STOWED", "DEPLOYING", "LIVE", "STOWING", "TEACH", "FAULT"};

static State state = State::FAULT;
static const char* faultReason = "booting";
static Settings settings;
static Adafruit_NeoPixel led(1, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800);
static bool tofPresent = false;

static Pose actual;  // last encoder reading
static bool actualOk = false;

// The queue of moves currently being run.
static Pose route[8];
static float routeSpeed[8];
static int routeLen = 0, routeIdx = 0;

static constexpr float D2R = PI / 180.0f;

// ---------------------------------------------------------------- helpers

static bool readActual() {
  actualOk = encReadPose(actual);
  return actualOk;
}

static void printState() {
  Pose p = actualOk ? actual : axesCommanded();
  Serial.printf("STATE %s yaw=%.1f sh=%.1f el=%.1f wr=%.1f dist=%d", STATE_NAMES[(int)state], p.j[YAW], p.j[SH],
                p.j[EL], p.j[WR], trackingDistance());
  if (state == State::FAULT) Serial.printf(" reason=%s", faultReason);
  Serial.println();
}

static void setState(State s) {
  if (s == state) return;
  state = s;
  printState();
}

static void fault(const char* reason) {
  axesStop();
  axesSetMode(DriveMode::Brake);
  routeLen = routeIdx = 0;
  faultReason = reason;
  if (state == State::FAULT) {
    printState();  // already faulted, but say why again
    return;
  }
  setState(State::FAULT);
}

static void addLeg(const Pose& p, float speed) {
  if (routeLen < 8) {
    route[routeLen] = p;
    routeSpeed[routeLen] = speed;
    routeLen++;
  }
}

static bool startNextLeg() {
  if (routeIdx >= routeLen) return false;
  Pose from = axesCommanded();
  axesStartMove(route[routeIdx], legDuration(from, route[routeIdx], routeSpeed[routeIdx]));
  routeIdx++;
  return true;
}

// The route: WAYPOINTS from config.h, then (100 mm above target), then TARGET. Returns how many.
static int waypoints(Pose* w) {
  int n = 0;
  for (int i = 0; i < N_WAYPOINTS; i++) w[n++] = poseOf(WAYPOINTS[i]);
  Vec3 above = fk(settings.target);
  above.z += ABOVE_TARGET_MM;
  Pose p;
  if (ik(above, p) && inLimits(p)) w[n++] = p;
  w[n++] = settings.target;
  return n;
}

// Speed for the leg between waypoint i and i+1 (either direction).
static float legSpeed(int i, int n) {
  if (i == n - 2) return APPROACH_SPEED;
  return i < N_WAYPOINTS ? LEG_SPEED[i] : NEAR_SPEED;
}

// Start from wherever the arm is: go to the nearest waypoint, then walk the list.
static void beginRoute(bool deploy) {
  Pose w[N_WAYPOINTS + 2];
  int n = waypoints(w);
  int k = 0;
  for (int i = 1; i < n; i++)
    if (maxJointDelta(actual, w[i]) < maxJointDelta(actual, w[k])) k = i;

  routeLen = routeIdx = 0;
  if (maxJointDelta(actual, w[k]) > 1.0f) addLeg(w[k], k == n - 1 ? APPROACH_SPEED : NEAR_SPEED);
  if (deploy)
    for (int i = k + 1; i < n; i++) addLeg(w[i], legSpeed(i - 1, n));
  else
    for (int i = k - 1; i >= 0; i--) addLeg(w[i], legSpeed(i, n));
}

// Make the drivers match reality before moving: no homing needed with absolute encoders.
static bool prepareToMove() {
  if (!readActual()) {
    fault("encoder read failed");
    return false;
  }
  axesSync(actual);
  axesSetMode(DriveMode::Hold);
  return true;
}

// ---------------------------------------------------------------- actions

static const char* deploy() {
  if (!settings.calibrated) return "not calibrated - stow the arm by hand and send CAL";
  if (!settings.taught) return "no target yet - press Teach";
  if (state == State::LIVE || state == State::DEPLOYING) return nullptr;
  axesStop();
  if (!prepareToMove()) return "encoder read failed";
  trackingReset();
  beginRoute(true);
  setState(State::DEPLOYING);
  if (!startNextLeg()) setState(State::LIVE);
  return nullptr;
}

static const char* stow() {
  if (!settings.calibrated) return "not calibrated - stow the arm by hand and send CAL";
  if (state == State::STOWED || state == State::STOWING) return nullptr;
  axesStop();
  if (!prepareToMove()) return "encoder read failed";
  beginRoute(false);
  setState(State::STOWING);
  if (!startNextLeg()) {
    axesSetMode(DriveMode::Brake);
    setState(State::STOWED);
  }
  return nullptr;
}

// First press: motors go limp so you can drag the mic into place. Second press: save it.
static const char* teach() {
  if (state == State::DEPLOYING || state == State::STOWING) return "wait until the arm stops";
  if (!settings.calibrated) return "not calibrated - stow the arm by hand and send CAL";
  if (state != State::TEACH) {
    axesStop();
    axesSetMode(DriveMode::Off);
    setState(State::TEACH);
    return nullptr;
  }
  if (!readActual()) return "encoder read failed";
  const char* why = nullptr;
  if (!inLimits(actual, &why)) return why;
  Vec3 above = fk(actual);
  above.z += ABOVE_TARGET_MM;
  Pose p;
  if (!ik(above, p) || !inLimits(p)) return "no room to approach from above - move it lower or closer";
  settings.target = actual;
  settings.target.j[WR] = 0;  // live means the mic hangs straight down
  settings.taught = true;
  if (!settingsSave(settings)) return "could not save to flash";
  axesSync(actual);
  axesSetMode(DriveMode::Hold);
  trackingReset();
  setState(State::LIVE);
  return nullptr;
}

static const char* calibrate() {
  if (state == State::DEPLOYING || state == State::STOWING) return "wait until the arm stops";
  if (!encCalibrate(settings.calRaw)) return "encoder read failed";
  settings.calibrated = true;
  if (!settingsSave(settings)) return "could not save to flash";
  readActual();
  axesSync(actual);
  axesSetMode(DriveMode::Brake);
  setState(State::STOWED);
  return nullptr;
}

static const char* jog(int j, float deg) {
  if (j < 0 || j >= N_JOINTS) return "joint must be 0 (yaw), 1 (shoulder), 2 (elbow) or 3 (wrist)";
  if (state != State::FAULT && state != State::LIVE) return "JOG works in FAULT or LIVE";
  if (axesBusy()) return "wait until the arm stops";
  if (!prepareToMove()) return "encoder read failed";
  Pose to = actual;
  to.j[j] += deg;
  const char* why = nullptr;
  if (!inLimits(to, &why)) return why;
  routeLen = routeIdx = 0;
  addLeg(to, NEAR_SPEED);
  startNextLeg();
  return nullptr;
}

// Apply a ToF nudge: move the taught spot along the way the mic faces (horizontal only).
static void nudge(float offset) {
  float h = (settings.target.j[YAW] + settings.aimDeg) * D2R;
  Vec3 tip = fk(settings.target);
  tip.x += offset * cosf(h);
  tip.y += offset * sinf(h);
  Pose p;
  if (!ik(tip, p) || !inLimits(p)) return;
  routeLen = routeIdx = 0;
  addLeg(p, TRACK_SPEED);
  startNextLeg();
}

// ---------------------------------------------------------------- serial

static void reply(const char* err) {
  if (err) {
    Serial.print("ERR ");
    Serial.println(err);
  } else {
    Serial.println("OK");
  }
}

static void handleLine(char* line) {
  for (char* c = line; *c && *c != '='; c++) *c = toupper(*c);  // keyword is case-insensitive
  int j;
  float v;
  if (!strcmp(line, "DEPLOY")) reply(deploy());
  else if (!strcmp(line, "STOW")) reply(stow());
  else if (!strcmp(line, "TEACH")) reply(teach());
  else if (!strcmp(line, "STOP")) {
    fault("stopped");
    reply(nullptr);
  } else if (!strcmp(line, "STATUS?")) {
    if (!axesBusy()) readActual();
    printState();
  } else if (!strcmp(line, "CAL")) reply(calibrate());
  else if (sscanf(line, "SET DIST=%f", &v) == 1) {
    if (v < TRACK_MIN_DIST_MM || v > 400) return reply("dist must be 80-400 mm");
    settings.targetDist = v;
    reply(settingsSave(settings) ? nullptr : "could not save to flash");
  } else if (sscanf(line, "SET AIM=%f", &v) == 1) {
    settings.aimDeg = v;
    reply(settingsSave(settings) ? nullptr : "could not save to flash");
  } else if (sscanf(line, "JOG J=%d DEG=%f", &j, &v) == 2) reply(jog(j, v));
  else if (*line) reply("unknown command");
}

static void pollSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = 0;
      handleLine(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------- buttons and LED

struct Button {
  uint8_t pin;
  bool stable = true;  // pull-up: true = released
  bool last = true;
  uint32_t changedAt = 0;
  bool pressed(uint32_t now) {  // true once per press, after 30 ms of steady signal
    bool v = digitalRead(pin);
    if (v != last) {
      last = v;
      changedAt = now;
    }
    if (v != stable && now - changedAt > 30) {
      stable = v;
      return !v;
    }
    return false;
  }
};
static Button btnDeploy{PIN_BTN_DEPLOY}, btnTeach{PIN_BTN_TEACH};

static void pollButtons(uint32_t now) {
  bool d = btnDeploy.pressed(now), t = btnTeach.pressed(now);
  if (!d && !t) return;
  if (state == State::DEPLOYING || state == State::STOWING || axesBusy()) {
    fault("stopped by button");  // any button stops a moving arm
    return;
  }
  const char* err = nullptr;
  if (t) err = teach();
  else if (state == State::LIVE || state == State::TEACH) err = stow();
  else err = deploy();
  if (err) {
    Serial.print("ERR ");
    Serial.println(err);
  }
}

static void updateLed(uint32_t now) {
  uint32_t c;
  switch (state) {
    case State::STOWED: c = led.Color(0, 0, 8); break;  // dim blue
    case State::DEPLOYING:
    case State::STOWING: c = led.Color(40, 20, 0); break;  // amber
    case State::LIVE: c = led.Color(0, 30, 0); break;      // green
    case State::TEACH: c = led.Color(30, 0, 30); break;    // purple
    default: c = (now / 250) % 2 ? led.Color(60, 0, 0) : 0;  // blinking red
  }
  static uint32_t shown = 1;
  if (c != shown) {
    led.setPixelColor(0, c);
    led.show();
    shown = c;
  }
}

// ---------------------------------------------------------------- main loop

void setup() {
  Serial.begin(115200);
  led.begin();
  pinMode(PIN_BTN_DEPLOY, INPUT_PULLUP);
  pinMode(PIN_BTN_TEACH, INPUT_PULLUP);

  bool driversOk = axesBegin();
  const char* encWhy = nullptr;
  bool encOk = encBegin(&encWhy);
  tofPresent = encOk && tofBegin();
  settingsLoad(settings);
  if (settings.calibrated) encSetCalibration(settings.calRaw);

  if (!driversOk) fault("stepper driver not answering on UART");
  else if (!encOk) fault(encWhy);
  else if (!settings.calibrated) fault("not calibrated - stow the arm by hand and send CAL");
  else if (!readActual()) fault("encoder read failed");
  else if (maxJointDelta(actual, poseOf(WAYPOINTS[0])) > STOWED_TOLERANCE_DEG) fault("not stowed at power-up - send STOW");
  else {
    axesSync(actual);
    axesSetMode(DriveMode::Brake);
    state = State::STOWED;
  }
  if (!tofPresent) Serial.println("# distance sensor not found - tracking disabled");
  printState();
}

void loop() {
  uint32_t now = millis();
  pollSerial();
  pollButtons(now);
  updateLed(now);

  // Safety checks at a steady rate whenever the motors are in charge of the arm.
  static uint32_t lastCheck = 0;
  bool moving = axesBusy();
  if (now - lastCheck >= SAFETY_PERIOD_MS && (moving || state == State::LIVE)) {
    lastCheck = now;
    if (!readActual()) return fault("encoder read failed");
    const char* why = safetyCheck(axesCommanded(), actual, moving, axesProgress());
    if (why) return fault(why);
  }
  if (moving) return;

  // A leg just finished: start the next one, or arrive.
  if (routeLen) {
    if (startNextLeg()) return;
    routeLen = routeIdx = 0;
    if (state == State::DEPLOYING) setState(State::LIVE);
    if (state == State::STOWING) {
      axesSetMode(DriveMode::Brake);
      setState(State::STOWED);
    }
    return;
  }

  float offset;
  if (state == State::LIVE && tofPresent && trackingUpdate(now, settings.targetDist, offset)) nudge(offset);
}
