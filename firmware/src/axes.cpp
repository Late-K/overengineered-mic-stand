#include "axes.h"

#include <TMCStepper.h>
#include <hardware/gpio.h>

static TMC2209Stepper drvYaw(&Serial2, R_SENSE, 0b00);  // address set by MS1/MS2 on each board
static TMC2209Stepper drvSh(&Serial2, R_SENSE, 0b01);
static TMC2209Stepper drvEl(&Serial2, R_SENSE, 0b10);
static TMC2209Stepper drvWr(&Serial2, R_SENSE, 0b11);
static TMC2209Stepper* const drivers[N_JOINTS] = {&drvYaw, &drvSh, &drvEl, &drvWr};

// ---- shared between the cores. RP2040 has no data cache, so volatile + a barrier is enough.
struct Move {
  int32_t from[N_JOINTS], to[N_JOINTS];
  uint32_t durUs;
};
static Move pending;                     // written by core 0, then havePending is set
static volatile bool havePending = false;
static volatile bool abortReq = false;
static volatile bool busy = false;
static volatile int32_t stepPos[N_JOINTS] = {0, 0, 0, 0};  // written by core 1 only while busy
static volatile float progress = 1.0f;

// ---------------------------------------------------------------- core 0

bool axesBegin() {
  for (int i = 0; i < N_JOINTS; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    pinMode(PIN_DIR[i], OUTPUT);
    pinMode(PIN_DIAG[i], INPUT);
  }
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);

  Serial2.setTX(PIN_TMC_TX);
  Serial2.setRX(PIN_TMC_RX);
  Serial2.begin(115200);

  bool ok = true;
  for (int i = 0; i < N_JOINTS; i++) {
    auto* d = drivers[i];
    d->begin();
    d->pdn_disable(true);        // PDN_UART pin is used for UART
    d->mstep_reg_select(true);   // microsteps from the register, not MS1/MS2 (those set the address)
    d->I_scale_analog(false);    // current set only by RUN_CURRENT_MA, the Vref pot is ignored
    d->toff(4);
    d->blank_time(24);
    d->rms_current(RUN_CURRENT_MA[i], HOLD_FRAC);
    d->microsteps(MICROSTEPS);
    d->intpol(true);             // interpolate to 256 microsteps: smooth and quiet
    d->en_spreadCycle(false);    // StealthChop
    d->pwm_autoscale(true);
    d->pwm_autograd(true);
    d->iholddelay(8);
    d->TPOWERDOWN(20);
    d->TCOOLTHRS(0xFFFFF);       // StallGuard output active at all speeds
    d->SGTHRS(SG_THRESHOLD);
    if (d->test_connection() != 0) ok = false;
  }
  return ok;
}

static void brakeAtStandstill(TMC2209Stepper* d) {
  d->ihold(0);
  d->freewheel(2);  // standstill with zero current: short the coils through the low-side FETs
}

void axesSetMode(DriveMode m) {
  for (int i = 0; i < N_JOINTS; i++) {
    // The wrist never holds with current: a hanging mic is stable on its own, and when folded
    // it rests on its stop. So it is silent whenever it isn't moving, including while LIVE.
    if (m == DriveMode::Brake || i == WR) {
      if (i == WR) drivers[i]->rms_current(RUN_CURRENT_MA[i], HOLD_FRAC);  // run current for moves
      brakeAtStandstill(drivers[i]);
    } else {
      drivers[i]->rms_current(RUN_CURRENT_MA[i], HOLD_FRAC);
      drivers[i]->freewheel(0);
    }
  }
  digitalWrite(PIN_EN, m == DriveMode::Off ? HIGH : LOW);
}

void axesSync(const Pose& actual) {
  if (busy) return;
  for (int i = 0; i < N_JOINTS; i++) stepPos[i] = lroundf(actual.j[i] * STEPS_PER_DEG[i]);
}

bool axesStartMove(const Pose& to, float seconds) {
  if (busy || havePending) return false;
  for (int i = 0; i < N_JOINTS; i++) {
    pending.from[i] = stepPos[i];
    pending.to[i] = lroundf(to.j[i] * STEPS_PER_DEG[i]);
  }
  pending.durUs = (uint32_t)(fmaxf(seconds, MIN_LEG_S) * 1e6f);
  progress = 0;
  __sync_synchronize();
  havePending = true;
  return true;
}

void axesStop() {
  abortReq = true;
  uint32_t t = micros();
  // Core 1 acknowledges within microseconds; the timeout only matters during boot, before it runs.
  while (abortReq && micros() - t < 10000) tight_loop_contents();
  havePending = false;
}

bool axesBusy() { return busy || havePending; }

float axesProgress() { return progress; }

Pose axesCommanded() {
  Pose p;
  for (int i = 0; i < N_JOINTS; i++) p.j[i] = stepPos[i] / STEPS_PER_DEG[i];
  return p;
}

// Minimum-jerk moves peak at 1.875x their average speed and 5.77 * distance / T^2 acceleration.
float legDuration(const Pose& from, const Pose& to, float speed_mm_s) {
  constexpr int N = 16;
  float len = 0, lenBottom = 0;
  Vec3 prevTip = fk(from), prevBottom = fk(from, Point::MicBottom);
  for (int i = 1; i <= N; i++) {
    Pose p = lerpPose(from, to, (float)i / N);
    Vec3 tip = fk(p), bottom = fk(p, Point::MicBottom);
    len += dist3(tip, prevTip);
    lenBottom += dist3(bottom, prevBottom);
    prevTip = tip;
    prevBottom = bottom;
  }
  len = fmaxf(len, lenBottom);
  float t = 1.875f * len / speed_mm_s;
  for (int j = 0; j < N_JOINTS; j++) {
    float maxDegS = MOTOR_MAX_RPS * 360.0f / RATIO[j];
    t = fmaxf(t, 1.875f * fabsf(to.j[j] - from.j[j]) / maxDegS);
  }
  t = fmaxf(t, sqrtf(5.774f * len / TIP_ACCEL));
  return fmaxf(t, MIN_LEG_S);
}

// ---------------------------------------------------------------- core 1: step generation

static Move active;
static uint32_t t0;

void setup1() {}

void loop1() {
  if (abortReq) {
    busy = false;
    havePending = false;
    progress = 1;
    abortReq = false;
  }
  if (havePending) {
    __sync_synchronize();
    active = pending;
    havePending = false;
    t0 = micros();
    busy = true;
  }
  if (!busy) return;

  uint32_t elapsed = micros() - t0;
  float u = elapsed >= active.durUs ? 1.0f : (float)elapsed / active.durUs;
  float s = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));  // minimum-jerk: zero speed and accel at both ends
  bool done = u >= 1.0f;

  for (int i = 0; i < N_JOINTS; i++) {
    int32_t want = active.from[i] + lroundf((active.to[i] - active.from[i]) * s);
    int32_t p = stepPos[i];
    if (want != p) {
      int dir = want > p ? 1 : -1;
      gpio_put(PIN_DIR[i], dir * MOTOR_DIR[i] > 0);
      busy_wait_us_32(1);
      gpio_put(PIN_STEP[i], 1);
      busy_wait_us_32(2);
      gpio_put(PIN_STEP[i], 0);
      stepPos[i] = p + dir;
    }
    if (stepPos[i] != active.to[i]) done = false;
  }
  progress = u;
  if (done) busy = false;
}
