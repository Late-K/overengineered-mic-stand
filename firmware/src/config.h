// Every pin, dimension, limit and speed in one place.
// Geometry, limits, route and speeds match sim/config.json - if you change the arm there, change it here too.
#pragma once
#include <Arduino.h>

enum Joint : uint8_t { YAW = 0, SH = 1, EL = 2, WR = 3 };
constexpr int N_JOINTS = 4;

// ---------------------------------------------------------------- pins (Pico GPIO numbers)
constexpr uint8_t PIN_STEP[N_JOINTS] = {6, 8, 10, 2};    // yaw, shoulder, elbow, wrist
constexpr uint8_t PIN_DIR[N_JOINTS] = {7, 9, 11, 3};
constexpr uint8_t PIN_EN = 12;                           // shared by all drivers, LOW = enabled
constexpr uint8_t PIN_DIAG[N_JOINTS] = {13, 14, 15, 26}; // StallGuard outputs
constexpr uint8_t PIN_TMC_TX = 4;                        // UART1 TX -> 1k resistor -> PDN_UART on all four drivers
constexpr uint8_t PIN_TMC_RX = 5;                        // UART1 RX -> PDN_UART directly
constexpr uint8_t PIN_SDA = 16;                          // I2C0 -> TCA9548A mux
constexpr uint8_t PIN_SCL = 17;
constexpr uint8_t PIN_BTN_DEPLOY = 18;                   // to GND, internal pull-up
constexpr uint8_t PIN_BTN_TEACH = 19;
constexpr uint8_t PIN_STATUS_LED = 22;                   // WS2812 data in

// ---------------------------------------------------------------- I2C devices
constexpr uint32_t I2C_HZ = 100000;            // drop to 50000 if the long cable up the arm gives errors
constexpr uint8_t MUX_ADDR = 0x70;
constexpr uint8_t AS5600_ADDR = 0x36;
constexpr uint8_t ENC_MUX_CH[N_JOINTS] = {0, 1, 2, 4};
constexpr uint8_t TOF_MUX_CH = 3;

// ---------------------------------------------------------------- arm geometry (mm)
constexpr float UPPER_ARM = 400;
constexpr float FOREARM = 400;
constexpr float HEAD_OFFSET_R = 90;  // head bracket past the forearm end; room for the flipped-up mic beside it
constexpr float MIC_DROP = 120;      // wrist -> capsule centre (mic hangs below the wrist when live)
constexpr float MIC_HANG = 220;      // wrist -> far end of the mic

// ---------------------------------------------------------------- drive
// Arm joints: 20T->60T then 16T->96T belt stages. Wrist: NEMA11 with a 27:1 planetary gearbox.
constexpr float RATIO[N_JOINTS] = {18, 18, 18, 27};
constexpr int MICROSTEPS = 8;              // the TMC2209 interpolates to 256 internally, so 8 is still silent
constexpr float STEPS_PER_DEG[N_JOINTS] = {
    200.0f * MICROSTEPS * RATIO[0] / 360.0f,
    200.0f * MICROSTEPS * RATIO[1] / 360.0f,
    200.0f * MICROSTEPS * RATIO[2] / 360.0f,
    200.0f * MICROSTEPS * RATIO[3] / 360.0f,
};
constexpr int8_t MOTOR_DIR[N_JOINTS] = {1, 1, 1, 1};  // flip to -1 if a JOG moves a joint the wrong way
constexpr int8_t ENC_DIR[N_JOINTS] = {1, 1, 1, 1};    // flip to -1 if an encoder reads backwards
constexpr float R_SENSE = 0.11f;                       // BTT / FYSETC TMC2209 boards
constexpr uint16_t RUN_CURRENT_MA[N_JOINTS] = {1200, 1200, 1200, 500};  // RMS; 17HS4401 1.7 A, NEMA11 ~0.67 A
constexpr float HOLD_FRAC = 0.3f;           // arm holding current while LIVE (the wrist always brakes at zero current)
constexpr float MOTOR_MAX_RPS = 5.0f;       // quiet StealthChop range on 24 V

// ---------------------------------------------------------------- joint limits (deg)
// Absolute angles: shoulder/elbow measured up from horizontal. Yaw runs past 360 so the arm
// swings round in front of the main desk, never behind you. Wrist: 0 = mic hanging, 180 = straight up;
// when parked the mic leans just past vertical onto a stop at about 185.
constexpr float LIMIT_MIN[N_JOINTS] = {180, -95, -250, 0};
constexpr float LIMIT_MAX[N_JOINTS] = {420, 80, 90, 190};
constexpr float FOLD_MIN = 5;    // shoulder - elbow; keeps the push-rod parallelogram away from dead centre
constexpr float FOLD_MAX = 160;

// ---------------------------------------------------------------- route (yaw, shoulder, elbow, wrist)
// Deploy runs WAYPOINTS[0] -> ... -> (100 mm above target) -> TARGET; stow runs it backwards.
constexpr int N_WAYPOINTS = 2;
constexpr float WAYPOINTS[N_WAYPOINTS][N_JOINTS] = {
    {180, 78, -80, 185},  // stowed: upper arm up, forearm folded down beside it, mic flipped onto its stop
    {330, 78, 20, 0},     // unfolded: turned towards you, forearm raised, mic hanging
};
constexpr float ABOVE_TARGET_MM = 100;             // the mic drops into place from this far above
constexpr float STOWED_TOLERANCE_DEG = 5;          // how close counts as "already stowed" at boot

// ---------------------------------------------------------------- speeds (mm/s at the fastest point of the arm)
constexpr float FAR_SPEED = 600;       // unfolding at the clamp, away from you
constexpr float NEAR_SPEED = 150;      // swinging in towards you
constexpr float APPROACH_SPEED = 75;   // final drop onto the target
constexpr float TRACK_SPEED = 50;      // ToF nudges
// Leg i runs from WAYPOINTS[i] to the next point; the final drop onto the target always uses APPROACH_SPEED.
constexpr float LEG_SPEED[N_WAYPOINTS] = {FAR_SPEED, NEAR_SPEED};
constexpr float TIP_ACCEL = 400;       // mm/s^2
constexpr float MIN_LEG_S = 0.3f;

// ---------------------------------------------------------------- distance tracking
constexpr float DEFAULT_TARGET_DIST_MM = 150;  // change at runtime with SET dist=
constexpr float DEFAULT_AIM_DEG = 82;          // which way the mic faces relative to the arm, seen from above
                                               // (printed by sim/arm_sim.py; change with SET aim=)
constexpr float TRACK_DEADBAND_MM = 30;
constexpr uint32_t TRACK_HOLD_MS = 2000;       // error must last this long before the arm moves
constexpr float TRACK_RANGE_MM = 190;          // follows your whole 38 cm lean either side of the taught spot
constexpr float TRACK_MIN_DIST_MM = 80;        // closer than this -> back off immediately
constexpr uint16_t TOF_MAX_VALID_MM = 600;     // anything further means nobody is there

// ---------------------------------------------------------------- safety
constexpr float MAX_FOLLOW_ERR_DEG = 3;   // encoder vs commanded; more = collision or missed steps
constexpr uint8_t SG_THRESHOLD = 0;       // StallGuard sensitivity, 0 = off. Tune per firmware/README.md
constexpr uint32_t SAFETY_PERIOD_MS = 20;
