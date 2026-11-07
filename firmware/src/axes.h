// Three TMC2209 stepper axes. Core 0 plans moves; core 1 does nothing but generate step pulses.
#pragma once
#include "kinematics.h"

enum class DriveMode {
  Hold,   // StealthChop, reduced holding current (LIVE)
  Brake,  // zero current, coils shorted: near-zero power, resists being moved (STOWED, FAULT)
  Off,    // drivers disabled, the arm is free to drag (TEACH)
};

bool axesBegin();                     // false if a driver doesn't answer on UART
void axesSetMode(DriveMode m);
void axesSync(const Pose& actual);    // set step counters from the encoders; only while idle
bool axesStartMove(const Pose& to, float seconds);  // smooth minimum-jerk move, all joints arrive together
void axesStop();                      // abort immediately
bool axesBusy();
float axesProgress();                 // 0..1 through the current move
Pose axesCommanded();
float legDuration(const Pose& from, const Pose& to, float speed_mm_s);
