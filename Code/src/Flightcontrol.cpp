#include "FlightControl.h"
#include <math.h>

//------------------- TUNING VALUES -------------------

//Start conservative for first testing
static const float MAX_ANGLE_DEG = 8.0f;
static const float MAX_YAW_RATE_DPS = 50.0f;

//PD gains for roll and pitch
static const float KP_ROLL  = 0.005f;
static const float KD_ROLL  = 0.0012f;

static const float KP_PITCH = 0.005f;
static const float KD_PITCH = 0.0012f;

//P gain for yaw rate
//static const float KP_YAW_RATE = 0.0008f;
static const float KP_YAW_RATE = 0.0015f;
//Correction limits
static const float MAX_ROLL_CMD  = 0.06f;
static const float MAX_PITCH_CMD = 0.06f;
static const float MAX_YAW_CMD   = 0.08f;

//Safety limit
static const float MAX_SAFE_ANGLE_DEG = 45.0f;

//------------------- FUNCTIONS -------------------

float clampFloat(float x, float low, float high) {
  if (x < low) {
    return low;
  }

  if (x > high) {
    return high;
  }

  return x;
}

bool shouldStopMotors(const RcChannels &rc, const ImuData &imu) {
  if (!rc.valid) {
    return true;
  }

  if (rc.killSwitch) {
    return true;
  }

  if (!imu.valid) {
    return true;
  }

  if (fabsf(imu.roll_deg) > MAX_SAFE_ANGLE_DEG) {
    return true;
  }

  if (fabsf(imu.pitch_deg) > MAX_SAFE_ANGLE_DEG) {
    return true;
  }

  return false;
}

// ControlTargets getControlTargets(const RcChannels &rc) {
//   ControlTargets target;

//   //RC percent values already come from Comm.cpp
//   target.roll_deg    = rc.rollPercent  * MAX_ANGLE_DEG;
//   target.pitch_deg   = rc.pitchPercent * MAX_ANGLE_DEG;
//   target.yawRate_dps = rc.yawPercent   * MAX_YAW_RATE_DPS;

//   return target;
// }

ControlTargets getControlTargets(const RcChannels &rc) {
  ControlTargets target;

  //RC percent values already come from Comm.cpp
  target.roll_deg    = -rc.rollPercent * MAX_ANGLE_DEG;
  target.pitch_deg   =  rc.pitchPercent * MAX_ANGLE_DEG;
  target.yawRate_dps =  rc.yawPercent * MAX_YAW_RATE_DPS;

  return target;
}

// ControlCommands getControlCommands(const ControlTargets &target, const ImuData &imu) {
//   ControlCommands cmd;

//   float rollError  = target.roll_deg  - imu.roll_deg;
//   float pitchError = target.pitch_deg - imu.pitch_deg;
//   float yawError   = target.yawRate_dps - imu.gyroZ_dps;

//   //PD for roll/pitch: angle error minus gyro damping
//   cmd.rollCmd  = KP_ROLL  * rollError  - KD_ROLL  * imu.gyroX_dps;
//   cmd.pitchCmd = KP_PITCH * pitchError - KD_PITCH * imu.gyroY_dps;

//   //P yaw-rate control
//   cmd.yawCmd = KP_YAW_RATE * yawError;

//   cmd.rollCmd  = clampFloat(cmd.rollCmd,  -MAX_ROLL_CMD,  MAX_ROLL_CMD);
//   cmd.pitchCmd = clampFloat(cmd.pitchCmd, -MAX_PITCH_CMD, MAX_PITCH_CMD);
//   cmd.yawCmd   = clampFloat(cmd.yawCmd,   -MAX_YAW_CMD,   MAX_YAW_CMD);

//   return cmd;
// }

ControlCommands getControlCommands(const ControlTargets &target, const ImuData &imu) {
  ControlCommands cmd;

  float rollError  = target.roll_deg  - imu.roll_deg;
  float pitchError = target.pitch_deg + imu.pitch_deg;
  float yawError   = target.yawRate_dps + imu.gyroZ_dps;

  //PD for roll/pitch: angle error with gyro damping
  cmd.rollCmd  = KP_ROLL  * rollError  - KD_ROLL  * imu.gyroX_dps;
  cmd.pitchCmd = KP_PITCH * pitchError + KD_PITCH * imu.gyroY_dps;

  //P yaw-rate control
  cmd.yawCmd = KP_YAW_RATE * yawError;

  cmd.rollCmd  = clampFloat(cmd.rollCmd,  -MAX_ROLL_CMD,  MAX_ROLL_CMD);
  cmd.pitchCmd = clampFloat(cmd.pitchCmd, -MAX_PITCH_CMD, MAX_PITCH_CMD);
  cmd.yawCmd   = clampFloat(cmd.yawCmd,   -MAX_YAW_CMD,   MAX_YAW_CMD);

  return cmd;
}

QuadMotorMix mixQuadX(float throttle, const ControlCommands &cmd) {
  QuadMotorMix mix;

  // Actual motor layout:
  // m1 = QM1 = stern_star = right rear
  // m2 = QM2 = bow_star   = right front
  // m3 = QM3 = stern_port = left rear
  // m4 = QM4 = bow_port   = left front

  mix.m1 = throttle - cmd.pitchCmd - cmd.rollCmd + cmd.yawCmd; // right rear
  mix.m2 = throttle + cmd.pitchCmd - cmd.rollCmd - cmd.yawCmd; // right front
  mix.m3 = throttle - cmd.pitchCmd + cmd.rollCmd - cmd.yawCmd; // left rear
  mix.m4 = throttle + cmd.pitchCmd + cmd.rollCmd + cmd.yawCmd; // left front

  mix.m1 = clampFloat(mix.m1, 0.0f, 1.0f);
  mix.m2 = clampFloat(mix.m2, 0.0f, 1.0f);
  mix.m3 = clampFloat(mix.m3, 0.0f, 1.0f);
  mix.m4 = clampFloat(mix.m4, 0.0f, 1.0f);

  return mix;
}