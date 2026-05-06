#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include <Arduino.h>
#include "Comm.h"
#include "Sensors.h"

struct QuadMotorMix {
  float m1;
  float m2;
  float m3;
  float m4;
};

struct ControlTargets {
  float roll_deg;
  float pitch_deg;
  float yawRate_dps;
};

struct ControlCommands {
  float rollCmd;
  float pitchCmd;
  float yawCmd;
};

float clampFloat(float x, float low, float high);

bool shouldStopMotors(const RcChannels &rc, const ImuData &imu);

ControlTargets getControlTargets(const RcChannels &rc);

ControlCommands getControlCommands(const ControlTargets &target, const ImuData &imu);

QuadMotorMix mixQuadX(float throttle, const ControlCommands &cmd);

#endif