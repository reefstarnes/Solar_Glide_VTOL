#pragma once

#include <Arduino.h>

// PWM config for all motors (ESC control)

// Convert 0.0–1.0 throttle → ESC pulse width (microseconds)
uint16_t throttleTo_PWM_PW(float t);

// Convert ESC pulse width (µs) → LEDC duty value
float PWM_PW_To_DutyPercent(uint16_t us);

uint32_t DutyPercentToDutyCounts(float dp);

// Simple motor wrapper around one LEDC channel + pin
class Motor {
public:
  Motor(uint8_t pin, uint8_t channel);

  // Call this once in setup()
  void begin();

  // Throttle in [0.0, 1.0] (values are clamped)
  void setThrottle(float t);

  //prints variables
  void printMotor(const char* headerString) const;

private:
  uint8_t pin;
  uint8_t channel;
  float lastThrottle = 0.0f;
  uint16_t lastPulseUs = 0;
  float lastDutyPercent = 0.0f;
  uint32_t lastDutyCounts = 0;
};
