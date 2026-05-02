#include <math.h>
#include "Motor.h"
#include "Config.h"

uint16_t throttleTo_PWM_PW(float t) {
  //t is normalized throttle, e.g. 0.0, 0.1, 0.5, 1.0.

  //Clamp throttle to [0, 1]. Safety feature from idiots using this function, so not neccessary in theory.
  if (t < 0.0f){
    t = 0.0f;
  }
  if (t > 1.0f){
    t = 1.0f;
  }

  //throttle to pwm pulse width
  float usFloat = PW_IDLE_THROTTLE + (PW_MAX_THROTTLE - PW_IDLE_THROTTLE) * t;


  //Round to nearest whole microsecond
  return static_cast<uint16_t>(roundf(usFloat));
}

float PWM_PW_To_DutyPercent(uint16_t us) {
    //duty% = us * f * 100 * 0.000001  (since us is in microseconds)
    float dutyPercent = (us * 0.000001) * PWM_FREQ_HZ * 100;
    return dutyPercent;
}

uint32_t DutyPercentToDutyCounts(float dp){
  //maxDuty = 2^(PWM_RESOLUTION) - 1  -> largest duty count (e.g. 4095 for 12-bit, 65535 for 16-bit)
  const uint32_t maxDuty = (1 << PWM_RESOLUTION) - 1;

  float countsF = (dp * 0.01f) * static_cast<float>(maxDuty);

  return static_cast<uint32_t>(roundf(countsF));

}

// -------- Motor methods --------

Motor::Motor(uint8_t pin, uint8_t channel)
: pin(pin), channel(channel) {}

void Motor::begin() {
  ledcSetup(channel, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttachPin(pin, channel);
}

void Motor::setThrottle(float t) {
    //compute throttle duty cycle for pwm, and update member variables along the way ( ͡° ͜ʖ ͡° )
    lastThrottle = t;
    lastPulseUs     = throttleTo_PWM_PW(t);
    lastDutyPercent = PWM_PW_To_DutyPercent(lastPulseUs);
    lastDutyCounts  = DutyPercentToDutyCounts(lastDutyPercent);

    //set based on duty cycle calculated
    ledcWrite(channel, lastDutyCounts);
}

void Motor::printMotor(const char* headerString) const {
    if (headerString) {
        Serial.print(headerString);
        Serial.print(" ");
    }

    Serial.print(F("[Motor] pin="));
    Serial.print(pin);
    Serial.print(F(" ch="));
    Serial.print(channel);

    Serial.print(F(" thr="));
    Serial.print(lastThrottle, 3);

    Serial.print(F(" us="));
    Serial.print(lastPulseUs);

    Serial.print(F(" duty%="));
    Serial.print(lastDutyPercent, 2);

    Serial.print(F(" counts="));
    Serial.print(lastDutyCounts);

    Serial.println();
}
/*

[1] Erintse, “Using ESP32 to Control Multi-channel LED Dimming in Arduino IDE Environment,” Instructables, 2023 (accessed 1 Dec 2025).

*/