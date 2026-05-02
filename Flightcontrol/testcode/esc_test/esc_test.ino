#include <ESP32Servo.h>

Servo esc;
const int escPin = 18;

void setup() {
  Serial.begin(115200);

  esc.setPeriodHertz(50);          // standard ESC/PWM refresh rate
  esc.attach(escPin, 1000, 2000);  // min and max pulse width in us

  Serial.println("Starting ESC test...");

  esc.writeMicroseconds(1000);     // minimum throttle
  delay(5000);                     // let ESC arm
}

void loop() {
  esc.writeMicroseconds(1000);
  delay(3000);

  esc.writeMicroseconds(1200);
  delay(3000);

  esc.writeMicroseconds(1400);
  delay(3000);

  esc.writeMicroseconds(1000);
  delay(3000);
}