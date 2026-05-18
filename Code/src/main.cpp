/*
Title: ESP32-S3 + BMP581 + BNO085 I2C test (modular)
Author: REEF STARNES
Created: 24 NOV '25
Purpose:
  Reads temperature, pressure, and altitude from BMP581,
  reads orientation from BNO085,
  prints nicely over Serial, and
  blinks an LED on GPIO38 once per reading.
*/

//----------- HEADER FILES-----------
#include <Arduino.h>
#include <Wire.h>
//#include <Adafruit_BMP5xx.h>
//#include <Adafruit_BNO08x.h>
//#include <IBusBM.h>
#include <math.h>

#include "Motor.h"
#include "Config.h"
#include "Sensors.h"
#include "Comm.h"
#include "FlightControl.h"

//----------- FUNCTION PROTOTYPES-----------
void initUSBSerial();
void toggleLed();
void LetThereBeLight();
void stopAllMotors();
void writeQuadMotors(const QuadMotorMix &mix);
void printAllDebug(const BmpData &bmpData,
                   const ImuData &imuData,
                   const BatteryData &battData,
                   const RcChannels &rcChannels);

//----------- GLOBAL OBJECTS-----------

Motor motorQM1(QM1_PIN, QM1_CH);  //stern_star (Right Rear)
Motor motorQM2(QM2_PIN, QM2_CH);  //bow_star (Right Front)
Motor motorQM3(QM3_PIN, QM3_CH); //stern_port (Left Rear)
Motor motorQM4(QM4_PIN, QM4_CH); //bow_port (Left_Front)
Motor motorGM (GM_PIN,  GM_CH);

static bool ledStatus = true;
#define SAFETY_CUTOFF 0.9   //cutsoff motors @x% throttle

//------------------- SETUP -------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);

  motorQM1.begin();
  motorQM2.begin();
  motorQM3.begin();
  motorQM4.begin();
  motorGM.begin();

  motorQM1.setThrottle(0.0);
  motorQM2.setThrottle(0.0);
  motorQM3.setThrottle(0.0);
  motorQM4.setThrottle(0.0);
  motorGM.setThrottle(0.0);

  initUSBSerial();
  initI2C();
  initSensors();
  initComm();

  Serial.println(F("Setup complete."));
  LetThereBeLight(); //led indicator that setup is complete
  
}

//------------------- LOOP -------------------
void loop() {

  //create variables

  BmpData     bmpData = {};
  ImuData     imuData = {};
  BatteryData battData = {};
  //RcChannels rcChannels = {0, 0, 0, 0.0, 0, 0, 0, 0.0, 0.0, 0.0};
  RcChannels rcChannels = {};
  //long int loop_count = 0; //for debugging
  while (1)
  {
    //loop_count++;
    //Read sensors
    readBMP581(bmpData);
    readBNO085(imuData);
    readBattery(battData);
    //Read channel data
    updateComm(rcChannels);

    //send Battery reading
    /*sendCommBatteryPercent(battData.batt_percent, rcChannels);*/

    //error handling
    if (!rcChannels.valid || !bmpData.valid || !imuData.valid || !battData.valid)
    {
      //error handle here
    }

if (shouldStopMotors(rcChannels, imuData)) {
  stopAllMotors();
}
else {
  float qThrottle = rcChannels.throttlePercent;

  //Bench safety cutoff
  if (qThrottle > SAFETY_CUTOFF) {
    //stopAllMotors();
    qThrottle = SAFETY_CUTOFF;
  }

  //Do not spin at very low throttle
  else if (qThrottle < 0.03f) {
    stopAllMotors();
  }

  else {
    ControlTargets targets = getControlTargets(rcChannels);
    ControlCommands commands = getControlCommands(targets, imuData);
    QuadMotorMix mix = mixQuadX(qThrottle, commands);

    writeQuadMotors(mix);

    //Keep glide motor off during quad mode
    motorGM.setThrottle(0.0f);
  }
}

    //void printAlldebug();

    toggleLed();
    delay(10);
    // //for debugging
    // if (loop_count > 3000)
    // {
    //   while (1)
    //   {
    //     //do nothing
    //   }
      
    // }



  static uint32_t lastDebugPrintMs = 0;

  if (millis() - lastDebugPrintMs >= 250) {
    lastDebugPrintMs = millis();
    printAllDebug(bmpData, imuData, battData, rcChannels);
  }
    
  }
  

}


//------------------- FUNCTION DECLATIONS -------------------

void stopAllMotors() {
  motorQM1.setThrottle(0.0f);
  motorQM2.setThrottle(0.0f);
  motorQM3.setThrottle(0.0f);
  motorQM4.setThrottle(0.0f);
  motorGM.setThrottle(0.0f);
}

void writeQuadMotors(const QuadMotorMix &mix) {
  motorQM1.setThrottle(mix.m1);
  motorQM2.setThrottle(mix.m2);
  motorQM3.setThrottle(mix.m3);
  motorQM4.setThrottle(mix.m4);
}

void initUSBSerial() {
  Serial.begin(BAUD_RATE);
  delay(500);
  Serial.println();
  Serial.println(F("Serial Begun"));
}

void toggleLed()
{
    ledStatus = !ledStatus; //flip the state
    digitalWrite(LED_PIN, ledStatus);
}

void LetThereBeLight(){
    for (int c = 0; c < 3; c++) {
        for (int i = 0; i < 40; i++) {
            toggleLed();
            delay(25);
        }
        digitalWrite(LED_PIN, LOW);
        delay(300);
    }

    for (int i = 0; i < 8; i++) {
        toggleLed();
        delay(120);
    }
}

void printAllDebug(const BmpData &bmpData,
                   const ImuData &imuData,
                   const BatteryData &battData,
                   const RcChannels &rcChannels) {
  Serial.println(F("=================================================="));

  Serial.println(F("♠♠♠ MOTORS ♠♠♠"));
  motorQM1.printMotor("QM1");
  motorQM2.printMotor("QM2");
  motorQM3.printMotor("QM3");
  motorQM4.printMotor("QM4");
  motorGM.printMotor("GM");
  Serial.println();

  Serial.println(F("♥♥♥ SENSORS ♥♥♥"));
  printBMP581Data(bmpData);
  printBNO085Data(imuData);
  printBatteryData(battData);
  Serial.println();

  Serial.println(F("♦♦♦ RC DATA ♦♦♦"));
  printCommData(rcChannels);

  Serial.println(F("♣♣♣ END FRAME ♣♣♣"));
}