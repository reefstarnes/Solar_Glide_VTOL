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


//----------- FUNCTION PROTOTYPES-----------
void initUSBSerial();
void toggleLed();
void LetThereBeLight();

//----------- GLOBAL OBJECTS-----------

Motor motorQM1(QM1_PIN, QM1_CH);
Motor motorQM2(QM2_PIN, QM2_CH);
Motor motorQM3(QM3_PIN, QM3_CH);
Motor motorQM4(QM4_PIN, QM4_CH);
Motor motorGM (GM_PIN,  GM_CH);

static bool ledStatus = true;

//------------------- SETUP -------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);

  initUSBSerial();
  initI2C();
  initSensors();
  initComm();

  motorQM1.begin();
  motorQM2.begin();
  motorQM3.begin();
  motorQM4.begin();
  motorGM.begin();

  Serial.println(F("Setup complete."));
  LetThereBeLight(); //led indicator that setup is complete
  
}

//------------------- LOOP -------------------
void loop() {

  //create variables

  BmpData     bmpData;
  ImuData     imuData;
  BatteryData battData;
  RcChannels rcChannels = {0, 0, 0, 0, false};

  while (1)
  {

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

    //abritary throttle set values.
    motorQM1.setThrottle(0.0);
    motorQM2.setThrottle(0.25);
    motorQM3.setThrottle(0.5);
    motorQM4.setThrottle(0.75);
    motorGM.setThrottle(1.0);
    

    // Print results
    Serial.println(F("=================================================="));
    Serial.println("♠♠♠ MOTORS ♠♠♠");
    motorQM1.printMotor("QM1");
    motorQM2.printMotor("QM2");
    motorQM3.printMotor("QM3");
    motorQM4.printMotor("QM4");
    motorGM.printMotor("GM");
    Serial.println();

    Serial.println("♥♥♥ SENSORS ♥♥♥");
    printBMP581Data(bmpData);
    printBNO085Data(imuData);
    printBatteryData(battData);
    Serial.println();

    Serial.println("♦♦♦ RC DATA ♦♦♦");
    printCommData(rcChannels);
    Serial.println("♣♣♣ END FRAME ♣♣♣");

    toggleLed();
    delay(200);  // 0.25 seconds between prints

  }
  

}


//------------------- FUNCTION DECLATIONS -------------------

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