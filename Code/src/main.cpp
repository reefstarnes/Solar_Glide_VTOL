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

  BmpData     bmpData;
  
  ImuData     imuData;
  BatteryData battData;
  RcChannels rcChannels = {0, 0, 0, 0, false, 0, false};
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

    if (!rcChannels.valid || rcChannels.killSwitch) {
      motorQM1.setThrottle(0.0f);
      motorQM2.setThrottle(0.0f);
      motorQM3.setThrottle(0.0f);
      motorQM4.setThrottle(0.0f);
      motorGM.setThrottle(0.0f);
    }
    else {
      float qThrottle = rcChannels.throttlePercent;

      //Safety cutoff: anything above 15% throttle gets forced to 0
      if (qThrottle > 0.15f) {
        qThrottle = 0.0f;
      }

      motorQM1.setThrottle(qThrottle);
      motorQM2.setThrottle(qThrottle);
      motorQM3.setThrottle(qThrottle);
      motorQM4.setThrottle(qThrottle);
      motorGM.setThrottle(0.0f);
    }

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
    delay(20);
    // //for debugging
    // if (loop_count > 3000)
    // {
    //   while (1)
    //   {
    //     //do nothing
    //   }
      
    // }
    
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