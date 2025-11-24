/*
Title: ESP32-S3 + BMP581 I2C test
Author: REEF STARNES
Created: 24 NOV '25
Purpose:
 Reads temperature, pressure, and altitude, prints nicely over Serial,
 and blinks an LED on GPIO38 once per reading.
*/

//----------- SOURCES-----------

/*board file: https://github.com/schreibfaul1/ESP32-MiniWebRadio/issues/381
*/

//----------- HEADER FILES-----------
#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BNO08x.h>
#include <Arduino.h>

//----------- CONST PIN DEFINITIONS-----------
#define LED_PIN         38
#define BAR_INT_PIN     8
#define BATT_A_INT      4

#define I2C_SDA_PIN     15
#define I2C_SCL_PIN     16

#define TX_PIN          1
#define RX_PIN          2

#define QM1_PIN         11
#define QM2_PIN         12
#define QM3_PIN         13
#define QM4_PIN         14
#define GM_PIN          21

#define XTRA1_PIN       32
#define XTRA2_PIN       33
#define XTRA3_PIN       34

//----------- CONST ADDRESS DEFINITIONS-----------
// BMP581 I2C address: 0x46 if SDO -> GND, 0x47 if SDO -> VDDIO. We have grounded this pin
#define BMP581_I2C_ADDR 0x46    // <-- CHANGE to 0x47 if you wired SDO high
#define BNO085_I2C_ADDR 0x4A

//----------- CONST NUMERICAL DEFINITIONS-----------
// Sea-level pressure for altitude calculation (hPa)
#define SEALEVELPRESSURE_HPA 1013.25
#define BAUD_RATE 115200

Adafruit_BMP5xx bmp;
Adafruit_BNO08x bno = Adafruit_BNO08x(-1);  // -1 = no hardware reset pin

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(BAUD_RATE);
  // optional: wait for Serial in Arduino IDE, harmless in PlatformIO
  while (!Serial) {
    delay(10);
  }

  Serial.println();
  Serial.println(F("BMP581 test starting..."));

  // Start I2C on custom pins
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Init sensor over I2C
  if (!bmp.begin(BMP581_I2C_ADDR, &Wire)) {
    Serial.println(F("ERROR: Could not find BMP581. Check wiring and I2C address."));
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(250);  // fast blink = error
    }
  }

  Serial.println(F("BMP581 detected. Configuring..."));

  // Basic, sane config (you can tweak later)
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_4X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_10_HZ);  // ~10 samples/sec
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  Serial.println(F("Setup complete. Reading sensor..."));
}

void loop() {
  // Turn LED on while we take a reading
  digitalWrite(LED_PIN, HIGH);

  // Trigger a measurement + read compensated data
  if (!bmp.performReading()) {
    Serial.println(F("Failed to read from BMP581."));
  } else {
    // bmp.temperature is in °C
    // bmp.pressure is in hPa (per Adafruit_BMP5xx docs)
    float temperatureC = bmp.temperature;
    float pressure_hPa = bmp.pressure;
    float pressure_Pa  = pressure_hPa * 100.0f;

    // Simple altitude estimate (relative to SEALEVELPRESSURE_HPA)
    float altitude_m = bmp.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.println(F("--------------------------------------------------"));
    Serial.print (F("Temperature : "));
    Serial.print (temperatureC, 2);
    Serial.println(F(" °C"));

    Serial.print (F("Pressure    : "));
    Serial.print (pressure_hPa, 2);
    Serial.print (F(" hPa  ("));
    Serial.print (pressure_Pa, 1);
    Serial.println(F(" Pa)"));

    Serial.print (F("Altitude    : "));
    Serial.print (altitude_m, 2);
    Serial.println(F(" m (approx)"));
    Serial.println();
  }

  // Turn LED off between reads
  digitalWrite(LED_PIN, LOW);

  delay(1000);  // 1 second between prints
}
