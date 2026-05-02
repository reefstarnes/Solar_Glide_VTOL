#pragma once

#include <Arduino.h>

//----------- SENSOR DATA STRUCTS-----------

struct BmpData {
  bool  valid;
  float temperatureF;
  float pressure_hPa;
  float pressure_Pa;
  float altitude_m;   // relative height (zeroed at first reading)
};

struct ImuData {
  bool  valid;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
};

struct BatteryData {
  bool     valid;
  uint16_t raw;
  float    v_adc;        // voltage at ADC pin
  float    v_batt;       // estimated battery pack voltage
  float    batt_percent; // state of charge estimate (%)
};

//----------- FUNCTION PROTOTYPES-----------

void initI2C();
bool initSensors();

bool readBMP581(BmpData &out);
bool readBNO085(ImuData &out);
bool readBattery(BatteryData &out);

bool printBMP581Data(const BmpData &d);
bool printBNO085Data(const ImuData &d);
bool printBatteryData(const BatteryData &d);
