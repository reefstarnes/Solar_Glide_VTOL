// Sensors.cpp
#include "Config.h"
#include "Sensors.h"

#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

//----------- GLOBAL SENSOR OBJECTS-----------

//BMP581 barometer
static Adafruit_BMP5xx bmp;

//BNO085 IMU, -1 means no reset pin
static Adafruit_BNO08x bno = Adafruit_BNO08x(-1);

// BNO085 event container
static sh2_SensorValue_t bno_sensor_value;

// Altitude zeroing state (we make first reading be "0 m")
static bool  g_bmpAltZeroSet = false;
static float g_bmpAltZero    = 0.0f;

//--------------------------------------------------
// INIT FUNCTIONS
//--------------------------------------------------

void initI2C() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(200000);
  delay(500);
}

bool initSensors() {
  // ---------- BMP581 ----------
  if (!bmp.begin(BMP581_I2C_ADDR, &Wire)) {
    Serial.println(F("ERROR: Could not find BMP581. Check wiring and I2C address."));
    return false;
  }
  Serial.println(F("BMP581 detected. Configuring..."));

    /* Temperature Oversampling Settings:
   * BMP5XX_OVERSAMPLING_1X   - 1x oversampling (fastest, least accurate)
   * BMP5XX_OVERSAMPLING_2X   - 2x oversampling  
   * BMP5XX_OVERSAMPLING_4X   - 4x oversampling
   * BMP5XX_OVERSAMPLING_8X   - 8x oversampling
   * BMP5XX_OVERSAMPLING_16X  - 16x oversampling
   * BMP5XX_OVERSAMPLING_32X  - 32x oversampling
   * BMP5XX_OVERSAMPLING_64X  - 64x oversampling
   * BMP5XX_OVERSAMPLING_128X - 128x oversampling (slowest, most accurate) [2]
   * Averages internally the samples, like 64 samples that are averaged and sent.
   */
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_4X); ///CHANGED FROM 16 to 4

  /* Pressure Oversampling Settings (same options as temperature):
   * Higher oversampling = better accuracy but slower readings
   * Recommended: 16X for good balance of speed/accuracy
   */
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_4X); ///CHANGED FROM 16 to 4

  /* IIR Filter Coefficient Settings:
   * BMP5XX_IIR_FILTER_BYPASS   - No filtering (fastest response)
   * BMP5XX_IIR_FILTER_COEFF_1  - Light filtering
   * BMP5XX_IIR_FILTER_COEFF_3  - Medium filtering
   * BMP5XX_IIR_FILTER_COEFF_7  - More filtering
   * BMP5XX_IIR_FILTER_COEFF_15 - Heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_31 - Very heavy filtering
   * BMP5XX_IIR_FILTER_COEFF_63 - Maximum filtering
   * BMP5XX_IIR_FILTER_COEFF_127- Maximum filtering (slowest response) [2]
   * LPF for filtering noise
   */
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);


  
  /* Output Data Rate Settings (Hz):
   * BMP5XX_ODR_240_HZ, BMP5XX_ODR_218_5_HZ, BMP5XX_ODR_199_1_HZ
   * BMP5XX_ODR_179_2_HZ, BMP5XX_ODR_160_HZ, BMP5XX_ODR_149_3_HZ
   * BMP5XX_ODR_140_HZ, BMP5XX_ODR_129_8_HZ, BMP5XX_ODR_120_HZ
   * BMP5XX_ODR_110_1_HZ, BMP5XX_ODR_100_2_HZ, BMP5XX_ODR_89_6_HZ
   * BMP5XX_ODR_80_HZ, BMP5XX_ODR_70_HZ, BMP5XX_ODR_60_HZ, BMP5XX_ODR_50_HZ
   * BMP5XX_ODR_45_HZ, BMP5XX_ODR_40_HZ, BMP5XX_ODR_35_HZ, BMP5XX_ODR_30_HZ
   * BMP5XX_ODR_25_HZ, BMP5XX_ODR_20_HZ, BMP5XX_ODR_15_HZ, BMP5XX_ODR_10_HZ
   * BMP5XX_ODR_05_HZ, BMP5XX_ODR_04_HZ, BMP5XX_ODR_03_HZ, BMP5XX_ODR_02_HZ
   * BMP5XX_ODR_01_HZ, BMP5XX_ODR_0_5_HZ, BMP5XX_ODR_0_250_HZ, BMP5XX_ODR_0_125_HZ [2]
   */
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);  /// CHANGED FROM 10 samples/second to 50!!


  /* Power Mode Settings:
   * BMP5XX_POWERMODE_STANDBY     - Standby mode (no measurements)
   * BMP5XX_POWERMODE_NORMAL      - Normal mode (periodic measurements)
   * BMP5XX_POWERMODE_FORCED      - Forced mode (single measurement then standby)
   * BMP5XX_POWERMODE_CONTINUOUS  - Continuous mode (fastest measurements)
   * BMP5XX_POWERMODE_DEEP_STANDBY - Deep standby (lowest power) [2]
   */
  bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);

  // ---------- BNO085 ----------
  Serial.println(F("Looking for BNO085..."));

  if (!bno.begin_I2C(BNO085_I2C_ADDR, &Wire)) {
    Serial.println(F("ERROR: Could not find BNO085. Check wiring and I2C address."));
    return false;
  }
  Serial.println(F("BNO085 detected. Configuring reports..."));

  // Enable fused rotation vector (quaternion). We’ll convert to yaw/pitch/roll.
  if (!bno.enableReport(SH2_ROTATION_VECTOR, 10000)) {
    Serial.println(F("ERROR: Could not enable BNO085 rotation vector report."));
    return false;
  }

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000)) {
    Serial.println(F("ERROR: Could not enable BNO085 gyro report."));
    return false;
  }

  // ---------- Battery ADC ----------
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db); //range roughly 0-3.3V
  return true;
}

//--------------------------------------------------
// SENSOR READ FUNCTIONS
//--------------------------------------------------

bool readBMP581(BmpData &out) {
  if (!bmp.performReading()) {
    out.valid = false;
    return false;
  }

  out.temperatureF = bmp.temperature* 9.0f/5.0f + 32.0f; //convert celsius to farnenheit
  out.pressure_hPa = bmp.pressure;                 // hPa
  out.pressure_Pa  = out.pressure_hPa * 100.0f;    // Pa

  //Raw absolute altitude from BMP581
  float rawAlt_m = bmp.readAltitude(SEALEVELPRESSURE_HPA);

  //First valid reading becomes our "zero height"
  if (!g_bmpAltZeroSet) {
    g_bmpAltZero    = rawAlt_m;
    g_bmpAltZeroSet = true;
  }

  // Store relative height (0 m at startup)
  out.altitude_m = rawAlt_m - g_bmpAltZero;
  out.valid = true;

  return true;
}

bool readBNO085(ImuData &out) {
  //Keep latest values because BNO085 sends rotation and gyro as separate events
  static ImuData latest = {};
  static bool haveRotation = false;
  static bool haveGyro = false;

  bool gotNewData = false;

  //Drain all waiting BNO085 events
  while (bno.getSensorEvent(&bno_sensor_value)) {
    gotNewData = true;

    //Handle rotation vector event
    if (bno_sensor_value.sensorId == SH2_ROTATION_VECTOR) {

      //Quaternion from fused rotation vector
      float qw = bno_sensor_value.un.rotationVector.real;
      float qx = bno_sensor_value.un.rotationVector.i;
      float qy = bno_sensor_value.un.rotationVector.j;
      float qz = bno_sensor_value.un.rotationVector.k;

      float ysqr = qy * qy;

      //roll, x-axis rotation
      float t0   = +2.0f * (qw * qx + qy * qz);
      float t1   = +1.0f - 2.0f * (qx * qx + ysqr);
      float roll = atan2f(t0, t1);

      //pitch, y-axis rotation
      float t2 = +2.0f * (qw * qy - qz * qx);
      t2       = t2 >  1.0f ?  1.0f : t2;
      t2       = t2 < -1.0f ? -1.0f : t2;
      float pitch = asinf(t2);

      //yaw, z-axis rotation
      float t3  = +2.0f * (qw * qz + qx * qy);
      float t4  = +1.0f - 2.0f * (ysqr + qz * qz);
      float yaw = atan2f(t3, t4);

      //Apply angle offsets
      latest.roll_deg  = (roll  * RAD_TO_DEG) - ROLL_OFFSET_DEG;
      latest.pitch_deg = (pitch * RAD_TO_DEG) - PITCH_OFFSET_DEG;
      latest.yaw_deg   = yaw   * RAD_TO_DEG;

      haveRotation = true;
    }

    //Handle gyro event
else if (bno_sensor_value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
  float rawGyroX = bno_sensor_value.un.gyroscope.x * RAD_TO_DEG;
  float rawGyroY = bno_sensor_value.un.gyroscope.y * RAD_TO_DEG;
  float rawGyroZ = bno_sensor_value.un.gyroscope.z * RAD_TO_DEG;

  const float alpha = 0.025f; //lower = smoother, higher = faster

  //First gyro reading should initialize directly, not ramp up from 0
  if (!haveGyro) {
    latest.gyroX_dps = rawGyroX;
    latest.gyroY_dps = rawGyroY;
    latest.gyroZ_dps = rawGyroZ;
  }
  else {
    latest.gyroX_dps = latest.gyroX_dps + alpha * (rawGyroX - latest.gyroX_dps);
    latest.gyroY_dps = latest.gyroY_dps + alpha * (rawGyroY - latest.gyroY_dps);
    latest.gyroZ_dps = latest.gyroZ_dps + alpha * (rawGyroZ - latest.gyroZ_dps);
  }

  haveGyro = true;
}
  }

  latest.valid = haveRotation && haveGyro;
  out = latest;

  //true means this call actually received at least one fresh IMU packet
  //return gotNewData && latest.valid;
  return latest.valid;
}

bool readBattery(BatteryData &out) {
  //Raw ADC value for debugging
  uint16_t raw = analogRead(BATT_ADC_PIN);
  
  //read milivolts w/ arduino library
  uint32_t mv = analogReadMilliVolts(BATT_ADC_PIN);

  float v_adc  = mv / 1000.0f;               //V at ADC pin
  float v_batt = v_adc * BATT_DIVIDER_RATIO; // actual battery voltage, based on voltage divider. Note each resistor is designed at +-1% tolerance. 

  out.raw = raw;
  out.v_adc = v_adc;
  out.v_batt = v_batt;
  out.valid = true;

  //battery charge percentage estimation, The LiPo state-of-charge curve was taken from Roho’s fitted model on Electronics StackExchange [1].
  float cellV = v_batt / (float)BATT_NUM_CELLS;
  float rawCurve = 123.0f - 123.0f / powf(1.0f + powf((cellV / 3.7f), 80.0f), 0.165f);
  out.batt_percent = 100.0f * (rawCurve - BATT_MIN_RAW_PERCENTAGE) / (BATT_MAX_RAW_PERCENTAGE - BATT_MIN_RAW_PERCENTAGE);

  //clamp if below 0% or above 100% for any reason
  if (out.batt_percent < 0.0f){
    out.batt_percent = 0.0f;
  }  
  if (out.batt_percent > 100.0f){
    out.batt_percent = 100.0f;
  }

  return true;
}

//--------------------------------------------------
// PRINT FUNCTIONS
//--------------------------------------------------

bool printBMP581Data(const BmpData &d) {
  Serial.println(F("[BMP581]"));

  if (!d.valid) {
    Serial.println(F("  Failed to read BMP581.\n"));
    return false;
  }

  Serial.print(F("  Temperature : "));
  Serial.print(d.temperatureF, 2);
  Serial.println(F(" °F"));

  Serial.print(F("  Pressure    : "));
  Serial.print(d.pressure_hPa, 2); //two decimal places displayed
  Serial.print(F(" hPa  ("));
  Serial.print(d.pressure_Pa, 2); //two decimal places displayed
  Serial.println(F(" Pa)"));

  //Note: this is relative height (0 m at startup)
  Serial.print(F("  Altitude    : "));
  Serial.print(d.altitude_m, 2);
  Serial.println(F(" m (relative)\n"));
  return true;
}

bool printBNO085Data(const ImuData &d) {
  Serial.println(F("[BNO085] (Rotation Vector + Gyro)"));

  if (!d.valid) {
    Serial.println(F("  No valid IMU data.\n"));
    return false;
  }

  Serial.print(F("  Yaw   (Z) : "));
  Serial.print(d.yaw_deg, 2);
  Serial.println(F(" °"));

  Serial.print(F("  Pitch (Y) : "));
  Serial.print(d.pitch_deg, 2);
  Serial.println(F(" °"));

  Serial.print(F("  Roll  (X) : "));
  Serial.print(d.roll_deg, 2);
  Serial.println(F(" °\n"));

  Serial.print(F("  Gyro X : "));


  Serial.print(d.gyroX_dps, 2);
  Serial.println(F(" deg/s"));

  Serial.print(F("  Gyro Y : "));
  Serial.print(d.gyroY_dps, 2);
  Serial.println(F(" deg/s"));

  Serial.print(F("  Gyro Z : "));
  Serial.print(d.gyroZ_dps, 2);
  Serial.println(F(" deg/s\n"));

  return true;
}

bool printBatteryData(const BatteryData &d){
  Serial.println(F("[Battery ADC]"));

  if (!d.valid) {
    Serial.println(F("  No valid battery data.\n"));
    return false;
  }

  Serial.print(F("  Raw ADC    : "));
  Serial.println(d.raw);

  Serial.print(F("  V_adc pin  : "));
  Serial.print(d.v_adc, 3);
  Serial.println(F(" V"));

  Serial.print(F(" s V_batt est : "));
  Serial.print(d.v_batt, 2);
  Serial.println(F(" V"));

  Serial.print(F("  SoC est    : "));
  Serial.print(d.batt_percent, 1);
  Serial.println(F(" %\n"));
  return true;
}

/*
[1] Roho, “Calculate battery percentage on LiPo battery (answer),” Electronics StackExchange, Mar. 6, 2021. Accessed: Dec. 1, 2025. [Online]. Available: https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery

[2] Adafruit, “Adafruit_BMP5xx: Arduino library for the BMP580 / BMP581 / BMP585 … barometric pressure sensors,” GitHub repository, 2025. [Online]. Available: https://github.com/adafruit/Adafruit_BMP5xx





*/