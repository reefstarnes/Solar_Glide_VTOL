#pragma once

#include <math.h>     // for powf (used in the BATT_* macros)

//----------- CONST PIN DEFINITIONS-----------
#define LED_PIN         38
#define BAR_INT_PIN     8
#define BATT_ADC_PIN    4

#define I2C_SDA_PIN     15
#define I2C_SCL_PIN     16

#define COMM_TX_PIN          1
#define COMM_RX_PIN          2

#define QM1_PIN         11
#define QM2_PIN         12
#define QM3_PIN         13
#define QM4_PIN         14
#define GM_PIN          21

#define XTRA1_PIN       32
#define XTRA2_PIN       33
#define XTRA3_PIN       34

//----------- CONST COUNTER CHANNEL DEFINITIONS-----------
//these are channels for esp32 counters for pwm signal generation
#define QM1_CH 0
#define QM2_CH 1
#define QM3_CH 2
#define QM4_CH 3
#define GM_CH  4

//------------- RC CHANNEL MAP -------------
// These are *1-based* to match radio UI labels.
#define CH_ROLL       1   // channel 1
#define CH_PITCH      2   // channel 2
#define CH_THROTTLE   3   // channel 3
#define CH_YAW        4   // channel 4


//----------- CONST ADDRESS DEFINITIONS-----------
// BMP581 I2C address: 0x46 if SDO -> GND, 0x47 if SDO -> VDDIO. We have grounded this pin.
#define BMP581_I2C_ADDR 0x46
// Check your wiring: ADR high is usually 0x4A, ADR low is usually 0x4B.
#define BNO085_I2C_ADDR 0x4A

//----------- CONST NUMERICAL DEFINITIONS-----------
// Sea-level pressure for altitude calculation (hPa)
#define SEALEVELPRESSURE_HPA 1013.25f
#define BAUD_RATE            115200

#define BATT_DIVIDER_HIGH_OHM 150000.0f
#define BATT_DIVIDER_LOW_OHM  20000.0f
#define BATT_DIVIDER_RATIO    ((BATT_DIVIDER_HIGH_OHM + BATT_DIVIDER_LOW_OHM) / BATT_DIVIDER_LOW_OHM)  // = 8.5f
#define BATT_NUM_CELLS        6
#define BATT_MIN_CELL_VOLTAGE 3.3f
#define BATT_MAX_CELL_VOLTAGE 4.2f

#define ROLL_OFFSET_DEG 0.0 //subtracts val from reading
#define PITCH_OFFSET_DEG 1.70 //subtracts val from reading

//battery % approximation from Sam Gibson stackexchange: https://electronics.stackexchange.com/questions/435837/calculate-battery-percentage-on-lipo-battery
#define BATT_MIN_RAW_PERCENTAGE ( 123.0f - 123.0f / powf( (1.0f + powf(( (BATT_MIN_CELL_VOLTAGE) / 3.7f ), 80.0f)), 0.165f ) )
#define BATT_MAX_RAW_PERCENTAGE ( 123.0f - 123.0f / powf( (1.0f + powf(( (BATT_MAX_CELL_VOLTAGE) / 3.7f ), 80.0f)), 0.165f ) )

#define PWM_FREQ_HZ      50//400
#define PWM_RESOLUTION   12
#define PWM_TIMER        0
#define PW_IDLE_THROTTLE 1000
#define PW_MAX_THROTTLE  2000

//duty cycle [%] = 100*f*t_on