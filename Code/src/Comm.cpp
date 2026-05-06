#include <Arduino.h>
#include <IBusBM.h>
#include <math.h>

#include "Config.h"
#include "Comm.h"

// ---------- INTERNAL STATE ----------

static IBusBM g_ibus;       // one instance handles both RX + telemetry
/*
static uint8_t g_battSensorAddr = 0;   // telemetry sensor slot
*/

// ---------- PRIVATE FUNCTIONS ----------
static float stickToPercent(int16_t raw) {
    //Converts 1000-2000us stick value to about -1.0 to +1.0
    const float center = 1500.0f;
    const float deadband = 25.0f;

    float delta = (float)raw - center;

    if (fabsf(delta) < deadband) {
        return 0.0f;
    }

    if (delta > 0.0f) {
        delta = delta - deadband;
    }
    else {
        delta = delta + deadband;
    }

    float x = delta / (500.0f - deadband);

    if (x < -1.0f) {
        x = -1.0f;
    }

    if (x > 1.0f) {
        x = 1.0f;
    }

    return x;
}

// ---------- PUBLIC FUNCTIONS ----------

void initComm() {
    //1 start bit, 8 data bits, no parity, 1 stop bit
    Serial1.begin(BAUD_RATE, SERIAL_8N1, COMM_RX_PIN, COMM_TX_PIN);

    //Disable the internal timer ISR and use polled mode, the timer ISR was throwing errors. 
    g_ibus.begin(Serial1, IBUSBM_NOTIMER);


    /*
    // Register an external voltage telemetry sensor (battery)
    // IBUSS_EXTV expects value = V * 100 (e.g., 12.34 V -> 1234)
    g_battSensorAddr = g_ibus.addSensor(IBUSS_EXTV);
    */
}


bool updateComm(RcChannels &rc) {
    // Process incoming/outgoing iBus frames
    g_ibus.loop();

    // Read channels using 1-based defines, convert to 0-based index
    int chRoll = g_ibus.readChannel(CH_ROLL - 1);
    int chPitch = g_ibus.readChannel(CH_PITCH - 1);
    int chThrottle = g_ibus.readChannel(CH_THROTTLE - 1);
    int chYaw = g_ibus.readChannel(CH_YAW - 1);
    int chKill = g_ibus.readChannel(CH_KILL - 1);
    
    // If any channel is negative, treat as invalid
    if (chRoll < 0) {
        Serial.println(F("Invalid Channel Data (Roll)"));
        rc.valid = false;
        return false;
    }

    if (chPitch < 0) {
        Serial.println(F("Invalid Channel Data (Pitch)"));
        rc.valid = false;
        return false;
    }

    if (chThrottle < 0) {
        Serial.println(F("Invalid Channel Data (Throttle)"));
        rc.valid = false;
        return false;
    }

    if (chYaw < 0) {
        Serial.println(F("Invalid Channel Data (Yaw)"));
        rc.valid = false;
        return false;
    }

    if (chKill < 0) {
        Serial.println(F("Invalid Channel Data (Kill Switch)"));
        rc.valid = false;
        return false;
    }


    //Store raw values, and convert to integars 
    rc.roll = static_cast<int16_t>(chRoll);
    rc.pitch = static_cast<int16_t>(chPitch);
    rc.throttleRaw = static_cast<int16_t>(chThrottle);
    rc.yaw = static_cast<int16_t>(chYaw);
    rc.killRaw = static_cast<int16_t>(chKill);
    rc.killSwitch = (rc.killRaw > KILL_ACTIVE_US);
    rc.rollPercent = stickToPercent(rc.roll);
    rc.pitchPercent = stickToPercent(rc.pitch);
    rc.yawPercent = stickToPercent(rc.yaw);

    //Map raw throttle µs -> 0.0–1.0
    const float usMin = static_cast<float>(PW_IDLE_THROTTLE); // e.g. 1000
    const float usMax = static_cast<float>(PW_MAX_THROTTLE);  // e.g. 2000;

    float x = (static_cast<float>(rc.throttleRaw) - usMin) / (usMax - usMin);

    //Clamp to [0, 1]
    if (x < 0.0f){
        x = 0.0f;
    }
    if (x > 1.0f){
        x = 1.0f;   
    }

    rc.throttlePercent = x;
    rc.valid = true;
    return true;
}

/*
void sendCommBatteryPercent(float v_batt, RcChannels &rc) {
    if (g_battSensorAddr == 0) return;                          // sensor not registered
    if (!isfinite(v_batt) || v_batt <= 0.0f) return;            // ignore bad battery

    // IBUSS_EXTV expects volts * 100 (12.34 V -> 1234)
    float    scaled     = v_batt * 100.0f;
    uint16_t vbatt_x100 = (uint16_t)roundf(scaled);

    g_ibus.setSensorMeasurement(g_battSensorAddr, vbatt_x100);
}
*/

bool printCommData(const RcChannels &rc) {
    Serial.print(F("[RC] valid="));
    if (rc.valid) {
        Serial.print(F("YES"));
    }   
    else {
        Serial.print(F("NO"));
        return false;
    }

    Serial.print(F("  roll="));
    Serial.print(rc.roll);

    Serial.print(F("  pitch="));
    Serial.print(rc.pitch);

    Serial.print(F("  yaw="));
    Serial.print(rc.yaw);

    Serial.print(F("  thrRaw="));
    Serial.print(rc.throttleRaw);

    Serial.print(F("  thrNorm="));
    Serial.print(rc.throttlePercent, 3);   // 3 decimal places (0.000–1.000)

    Serial.print(F("  killRaw="));
    Serial.print(rc.killRaw);

    Serial.print(F("  kill="));
    if (rc.killSwitch) {
        Serial.print(F("ON"));
    }
    else {
        Serial.print(F("OFF"));
    }

    Serial.print(F("  rollNorm="));
    Serial.print(rc.rollPercent, 3);

    Serial.print(F("  pitchNorm="));
    Serial.print(rc.pitchPercent, 3);

    Serial.print(F("  yawNorm="));
    Serial.print(rc.yawPercent, 3);

    Serial.println();
    return true;
}
