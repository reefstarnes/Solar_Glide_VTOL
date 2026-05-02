// Comm.h
#pragma once
#include <stdint.h>

struct RcChannels {
    int16_t roll;
    int16_t pitch;
    int16_t throttleRaw;
    float throttlePercent;
    int16_t yaw;
    bool    valid;
};

void initComm();
bool updateComm(RcChannels &rc);   // <-- pass-by-reference
//void sendCommBatteryPercent(float v_batt, RcChannels &rc);
bool printCommData(const RcChannels &rc);