#include "motor_desc.h"
#include "controller_helper.h"

uint32_t dutycycleToInt(uint8_t val) {
    if (val >= DUTYCYCLE_95) return 95;
    uint32_t out = 15;
    for (uint8_t i = DUTYCYCLE_15; i <= DUTYCYCLE_95; i++) {
        if (val == i) return out;
        out += (i % 2 == 0) ? 2 : 3;
    }
    return 15;
}

uint8_t intToDutycycle(uint32_t val) {
    if (val < 15) return DUTYCYCLE_15;
    if (val > 95) return DUTYCYCLE_95;

    int32_t out = 15;
            
    for (uint8_t i = DUTYCYCLE_15; i <= DUTYCYCLE_95; i++) {
        if ((int32_t)val - out < 2) {
            return i;
        } 
        out += (i % 2 == 0) ? 2 : 3;
    }

    return DUTYCYCLE_95;
}

uint32_t throwerFreqToInt(uint8_t val) {
    switch (val) {
        case THROWER_FREQ_8KHz : return 8000;
        case THROWER_FREQ_9KHz : return 9000;
        case THROWER_FREQ_10KHz: return 10000;
        case THROWER_FREQ_11KHz: return 11000;
        case THROWER_FREQ_12KHz: return 12000;
        case THROWER_FREQ_14KHz: return 14000;
        case THROWER_FREQ_15KHz: return 15000;
        case THROWER_FREQ_16KHz: return 16000;
        default: return 8000;
    }
}

uint32_t dosingFreqToInt(uint8_t val) {
    switch (val) {
        case DOSING_FREQ_4KHz : return 4000;
        case DOSING_FREQ_7KHz : return 7000;
        case DOSING_FREQ_11KHz: return 11000;
        case DOSING_FREQ_16KHz: return 16000;
        case DOSING_FREQ_22KHz: return 22000;
        case DOSING_FREQ_29KHz: return 29000;
        case DOSING_FREQ_37KHz: return 37000;
        case DOSING_FREQ_46KHz: return 46000;
        default: return 4000;
    }
}

uint8_t intToThrowFreq(uint32_t val) {
    if (val < 8000) return THROWER_FREQ_8KHz;
    if (val > 16000) return THROWER_FREQ_16KHz;
    uint32_t out = 8000;
    for (uint8_t i = 0; i <= 7; i++) {
        if (val - out <= 1000) return i;
        out = throwerFreqToInt(i);
    }
    return THROWER_FREQ_8KHz;
}

uint8_t intToDosingFreq(uint32_t val) {
    if (val < 4000) return DOSING_FREQ_4KHz;
    if (val > 46000) return DOSING_FREQ_46KHz;

    uint32_t out = 4000;

    for (uint8_t i = DOSING_FREQ_4KHz; i <= DOSING_FREQ_46KHz; i++) {
        if (val - out <= (1000 * i + 3000)) return i;
        out = dosingFreqToInt(i);
    }

    return DOSING_FREQ_4KHz;
}