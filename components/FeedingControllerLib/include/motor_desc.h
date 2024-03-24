#ifndef MOTOR_DESC_H
#define MOTOR_DESC_H

#include <stdint.h>

#define DUTYCYCLE_15 0
#define DUTYCYCLE_17 1
#define DUTYCYCLE_20 2
#define DUTYCYCLE_22 3
#define DUTYCYCLE_25 4
#define DUTYCYCLE_27 5
#define DUTYCYCLE_30 6
#define DUTYCYCLE_32 7
#define DUTYCYCLE_35 8
#define DUTYCYCLE_37 9
#define DUTYCYCLE_40 10
#define DUTYCYCLE_42 11
#define DUTYCYCLE_45 12
#define DUTYCYCLE_47 13
#define DUTYCYCLE_52 14
#define DUTYCYCLE_55 15
#define DUTYCYCLE_57 16
#define DUTYCYCLE_60 17
#define DUTYCYCLE_62 18
#define DUTYCYCLE_65 19
#define DUTYCYCLE_67 20
#define DUTYCYCLE_70 21
#define DUTYCYCLE_72 22
#define DUTYCYCLE_75 23
#define DUTYCYCLE_77 24
#define DUTYCYCLE_80 25
#define DUTYCYCLE_82 26
#define DUTYCYCLE_85 27
#define DUTYCYCLE_87 28
#define DUTYCYCLE_90 29
#define DUTYCYCLE_92 30
#define DUTYCYCLE_95 31

#define THROWER_FREQ_8KHz 0
#define THROWER_FREQ_9KHz 1
#define THROWER_FREQ_10KHz 2
#define THROWER_FREQ_11KHz 3
#define THROWER_FREQ_12KHz 4
#define THROWER_FREQ_14KHz 5
#define THROWER_FREQ_15KHz 6
#define THROWER_FREQ_16KHz 7

#define DOSING_FREQ_4KHz 0
#define DOSING_FREQ_7KHz 1
#define DOSING_FREQ_11KHz 2
#define DOSING_FREQ_16KHz 3
#define DOSING_FREQ_22KHz 4
#define DOSING_FREQ_29KHz 5
#define DOSING_FREQ_37KHz 6
#define DOSING_FREQ_46KHz 7

typedef struct Thrower {
    uint8_t dutycycle;
    uint8_t freq;
} Thrower;

typedef struct Dosing {
    uint8_t dutycycle;
    uint8_t freq;
} Dosing;

#endif