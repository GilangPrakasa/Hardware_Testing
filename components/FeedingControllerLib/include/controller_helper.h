#ifndef CONTROLLER_HELPER_H
#define CONTROLLER_HELPER_H

#include <stdint.h>

uint32_t dutycycleToInt(uint8_t val);
uint8_t intToDutycycle(uint32_t val);

uint32_t throwerFreqToInt(uint8_t val);
uint32_t dosingFreqToInt(uint8_t val);

uint8_t intToThrowFreq(uint32_t val);
uint8_t intToDosingFreq(uint32_t val);

#endif // CONTROLLER_HELPER_H