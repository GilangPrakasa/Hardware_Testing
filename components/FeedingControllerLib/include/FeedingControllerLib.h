#ifndef FEEDING_CONTROLLER_LIB_H
#define FEEDING_CONTROLLER_LIB_H

#include <string.h>
#include <esp_err.h>
#include <i2cdev.h>
#include "fclib.h"
#include "motor_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

bool startFeedingController(gpio_num_t sdaPin, gpio_num_t sclPin, OnIntrEventCb cb_func);
bool closeFeedingController();
bool readFeedingControllerVersion(uint8_t* ver);
uint8_t getDrvError();

bool setThrowerPwm(uint32_t pwm);
bool setThrowerFreq(uint32_t freq);
bool fcRunThrower();
bool fcStopThrower();
uint32_t readThrowerPwm();
uint32_t readThrowerFreq();

bool setDosingPwm(uint32_t pwm);
bool setDosingFreq(uint32_t freq);
bool fcRunDosing();
bool fcStopDosing();
uint32_t readDosingPwm();
uint32_t readDosingFreq();

uint8_t readThrowerOverCurrentThreshold();
bool setThrowerOverCurrentThreshold(uint8_t value);
uint8_t readThrowerMaxCurrentValue();

uint8_t readDosingOverCurrentThreshold();
bool setDosingOverCurrentThreshold(uint8_t value);
uint8_t readDosingMaxCurrentValue();

bool resetThrowerCurrentMeasurement();
bool resetDosingCurrentMeasurement();

#ifdef __cplusplus
}
#endif
#endif // FEEDING_CONTROLLER_LIB_H