#ifndef FEEDER_CONTROLLER_H_
#define FEEDER_CONTROLLER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// * MOTOR COMMANDS
#define MOTOR_CMD_START_FEEDING 0
#define MOTOR_CMD_STOP_FEEDING 1
#define MOTOR_CMD_SET_DRIVER_CONFIG_TO_DEFAULT 2
#define MOTOR_CMD_RUN_THROWER 3
#define MOTOR_CMD_RUN_DOSING 4
#define MOTOR_CMD_STOP_ALL_MOTOR 5

// * MOTOR STATES
#define MOTOR_STATE_IDLE 0
#define MOTOR_STATE_STARTING 1
#define MOTOR_STATE_STOPPING 2
#define MOTOR_STATE_RUNNING 3

// CURRENT SENSOR
#define OVERCURRENT_REAL_TO_REGISTER_CONVERSION 10
#define THROWER_OVERCURRENT_MAX_VALUE 20
#define DOSING_OVERCURRENT_MAX_VALUE 10
#define OVERCURRENT_MIN_VALUE 0.1

#define THROWER_OC_MAX_REG THROWER_OVERCURRENT_MAX_VALUE*OVERCURRENT_REAL_TO_REGISTER_CONVERSION
#define DOSING_OC_MAX_REG DOSING_OVERCURRENT_MAX_VALUE*OVERCURRENT_REAL_TO_REGISTER_CONVERSION

struct MotorConfig {
    uint8_t throwerPower;
    uint8_t throwerExtra;
};

extern void (*onFeedingErrorCb)();

void initMotorController(uint8_t disableOvercurrentDetection);
void changeThrowerMaxPWM(uint8_t throwerMax);
void changeDosingPWM(uint8_t dosingPower);
void resetMotorController();
void updateMotorState(int32_t state);
void updateMotorConfig(struct MotorConfig* cfg);
bool setThrowerOverCurrentLimit(uint8_t);
bool setDosingOverCurrentLimit(uint8_t);
bool readControllerFirmwareVersion(uint8_t * version);

bool setThrowerAutomaticOverCurrentLimit();
bool setDosingAutomaticOverCurrentLimit();
bool setOvercurrentLimitBasedOnMaxValue();
uint8_t calculateOvercurrentOffset(uint8_t maxValue);

uint8_t readDosingOverCurrentLimit();
uint8_t readThrowerOverCurrentLimit();

bool controllerOK();
bool dosingNoLongerStall();
bool throwerNoLongerStall();

bool resetCurrentMax();

void th_pwm1en1(int pwm);
void th_pwm1en0();
void th_pwm0en0();
void th_pwm0en1(); 

void dos_pwm1en1();
void dos_pwm1en0();
void dos_pwm0en0();
void dos_pwm0en1();
void th_start(int pwm);

uint32_t readpwmdos();
uint32_t readpwmth();

#ifdef __cplusplus
}
#endif
#endif // FEEDER_CONTROLLER_H_