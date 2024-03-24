#include <math.h>
#include <Arduino.h>

// #define ENABLE_DRIVER_SERIAL_DEBUG
#ifdef ENABLE_DRIVER_SERIAL_DEBUG
#include "driver/uart.h"
#endif // ENABLE_DRIVER_SERIAL_DEBUG

#include <esp_log.h>
#include "FeedingControllerLib.h"
#include "motor_controller.h"
#include "beeper.h"
#include "led_status.h"


// * MOTOR TRANSITION SOFTNESS
#define SOFT    10
#define NORMAL  20
#define HARD    30

#define THROWER_RAMPUP_NUMBER_OF_STEP 9

#define CONFIG_MOTOR_CTRL_RESET_GPIO 25
#define CONFIG_I2C_SDA_GPIO 21
#define CONFIG_I2C_SCL_GPIO 22
#define CONFIG_FEEDING_MOTOR_CONTROL_TASK_HEAP 3072

static const char *TAG = "MotorController";

static TaskHandle_t controllerTaskHandle;

// static uint32_t throwerRampDuration = 3000;
// static uint32_t throwerPow          = 80;
static uint32_t dosingPow           = 50;
// static uint32_t throwerFreq         = 8000;
// static uint32_t dosingFreq          = 4000;
static uint8_t throwerMax           = 100;

static uint32_t throwerPowInternal  = 0;

static bool controllerStatus = false;

static uint8_t disableOverCurrent = false;

void (*onFeedingErrorCb)() = NULL;

static void onMotorFail() {
    ESP_LOGE(TAG, "Alert motor triggerred!!!");
    uint8_t errStatus = getDrvError();
    // state_t feederStateBuff;

    switch (errStatus) {
        case DRV_ERR_NORMAL:
            ESP_LOGW(TAG, "Wrong alert!");
            return;
        case DRV_ERR_THROWER_OVERCURRENT:
            ESP_LOGW(TAG, "Thrower Overcurrent");
            break;
        // case DRV_ERR_THROWER_NO_LOAD:
        //     feederStateBuff = EFEEDER_THROWER_ERROR;
        //     break;
        // case DRV_ERR_THROWER_NOT_DETECTED:
        //     feederStateBuff = EFEEDER_THROWER_ERROR;
            break;
        case DRV_ERR_DOSING_OVERCURRENT:
            ESP_LOGW(TAG, "Dosing Overcurrent");
            break;
        // case DRV_ERR_DOSING_NO_LOAD:
        //     feederStateBuff = EFEEDER_NO_FEED_FLOW;
        //     break;
        // case DRV_ERR_DOSING_NOT_DETECTED:
        //     feederStateBuff = EFEEDER_DOSING_ERROR;
        //     break;
        // case DRV_ERR_POWER_OVERVOLTAGE:
        //     feederStateBuff = EFEEDER_OVER_VOLTAGE;
        //     break;
        // case DRV_ERR_POWER_UNDERVOLTAGE:
        //     feederStateBuff = EFEEDER_UNDER_VOLTAGE;
        //     break;
        default: return;
    }

    // if(!disableOverCurrent)
    // {
    //     setEfeederState(__func__, feederStateBuff, NULL);
    //     onFeedingErrorCb();
    // }
}

static void onDriverFail() 
{
    beepdrivererror();
    leddrivererror();
    for (uint8_t i = 2; i > 0; i--) {
        ESP_LOGE(TAG, "Restarting cobox in %u", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    onFeedingErrorCb();
    esp_restart();
}

#define checkDrv(func) do { if (!func) onDriverFail(); } while (0)

void startThrower(uint8_t pwm, uint32_t maxRampTimeMs) {
    uint8_t numOfStep = THROWER_RAMPUP_NUMBER_OF_STEP;
    // throwerPowInternal = 24;
    checkDrv(setThrowerPwm(throwerPowInternal));
    ESP_LOGI(TAG, "pwm thrower internal %d", throwerPowInternal);
    checkDrv(fcRunThrower());
    uint32_t throwerTimeStart = millis();    
    uint8_t i = 0;
    while (millis() - throwerTimeStart < maxRampTimeMs) {
        if(millis() - throwerTimeStart > (maxRampTimeMs/numOfStep*i)) {
            throwerPowInternal = pwm/numOfStep*(i+1);
            if (throwerPowInternal > pwm) throwerPowInternal = pwm;
            checkDrv(setThrowerPwm(throwerPowInternal));
            ESP_LOGI(TAG, "pwm thrower ramped %d", throwerPowInternal);
            i = i+1;
        }
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
    uint32_t throwerExtraDuration = millis() - throwerTimeStart;
    ESP_LOGI(TAG, "thrower extra duration %d", throwerExtraDuration);
}

// static void stopThrower(uint8_t pwm, uint32_t maxRampTimeMs) {
//     vTaskDelay(maxRampTimeMs / portTICK_PERIOD_MS);
//     throwerPowInternal = 0;
//     checkDrv(fcStopThrower());
// }

void th_start(int pwm)
{
    checkDrv(setThrowerPwm(pwm));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    checkDrv(fcRunThrower());
}

void th_pwm1en1(int pwm)
{
    checkDrv(setThrowerPwm(pwm));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    checkDrv(fcRunThrower());
}

void th_pwm1en0()
{
    checkDrv(setThrowerPwm(100));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    checkDrv(fcStopThrower());
}

void th_pwm0en0()
{
    checkDrv(setThrowerPwm(0));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    checkDrv(fcStopThrower());
}

void th_pwm0en1()
{
    checkDrv(setThrowerPwm(0));
    vTaskDelay(50 / portTICK_PERIOD_MS);
    checkDrv(fcRunThrower());
}

void dos_pwm1en1()
{
    checkDrv(setDosingPwm(100));
    vTaskDelay(50/ portTICK_PERIOD_MS);
    checkDrv(fcRunDosing());
}

void dos_pwm1en0()
{
    checkDrv(setDosingPwm(100));
    vTaskDelay(50/ portTICK_PERIOD_MS);
    checkDrv(fcStopDosing());
}

void dos_pwm0en0()
{
    checkDrv(setDosingPwm(0));
    vTaskDelay(50/ portTICK_PERIOD_MS);
    checkDrv(fcStopDosing());
}

void dos_pwm0en1()
{
    checkDrv(setDosingPwm(0));
    vTaskDelay(50/ portTICK_PERIOD_MS);
    checkDrv(fcRunDosing());
}

uint32_t readpwmdos()
{
    return readDosingPwm();
}

uint32_t readpwmth()
{
    return readThrowerPwm();
}

#ifdef ENABLE_DRIVER_SERIAL_DEBUG
static void driverSerialDiagTask(void *pvParameter) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, -1, GPIO_NUM_16, -1, -1);

    static const char *RX_TASK_TAG = "RX_TASK";
    uint8_t* data = (uint8_t*) malloc(1024+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, 1024, 500 / portTICK_RATE_MS);
        char *newData = (char *)malloc(rxBytes + 1);  // +1 for the null terminator
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGW(RX_TASK_TAG, "Read %d bytes: \n'%s'", rxBytes, data);

            int j = 0;
            for (int i = 0; i < rxBytes; i++) {
                if (data[i] != ' ' && data[i] != '\n' && data[i] != '\r') {
                    // Copy non-space and non-newline characters to the new string
                    newData[j++] = data[i];
                }
            }
            newData[j] = '\0';
            setEfeederState(__func__, EFEEDER_ATMEGA_SERIAL, (const char *)newData);

        }
        free(newData);
    }
    free(data);
}
#endif // ENABLE_DRIVER_SERIAL_DEBUG

static void controllerTask(void *pvParameter) {
    ESP_LOGD(TAG, "Preparing controller...");
    checkDrv(startFeedingController(CONFIG_I2C_SDA_GPIO, CONFIG_I2C_SCL_GPIO, onMotorFail));
    ESP_LOGD(TAG, "Controller is ready!!!");
    controllerStatus = true;
    for (;;) {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) > 0) {};
    }
	vTaskDelete(NULL);
}

void initMotorController(uint8_t disableOC) {
#ifdef ENABLE_DRIVER_SERIAL_DEBUG
    xTaskCreate(driverSerialDiagTask, "Driver Serial Diag", 1024*2, NULL, 10, NULL);
#endif // ENABLE_DRIVER_SERIAL_DEBUG
    disableOverCurrent = disableOC;
    xTaskCreatePinnedToCore(controllerTask, "Feeding Controller Task", CONFIG_FEEDING_MOTOR_CONTROL_TASK_HEAP, NULL, 5, &controllerTaskHandle, 1);
}

void changeThrowerMaxPWM(uint8_t max) {
    throwerMax = max;
    ESP_LOGI(TAG, "thrower max: %d", throwerMax);
    return;
}

void changeDosingPWM(uint8_t power) {
    dosingPow = power;
    ESP_LOGI(TAG, "dosing power: %d", dosingPow);
    return;
}

void resetMotorController() {
    gpio_reset_pin(CONFIG_MOTOR_CTRL_RESET_GPIO);
    gpio_set_direction(CONFIG_MOTOR_CTRL_RESET_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(CONFIG_MOTOR_CTRL_RESET_GPIO, 0);
    ets_delay_us(1000);
    gpio_set_level(CONFIG_MOTOR_CTRL_RESET_GPIO, 1);
}

bool readControllerFirmwareVersion(uint8_t * version) {
    return readFeedingControllerVersion(version);
}

bool setThrowerAutomaticOverCurrentLimit(){
    uint8_t maxValue = readThrowerMaxCurrentValue();
    ESP_LOGI(TAG, "tmcv: %d", maxValue);
    maxValue += calculateOvercurrentOffset(maxValue); 
    if(maxValue > THROWER_OC_MAX_REG) {
        maxValue = THROWER_OC_MAX_REG;
        ESP_LOGI(TAG, "toc melebihi nilai max");
    }
    ESP_LOGI(TAG, "toct: %d", maxValue);
    return setThrowerOverCurrentThreshold(maxValue);
}

bool setDosingAutomaticOverCurrentLimit(){
    uint8_t dosingMaxValue = readDosingMaxCurrentValue();
    ESP_LOGI(TAG, "dmcv: %d", dosingMaxValue);
    dosingMaxValue += calculateOvercurrentOffset(dosingMaxValue); 
    if(dosingMaxValue > DOSING_OC_MAX_REG) {
        dosingMaxValue = DOSING_OC_MAX_REG;
        ESP_LOGI(TAG, "doc melebihi nilai max");
    }
    ESP_LOGI(TAG, "doct: %d", dosingMaxValue);
    return setDosingOverCurrentThreshold(dosingMaxValue);
}

bool setOvercurrentLimitBasedOnMaxValue() {
    return setThrowerAutomaticOverCurrentLimit() && setDosingAutomaticOverCurrentLimit();
}

uint8_t calculateOvercurrentOffset(uint8_t maxValue) {

    // arus < 1 A       : offset = 0.4
    // 1 A < arus < 2 A : offset = 0.7
    // 2 A < arus       : offset = 1
    if(maxValue < 10) return 4;
    else if(maxValue < 20) return 7;
    else return 10;
}

uint8_t readDosingOverCurrentLimit() {
    return readDosingOverCurrentThreshold();
}

uint8_t readThrowerOverCurrentLimit() {
    return readThrowerOverCurrentThreshold();
}

bool controllerOK() {
    return controllerStatus;
}

bool throwerNoLongerStall() {
    uint8_t throwerMaxValue = readThrowerMaxCurrentValue();
    uint8_t throwerCurrentLimit = readThrowerOverCurrentLimit();
    return (throwerMaxValue <= throwerCurrentLimit);
}

bool dosingNoLongerStall() {
    uint8_t dosingMaxValue = readDosingMaxCurrentValue();
    uint8_t dosingCurrentLimit = readDosingOverCurrentLimit();
    return (dosingMaxValue <= dosingCurrentLimit);
}

bool resetCurrentMax(){
    ESP_LOGI(TAG, "reset current max");
    bool success = resetThrowerCurrentMeasurement();
    success = success && resetDosingCurrentMeasurement();
    
    return success;
}