#include <esp_log.h>
#include "controller_helper.h"
#include "FeedingControllerLib.h"

static i2c_dev_t devDesc;

static const char *TAG = "FeedingController";

uint8_t drvError = DRV_ERR_NORMAL;

static bool checkTransmission(esp_err_t func) {
    return (func == ESP_OK);
}

bool startFeedingController(gpio_num_t sdaPin, gpio_num_t sclPin, OnIntrEventCb cb_func) {
    bool res = true;
    uint8_t version[3];
    memset(&devDesc, 0, sizeof(i2c_dev_t));
    res &= checkTransmission(feed_control_init_interrupt(&devDesc, cb_func));
    res &= checkTransmission(feed_control_init_desc(&devDesc, I2C_NUM_0, sdaPin, sclPin));
    res &= checkTransmission(read_ctrl_version(&devDesc, version));
    ESP_LOGI(TAG, "Controller version: v%u.%u.%u", version[0], version[1], version[2]);
    return res;
}

uint8_t getDrvError() {
    return drvError;
}

bool closeFeedingController() {
    return checkTransmission(feed_control_free_desc(&devDesc));
}

bool readFeedingControllerVersion(uint8_t* ver) {
    return checkTransmission(read_ctrl_version(&devDesc, ver));
}

bool setThrowerPwm(uint32_t pwm) {
    return checkTransmission(set_tcnr_pdcs(&devDesc, intToDutycycle(pwm)));
}

bool setThrowerFreq(uint32_t freq) {
    return checkTransmission(set_tcnr_ofsl(&devDesc, intToThrowFreq(freq)));
}

bool fcRunThrower() {
    return checkTransmission(set_tocr_oena(&devDesc, 1));
}

bool fcStopThrower() {
    return checkTransmission(set_tocr_oena(&devDesc, 0));
}

uint32_t readThrowerPwm() {
    uint8_t res;
    checkTransmission(read_tcnr_pdcs(&devDesc, &res));
    return dutycycleToInt(res);
}

uint32_t readThrowerFreq() {
    uint8_t res;
    checkTransmission(read_tcnr_ofsl(&devDesc, &res));
    return throwerFreqToInt(res);
}

bool setDosingPwm(uint32_t pwm) {
    return checkTransmission(set_dcnr_pdcs(&devDesc, intToDutycycle(pwm)));
}

bool setDosingFreq(uint32_t freq) {
    return checkTransmission(set_dcnr_ofsl(&devDesc, intToDosingFreq(freq)));
}

bool fcRunDosing() {
    return checkTransmission(set_docr_oena(&devDesc, 1));
}

bool fcStopDosing() {
    return checkTransmission(set_docr_oena(&devDesc, 0));
}

uint32_t readDosingPwm() {
    uint8_t res;
    checkTransmission(read_dcnr_pdcs(&devDesc, &res));
    return dutycycleToInt(res);
}

uint32_t readDosingFreq() {
    uint8_t res;
    checkTransmission(read_dcnr_ofsl(&devDesc, &res));
    return dosingFreqToInt(res);
}

uint8_t readThrowerOverCurrentThreshold() {
    uint8_t res;
    checkTransmission(read_toct(&devDesc, &res));
    return res;
}

bool setThrowerOverCurrentThreshold(uint8_t value) {
    return checkTransmission(set_toct(&devDesc, value));
}

uint8_t readThrowerMaxCurrentValue() {
    uint8_t res;
    checkTransmission(read_tmcv(&devDesc, &res));
    ESP_LOGI(TAG, "thrower max current: %d", res);
    return res;
}

uint8_t readDosingOverCurrentThreshold() {
    uint8_t res;
    checkTransmission(read_doct(&devDesc, &res));
    return res;
}

bool setDosingOverCurrentThreshold(uint8_t value) {
    return checkTransmission(set_doct(&devDesc, value));
}

uint8_t readDosingMaxCurrentValue() {
    uint8_t res;
    checkTransmission(read_dmcv(&devDesc, &res));
    ESP_LOGI(TAG, "dos max current: %d", res);
    return res;
}

bool resetDosingCurrentMeasurement() {
    return checkTransmission(set_dmcv(&devDesc, 0));
}

bool resetThrowerCurrentMeasurement() {
    return checkTransmission(set_tmcv(&devDesc, 0));
}
