#ifndef FEEDING_TASK_H
#define FEEDING_TASK_H

#include <string.h>
#include <esp_err.h>
#include <i2cdev.h>

#include "drv_err_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*OnIntrEventCb)();

extern uint8_t drvError;

esp_err_t feed_control_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sdapin, gpio_num_t sclpin);
esp_err_t feed_control_free_desc(i2c_dev_t *dev);

esp_err_t feed_control_init_interrupt(i2c_dev_t *dev, OnIntrEventCb cb_func);

esp_err_t read_ctrl_version(i2c_dev_t *dev, uint8_t vers[]);

esp_err_t set_tcnr_pdcs(i2c_dev_t *dev, uint8_t pwm);
esp_err_t set_dcnr_pdcs(i2c_dev_t *dev, uint8_t pwm);
esp_err_t set_tcnr_ofsl(i2c_dev_t *dev, uint8_t freq);
esp_err_t set_dcnr_ofsl(i2c_dev_t *dev, uint8_t freq);

esp_err_t set_tocr_oena(i2c_dev_t *dev, bool start);
esp_err_t set_docr_oena(i2c_dev_t *dev, bool start);

esp_err_t read_tocr(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_docr(i2c_dev_t *dev, uint8_t *val);

esp_err_t read_tcnr_pdcs(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_dcnr_pdcs(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_tcnr_ofsl(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_dcnr_ofsl(i2c_dev_t *dev, uint8_t *val);

esp_err_t read_astr(i2c_dev_t *dev, uint8_t *val);

esp_err_t set_toct(i2c_dev_t *dev, uint8_t val);
esp_err_t read_toct(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_tmcv(i2c_dev_t *dev, uint8_t *val);

esp_err_t set_doct(i2c_dev_t *dev, uint8_t val);
esp_err_t read_doct(i2c_dev_t *dev, uint8_t *val);
esp_err_t read_dmcv(i2c_dev_t *dev, uint8_t *val);

esp_err_t set_dmcv(i2c_dev_t *dev, uint8_t val);
esp_err_t set_tmcv(i2c_dev_t *dev, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif // FEEDING_TASK_H