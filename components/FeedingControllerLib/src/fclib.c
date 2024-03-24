#include "fclib.h"

#include <driver/gpio.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>

#include "register_defs.h"

#define CHECK_ARG(ARG)                          \
    do {                                        \
        if (!(ARG)) return ESP_ERR_INVALID_ARG; \
    } while (0)
#define I2C_FREQ_HZ 10000
#define FEEDING_CONTROLLER_ADDR 0x5E

#define MAX_RETRY_TRANSMISSION 3

#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_NUM_35) | (1ULL << GPIO_NUM_34))

static TaskHandle_t feeding_controller_task_handle = NULL;
static QueueHandle_t intr_queue_handle = NULL;

static OnIntrEventCb cb_func_;

static esp_err_t read_register(i2c_dev_t *dev, uint8_t reg, uint8_t *val) {
    CHECK_ARG(dev);
    esp_err_t res = ESP_OK;
    for (uint8_t i = 0; i < MAX_RETRY_TRANSMISSION; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        I2C_DEV_TAKE_MUTEX(dev);
        res = i2c_dev_read_reg(dev, reg, val, 1);
        I2C_DEV_GIVE_MUTEX(dev);
        if (res != ESP_OK) {
            ESP_LOGE("FCLIB", "Transmission read failed(%u), retrying.", i);
        } else {
            break;
        }
    }
    return res;
}

static esp_err_t update_register(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val) {
    CHECK_ARG(dev);
    esp_err_t res = ESP_OK;
    uint8_t old;
    res |= read_register(dev, reg, &old);

    for (uint8_t i = 0; i < MAX_RETRY_TRANSMISSION; ++i) {
        uint8_t buf = (old & mask) | val;
        I2C_DEV_TAKE_MUTEX(dev);
        res |= i2c_dev_write_reg(dev, reg, &buf, 1);
        I2C_DEV_GIVE_MUTEX(dev);
        if (res != ESP_OK) {
            ESP_LOGE("FCLIB", "Transmission write failed(%u), retrying.", i);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            break;
        }
    }
    return res;
}

esp_err_t feed_control_init_desc(i2c_dev_t *dev, i2c_port_t port, gpio_num_t sdapin, gpio_num_t sclpin) {
    CHECK_ARG(dev);
    dev->port = port;
    dev->addr = FEEDING_CONTROLLER_ADDR;
    dev->cfg.sda_io_num = sdapin;
    dev->cfg.scl_io_num = sclpin;
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
    return i2c_dev_create_mutex(dev);
}

static void IRAM_ATTR feed_control_isr_handler(void *arg) {
    uint8_t interrupt_source = (uint8_t)arg;
    xQueueSendFromISR(intr_queue_handle, &interrupt_source, NULL);
}

static void on_motor_error(i2c_dev_t *dev, uint8_t intr_source) {
    uint8_t error_bits;
    read_astr(dev, &error_bits);
    // if (error_bits & PUND_BITS) drvError = DRV_ERR_POWER_UNDERVOLTAGE;
    // else if (error_bits & POVR_BITS) drvError = DRV_ERR_POWER_OVERVOLTAGE;
    // else if (error_bits & DNDT_BITS) drvError = DRV_ERR_DOSING_NOT_DETECTED;
    // else if (error_bits & DNLC_BITS) drvError = DRV_ERR_DOSING_NO_LOAD;
    // else
    if (error_bits & DOVC_BITS) drvError = DRV_ERR_DOSING_OVERCURRENT;
    // else if (error_bits & TNDT_BITS) drvError = DRV_ERR_THROWER_NOT_DETECTED;
    // else if (error_bits & TNLC_BITS) drvError = DRV_ERR_THROWER_NO_LOAD;
    else if (error_bits & TOVC_BITS)
        drvError = DRV_ERR_THROWER_OVERCURRENT;
    else
        drvError = DRV_ERR_NORMAL;
    ESP_LOGE("FCLIB", "source:%d, errorbits:0x%02x, drvError:%d\n", intr_source, error_bits, drvError);
    if (cb_func_ != NULL) cb_func_();
}

static void feeding_controller_task(void *pvParam) {
    i2c_dev_t *dev = (i2c_dev_t *)pvParam;
    for (;;) {
        uint8_t interrupt_source;
        if (xQueueReceive(intr_queue_handle, &interrupt_source, portMAX_DELAY)) {
            on_motor_error(dev, interrupt_source);
        }
    }
}

esp_err_t feed_control_init_interrupt(i2c_dev_t *dev, OnIntrEventCb cb_func) {
    cb_func_ = cb_func;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_NUM_35, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_NUM_34, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_35, feed_control_isr_handler, (void *)0);
    gpio_isr_handler_add(GPIO_NUM_34, feed_control_isr_handler, (void *)1);

    intr_queue_handle = xQueueCreate(5, sizeof(uint8_t));
    xTaskCreatePinnedToCore(feeding_controller_task, "feeding_controller_task", 2048 * 2, dev, 10, &feeding_controller_task_handle, 1);

    return ESP_OK;
}

esp_err_t feed_control_free_desc(i2c_dev_t *dev) {
    CHECK_ARG(dev);

    gpio_isr_handler_remove(GPIO_NUM_35);
    gpio_isr_handler_remove(GPIO_NUM_34);

    if (feeding_controller_task_handle != NULL) {
        vTaskDelete(feeding_controller_task_handle);
        feeding_controller_task_handle = NULL;
    }

    if (intr_queue_handle != NULL) {
        vQueueDelete(intr_queue_handle);
        intr_queue_handle = NULL;
    }

    return i2c_dev_delete_mutex(dev);
}

esp_err_t read_ctrl_version(i2c_dev_t *dev, uint8_t *vers) {
    I2C_DEV_CHECK(dev, read_register(dev, VMAJ_ADDR, &vers[0]));
    I2C_DEV_CHECK(dev, read_register(dev, VMIN_ADDR, &vers[1]));
    I2C_DEV_CHECK(dev, read_register(dev, VPAT_ADDR, &vers[2]));
    return ESP_OK;
}

esp_err_t set_tcnr_pdcs(i2c_dev_t *dev, uint8_t pwm) {
    return update_register(dev, TCNR_ADDR, PDCS_MASK, pwm);
}

esp_err_t set_dcnr_pdcs(i2c_dev_t *dev, uint8_t pwm) {
    return update_register(dev, DCNR_ADDR, PDCS_MASK, pwm);
}

esp_err_t set_tcnr_ofsl(i2c_dev_t *dev, uint8_t freq) {
    return update_register(dev, TCNR_ADDR, OFSL_MASK, freq << 5);
}

esp_err_t set_dcnr_ofsl(i2c_dev_t *dev, uint8_t freq) {
    return update_register(dev, DCNR_ADDR, OFSL_MASK, freq << 5);
}

esp_err_t set_tocr_oena(i2c_dev_t *dev, bool start) {
    return update_register(dev, TOCR_ADDR, OENA_MASK, start ? 1 : 0);
}

esp_err_t set_docr_oena(i2c_dev_t *dev, bool start) {
    return update_register(dev, DOCR_ADDR, OENA_MASK, start ? 1 : 0);
}

esp_err_t read_tocr(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, TOCR_ADDR, val);
    *val &= OENA_BITS;
    return ESP_OK;
}

esp_err_t read_docr(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, DOCR_ADDR, val);
    *val &= OENA_BITS;
    return ESP_OK;
}

esp_err_t read_tcnr_pdcs(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, TCNR_ADDR, val);
    *val &= PDCS_BITS;
    return ESP_OK;
}

esp_err_t read_dcnr_pdcs(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, DCNR_ADDR, val);
    *val &= PDCS_BITS;
    return ESP_OK;
}

esp_err_t read_tcnr_ofsl(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, TCNR_ADDR, val);
    *val &= OFSL_BITS;
    *val >>= 5;
    return ESP_OK;
}

esp_err_t read_dcnr_ofsl(i2c_dev_t *dev, uint8_t *val) {
    read_register(dev, DCNR_ADDR, val);
    *val &= OFSL_BITS;
    *val >>= 5;
    return ESP_OK;
}

esp_err_t read_astr(i2c_dev_t *dev, uint8_t *val) {
    return read_register(dev, ASTR_ADDR, val);
}

esp_err_t read_toct(i2c_dev_t *dev, uint8_t *val) {
    return read_register(dev, TOCT_ADDR, val);
}

esp_err_t read_tmcv(i2c_dev_t *dev, uint8_t *val) {
    return read_register(dev, TMCV_ADDR, val);
}

esp_err_t set_toct(i2c_dev_t *dev, uint8_t val) {
    return update_register(dev, TOCT_ADDR, OCTV_MASK, val);
}

esp_err_t read_doct(i2c_dev_t *dev, uint8_t *val) {
    return read_register(dev, DOCT_ADDR, val);
}

esp_err_t read_dmcv(i2c_dev_t *dev, uint8_t *val) {
    return read_register(dev, DMCV_ADDR, val);
}

esp_err_t set_doct(i2c_dev_t *dev, uint8_t val) {
    return update_register(dev, DOCT_ADDR, OCDV_MASK, val);
}

esp_err_t set_dmcv(i2c_dev_t *dev, uint8_t val) {
    return update_register(dev, DMCV_ADDR, DMCV_MASK, val);
}

esp_err_t set_tmcv(i2c_dev_t *dev, uint8_t val) {
    return update_register(dev, TMCV_ADDR, TMCV_MASK, val);
}