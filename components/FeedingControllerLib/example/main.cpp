#include "FeedingControllerLib.h"

static const char *TAG = "FeedDrv";
static TaskHandle_t controllerTaskHandle = NULL;
static xQueueHandle isrGpio = NULL;

uint32_t throwerPwmVal;
uint32_t dosingPwmVal;
uint32_t throwerFreqVal;
uint32_t dosingFreqVal;
MotorStatus throwerState;
MotorStatus dosingState;

static void IRAM_ATTR incPwm(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(isrGpio, &gpio_num, NULL);
}

void FeedingControllerTask(void *pvParameters) {
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    TickType_t wait = 100 / portTICK_PERIOD_MS;

    FeedingControllerLib fc(&dev, I2C_NUM_0, SDA_PIN_DEFAULT, SCL_PIN_DEFAULT);
    fc.start();

    uint32_t value = 0;
    uint32_t io_num = 0;

    for (;;)
    {
        if(xQueueReceive(isrGpio, &io_num, wait)) {
            if (value > 31) value = 0;

            ESP_LOGI(TAG, "value: %u", value);
            fc.setThrowerPwm(value*2);
            fc.setDosingPwm(value*2);
            fc.setThrowerFreq(value*1000);
            fc.setDosingFreq(value*2000);
            fc.runThrower();
            fc.runDosing();
            
            value++;
        }

        throwerPwmVal = fc.readThrowerPwm();
        dosingPwmVal = fc.readDosingPwm();
        throwerFreqVal = fc.readThrowerFreq();
        dosingFreqVal = fc.readDosingFreq();
        throwerState = fc.getThrowerStatus();
        dosingState = fc.getDosingStatus();

        ESP_LOGI(TAG, "Thrower PWM: %u", throwerPwmVal);
        ESP_LOGI(TAG, "Dosing PWM: %u", dosingPwmVal);
        ESP_LOGI(TAG, "Thrower Freq: %u", throwerFreqVal);
        ESP_LOGI(TAG, "Dosing Freq: %u", dosingFreqVal);
        ESP_LOGI(TAG, "Thrower Stat: %u", (uint8_t)throwerState);
        ESP_LOGI(TAG, "Dosing Stat: %u\n", (uint8_t)dosingState);
        vTaskDelay(pdMS_TO_TICKS(2000));

        fc.stopThrower();
        fc.stopDosing();
    }
}

void app_main() {
    gpio_config_t io_conf = {};

    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL<<0);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    isrGpio = xQueueCreate(10, sizeof(uint32_t));

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(FeedingControllerTask, "Feeding Controller Task", configMINIMAL_STACK_SIZE * 8, NULL, 5, &controllerTaskHandle);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_0, incPwm, NULL);
}