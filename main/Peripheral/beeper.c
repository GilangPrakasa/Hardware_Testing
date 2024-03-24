#include "driver/gpio.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "beeper.h"

#define CONFIG_BEEPER_GPIO 12
#define CONFIG_BEEPER_STATUS_HEAP 2048

static const char *TAG = "Beeper";

static TaskHandle_t beeperTaskHandle;

// static state_t beeperStateInternal = EFEEDER_OK;
static bool beeperState = false;
static bool isCmdFromState = false;
static bool beeperEnable = false;

void beep(uint32_t periodMs) {
    gpio_set_level(CONFIG_BEEPER_GPIO, 1);
    vTaskDelay(periodMs / portTICK_PERIOD_MS);
    gpio_set_level(CONFIG_BEEPER_GPIO, 0);
}

void turnOffBeeper() {
    isCmdFromState = false;
    beeperState = false;
    if(beeperTaskHandle != NULL){
        xTaskNotifyGive(beeperTaskHandle);
    }
}

void beepBeeper() {
    isCmdFromState = false;
    beeperState = true;
    if(beeperTaskHandle != NULL){
        xTaskNotifyGive(beeperTaskHandle);
    }
}

void beeperStartup() {
    for (uint8_t i = 0; i < 4; i++) {
        beep(25);
        gpio_set_level(CONFIG_BEEPER_GPIO, 0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }

    vTaskDelay(200 / portTICK_PERIOD_MS);

    for (uint8_t i = 0; i < 4; i++) {
        beep(50);
        gpio_set_level(CONFIG_BEEPER_GPIO, 0);
        vTaskDelay(25 / portTICK_PERIOD_MS);
    }
}

void beepdrivererror()
{
    gpio_set_level(CONFIG_BEEPER_GPIO, 1);
}

void statebeep(uint8_t loop) {
    for (uint8_t i = 0; i < loop; i++) {
        beep(100);
        gpio_set_level(CONFIG_BEEPER_GPIO, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}


void beeperTask(void *pvParameter) {
    ESP_LOGD(TAG, "Initializing beeper");
    gpio_reset_pin(CONFIG_BEEPER_GPIO);
    gpio_set_direction(CONFIG_BEEPER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_BEEPER_GPIO, 0);
    ESP_LOGD(TAG, "Beeper initialized");
    vTaskDelete(NULL);
}

void initBeeper(bool en) {
    enableBeeper(en);    
    xTaskCreate(beeperTask, "Beeper Status", CONFIG_BEEPER_STATUS_HEAP, NULL, 1, &beeperTaskHandle);
}

void enableBeeper(bool en) {
    beeperEnable = en;
}