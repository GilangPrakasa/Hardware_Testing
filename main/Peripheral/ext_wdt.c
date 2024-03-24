#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "ext_wdt.h"

static uint32_t interval_ms = 500;
static TaskHandle_t ext_wdt_task_handler = NULL;

static void ext_wdt_task(void* pvParam) {
    gpio_reset_pin(GPIO_NUM_15);
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);    
    gpio_set_level(GPIO_NUM_15, 0);
    // vTaskDelay(100 / portTICK_PERIOD_MS);

    while (true) {
        gpio_set_level(GPIO_NUM_15, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(GPIO_NUM_15, 0);
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }  
}

void ext_wdt_start(uint32_t pulse_interval_ms) {
    interval_ms = pulse_interval_ms;
    xTaskCreate(
        ext_wdt_task,
        "ext_wdt_task",
        (configMINIMAL_STACK_SIZE * 3),
        NULL,
        0,
        &ext_wdt_task_handler
    );
}

void ext_wdt_stop(void) {
    if (ext_wdt_task_handler != NULL) {
        vTaskDelete(ext_wdt_task_handler);
        ext_wdt_task_handler = NULL;
    }
}