#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Arduino.h"
#include "pcf8574.h"
#include "hd44780.h"

#define GPIO_I2C_SDA 21
#define GPIO_I2C_SCL 22

static i2c_dev_t dev;
static i2c_dev_t pcf8574;
static hd44780_t lcd;

#include "led_strip.h"

#include "led_status.h"

#define LED_TYPE LED_STRIP_WS2812
#define LED_MINIMAL_PERIOD_MS (portTICK_PERIOD_MS * 5)
#define waitLed(condition) do { if (!(condition)) return; } while (0)

// * LED status mode
#define LED_SOLID    0
#define LED_BLINKING 2
#define LED_PULSING  10
#define LED_GLOWING  255

// * LED status period macros
#define LED_PERIOD_FOREVER  0xFFFFFFFF
#define LED_PERIOD_MAX      0x7FFFFFFF
#define LED_PERIOD_CRITICAL (LED_PERIOD_MAX)
#define LED_PERIOD_ERROR    (60 * 1000)
#define LED_PERIOD_POWER_OFF 5000
#define LED_PERIOD_WARNING  2000
#define LED_PERIOD_NOTIFY   2000

#define LED_IDLE_TIME_BEFORE_REPEATITION 2000

#define CONFIG_NEOPIXEL_GPIO 4
#define CONFIG_LED_STATUS_HEAP 2048

static const char *TAG = "led_status";

static led_strip_t* led;
static TaskHandle_t ledTaskHandle = NULL;

// static state_t ledStateInternal = EFEEDER_OK;
// static state_t ledStatePriority = EFEEDER_OK;

static uint8_t abortLedTask = 0;
static bool ledStatusEnable = false;

static uint8_t ledMode = EFEEDER_STANDALONE;

// static state_t idleAPMode = EFEEDER_DISCONNECTED_FROM_AP;

// * Color term from https://en.wikipedia.org/wiki/Color_term
#define COLOR_RED             0 
#define COLOR_ORANGE          1
#define COLOR_YELLOW          2
#define COLOR_CHARTREUSEGREEN 3
#define COLOR_GREEN           4
#define COLOR_SPRINGGREEN     5
#define COLOR_CYAN            6
#define COLOR_AZURE           7
#define COLOR_BLUE            8
#define COLOR_VIOLET          9
#define COLOR_MAGENTA         10
#define COLOR_ROSE            11
#define COLOR_WHITE           12
#define COLOR_NONE            13

static const rgb_t colors[] = {
    {.r = 0xff, .g = 0x00, .b = 0x00}, // COLOR_RED
    {.r = 0xff, .g = 0x44, .b = 0x00}, // COLOR_ORANGE
    {.r = 0xff, .g = 0xaa, .b = 0x00}, // COLOR_YELLOW
    {.r = 0x99, .g = 0xff, .b = 0x00}, // COLOR_CHARTREUSEGREEN
    {.r = 0x00, .g = 0xff, .b = 0x00}, // COLOR_GREEN
    {.r = 0x00, .g = 0xff, .b = 0x33}, // COLOR_SPRINGGREEN
    {.r = 0x00, .g = 0xff, .b = 0xaa}, // COLOR_CYAN
    {.r = 0x00, .g = 0x77, .b = 0xff}, // COLOR_AZURE
    {.r = 0x00, .g = 0x00, .b = 0xff}, // COLOR_BLUE
    {.r = 0x77, .g = 0x00, .b = 0xff}, // COLOR_VIOLET
    {.r = 0xbb, .g = 0x00, .b = 0xff}, // COLOR_MAGENTA
    {.r = 0xff, .g = 0x00, .b = 0xfe}, // COLOR_ROSE
    {.r = 0xff, .g = 0xfe, .b = 0xfd}, // COLOR_WHITE
    {.r = 0x00, .g = 0x00, .b = 0x00}  // COLOR_NONE
};

static uint8_t delayLedMs(uint32_t period) {
    uint32_t buffMs = 0;
    while (!abortLedTask && (buffMs < period)) {
        vTaskDelay(LED_MINIMAL_PERIOD_MS / portTICK_PERIOD_MS);
        buffMs += LED_MINIMAL_PERIOD_MS;
    }
    return (buffMs < period) ? 0 : 1;
}

static void fadeInLed(const rgb_t rgbColor, uint32_t transitionIntervalMs) {
    rgb_t ledColor = colors[COLOR_NONE];
    for (uint8_t factor = 255; rgb_to_code(ledColor) != rgb_to_code(rgbColor); factor--) {
        ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, ledColor));
        ESP_ERROR_CHECK(led_strip_flush(led));
        waitLed(delayLedMs(transitionIntervalMs));
        ledColor = colors[COLOR_NONE];
        ledColor = rgb_add_rgb(ledColor, rgb_fade(rgbColor, factor));
    }
}

static void fadeOutLed(const rgb_t rgbColor, uint32_t transitionIntervalMs) {
    for (rgb_t ledColor = rgbColor; !rgb_is_zero(ledColor); ledColor = rgb_fade(ledColor, 1)) {
        ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, ledColor));
        ESP_ERROR_CHECK(led_strip_flush(led));
        waitLed(delayLedMs(transitionIntervalMs));
    }
}

static void solidLed(const rgb_t rgbColor, uint32_t period) {
    ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, rgbColor));
    ESP_ERROR_CHECK(led_strip_flush(led));
    if (period == LED_PERIOD_FOREVER) return;
    waitLed(delayLedMs(period)); 
}

static void glowingLed(const rgb_t rgbColor, uint32_t period) {
    uint32_t transitionDurationMs = LED_MINIMAL_PERIOD_MS;
    for (uint32_t elapsed = 0; elapsed < period; elapsed += (transitionDurationMs * 255 * 2)) {
        fadeInLed(rgbColor, transitionDurationMs);
        fadeOutLed(rgbColor, transitionDurationMs);
        waitLed(!abortLedTask);
    }
}

static void blinkingLed(const rgb_t rgbColor, uint32_t period, uint8_t blinkRate) {
    if (blinkRate == LED_SOLID) return;

    uint32_t durationPerBlink = 1000 / blinkRate;

    for (uint32_t elapsedMs = 0; elapsedMs < period; elapsedMs += durationPerBlink) {
        ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, colors[COLOR_NONE]));
        ESP_ERROR_CHECK(led_strip_flush(led));
        waitLed(delayLedMs(durationPerBlink / 2));
        ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, rgbColor));
        ESP_ERROR_CHECK(led_strip_flush(led));
        waitLed(delayLedMs(durationPerBlink / 2));
    }
}

static void blinkingLedWithIdleTime(const rgb_t rgbColor, uint32_t period, uint8_t blinkNumber, uint32_t idleTime) {
    
    uint32_t durationPerBlink = 500;

    for (uint32_t elapsedMs = 0; elapsedMs < period; elapsedMs += durationPerBlink) {
        for (uint8_t blinkCount = 0; blinkCount < blinkNumber; blinkCount++) {
            ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, colors[COLOR_NONE]));
            ESP_ERROR_CHECK(led_strip_flush(led));
            waitLed(delayLedMs(durationPerBlink / 2));
            waitLed(delayLedMs(durationPerBlink / 2));
            ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, rgbColor));
            ESP_ERROR_CHECK(led_strip_flush(led));
            waitLed(delayLedMs(durationPerBlink / 2));
        }
        ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, colors[COLOR_NONE]));
        ESP_ERROR_CHECK(led_strip_flush(led));
        waitLed(delayLedMs(idleTime));
    } 
}

void ledStatusTask(void *pvParameter) {
    ESP_LOGI(TAG, "Initializing RGB Led Status");
    led_strip_install();

    led_strip_t ledStrip = {
        .type = LED_STRIP_WS2812,
        .length = 1,
        .gpio = CONFIG_NEOPIXEL_GPIO,
        .buf = NULL,
    #ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
    #endif
    };

    led = &ledStrip;

    ESP_ERROR_CHECK(led_strip_init(led));
    ESP_LOGI(TAG, "RGB Led Status initialized");

    // state_t prevState = EFEEDER_OK;

    // for (;;) {
        // if ((ulTaskNotifyTake(pdTRUE, 0) > 0) && (ledStateInternal != prevState)) {
        //     prevState = ledStateInternal;
        //     abortLedTask = 0;
        //     executeCommand();
        // } else {
        //     backToLastCommand();
        // }

        // vTaskDelay(LED_MINIMAL_PERIOD_MS / portTICK_PERIOD_MS);
    // }
	vTaskDelete(NULL);
    ledTaskHandle = NULL;
}

void initLedStatus(bool en) {
    enableLedStatus(en);    
    ESP_LOGI(TAG, "Initializing RGB Led Status");
    led_strip_install();

    static led_strip_t ledStrip = {
        .type = LED_STRIP_WS2812,
        .length = 1,
        .gpio = CONFIG_NEOPIXEL_GPIO,
        .buf = NULL,
    #ifdef LED_STRIP_BRIGHTNESS
        .brightness = 255,
    #endif
    };

    led = &ledStrip;

    ESP_ERROR_CHECK(led_strip_init(led));
    ESP_LOGI(TAG, "RGB Led Status initialized");
    vTaskDelay(10 / portTICK_RATE_MS);
}

void enableLedStatus(bool en) {
    ledStatusEnable = en;
}

void ledsuccessbooting()
{
    if (ledTaskHandle != NULL) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    ESP_ERROR_CHECK(led_strip_fill(led, 0, led->length, colors[COLOR_GREEN]));
    vTaskDelay(5 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(led_strip_flush(led));
}

static void blinkLeddrivererrorTask(void* pvParam) {
    for (;;) {
        blinkingLedWithIdleTime(colors[COLOR_RED], LED_PERIOD_FOREVER, 6, LED_IDLE_TIME_BEFORE_REPEATITION);
    }
}

static void blinkLed2Task(void* pvParam) {
    uint8_t* blinkNumber = (uint8_t*) pvParam;
    for (;;) {
        blinkingLedWithIdleTime(colors[COLOR_YELLOW], LED_PERIOD_FOREVER, *blinkNumber, LED_IDLE_TIME_BEFORE_REPEATITION);
    }
}

static void blinkLedstateTask(void* pvParam)
{
    uint8_t* blinkNumber = (uint8_t*) pvParam;
    for (;;) {
        blinkingLedWithIdleTime(colors[COLOR_BLUE], LED_PERIOD_FOREVER, *blinkNumber, LED_IDLE_TIME_BEFORE_REPEATITION);
    }
}

static void blinkLederrorstateTask(void* pvParam)
{
    uint8_t* blinkNumber = (uint8_t*) pvParam;
    for (;;) {
        blinkingLedWithIdleTime(colors[COLOR_BLUE], LED_PERIOD_FOREVER, *blinkNumber, LED_IDLE_TIME_BEFORE_REPEATITION);
    }
}

void leddrivererror()
{
    if (ledTaskHandle != NULL) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    xTaskCreate(blinkLeddrivererrorTask, "Blink LED", CONFIG_LED_STATUS_HEAP, NULL, 1, &ledTaskHandle);
}

void ledrunmotor(uint8_t blinkNumber)
{
    if (ledTaskHandle != NULL) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    xTaskCreate(blinkLed2Task, "Blink LED", CONFIG_LED_STATUS_HEAP, (void*) &blinkNumber, 1, &ledTaskHandle);
}

void ledrunstate(uint8_t blinkNumber)
{
    if (ledTaskHandle != NULL) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    xTaskCreate(blinkLedstateTask, "Blink LED", CONFIG_LED_STATUS_HEAP, (void*) &blinkNumber, 1, &ledTaskHandle);
}

void ledrunerrorstate(uint8_t blinkNumber)
{
    if (ledTaskHandle != NULL) {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = NULL;
    }
    xTaskCreate(blinkLederrorstateTask, "Blink LED", CONFIG_LED_STATUS_HEAP, (void*) &blinkNumber, 1, &ledTaskHandle);
}