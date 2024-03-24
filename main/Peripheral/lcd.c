#include "lcd.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/timers.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

#include "eventbus.h"
#include "hd44780.h"
#include "i2cdev.h"
#include "pcf8574.h"
// #include "timehelper.h"
// #include "timekeeping.h"

// LCD icon code
#define ICON_MENU_CURSOR 0x7E
#define ICON_SCHEDULE_ON 0
#define ICON_SCHEDULE_OFF 1
#define ICON_TANK_CAPACITY_100 2
#define ICON_TANK_CAPACITY_50 3
#define ICON_TANK_CAPACITY_20 4
#define ICON_TANK_CAPACITY_0 5

#define CONFIG_PCF8574_I2C_ADDRESS 0x38
#define CONFIG_LCD_OBJ_QUEUE_LEN 20
#define CONFIG_LCD_NUM_ROWS 4
#define CONFIG_I2C_SDA_GPIO 21
#define CONFIG_I2C_SCL_GPIO 22
#define CONFIG_DEFAULT_LCD_SLEEP_PERIOD_MS 15000
#define CONFIG_LCD_RENDER_TASK_HEAP 3072
#define CONFIG_WIFI_INFO "5.9.5-beta.3"


#define CHAR_DATA_SIZE 6
static const uint8_t char_data[CHAR_DATA_SIZE][8] = {
    [ICON_SCHEDULE_ON] = {0x08, 0x0C, 0x0E, 0x0F, 0x0E, 0x0C, 0x08, 0x00},
    [ICON_SCHEDULE_OFF] = {0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x00},
    [ICON_TANK_CAPACITY_100] = {0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00},
    [ICON_TANK_CAPACITY_50] = {0x11, 0x11, 0x1F, 0x1F, 0x1F, 0x0E, 0x04, 0x00},
    [ICON_TANK_CAPACITY_20] = {0x11, 0x11, 0x11, 0x11, 0x1F, 0x0E, 0x04, 0x00},
    [ICON_TANK_CAPACITY_0] = {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04, 0x00}};

static esp_err_t lcd_write_cb(const hd44780_t* lcd, uint8_t data);

static const hd44780_t lcd_temp = {
    .write_cb = lcd_write_cb,
    .pins = {.rs = 0, .e = 2, .d4 = 4, .d5 = 5, .d6 = 6, .d7 = 7, .bl = 3},
    .font = HD44780_FONT_5X8,
    .lines = CONFIG_LCD_NUM_ROWS,
    .backlight = true,
};

typedef struct {
    uint8_t x;
    uint8_t y;
    bool is_str;
    char str[21];
} lcd_UI_obj_t;

static i2c_dev_t pcf8574;
static hd44780_t lcd;

static TaskHandle_t lcd_task_handle = NULL;
static QueueHandle_t lcd_queue = NULL;
static TimerHandle_t lcd_sleep_timer = NULL;
static SemaphoreHandle_t lcd_mutex = NULL;

static uint8_t lcd_freeze_state = 0;
static bool lcd_enabled = false;

// Item history
static bool lcd_backlit_status = false;
static lcd_UI_obj_t menu_cursor_history = {.is_str = true, .str = {ICON_MENU_CURSOR}};
static bool cursor_state = true;
static bool schedule_state = true;
static bool schedule_method = true;
static bool connection_method = true;
static uint8_t menu_index = 0;
static time_t feeding_run_duration = 0;
static time_t feeding_pause_duration = 0;
static int32_t ap_connection_state = EFEEDER_DISCONNECTED_FROM_AP;
static char err_msg[21];
static char act_msg[21];
static char freeze_msg[21];
static char process_msg_1[21];
static char process_msg_2[21];
static char process_msg_3[21];

static const char* TAG = "lcd";

static void lcd_task(void* pvParameter) {
    lcd_UI_obj_t lcd_ui_obj;

    for (;;) {
        xQueueReceive(lcd_queue, &lcd_ui_obj, portMAX_DELAY);
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
        hd44780_gotoxy(&lcd, lcd_ui_obj.x, lcd_ui_obj.y);
        (lcd_ui_obj.is_str) ? hd44780_puts(&lcd, lcd_ui_obj.str) : hd44780_putc(&lcd, lcd_ui_obj.str[0]);
        xSemaphoreGive(lcd_mutex);
    }
    vTaskDelete(NULL);
}

// static void lcd_backlit_sleep(TimerHandle_t xTimer) {
//     lcd_set_backlit(0);
// }

static esp_err_t lcd_write_cb(const hd44780_t* lcd, uint8_t data) {
    return pcf8574_port_write(&pcf8574, data);
}

static esp_err_t lcd_init(uint8_t addr) {
    memset(&pcf8574, 0, sizeof(pcf8574));

    // memset(&pcf8574, 0, sizeof(i2c_dev_t));
    // pcf8574.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // pcf8574.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;

    ESP_ERROR_CHECK(pcf8574_init_desc(&pcf8574, addr, I2C_NUM_0, (gpio_num_t)CONFIG_I2C_SDA_GPIO, (gpio_num_t)CONFIG_I2C_SCL_GPIO));

    // Check if pcf8574 available
    uint8_t buf;
    esp_err_t res = pcf8574_port_read(&pcf8574, &buf);
    if (res != ESP_OK) {
        ESP_LOGW(TAG, "LCD driver 0x%02x not detected!", addr);
        ESP_ERROR_CHECK(pcf8574_free_desc(&pcf8574));
        return res;
    }

    lcd = lcd_temp;
    return hd44780_init(&lcd);
}

static bool lcd_detect_i2c(void) {
    if (lcd_init(CONFIG_PCF8574_I2C_ADDRESS) == ESP_OK) return true;
    // i2c address of lcd legacy
    if (lcd_init(0x20) == ESP_OK) return true;
    if (lcd_init(0x38) == ESP_OK) return true;
    return false;
}

void lcd_start(void) {
    if (!lcd_detect_i2c()) return;
    lcd_queue = xQueueCreate(CONFIG_LCD_OBJ_QUEUE_LEN, sizeof(lcd_UI_obj_t));
    // lcd_sleep_timer = xTimerCreate("Sleep LCD", pdMS_TO_TICKS(CONFIG_DEFAULT_LCD_SLEEP_PERIOD_MS), pdFALSE, (void*)5, lcd_backlit_sleep);

    lcd_mutex = xSemaphoreCreateMutex();
    // xTimerStart(lcd_sleep_timer, portMAX_DELAY);
    xTaskCreatePinnedToCore(lcd_task, "RenderLCD Task", CONFIG_LCD_RENDER_TASK_HEAP, NULL, 3, NULL, 1);
    lcd_set_backlit(1);

    lcd_enabled = true;
    for (uint8_t i = 0; i < CHAR_DATA_SIZE; i++) hd44780_upload_character(&lcd, i, char_data[i]);

    for (size_t i = 0; i < 16; i++)
    {
        lcd_clear();
        lcd_write(i, 0, "+");
        lcd_write(i, 1, "+");
        lcd_write(i, 2, "+");
        lcd_write(i, 3, "+");
        lcd_write(i+2, 0, "-");
        lcd_write(i+2, 1, "-");
        lcd_write(i+2, 2, "-");
        lcd_write(i+2, 3, "-");
        vTaskDelay(pdMS_TO_TICKS(200));
    } 
}

void lcd_main(const char* str0, const char* str1, const char* str2) 
{
    lcd_clear();
    lcd_write(0, 0, str0);
    lcd_write(0, 1, "--------------------");
    lcd_write(0, 2, str1);
    lcd_write(0, 3, str2);
}

void lcd_stop(void) {
    lcd_set_backlit(0);
    lcd_clear();
    ESP_ERROR_CHECK(pcf8574_free_desc(&pcf8574));

    lcd_enabled = false;

    if (lcd_sleep_timer != NULL) {
        xTimerDelete(lcd_sleep_timer, portMAX_DELAY);
        lcd_sleep_timer = NULL;
    }

    if (lcd_task_handle != NULL) {
        vTaskDelete(lcd_task_handle);
        lcd_task_handle = NULL;
    }

    if (lcd_mutex != NULL) {
        vSemaphoreDelete(lcd_mutex);
        lcd_mutex = NULL;
    }
}

void lcd_clear(void) {
    if (!lcd_enabled) return;
    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    hd44780_clear(&lcd);
    xSemaphoreGive(lcd_mutex);
}

void lcd_write(uint8_t x, uint8_t y, const char* str) {
    if (!lcd_enabled) return;
    lcd_UI_obj_t lcd_ui_obj = {
        .x = x,
        .y = y,
        .is_str = true,
    };
    strcpy(lcd_ui_obj.str, str);
    xQueueSend(lcd_queue, &lcd_ui_obj, portMAX_DELAY);
}

void lcd_write_icon(uint8_t x, uint8_t y, const uint8_t chr) {
    if (!lcd_enabled) return;
    lcd_UI_obj_t lcd_ui_obj = {
        .x = x,
        .y = y,
        .is_str = false,
        .str[0] = chr,
    };
    xQueueSend(lcd_queue, &lcd_ui_obj, portMAX_DELAY);
}

void lcd_erase_object(uint8_t x, uint8_t y, uint8_t len_to_erase) {
    if (!lcd_enabled) return;
    char blank[len_to_erase + 1];
    sprintf(blank, "%.*s", len_to_erase, "                    ");
    lcd_write(x, y, blank);
}

void lcd_set_backlit(uint8_t on) {
    if (!lcd_enabled) return;
    lcd_backlit_status = on;
    hd44780_switch_backlight(&lcd, on);
    if (on && (!lcd_freeze_state)) xTimerReset(lcd_sleep_timer, 10);
}

void lcd_set_menu_cursor(uint8_t x, uint8_t y, bool erase) {
    if (!erase) {
        lcd_write(x, y, menu_cursor_history.str);
        menu_cursor_history.x = x;
        menu_cursor_history.y = y;
        cursor_state = true;
    } else {
        lcd_erase_object(x, y, 1);
    }
}

void lcd_disable_menu_cursor(void) {
    cursor_state = false;
}

void lcd_set_item_cursor(uint8_t x, uint8_t y, int32_t item, uint8_t itemLen) {
    char msg[itemLen + 1];
    sprintf(msg, "%0*d", itemLen, item);

    lcd_erase_object(x, y, itemLen);
    vTaskDelay(pdMS_TO_TICKS(100));
    lcd_write(x, y, msg);
    vTaskDelay(pdMS_TO_TICKS(200));
}

bool lcd_is_frozen(void) {
    return lcd_freeze_state;
}

// esp_err_t lcd_refresh_screen(void) {
//     if (!lcd_enabled) return ESP_FAIL;
//     lcd = lcd_temp;
//     xQueueReset(lcd_queue);
//     xSemaphoreTake(lcd_mutex, portMAX_DELAY);
//     esp_err_t err = hd44780_init(&lcd);
//     for (uint8_t i = 0; i < CHAR_DATA_SIZE; i++) hd44780_upload_character(&lcd, i, char_data[i]);
//     xSemaphoreGive(lcd_mutex);

//     // time_t t = getTime();
//     if (lcd_freeze_state == 1) {
//         lcd_write(0, 0, err_msg);
//         lcd_write(0, 2, act_msg);
//         lcd_write(0, 1, freeze_msg);
//         // lcd_write_time(0, 3, t);
//         return err;
//     } else if (lcd_freeze_state == 2) {
//         lcd_clear();
//         lcd_write(0, 0, process_msg_1);
//         lcd_write(0, 1, process_msg_2);
//         lcd_write(0, 2, process_msg_3);
//         return err;
//     }

//     // Redisplay screen
//     lcd_write_time(0, 3, t);

//     lcd_set_connection_method_icon(connection_method);

//     if (ap_connection_state) {
//         lcd_set_AP_connection_state_icon();
//     }

//     if (menu_index) {
//         if (schedule_method) {
//             lcd_render_schedule_cnt_menu(feeding_run_duration, feeding_pause_duration);
//         } else {
//             lcd_render_schedule_bsc_menu();
//         }
//         lcd_write(3, 2, "Pengaturan timer ");
//     } else {
//         lcd_set_menu_index(0);

//         lcd_write(0, 0, " Jam:");
//         lcd_write(0, 1, " Tgl:");
//         lcd_write_time(5, 0, t);
//         lcd_write_date(5, 1, t);
//         lcd_write(15, 0, "T:");
//         char tz[5];
//         sprintf(tz, "%0*d", 3, getTimezoneOffset());
//         lcd_write(17, 0, tz);
//         lcd_erase_object(13, 0, 2);
//         lcd_erase_object(15, 1, 5);
//         lcd_write(3, 2, "Pengaturan jam   ");
//     }

//     if (schedule_method) {
//         lcd_write(13, 3, "CNT");
//     } else {
//         lcd_write(13, 3, "---");
//     }

//     if (cursor_state) {
//         lcd_set_menu_cursor(menu_cursor_history.x, menu_cursor_history.y, 0);
//     } else {
//         lcd_write(0, 2, "<>");
//     }

//     lcd_set_feeding_icon(false);
//     lcd_set_schedule_icon(schedule_state);

//     return err;
// }

// void lcd_render_welcome_screen(const char* feeder_name) {
//     lcd_clear();
//     lcd_write(1, 0, feeder_name);
//     lcd_write(0, 1, " Firmware Version");
//     lcd_write(1, 2, CONFIG_WIFI_INFO);
//     lcd_write(0, 3, " Mohon tunggu...    ");

//     // eventbus_register_ANY(lcd_event_loop);
// }

// void lcd_write_time(uint8_t x, uint8_t y, time_t t) {
//     char msg[9];
//     struct tm* timeinfo = localtime(&t);
//     strftime(msg, sizeof(msg), "%H:%M:%S", timeinfo);
//     lcd_write(x, y, msg);
// }

// void lcd_write_date(uint8_t x, uint8_t y, time_t t) {
//     char msg[11];
//     struct tm* timeinfo = localtime(&t);
//     strftime(msg, sizeof(msg), "%d/%m/%Y", timeinfo);
//     lcd_write(x, y, msg);
// }

// void lcd_set_feeding_icon(uint8_t stat) {
//     lcd_write_icon(19, 3, ICON_TANK_CAPACITY_50);
//     if (!stat) return;  // Animate icon if feeding
//     vTaskDelay(pdMS_TO_TICKS(300));
//     lcd_write_icon(19, 3, ICON_TANK_CAPACITY_20);
//     vTaskDelay(pdMS_TO_TICKS(200));
//     lcd_write_icon(19, 3, ICON_TANK_CAPACITY_0);
//     vTaskDelay(pdMS_TO_TICKS(100));
//     lcd_write_icon(19, 3, ICON_TANK_CAPACITY_100);
//     vTaskDelay(pdMS_TO_TICKS(400));
// }

// void lcd_set_schedule_icon(bool stat) {
//     uint8_t iconState = (stat) ? ICON_SCHEDULE_ON : ICON_SCHEDULE_OFF;
//     schedule_state = stat;
//     lcd_write_icon(17, 3, iconState);
// }

// void lcd_set_connection_method_icon(uint8_t method) {
//     const char* icon = (method) ? "SA" : "AP";
//     connection_method = method;
//     if (method) ap_connection_state = 0;
//     lcd_write(9, 3, icon);
// }

// void lcd_set_AP_connection_state_icon(void) {
//     switch (ap_connection_state) {
//         case EFEEDER_MQTT_CONNECTED:
//             lcd_write(9, 3, "AP>");
//             break;
//         case EFEEDER_DISCONNECTED_FROM_AP:
//             lcd_write(9, 3, "AP-");
//             break;
//         case EFEEDER_CONNECTED_TO_AP_NOT_INTERNET:
//             lcd_write(9, 3, "AP+");
//             break;
//         default:
//             return;
//     }
// }

// void lcd_render_schedule_cnt_menu(time_t run, time_t pause) {
//     feeding_run_duration = run;
//     feeding_pause_duration = pause;
//     char runT[21];
//     char pauseT[21];

//     sprintf(runT, "Jalan:%03lum.%02lus     ", ((((run) % 86400) / 3600) * 60) + (((run) / 60) % 60),
//             ((run) % 60));
//     sprintf(pauseT, "Jeda :%03lum.%02lus     ", ((((pause) % 86400) / 3600) * 60) + (((pause) / 60) % 60),
//             ((pause) % 60));
//     lcd_write(1, 0, runT);
//     lcd_write(1, 1, pauseT);
//     lcd_write(13, 3, "CNT");
//     schedule_method = true;
// }

// void lcd_render_schedule_adv_menu(void) {
//     lcd_write(1, 0, "Menggunakan jadwal ");
//     lcd_write(1, 1, "advance.           ");
//     lcd_write(13, 3, "ADV");
// }

// void lcd_render_schedule_bsc_menu(void) {
//     lcd_write(1, 0, "Menggunakan jadwal ");
//     lcd_write(1, 1, "dari apps.         ");
//     lcd_write(13, 3, "---");
//     schedule_method = false;
// }

// void lcd_render_motor_setting_menu(uint32_t dosingThres, uint32_t throwerThres) {
//     char dosingT[21];
//     char throwerT[21];

//     sprintf(dosingT, "ds_thres:%05d...mA", dosingThres);
//     sprintf(throwerT, "th_thres:%05d...mA", throwerThres);
//     lcd_write(1, 0, dosingT);
//     lcd_write(1, 1, throwerT);
// }

// void lcd_render_run_menu(uint8_t stat) {
//     lcd_set_menu_cursor(0, 1, 1);
//     lcd_write(1, 1, "[OK]");

//     if (stat) {
//         lcd_write(0, 0, "Feeder berjalan...  ");
//         lcd_write(5, 1, "Hentikan       ");
//     } else {
//         lcd_write(0, 0, "Feeder berhenti     ");
//         lcd_write(5, 1, "Jalankan       ");
//     }
// }

// void lcd_set_menu_index(uint8_t index) {
//     menu_index = index;
// }