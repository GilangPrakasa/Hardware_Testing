#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void lcd_start(void);
void lcd_stop(void);

void lcd_clear();
void lcd_write(uint8_t x, uint8_t y, const char* str);
void lcd_write_icon(uint8_t x, uint8_t y, uint8_t chr);
void lcd_erase_object(uint8_t x, uint8_t y, uint8_t len_to_erase);
void lcd_set_backlit(uint8_t on);
void lcd_set_menu_cursor(uint8_t x, uint8_t y, bool erase);
void lcd_disable_menu_cursor(void);
void lcd_set_item_cursor(uint8_t x, uint8_t y, int32_t item, uint8_t itemLen);
bool lcd_is_frozen(void);
esp_err_t lcd_refresh_screen(void);

void lcd_main(const char* str0, const char* str1, const char* str2);
/* Menu components */
void lcd_render_welcome_screen(const char* feeder_name);
// void lcd_write_time(uint8_t x, uint8_t y, time_t t);
// void lcd_write_date(uint8_t x, uint8_t y, time_t t);
// void lcd_set_feeding_icon(uint8_t stat);
// void lcd_set_schedule_icon(bool stat);
// void lcd_set_connection_method_icon(uint8_t method);
// void lcd_set_AP_connection_state_icon(void);

// /* Feeder Menu */
// void lcd_render_schedule_cnt_menu(time_t run, time_t pause);
// void lcd_render_schedule_adv_menu(void);
// void lcd_render_schedule_bsc_menu(void);
// void lcd_render_motor_setting_menu(uint32_t dosingThres, uint32_t throwerThres);
// void lcd_render_run_menu(uint8_t stat);
// void lcd_set_menu_index(uint8_t index);

#ifdef __cplusplus
}
#endif