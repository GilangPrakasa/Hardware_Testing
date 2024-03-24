#ifndef FEEDER_STATE_H_
#define FEEDER_STATE_H_

#include <esp_event.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// NONE or Type 0
// * Feeder generic events
ESP_EVENT_DECLARE_BASE(NONE_TYPE_EVENT);
enum feeder_general_events {
    EFEEDER_OK = 000,
    EFEEDER_FAIL,
    EFEEDER_NONE_EVENT_LIMIT
};

// ERROR or Type 1
// * System critical error events
ESP_EVENT_DECLARE_BASE(ERROR_EVENT);
enum feeder_critical_error_events {
    EFEEDER_DRIVER_ERROR = 100,
    EFEEDER_THROWER_ERROR,
    EFEEDER_DOSING_ERROR,
    EFEEDER_UNDER_VOLTAGE,
    EFEEDER_OVER_VOLTAGE,
    EFEEDER_STORAGE_ERROR,
    EFEEDER_RTC_ERROR,
    EFEEDER_DATE_INVALID,
    EFEEDER_NO_FEED_FLOW,
    EFEEDER_THROWER_NOT_DETECTED,
    EFEEDER_DOSING_NOT_DETECTED,
    EFEEDER_FOTA_FAILED,
    EFEEDER_NO_CRITICAL_ERROR,
    EFEEDER_ERROR_EVENT_LIMIT
};

// WARN or Type 2
ESP_EVENT_DECLARE_BASE(WARNING_EVENT);

enum feeder_warning_events {
    EFEEDER_FW_UPGRADE_FAIL = 200,
    EFEEDER_NORFLASH_ERASE,
    EFEEDER_TRANSFER_FAIL,
    EFEEDER_ATMEGA_PING_FAILED,
    EFEEDER_STORAGE_CORRUPTED,
    EFEEDER_IDENTITY_ERASED,
    EFEEDER_WARNING_EVENT_LIMIT
};

// INFO or Type 3
ESP_EVENT_DECLARE_BASE(FEEDING_EVENT);
ESP_EVENT_DECLARE_BASE(SYSTEM_EVENT);
ESP_EVENT_DECLARE_BASE(INTERACTION_EVENT);

// * Feeding events
enum feeder_feeding_events {
    EFEEDER_IDLE = 300,
    EFEEDER_STARTING,
    EFEEDER_STOPPING,
    EFEEDER_FEEDING,
    EFEEDER_FEEDING_DONE,
    EFEEDER_CLEANING_DONE,
    EFEEDER_FEEDING_EVENT_LIMIT,
};

// * System events
enum feeder_system_events {
    EFEEDER_BOOTING = 310,
    EFEEDER_BOOTING_SUCCESS,
    EFEEDER_LOAD_APP,
    EFEEDER_FAIL_SAFE_MODE,

    EFEEDER_PROVISIONING,
    EFEEDER_WAITING_TO_BE_PROVISIONED,
    EFEEDER_SWITCH_TO_STATION,
    EFEEDER_SWITCH_TO_AP,

    EFEEDER_CONNECTED_TO_AP_NOT_INTERNET,
    EFEEDER_DISCONNECTED_FROM_AP,

    EFEEDER_FW_UPGRADE_READY,
    EFEEDER_FW_UPGRADING,
    EFEEDER_FW_UPGRADE_SUCCESS,

    EFEEDER_FEEDLOG_READY_TO_SENT,
    EFEEDER_SYSLOG_READY_TO_SENT,

    EFEEDER_POWER_OFF,
    EFEEDER_MQTT_CONNECTED,

    EFEEDER_FEEDLOG_PRODUCED,

    EFEEDER_SCHEDULE_START,
    EFEEDER_TEST_RUN_START,
    EFEEDER_CALIBRATE_FLOWRATE_START,
    EFEEDER_SET_FEEDING_STOP,

    EFEEDER_PUBLISHING_MQTT,

    EFEEDER_ATMEGA_PING,
    EFEEDER_ATMEGA_SERIAL,

    EFEEDER_DEVICE_CONFIG_UPDATED,
    EFEEDER_PUBLISHING_MQTT_FAILED,

    EFEEDER_TIME_SYNC_HAS_OFFSET,

    EFEEDER_RELOAD_SCHEDULE,

    EFEEDER_SYSTEM_EVENT_LIMIT
};

// * interaction
enum feeder_interaction_events {
    EFEEDER_TIME_GET_HTTP_REQUEST = 350,
    EFEEDER_TIME_GET_HTTP_RESPONSE,
    EFEEDER_TIME_POST_HTTP_REQUEST,
    EFEEDER_TIME_POST_HTTP_RESPONSE,

    EFEEDER_SETTING_GET_HTTP_REQUEST,
    EFEEDER_SETTING_GET_HTTP_RESPONSE,
    EFEEDER_SETTING_POST_HTTP_REQUEST,
    EFEEDER_SETTING_POST_HTTP_RESPONSE,

    EFEEDER_SCHEDULES_GET_HTTP_REQUEST,
    EFEEDER_SCHEDULES_GET_HTTP_RESPONSE,
    EFEEDER_SCHEDULES_POST_HTTP_REQUEST,
    EFEEDER_SCHEDULES_POST_HTTP_RESPONSE,

    EFEEDER_TEST_RUN_HTTP_REQUEST,
    EFEEDER_TEST_RUN_HTTP_RESPONSE,

    EFEEDER_CALIBRATE_POST_HTTP_REQUEST,
    EFEEDER_CALIBRATE_POST_HTTP_RESPONSE,

    EFEEDER_CURRENT_SENSOR_GET_HTTP_REQUEST,
    EFEEDER_CURRENT_SENSOR_GET_HTTP_RESPONSE,
    EFEEDER_CURRENT_SENSOR_POST_HTTP_REQUEST,
    EFEEDER_CURRENT_SENSOR_POST_HTTP_RESPONSE,

    EFEEDER_INFO_GET_HTTP_REQUEST,
    EFEEDER_INFO_GET_HTTP_RESPONSE,

    EFEEDER_DOWNLOAD_FEEDLOG,

    EFEEDER_CURRENT_SENSOR_GET_VIA_WEBBROWSER,
    EFEEDER_CURRENT_SENSOR_POST_VIA_WEBBROWSER,

    EFEEDER_RUN_BUTTON_PRESSED,
    EFEEDER_RUN_BUTTON_LONG_PRESSED,
    EFEEDER_DISP_ENTER_BUTTON_PRESSED,
    EFEEDER_DISP_MINUS_BUTTON_PRESSED,
    EFEEDER_DISP_PLUS_BUTTON_PRESSED,
    EFEEDER_DISP_MENU_BUTTON_PRESSED,

    EFEEDER_INTERACTION_LIMIT
};

// DEBUG or Type 4
// * Network activity events
ESP_EVENT_DECLARE_BASE(DEBUG_EVENT);
enum feeder_debug_events {
    EFEEDER_BL_CONNECTED = 400,
    EFEEDER_BL_DISCONNECTED,
    EFEEDER_BL_RECEIVING,
    EFEEDER_BL_SENDING,
    EFEEDER_WIFI_CONNECTED,
    EFEEDER_WIFI_DISCONNECTED,
    EFEEDER_WIFI_RECEIVING,
    EFEEDER_WIFI_SENDING,
    EFEEDER_TRANSFER_SUCCESS,
    EFEEDER_DEBUG_EVENT_LIMIT,
};

// VBRBOSE or Type 5
ESP_EVENT_DECLARE_BASE(VERBOSE_EVENT);
#define EFEEDER_VERBOSE_EVENT_LIMIT 599

// ERROR FIXED or Type 6
ESP_EVENT_DECLARE_BASE(ERROR_FIXED_EVENT);
enum feeder_critical_error_fixed_events {
    EFEEDER_DRIVER_ERROR_FIXED = 600,
    EFEEDER_THROWER_ERROR_FIXED,
    EFEEDER_DOSING_ERROR_FIXED,
    EFEEDER_UNDER_VOLTAGE_FIXED,
    EFEEDER_OVER_VOLTAGE_FIXED,
    EFEEDER_STORAGE_ERROR_FIXED,
    EFEEDER_RTC_ERROR_FIXED,
    EFEEDER_DATE_INVALID_FIXED,
    EFEEDER_NO_FEED_FLOW_FIXED,
    EFEEDER_THROWER_NOT_DETECTED_FIXED,
    EFEEDER_DOSING_NOT_DETECTED_FIXED,
    EFEEDER_ERROR_FIXED_EVENT_LIMIT,
};

typedef int32_t cobox_event_t;

typedef struct {
    uint32_t name;
    const char* str;
} EventStrMap_t;

const char* event_to_str(int type, int desc);
int32_t event_to_id(int type, int desc);

esp_err_t eventbus_init(void);
esp_err_t eventbus_post(const char* tag, cobox_event_t event, const char* message);
void eventbus_register_EVENT(esp_event_base_t event_base, int32_t event_id, esp_event_handler_t handler);
void eventbus_register_ANY(esp_event_handler_t handler);
void eventbus_register_INFO(esp_event_handler_t handler);

extern esp_event_loop_handle_t eventbus;

#ifdef __cplusplus
}
#endif

#endif  // FEEDER_STATE_H_