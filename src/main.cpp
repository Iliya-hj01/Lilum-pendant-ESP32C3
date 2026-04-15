#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_engine.h"
#include "simple_BLE.h"
#include "button_service.h"
#include "mic_service.h"
#include "IMU_service.h"
#include "battery_service.h"
#include "orchestra_shared_config.h"
#include "logging_config.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";
uint8_t animationNumber = 0; // Global variable for animation selection

// ── Gesture → white-mode toggle (mirrors button double-tap) ──
static bool     s_gesture_saved_manual_mode    = false;
static uint8_t  s_gesture_saved_manual_pattern = 0;
static uint8_t  s_gesture_saved_brightness     = LED_BRIGHTNESS_MAX;

static void gesture_enter_white_mode(void)
{
    s_gesture_saved_manual_mode    = g_manual_mode_active;
    s_gesture_saved_manual_pattern = g_manual_pattern;
    s_gesture_saved_brightness     = workingBrightness;
    led_engine_set_white_mode(true);
}

static void gesture_exit_white_mode(void)
{
    led_engine_set_white_mode(false);
    g_manual_mode_active = s_gesture_saved_manual_mode;
    g_manual_pattern     = s_gesture_saved_manual_pattern;
    workingBrightness    = s_gesture_saved_brightness;
}

void ledTask(void *pvParameter) {
    ESP_LOGI(TAG, "LED Task started");
    TickType_t xLastWakeTime = xTaskGetTickCount();

    led_engine_setup();
    while (1) {
        // Check for double-dip gesture → toggle white mode
        if (imu_gesture_double_dip()) {
            if (led_engine_is_white_mode()) {
                gesture_exit_white_mode();
            } else {
                gesture_enter_white_mode();
            }
        }

        led_engine_loop();
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(16));
    }
}

void printTask(void *pvParameter) {
    ESP_LOGI(TAG, "Print Task started");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        extern uint8_t g_animation_mode; // from led_engine

        // Build the log line dynamically based on logging_config.h
        char buf[256];
        int pos = 0;

        if (LOG_ENABLE_MAIN) {
            if (ORCHESTRA_ROLE == OrchestraRole::Master) {
                pos += snprintf(buf + pos, sizeof(buf) - pos, "Master | Anim: %u", g_animation_mode);
            } else {
                pos += snprintf(buf + pos, sizeof(buf) - pos, "Follower | Anim: %u", g_animation_mode);
            }
        }

        if (LOG_ENABLE_BLE) {
            if (ORCHESTRA_ROLE == OrchestraRole::Master) {
                pos += snprintf(buf + pos, sizeof(buf) - pos, " | Adv: %s",
                                ble_is_currently_advertising() ? "Y" : "N");
            } else {
                pos += snprintf(buf + pos, sizeof(buf) - pos, " | Sync: %s | Scan: %s | NextScan: %lu ms",
                                ble_is_follower_synced() ? "Y" : "N",
                                ble_is_currently_scanning() ? "Y" : "N",
                                (unsigned long)ble_ms_until_next_scan());
            }
        }

        if (LOG_ENABLE_MIC) {
            uint8_t vol = mic_get_volume();
            bool beat = mic_get_beat();
            uint16_t bpm = mic_get_bpm();
            pos += snprintf(buf + pos, sizeof(buf) - pos, " | Vol: %3u | BPM: %3u%s",
                            vol, bpm, beat ? " <BEAT>" : "");
        }

        if (LOG_ENABLE_IMU) {
            float ax, ay, az;
            imu_get_accel_mg(&ax, &ay, &az);
            pos += snprintf(buf + pos, sizeof(buf) - pos, " | XX:%+7.0f YY:%+7.0f ZZ:%+7.0f mg",
                            ax, ay, az);
        }

        if (LOG_ENABLE_GESTURE) {
            bool dip = imu_gesture_double_dip();
            pos += snprintf(buf + pos, sizeof(buf) - pos, " | G:%-7s%s",
                            imu_gesture_state_str(), dip ? " <<DOUBLE DIP>>" : "");
        }

        if (LOG_ENABLE_BATTERY) {
            float temp = battery_get_temperature();
            pos += snprintf(buf + pos, sizeof(buf) - pos,
                            " | Bat:%3d%% %s T:%.1f°C%s%s",
                            battery_get_level(),
                            battery_is_full() ? "FULL" : (battery_is_charging() ? "CHG" : "DIS"),
                            temp,
                            battery_temp_in_range() ? "" : " !TEMP",
                            battery_key_short_press() ? " <KEY>" : "");
        }

        ESP_LOGI(TAG, "%s", buf);
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LOG_INTERVAL_MS));
    }
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(100));  // Give serial a moment to initialize
    ESP_LOGI(TAG, ">>> APP MAIN START <<<");
    
    ESP_LOGI(TAG, "Calling initArduino...");
    initArduino();
    ESP_LOGI(TAG, "initArduino done");
    int rc;

    // BLE stack depends on NVS being initialized first.
    // If you already do this in app_main(), remove these two blocks from here.
    ESP_LOGI(TAG, "Initializing NVS");
    rc = nvs_flash_init();
    if (rc == ESP_ERR_NVS_NO_FREE_PAGES || rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(rc);
    ESP_LOGI(TAG, "NVS initialized");
    
    ESP_LOGI(TAG, "Calling ble_init...");
    ble_init();
    ble_set_logging(false);
    if (!LOG_ENABLE_BLE_STACK) {
        esp_log_level_set("NimBLE", ESP_LOG_WARN);
        esp_log_level_set("BLE_INIT", ESP_LOG_WARN);
    }
    ESP_LOGI(TAG, "ble_init completed");
    
    ESP_LOGI(TAG, "Starting LED Task");
    xTaskCreate(ledTask, // Task function
         "LED Task", // Name of the task
        4096, // Stack size in bytes
        &animationNumber, // Pass pointer to global variable
        1, // Task priority
        NULL // Task handle
    );

    ESP_LOGI(TAG, "Starting IMU Service");
    imu_service_init();

    ESP_LOGI(TAG, "Starting Mic Service");
    mic_service_init();

    ESP_LOGI(TAG, "Starting Button Service");
    button_service_init();

    ESP_LOGI(TAG, "Starting Battery Service");
    battery_service_init();

    ESP_LOGI(TAG, "Starting Print Task");
    xTaskCreate(printTask, // Task function
         "Print Task", // Name of the task
        3072, // Stack size in bytes
        NULL, // Parameter to pass to the task
        1, // Task priority
        NULL // Task handle
    );
}

