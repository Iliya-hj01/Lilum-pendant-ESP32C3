#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sketch.h"
#include "simple_BLE.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";
uint8_t animationNumber = 0; // Global variable for animation selection

void ledTask(void *pvParameter) {
    ESP_LOGI(TAG, "LED Task started");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // uint8_t* animNum = (uint8_t*)pvParameter; // Cast parameter to pointer

    sketchSetup();
    while (1) {
        // Your LED control code here
        sketchLoop(); // Call the function without parameters
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(16)); // Delay for 16 millisecond from xLastWakeTime, which is approximately 60 FPS
    }
}

void printTask(void *pvParameter) {
    ESP_LOGI(TAG, "Print Task started");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        ESP_LOGI(TAG, "Simple message from print task");
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "app starting");
    
    initArduino();
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
    ble_init();
    ESP_LOGI(TAG, "Starting LED Task");
    xTaskCreate(ledTask, // Task function
         "LED Task", // Name of the task
        4096, // Stack size in bytes
        &animationNumber, // Pass pointer to global variable
        1, // Task priority
        NULL // Task handle
    );
    ESP_LOGI(TAG, "Starting Print Task");
    xTaskCreate(printTask, // Task function
         "Print Task", // Name of the task
        2048, // Stack size in bytes
        NULL, // Parameter to pass to the task
        1, // Task priority
        NULL // Task handle
    );
}

