/**
 * @file main.c
 * @brief ESP32-C3 CAN Display Client - Main Application
 *
 * This application receives sensor data from a CAN bus network
 * and displays it on a 128x64 SSD1306 OLED display.
 *
 * Hardware:
 * - ESP32-C3 SuperMini
 * - SSD1306 OLED (128x64, I2C, 0x3C)
 * - TJA1050 CAN transceiver
 *
 * Pin Configuration:
 * - CAN TX: GPIO4
 * - CAN RX: GPIO5
 * - I2C SDA: GPIO6
 * - I2C SCL: GPIO7
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "sensor_data.h"
#include "can_receiver.h"
#include "display_driver.h"
#include "display_ui.h"

static const char *TAG = "MAIN";

// Global sensor data instance
sensor_readings_t g_sensor_data = {0};

/**
 * @brief Initialize sensor data structure
 */
bool sensor_data_init(void) {
    // Create mutex for thread-safe access
    g_sensor_data.mutex = xSemaphoreCreateMutex();
    if (g_sensor_data.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor data mutex");
        return false;
    }

    // Initialize all timestamps to 0 (will be marked stale)
    memset(g_sensor_data.last_update_ms, 0, sizeof(g_sensor_data.last_update_ms));

    ESP_LOGI(TAG, "Sensor data structure initialized");
    return true;
}

/**
 * @brief Check if sensor data is stale
 */
bool sensor_data_is_stale(sensor_index_t sensor_idx) {
    if (sensor_idx >= SENSOR_IDX_COUNT) return true;

    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t last_update = g_sensor_data.last_update_ms[sensor_idx];

    // Check if never updated (0) or timeout exceeded
    if (last_update == 0) return true;

    return (now - last_update) > SENSOR_DATA_STALE_TIMEOUT_MS;
}

/**
 * @brief Lock sensor data mutex
 */
bool sensor_data_lock(uint32_t timeout_ms) {
    return xSemaphoreTake(g_sensor_data.mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

/**
 * @brief Unlock sensor data mutex
 */
void sensor_data_unlock(void) {
    xSemaphoreGive(g_sensor_data.mutex);
}

void app_main(void) {
    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "ESP32-C3 CAN Display Client");
    ESP_LOGI(TAG, "===========================================");

    // Initialize NVS (required for some ESP-IDF components)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Step 1: Initialize sensor data structure
    ESP_LOGI(TAG, "Step 1/4: Initializing sensor data structure");
    if (!sensor_data_init()) {
        ESP_LOGE(TAG, "Failed to initialize sensor data");
        return;
    }

    // Step 2: Initialize CAN receiver
    ESP_LOGI(TAG, "Step 2/4: Initializing CAN receiver");
    ret = can_receiver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN receiver: %s", esp_err_to_name(ret));
        return;
    }

    // Step 3: Initialize display
    ESP_LOGI(TAG, "Step 3/4: Initializing display");
    ret = display_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize display: %s", esp_err_to_name(ret));
        // Continue anyway - we can still receive CAN data
    }

    // Step 4: Start tasks
    ESP_LOGI(TAG, "Step 4/4: Starting tasks");

    ret = can_receiver_start_task();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CAN receiver task");
        return;
    }

    if (display_is_initialized()) {
        ret = display_ui_start_task();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start display UI task");
            // Continue anyway - CAN receiver will still work
        }
    }

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "Initialization complete!");
    ESP_LOGI(TAG, "Waiting for CAN messages...");
    ESP_LOGI(TAG, "===========================================");

    // Main loop - monitor system health
    uint32_t stats_counter = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // 10 seconds

        stats_counter++;
        if (stats_counter % 6 == 0) { // Every minute
            ESP_LOGI(TAG, "System running - Free heap: %lu bytes", esp_get_free_heap_size());

            // Log sensor data status
            if (sensor_data_lock(pdMS_TO_TICKS(100))) {
                ESP_LOGI(TAG, "Sensor status: VEML=%s, BME_ENV=%s, BME_IAQ=%s, MQ3=%s, LD2410=%s",
                         sensor_data_is_stale(SENSOR_IDX_VEML7700) ? "STALE" : "OK",
                         sensor_data_is_stale(SENSOR_IDX_BME680_ENV) ? "STALE" : "OK",
                         sensor_data_is_stale(SENSOR_IDX_BME680_IAQ) ? "STALE" : "OK",
                         sensor_data_is_stale(SENSOR_IDX_MQ3_ALCOHOL) ? "STALE" : "OK",
                         sensor_data_is_stale(SENSOR_IDX_LD2410_PRESENCE) ? "STALE" : "OK");
                sensor_data_unlock();
            }
        }
    }
}
