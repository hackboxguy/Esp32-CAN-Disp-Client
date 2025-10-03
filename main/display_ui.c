/**
 * @file display_ui.c
 * @brief Display UI rendering implementation
 */

#include "display_ui.h"
#include "display_driver.h"
#include "sensor_data.h"

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DISPLAY_UI";

/**
 * @brief Get IAQ quality description
 */
static const char* get_iaq_description(uint16_t iaq) {
    if (iaq <= 50) return "Exc";       // Excellent
    if (iaq <= 100) return "Good";
    if (iaq <= 150) return "Fair";
    if (iaq <= 200) return "Poor";
    if (iaq <= 300) return "Bad";
    return "Severe";
}

void display_ui_render_main_screen(void) {
    display_clear();

    if (!sensor_data_lock(pdMS_TO_TICKS(50))) {
        ESP_LOGW(TAG, "Failed to lock sensor data");
        display_draw_string(10, 28, "DATA LOCKED", FONT_SMALL);
        display_update();
        return;
    }

    // ===== TOP SECTION: Lux Value (Large) =====
    if (!sensor_data_is_stale(SENSOR_IDX_VEML7700)) {
        char lux_str[16];
        if (g_sensor_data.lux < 1000) {
            // 0-999: Show as-is (e.g., "500 lux")
            snprintf(lux_str, sizeof(lux_str), "%.0f", g_sensor_data.lux);
        } else if (g_sensor_data.lux < 10000) {
            // 1000-9999: Show with 1 decimal (e.g., "5.2k lux")
            snprintf(lux_str, sizeof(lux_str), "%.1fk", g_sensor_data.lux / 1000.0f);
        } else if (g_sensor_data.lux < 100000) {
            // 10000-99999: Show as integer k (e.g., "50k lux", "90k lux")
            snprintf(lux_str, sizeof(lux_str), "%.0fk", g_sensor_data.lux / 1000.0f);
        } else {
            // 100000+: Show as integer k (e.g., "130k lux")
            snprintf(lux_str, sizeof(lux_str), "%.0fk", g_sensor_data.lux / 1000.0f);
        }
        display_draw_printf(2, 2, FONT_LARGE, "%s lux", lux_str);
    } else {
        display_draw_string(2, 2, "--- lux", FONT_LARGE);
    }

    // ===== MIDDLE SECTION: Temp, Humidity, Pressure (3 columns) =====
    uint8_t mid_y = 22;

    if (!sensor_data_is_stale(SENSOR_IDX_BME680_ENV)) {
        // Temperature
        display_draw_printf(0, mid_y, FONT_SMALL, "%.1fC", g_sensor_data.temperature);

        // Humidity
        display_draw_printf(42, mid_y, FONT_SMALL, "%.0f%%", g_sensor_data.humidity);

        // Pressure
        display_draw_printf(70, mid_y, FONT_SMALL, "%.0fhPa", g_sensor_data.pressure);
    } else {
        display_draw_string(10, mid_y, "--- C  ---%  ---hPa", FONT_SMALL);
    }

    // ===== BOTTOM SECTION: IAQ and Alcohol =====
    uint8_t bot_y = 36;

    // IAQ with quality indicator
    if (!sensor_data_is_stale(SENSOR_IDX_BME680_IAQ)) {
        const char *quality = get_iaq_description(g_sensor_data.iaq);
        display_draw_printf(0, bot_y, FONT_SMALL, "IAQ:%d %s", g_sensor_data.iaq, quality);
    } else {
        display_draw_string(0, bot_y, "IAQ: ---", FONT_SMALL);
    }

    // Alcohol level
    if (!sensor_data_is_stale(SENSOR_IDX_MQ3_ALCOHOL)) {
        display_draw_printf(0, bot_y + 10, FONT_SMALL, "Alc:%dppm", g_sensor_data.alcohol_ppm);

        // Alert if high alcohol detected (e.g., > 100 ppm)
        if (g_sensor_data.alcohol_ppm > 100) {
            display_draw_string(70, bot_y + 10, "ALERT!", FONT_SMALL);
        }
    } else {
        display_draw_string(0, bot_y + 10, "Alc: ---", FONT_SMALL);
    }

    // ===== STATUS LINE: Presence and staleness indicators =====
    uint8_t status_y = 56;

    // Presence indicator
    if (!sensor_data_is_stale(SENSOR_IDX_LD2410_PRESENCE)) {
        if (g_sensor_data.presence_detected) {
            display_draw_printf(0, status_y, FONT_SMALL, "P:%dcm", g_sensor_data.presence_distance);
        } else {
            display_draw_string(0, status_y, "P:None", FONT_SMALL);
        }
    }

    // CAN staleness warning - only check sensors that have sent at least one message
    bool any_stale = false;
    for (int i = 0; i < SENSOR_IDX_COUNT; i++) {
        // Skip sensors that have never sent a message (timestamp = 0)
        if (g_sensor_data.last_update_ms[i] > 0 && sensor_data_is_stale(i)) {
            any_stale = true;
            break;
        }
    }

    if (any_stale) {
        display_draw_string(90, status_y, "STALE", FONT_SMALL);
    }

    sensor_data_unlock();
    display_update();
}

void display_ui_render_presence_screen(void) {
    display_clear();

    if (!sensor_data_lock(pdMS_TO_TICKS(50))) {
        display_draw_string(10, 28, "DATA LOCKED", FONT_SMALL);
        display_update();
        return;
    }

    display_draw_string(10, 0, "PRESENCE DETAILS", FONT_SMALL);

    if (!sensor_data_is_stale(SENSOR_IDX_LD2410_PRESENCE)) {
        display_draw_printf(0, 12, FONT_SMALL, "Status: %s",
                          g_sensor_data.presence_detected ? "Detected" : "None");
        display_draw_printf(0, 22, FONT_SMALL, "Distance: %d cm", g_sensor_data.presence_distance);
        display_draw_printf(0, 32, FONT_SMALL, "Move Energy: %d", g_sensor_data.move_energy);
        display_draw_printf(0, 42, FONT_SMALL, "Static Energy: %d", g_sensor_data.static_energy);
    } else {
        display_draw_string(20, 28, "NO DATA", FONT_SMALL);
    }

    sensor_data_unlock();
    display_update();
}

void display_update_task(void *pvParameters) {
    ESP_LOGI(TAG, "Display update task started (rate: %d Hz)", DISPLAY_UPDATE_RATE_HZ);

    const TickType_t update_period = pdMS_TO_TICKS(1000 / DISPLAY_UPDATE_RATE_HZ);
    TickType_t last_wake_time = xTaskGetTickCount();

    // For now, always show main screen
    // In future, could add button to toggle between screens
    bool show_main_screen = true;

    while (1) {
        if (show_main_screen) {
            display_ui_render_main_screen();
        } else {
            display_ui_render_presence_screen();
        }

        vTaskDelayUntil(&last_wake_time, update_period);
    }
}

esp_err_t display_ui_start_task(void) {
    BaseType_t ret = xTaskCreate(
        display_update_task,
        "display_ui",
        DISPLAY_UI_TASK_STACK_SIZE,
        NULL,
        DISPLAY_UI_TASK_PRIORITY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create display update task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Display UI task created");
    return ESP_OK;
}
