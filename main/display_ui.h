/**
 * @file display_ui.h
 * @brief Display UI rendering and layout
 *
 * This module handles the visual layout and rendering of sensor data
 * on the SSD1306 display.
 */

#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include "esp_err.h"

// Task configuration
#define DISPLAY_UI_TASK_STACK_SIZE  4096
#define DISPLAY_UI_TASK_PRIORITY    4
#define DISPLAY_UPDATE_RATE_HZ      10

/**
 * @brief Start the display UI update task
 *
 * Creates the FreeRTOS task that periodically renders the UI
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_ui_start_task(void);

/**
 * @brief Display UI update task function
 *
 * This task runs at DISPLAY_UPDATE_RATE_HZ, reading sensor data
 * and rendering it to the display
 *
 * @param pvParameters Task parameters (unused)
 */
void display_update_task(void *pvParameters);

/**
 * @brief Render the main sensor display screen
 *
 * Layout:
 * - Top: Large lux value
 * - Middle: Temperature, Humidity, Pressure
 * - Bottom: IAQ and Alcohol level
 */
void display_ui_render_main_screen(void);

/**
 * @brief Render presence detection screen (alternative view)
 */
void display_ui_render_presence_screen(void);

#endif // DISPLAY_UI_H
