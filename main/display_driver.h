/**
 * @file display_driver.h
 * @brief SSD1306 OLED display driver wrapper
 *
 * This module provides initialization and basic drawing functions
 * for the SSD1306 128x64 OLED display over I2C.
 */

#ifndef DISPLAY_DRIVER_H
#define DISPLAY_DRIVER_H

#include <stdbool.h>
#include "esp_err.h"

// Display Configuration
#define DISPLAY_I2C_SDA_GPIO    6
#define DISPLAY_I2C_SCL_GPIO    7
#define DISPLAY_I2C_ADDR        0x3C
#define DISPLAY_I2C_FREQ_HZ     400000  // 400 kHz

#define DISPLAY_WIDTH           128
#define DISPLAY_HEIGHT          64

// Font sizes (for future use with u8g2)
typedef enum {
    FONT_SMALL = 0,
    FONT_MEDIUM,
    FONT_LARGE
} display_font_size_t;

/**
 * @brief Initialize the display driver
 *
 * Initializes I2C bus and SSD1306 display
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t display_init(void);

/**
 * @brief Clear the display buffer
 */
void display_clear(void);

/**
 * @brief Update the display (send buffer to screen)
 */
void display_update(void);

/**
 * @brief Draw a string at specified position
 *
 * @param x X coordinate (0-127)
 * @param y Y coordinate (0-63)
 * @param str String to draw
 * @param font Font size to use
 */
void display_draw_string(uint8_t x, uint8_t y, const char *str, display_font_size_t font);

/**
 * @brief Draw a formatted string at specified position
 *
 * @param x X coordinate
 * @param y Y coordinate
 * @param font Font size to use
 * @param format Printf-style format string
 * @param ... Variable arguments
 */
void display_draw_printf(uint8_t x, uint8_t y, display_font_size_t font, const char *format, ...);

/**
 * @brief Set display brightness
 *
 * @param brightness Brightness level (0-255)
 */
void display_set_brightness(uint8_t brightness);

/**
 * @brief Check if display is initialized
 *
 * @return true if initialized, false otherwise
 */
bool display_is_initialized(void);

#endif // DISPLAY_DRIVER_H
