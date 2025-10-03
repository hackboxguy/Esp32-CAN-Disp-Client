/**
 * @file display_driver.c
 * @brief SSD1306 display driver implementation (basic I2C version)
 *
 * This is a simple implementation using I2C directly.
 * For production, consider using u8g2 library for better font support.
 */

#include "display_driver.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "DISPLAY";

#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS   1000

// SSD1306 Commands
#define SSD1306_CMD_DISPLAY_OFF         0xAE
#define SSD1306_CMD_DISPLAY_ON          0xAF
#define SSD1306_CMD_SET_CONTRAST        0x81
#define SSD1306_CMD_NORMAL_DISPLAY      0xA6
#define SSD1306_CMD_SET_MUX_RATIO       0xA8
#define SSD1306_CMD_SET_DISPLAY_OFFSET  0xD3
#define SSD1306_CMD_SET_START_LINE      0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP   0xA1
#define SSD1306_CMD_SET_COM_SCAN_DEC    0xC8
#define SSD1306_CMD_SET_COM_PINS        0xDA
#define SSD1306_CMD_SET_PRECHARGE       0xD9
#define SSD1306_CMD_SET_VCOMH_DESELECT  0xDB
#define SSD1306_CMD_CHARGE_PUMP         0x8D
#define SSD1306_CMD_MEMORY_MODE         0x20
#define SSD1306_CMD_COLUMN_ADDR         0x21
#define SSD1306_CMD_PAGE_ADDR           0x22

static bool display_initialized = false;
static uint8_t display_buffer[DISPLAY_WIDTH * DISPLAY_HEIGHT / 8];

// Simple 5x7 font (ASCII 32-126)
static const uint8_t font_5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32: Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33: !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34: "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35: #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36: $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37: %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38: &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39: '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40: (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41: )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42: *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43: +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44: ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45: -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46: .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47: /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48: 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49: 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50: 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51: 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52: 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53: 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54: 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55: 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56: 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57: 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58: :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59: ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60: <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61: =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62: >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63: ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64: @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65: A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66: B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67: C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68: D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69: E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70: F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71: G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72: H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73: I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74: J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75: K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76: L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77: M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78: N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79: O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80: P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81: Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82: R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83: S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84: T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85: U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86: V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87: W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88: X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89: Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90: Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91: [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92: backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93: ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94: ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95: _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96: `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97: a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98: b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99: c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100: d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101: e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102: f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // 103: g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104: h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105: i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // 106: j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107: k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108: l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109: m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110: n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111: o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // 112: p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // 113: q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114: r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115: s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116: t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117: u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118: v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119: w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120: x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // 121: y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122: z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123: {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124: |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125: }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 126: ~
};

/**
 * @brief Check if I2C device is present at address
 */
static esp_err_t i2c_device_probe(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Write command to SSD1306
 */
static esp_err_t ssd1306_write_cmd(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd}; // 0x00 = command mode
    return i2c_master_write_to_device(I2C_MASTER_NUM, DISPLAY_I2C_ADDR,
                                      data, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

/**
 * @brief Write data to SSD1306
 */
static esp_err_t ssd1306_write_data(const uint8_t *data, size_t len) {
    uint8_t header = 0x40; // 0x40 = data mode

    // Allocate buffer for header + data
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;

    buf[0] = header;
    memcpy(&buf[1], data, len);

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, DISPLAY_I2C_ADDR,
                                                buf, len + 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    free(buf);
    return ret;
}

esp_err_t display_init(void) {
    ESP_LOGI(TAG, "Initializing SSD1306 display");
    ESP_LOGI(TAG, "I2C SDA: GPIO%d, SCL: GPIO%d, Addr: 0x%02X",
             DISPLAY_I2C_SDA_GPIO, DISPLAY_I2C_SCL_GPIO, DISPLAY_I2C_ADDR);

    // Configure I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = DISPLAY_I2C_SDA_GPIO,
        .scl_io_num = DISPLAY_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = DISPLAY_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for display to power up
    vTaskDelay(pdMS_TO_TICKS(100));

    // Check if SSD1306 is present on I2C bus
    ESP_LOGI(TAG, "Probing for SSD1306 at address 0x%02X...", DISPLAY_I2C_ADDR);
    ret = i2c_device_probe(DISPLAY_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SSD1306 not found at address 0x%02X: %s",
                 DISPLAY_I2C_ADDR, esp_err_to_name(ret));
        ESP_LOGE(TAG, "Please check display wiring and I2C connections");
        return ret;
    }
    ESP_LOGI(TAG, "SSD1306 detected successfully");

    // Initialize SSD1306 - Initialization sequence
    ret = ssd1306_write_cmd(SSD1306_CMD_DISPLAY_OFF);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send display off command: %s", esp_err_to_name(ret));
        return ret;
    }

    ssd1306_write_cmd(SSD1306_CMD_SET_MUX_RATIO);
    ssd1306_write_cmd(0x3F); // 64 lines
    ssd1306_write_cmd(SSD1306_CMD_SET_DISPLAY_OFFSET);
    ssd1306_write_cmd(0x00);
    ssd1306_write_cmd(SSD1306_CMD_SET_START_LINE | 0x00);
    ssd1306_write_cmd(SSD1306_CMD_SET_SEGMENT_REMAP);
    ssd1306_write_cmd(SSD1306_CMD_SET_COM_SCAN_DEC);
    ssd1306_write_cmd(SSD1306_CMD_SET_COM_PINS);
    ssd1306_write_cmd(0x12);
    ssd1306_write_cmd(SSD1306_CMD_SET_CONTRAST);
    ssd1306_write_cmd(0x7F);
    ssd1306_write_cmd(SSD1306_CMD_NORMAL_DISPLAY);
    ssd1306_write_cmd(SSD1306_CMD_CHARGE_PUMP);
    ssd1306_write_cmd(0x14); // Enable charge pump
    ssd1306_write_cmd(SSD1306_CMD_MEMORY_MODE);
    ssd1306_write_cmd(0x00); // Horizontal addressing mode

    ret = ssd1306_write_cmd(SSD1306_CMD_DISPLAY_ON);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send display on command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Clear display buffer
    memset(display_buffer, 0, sizeof(display_buffer));
    display_update();

    display_initialized = true;
    ESP_LOGI(TAG, "Display initialized successfully");
    return ESP_OK;
}

void display_clear(void) {
    if (!display_initialized) return;
    memset(display_buffer, 0, sizeof(display_buffer));
}

void display_update(void) {
    if (!display_initialized) return;

    // Set column and page address range
    ssd1306_write_cmd(SSD1306_CMD_COLUMN_ADDR);
    ssd1306_write_cmd(0);                   // Start column
    ssd1306_write_cmd(DISPLAY_WIDTH - 1);   // End column
    ssd1306_write_cmd(SSD1306_CMD_PAGE_ADDR);
    ssd1306_write_cmd(0);                   // Start page
    ssd1306_write_cmd(7);                   // End page (64/8 = 8 pages)

    // Write buffer to display
    ssd1306_write_data(display_buffer, sizeof(display_buffer));
}

void display_draw_string(uint8_t x, uint8_t y, const char *str, display_font_size_t font) {
    if (!display_initialized || !str) return;

    uint8_t scale = (font == FONT_LARGE) ? 2 : 1;

    while (*str) {
        char c = *str++;

        // Only handle printable ASCII (32-126)
        if (c < ' ' || c > '~') {
            c = ' ';
        }

        const uint8_t *glyph = font_5x7[c - ' '];

        // Draw character
        for (int col = 0; col < 5; col++) {
            for (int s = 0; s < scale; s++) {
                uint8_t px = x + col * scale + s;
                if (px >= DISPLAY_WIDTH) break;

                for (int row = 0; row < 8; row++) {
                    if (glyph[col] & (1 << row)) {
                        for (int sy = 0; sy < scale; sy++) {
                            uint8_t py = y + row * scale + sy;
                            if (py < DISPLAY_HEIGHT) {
                                display_buffer[px + (py / 8) * DISPLAY_WIDTH] |= (1 << (py % 8));
                            }
                        }
                    }
                }
            }
        }

        x += 6 * scale; // Character width + spacing
    }
}

void display_draw_printf(uint8_t x, uint8_t y, display_font_size_t font, const char *format, ...) {
    if (!display_initialized) return;

    char buffer[64];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    display_draw_string(x, y, buffer, font);
}

void display_set_brightness(uint8_t brightness) {
    if (!display_initialized) return;
    ssd1306_write_cmd(SSD1306_CMD_SET_CONTRAST);
    ssd1306_write_cmd(brightness);
}

bool display_is_initialized(void) {
    return display_initialized;
}
