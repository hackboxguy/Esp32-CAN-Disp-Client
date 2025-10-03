/**
 * @file can_protocol.h
 * @brief CAN bus protocol definitions for multi-sensor node
 *
 * This file defines the CAN message IDs and data structures for
 * receiving sensor data from the multi-sensor CAN node.
 *
 * CAN Bus Configuration: 500 kbps
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#include <stdint.h>

// CAN Message IDs
#define CAN_ID_VEML7700_LUX     0x0A2   // VEML7700 Lux Data (1 Hz)
#define CAN_ID_BME680_ENV       0x0A3   // BME680 Environmental (0.33 Hz)
#define CAN_ID_BME680_IAQ       0x0A4   // BME680 Air Quality (0.33 Hz)
#define CAN_ID_MQ3_ALCOHOL      0x0A5   // MQ-3 Alcohol (1 Hz)
#define CAN_ID_LD2410_PRESENCE  0x0A6   // LD2410 Presence (10 Hz)
#define CAN_ID_SYSTEM_STATUS    0x0A7   // System Status (0.1 Hz)

// Message data lengths
#define CAN_MSG_LEN_STANDARD    8

// Status values
#define SENSOR_STATUS_OK        0x00
#define SENSOR_STATUS_ERROR     0x01

// IAQ accuracy levels
#define IAQ_ACCURACY_STABILIZING    0
#define IAQ_ACCURACY_LOW            1
#define IAQ_ACCURACY_MEDIUM         2
#define IAQ_ACCURACY_HIGH           3

/**
 * @brief VEML7700 Lux Data Message (0x0A2)
 * Update rate: 1 Hz
 *
 * NEW FORMAT: 3-byte lux value to support up to 16.7M lux (0-16,777,215)
 * Typical range: 0-120,000 lux (direct sunlight)
 */
typedef struct __attribute__((packed)) {
    uint8_t lux_low;        // Byte 0: Lux value bits 0-7
    uint8_t lux_mid;        // Byte 1: Lux value bits 8-15
    uint8_t lux_high;       // Byte 2: Lux value bits 16-23
    uint8_t status;         // Byte 3: Status (0x00=OK, 0x01=Error)
    uint8_t sequence;       // Byte 4: Sequence counter
    uint8_t config_idx;     // Byte 5: Config index (0-20)
    uint16_t checksum;      // Byte 6-7: Checksum
} can_msg_veml7700_t;

/**
 * @brief BME680 Environmental Data Message (0x0A3)
 * Update rate: 0.33 Hz
 */
typedef struct __attribute__((packed)) {
    int16_t temperature;    // Byte 0-1: Temperature (�C � 100), little-endian
    uint8_t humidity;       // Byte 2: Humidity (%RH, 0-100)
    uint16_t pressure;      // Byte 3-4: Pressure (hPa � 10), little-endian
    uint8_t status;         // Byte 5: Status
    uint16_t checksum;      // Byte 6-7: Checksum
} can_msg_bme680_env_t;

/**
 * @brief BME680 Air Quality Data Message (0x0A4)
 * Update rate: 0.33 Hz
 */
typedef struct __attribute__((packed)) {
    uint16_t iaq;           // Byte 0-1: IAQ (0-500), little-endian
    uint8_t accuracy;       // Byte 2: Accuracy (0-3)
    uint16_t co2_equiv;     // Byte 3-4: CO2 equivalent (ppm), little-endian
    uint16_t breath_voc;    // Byte 5-6: Breath VOC (ppm), little-endian
    uint8_t status;         // Byte 7: Status
} can_msg_bme680_iaq_t;

/**
 * @brief MQ-3 Alcohol Sensor Data Message (0x0A5)
 * Update rate: 1 Hz
 */
typedef struct __attribute__((packed)) {
    uint16_t raw_adc;       // Byte 0-1: Raw ADC value, little-endian
    uint16_t rs_r0_ratio;   // Byte 2-3: Rs/R0 ratio (� 1000), little-endian
    uint16_t ppm_estimate;  // Byte 4-5: PPM estimate, little-endian
    uint8_t status;         // Byte 6: Status
    uint8_t sequence;       // Byte 7: Sequence counter
} can_msg_mq3_alcohol_t;

/**
 * @brief LD2410 Presence Detection Message (0x0A6)
 * Update rate: 10 Hz
 */
typedef struct __attribute__((packed)) {
    uint8_t presence;       // Byte 0: Presence detected (0/1)
    uint16_t distance;      // Byte 1-2: Distance (cm), little-endian
    uint8_t move_energy;    // Byte 3: Movement energy
    uint8_t static_energy;  // Byte 4: Static energy
    uint8_t status;         // Byte 5: Status
    uint16_t checksum;      // Byte 6-7: Checksum
} can_msg_ld2410_presence_t;

/**
 * @brief System Status Message (0x0A7)
 * Update rate: 0.1 Hz
 */
typedef struct __attribute__((packed)) {
    uint8_t active_sensors; // Byte 0: Active sensors bitmask
    uint8_t free_heap_kb;   // Byte 1: Free heap (KB)
    uint32_t uptime_sec;    // Byte 2-5: Uptime (seconds), little-endian
    uint16_t sequence;      // Byte 6-7: Sequence counter
} can_msg_system_status_t;

// Active sensors bitmask flags
#define SENSOR_ACTIVE_VEML7700  (1 << 0)
#define SENSOR_ACTIVE_BME680    (1 << 1)
#define SENSOR_ACTIVE_MQ3       (1 << 2)
#define SENSOR_ACTIVE_LD2410    (1 << 3)

#endif // CAN_PROTOCOL_H
