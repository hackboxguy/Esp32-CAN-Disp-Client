/**
 * @file sensor_data.h
 * @brief Shared sensor data structures and state management
 *
 * This file defines the shared data structure that holds the latest
 * sensor readings received from the CAN bus. The CAN receiver task
 * updates this structure, and the display task reads from it.
 */

#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Timeout for considering data stale (milliseconds)
#define SENSOR_DATA_STALE_TIMEOUT_MS    5000

// Sensor indices for tracking updates
typedef enum {
    SENSOR_IDX_VEML7700 = 0,
    SENSOR_IDX_BME680_ENV,
    SENSOR_IDX_BME680_IAQ,
    SENSOR_IDX_MQ3_ALCOHOL,
    SENSOR_IDX_LD2410_PRESENCE,
    SENSOR_IDX_SYSTEM_STATUS,
    SENSOR_IDX_COUNT
} sensor_index_t;

/**
 * @brief Main sensor readings structure
 *
 * This structure holds the latest decoded sensor values from all
 * CAN messages. Access must be protected by the mutex.
 */
typedef struct {
    // VEML7700 Light Sensor
    float lux;
    uint8_t lux_status;

    // BME680 Environmental
    float temperature;          // °C
    float humidity;             // %RH
    float pressure;             // hPa
    uint8_t env_status;

    // BME680 Air Quality
    uint16_t iaq;               // Indoor Air Quality (0-500)
    uint8_t iaq_accuracy;       // 0-3
    uint16_t co2_equiv;         // ppm
    uint16_t breath_voc;        // ppm
    uint8_t iaq_status;

    // MQ-3 Alcohol
    uint16_t alcohol_raw_adc;
    float alcohol_rs_r0;        // Rs/R0 ratio
    uint16_t alcohol_ppm;
    uint8_t alcohol_status;

    // LD2410 Presence
    bool presence_detected;
    uint16_t presence_distance; // cm
    uint8_t move_energy;
    uint8_t static_energy;
    uint8_t presence_status;

    // System Status
    uint8_t active_sensors_mask;
    uint8_t free_heap_kb;
    uint32_t uptime_sec;

    // Staleness tracking (timestamp of last update for each sensor)
    uint32_t last_update_ms[SENSOR_IDX_COUNT];

    // Mutex for thread-safe access
    SemaphoreHandle_t mutex;

} sensor_readings_t;

// Global sensor data instance
extern sensor_readings_t g_sensor_data;

/**
 * @brief Initialize the sensor data structure
 *
 * Must be called before any access to g_sensor_data
 * Creates the mutex for thread-safe access
 *
 * @return true if initialization successful, false otherwise
 */
bool sensor_data_init(void);

/**
 * @brief Check if sensor data is stale
 *
 * @param sensor_idx Sensor index to check
 * @return true if data is older than SENSOR_DATA_STALE_TIMEOUT_MS
 */
bool sensor_data_is_stale(sensor_index_t sensor_idx);

/**
 * @brief Lock the sensor data mutex
 *
 * Must be called before reading/writing sensor data
 *
 * @param timeout_ms Timeout in milliseconds (use portMAX_DELAY for infinite)
 * @return true if lock acquired, false on timeout
 */
bool sensor_data_lock(uint32_t timeout_ms);

/**
 * @brief Unlock the sensor data mutex
 */
void sensor_data_unlock(void);

#endif // SENSOR_DATA_H
