/**
 * @file can_receiver.c
 * @brief CAN bus receiver implementation
 */

#include "can_receiver.h"
#include "can_protocol.h"
#include "sensor_data.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"

static const char *TAG = "CAN_RX";

// TWAI configuration
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = CAN_TX_GPIO,
    .rx_io = CAN_RX_GPIO,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 20,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0,
    .intr_flags = ESP_INTR_FLAG_LEVEL1
};

/**
 * @brief Calculate simple checksum for verification
 */
static uint16_t calculate_checksum(const uint8_t *data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief Parse VEML7700 lux message
 */
static void parse_veml7700_msg(const uint8_t *data) {
    can_msg_veml7700_t *msg = (can_msg_veml7700_t *)data;

    // Verify checksum (checksum is calculated on bytes 0-5)
    uint16_t calc_checksum = calculate_checksum(data, 6);
    if (calc_checksum != msg->checksum) {
        ESP_LOGW(TAG, "VEML7700 checksum mismatch: calc=0x%04X, recv=0x%04X",
                 calc_checksum, msg->checksum);
        return;
    }

    // Decode 3-byte lux value (24-bit, little-endian)
    uint32_t lux_value = msg->lux_low |
                         (msg->lux_mid << 8) |
                         (msg->lux_high << 16);

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.lux = (float)lux_value;
        g_sensor_data.lux_status = msg->status;
        g_sensor_data.last_update_ms[SENSOR_IDX_VEML7700] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "VEML7700: Lux=%.0f, Status=%d, Seq=%d",
                 (float)lux_value, msg->status, msg->sequence);
    }
}

/**
 * @brief Parse BME680 environmental message
 */
static void parse_bme680_env_msg(const uint8_t *data) {
    can_msg_bme680_env_t *msg = (can_msg_bme680_env_t *)data;

    // Verify checksum
    uint16_t calc_checksum = calculate_checksum(data, 6);
    if (calc_checksum != msg->checksum) {
        ESP_LOGW(TAG, "BME680 ENV checksum mismatch");
        return;
    }

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.temperature = (float)msg->temperature / 100.0f;
        g_sensor_data.humidity = (float)msg->humidity;
        g_sensor_data.pressure = (float)msg->pressure / 10.0f;
        g_sensor_data.env_status = msg->status;
        g_sensor_data.last_update_ms[SENSOR_IDX_BME680_ENV] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "BME680 ENV: T=%.2f�C, H=%d%%, P=%.1fhPa",
                 g_sensor_data.temperature, msg->humidity, g_sensor_data.pressure);
    }
}

/**
 * @brief Parse BME680 air quality message
 */
static void parse_bme680_iaq_msg(const uint8_t *data) {
    can_msg_bme680_iaq_t *msg = (can_msg_bme680_iaq_t *)data;

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.iaq = msg->iaq;
        g_sensor_data.iaq_accuracy = msg->accuracy;
        g_sensor_data.co2_equiv = msg->co2_equiv;
        g_sensor_data.breath_voc = msg->breath_voc;
        g_sensor_data.iaq_status = msg->status;
        g_sensor_data.last_update_ms[SENSOR_IDX_BME680_IAQ] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "BME680 IAQ: IAQ=%d, Acc=%d, CO2=%dppm, VOC=%dppm",
                 msg->iaq, msg->accuracy, msg->co2_equiv, msg->breath_voc);
    }
}

/**
 * @brief Parse MQ-3 alcohol message
 */
static void parse_mq3_alcohol_msg(const uint8_t *data) {
    can_msg_mq3_alcohol_t *msg = (can_msg_mq3_alcohol_t *)data;

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.alcohol_raw_adc = msg->raw_adc;
        g_sensor_data.alcohol_rs_r0 = (float)msg->rs_r0_ratio / 1000.0f;
        g_sensor_data.alcohol_ppm = msg->ppm_estimate;
        g_sensor_data.alcohol_status = msg->status;
        g_sensor_data.last_update_ms[SENSOR_IDX_MQ3_ALCOHOL] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "MQ3: ADC=%d, Rs/R0=%.3f, PPM=%d",
                 msg->raw_adc, g_sensor_data.alcohol_rs_r0, msg->ppm_estimate);
    }
}

/**
 * @brief Parse LD2410 presence message
 */
static void parse_ld2410_presence_msg(const uint8_t *data) {
    can_msg_ld2410_presence_t *msg = (can_msg_ld2410_presence_t *)data;

    // Verify checksum
    uint16_t calc_checksum = calculate_checksum(data, 6);
    if (calc_checksum != msg->checksum) {
        ESP_LOGW(TAG, "LD2410 checksum mismatch");
        return;
    }

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.presence_detected = (msg->presence != 0);
        g_sensor_data.presence_distance = msg->distance;
        g_sensor_data.move_energy = msg->move_energy;
        g_sensor_data.static_energy = msg->static_energy;
        g_sensor_data.presence_status = msg->status;
        g_sensor_data.last_update_ms[SENSOR_IDX_LD2410_PRESENCE] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "LD2410: Present=%d, Dist=%dcm, Move=%d, Static=%d",
                 msg->presence, msg->distance, msg->move_energy, msg->static_energy);
    }
}

/**
 * @brief Parse system status message
 */
static void parse_system_status_msg(const uint8_t *data) {
    can_msg_system_status_t *msg = (can_msg_system_status_t *)data;

    if (sensor_data_lock(pdMS_TO_TICKS(100))) {
        g_sensor_data.active_sensors_mask = msg->active_sensors;
        g_sensor_data.free_heap_kb = msg->free_heap_kb;
        g_sensor_data.uptime_sec = msg->uptime_sec;
        g_sensor_data.last_update_ms[SENSOR_IDX_SYSTEM_STATUS] = xTaskGetTickCount() * portTICK_PERIOD_MS;
        sensor_data_unlock();

        ESP_LOGD(TAG, "System: Active=0x%02X, Heap=%dKB, Uptime=%lus",
                 msg->active_sensors, msg->free_heap_kb, msg->uptime_sec);
    }
}

/**
 * @brief Process received CAN message
 */
static void process_can_message(const twai_message_t *message) {
    // Only process standard format messages with correct length
    if (message->extd || message->rtr || message->data_length_code != CAN_MSG_LEN_STANDARD) {
        return;
    }

    //ESP_LOGI(TAG, "RX frame ID=0x%03lX DLC=%d", message->identifier, message->data_length_code);

    switch (message->identifier) {
        case CAN_ID_VEML7700_LUX:
            parse_veml7700_msg(message->data);
            break;

        case CAN_ID_BME680_ENV:
            parse_bme680_env_msg(message->data);
            break;

        case CAN_ID_BME680_IAQ:
            parse_bme680_iaq_msg(message->data);
            break;

        case CAN_ID_MQ3_ALCOHOL:
            parse_mq3_alcohol_msg(message->data);
            break;

        case CAN_ID_LD2410_PRESENCE:
            parse_ld2410_presence_msg(message->data);
            break;

        case CAN_ID_SYSTEM_STATUS:
            parse_system_status_msg(message->data);
            break;

        default:
            ESP_LOGD(TAG, "Unknown CAN ID: 0x%03lX", message->identifier);
            break;
    }
}

esp_err_t can_receiver_init(void) {
    ESP_LOGI(TAG, "Initializing TWAI (CAN) driver");
    ESP_LOGI(TAG, "TX: GPIO%d, RX: GPIO%d, Bitrate: %d bps",
             CAN_TX_GPIO, CAN_RX_GPIO, CAN_BITRATE);

    // Install TWAI driver
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start TWAI driver
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }

    ESP_LOGI(TAG, "TWAI driver started successfully");
    return ESP_OK;
}

void can_receive_task(void *pvParameters) {
    ESP_LOGI(TAG, "CAN receive task started");

    twai_message_t rx_msg;
    uint32_t no_msg_count = 0;

    while (1) {
        // Wait for message with timeout
        esp_err_t ret = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));

        if (ret == ESP_OK) {
            no_msg_count = 0;
            process_can_message(&rx_msg);
        } else if (ret == ESP_ERR_TIMEOUT) {
            no_msg_count++;
            if (no_msg_count % 10 == 0) {
                ESP_LOGW(TAG, "No CAN messages received for %lu seconds", no_msg_count);
            }
        } else {
            ESP_LOGE(TAG, "TWAI receive error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

esp_err_t can_receiver_start_task(void) {
    BaseType_t ret = xTaskCreate(
        can_receive_task,
        "can_rx",
        CAN_RX_TASK_STACK_SIZE,
        NULL,
        CAN_RX_TASK_PRIORITY,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN receive task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CAN receive task created");
    return ESP_OK;
}
