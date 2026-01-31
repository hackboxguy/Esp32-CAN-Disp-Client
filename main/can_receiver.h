/**
 * @file can_receiver.h
 * @brief CAN bus receiver and message parser
 *
 * This module handles initialization of the TWAI (CAN) driver,
 * receives CAN messages, and parses them into the shared sensor data structure.
 */

#ifndef CAN_RECEIVER_H
#define CAN_RECEIVER_H

#include <stdbool.h>
#include "esp_err.h"

// CAN Bus Configuration
#define CAN_TX_GPIO             5
#define CAN_RX_GPIO             4
#define CAN_BITRATE             500000  // 500 kbps

// Task configuration
#define CAN_RX_TASK_STACK_SIZE  4096
#define CAN_RX_TASK_PRIORITY    5

/**
 * @brief Initialize the CAN receiver
 *
 * Configures and starts the TWAI driver for receiving CAN messages
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_receiver_init(void);

/**
 * @brief Start the CAN receiver task
 *
 * Creates the FreeRTOS task that continuously receives and parses CAN messages
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t can_receiver_start_task(void);

/**
 * @brief CAN receiver task function
 *
 * This task runs continuously, receiving CAN messages and updating
 * the global sensor data structure
 *
 * @param pvParameters Task parameters (unused)
 */
void can_receive_task(void *pvParameters);

#endif // CAN_RECEIVER_H
