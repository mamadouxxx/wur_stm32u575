/*
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
 *
/**
 * @file error_handler.h
 * @brief Centralized firmware error handling module.
 *
 * This module provides a unified way to report warnings and critical
 * errors across the firmware. It is responsible for:
 *  - Logging errors (UART)
 *  - Visual indication (LED)
 *  - Notifying the system of fatal conditions
 *
 * This module does NOT decide recovery strategy (restart, degraded mode).
 * That responsibility belongs to the boot manager.
 */

#ifndef ERROR_HANDLER_H_
#define ERROR_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @enum ErrorCode_t
 * @brief Enumerates firmware error sources.
 */
typedef enum {
    ERR_NONE     = 0,   /**< No error */
    ERR_ADC      = 1,   /**< ADC failure */
    ERR_UART     = 2,   /**< UART failure */
    ERR_RF_TX    = 3,   /**< RF transmission failure */
    ERR_SENSOR   = 4,   /**< Sensor subsystem failure */
    ERR_UNKNOWN  = 255  /**< Unknown error */
} ErrorCode_t;

/**
 * @brief Reports a critical (fatal) firmware error.
 *
 * This function:
 *  - Logs the error through UART (if available)
 *  - Signals the error visually
 *  - Notifies the system that a fatal error occurred
 *
 * It does NOT block execution indefinitely.
 *
 * @param code  Error source identifier
 * @param msg   Optional human-readable message (can be NULL)
 */
void FW_Error_Handler(ErrorCode_t code, const char* msg);

/**
 * @brief Reports a non-fatal warning.
 *
 * This function logs the warning and signals it visually,
 * but allows normal execution to continue.
 *
 * @param code  Error source identifier
 * @param msg   Optional human-readable message (can be NULL)
 */
void Error_Warning(ErrorCode_t code, const char* msg);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_HANDLER_H_ */
