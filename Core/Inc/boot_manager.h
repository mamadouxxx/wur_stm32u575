/*
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
 *
 * @file boot_manager.h
 * @brief Firmware boot and recovery manager.
 *
 * This module is responsible for:
 *  - Tracking consecutive boot failures
 *  - Detecting reboot loops
 *  - Selecting normal or degraded execution mode
 *  - Deciding recovery strategy after critical errors
 *
 * It is designed for low-power autonomous systems.
 */

#ifndef BOOT_MANAGER_H_
#define BOOT_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/**
 * @enum BootMode_t
 * @brief Current firmware execution mode.
 */
typedef enum {
    BOOT_MODE_NORMAL = 0,    /**< Full functionality */
    BOOT_MODE_DEGRADED,      /**< Reduced functionality */
    BOOT_MODE_SAFE           /**< Minimal safe mode */
} BootMode_t;

/**
 * @brief Initialize the boot manager.
 *
 * Must be called very early in main().
 * Evaluates restart history and selects execution mode.
 */
void boot_manager_init(void);

/**
 * @brief Notify the boot manager of a fatal error.
 *
 * This function is called by the error handler when
 * a critical error occurs.
 */
void boot_manager_notify_fatal_error(void);

/**
 * @brief Get the current boot mode.
 *
 * @return Current system execution mode.
 */
BootMode_t boot_manager_get_mode(void);

/**
 * @brief Indicates whether the system is in degraded mode.
 *
 * @return true if degraded mode is active
 */
bool boot_manager_is_degraded(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_MANAGER_H_ */
