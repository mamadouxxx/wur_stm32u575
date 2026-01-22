/**
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
 *
 * @file boot_manager.c
 * @brief Boot and recovery logic implementation.
 */

#include "boot_manager.h"
#include "error_handler.h"
#include "main.h"

/* Configuration */
#define MAX_CONSECUTIVE_FAILURES  3

/* Internal state */
static uint8_t  restart_count = 0;
static BootMode_t current_mode = BOOT_MODE_NORMAL;

void boot_manager_init(void)
{
    restart_count++;

    if (restart_count >= MAX_CONSECUTIVE_FAILURES) {
        current_mode = BOOT_MODE_DEGRADED;
    } else {
        current_mode = BOOT_MODE_NORMAL;
    }
}

void boot_manager_notify_fatal_error(void)
{
    restart_count++;

    if (restart_count >= MAX_CONSECUTIVE_FAILURES) {
        current_mode = BOOT_MODE_SAFE;
    }

    /* Trigger controlled reset */
    NVIC_SystemReset();
}

BootMode_t boot_manager_get_mode(void)
{
    return current_mode;
}

bool boot_manager_is_degraded(void)
{
    return (current_mode == BOOT_MODE_DEGRADED);
}
