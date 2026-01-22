/*
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
 *
/**
 * @file error_handler.c
 * @brief Firmware error handling implementation.
 */

#include "error_handler.h"
#include "hal_uart.h"
#include "main.h"
#include <stdio.h>

/* Weak hook for system-level fatal error handling (boot manager) */
__attribute__((weak))
void system_fatal_error_notify(ErrorCode_t code)
{
    (void)code;
}

/**
 * @brief Blink error LED a given number of times.
 *
 * @param times      Number of blinks
 * @param delay_ms   Delay between toggles in milliseconds
 */
static void blink_led(uint8_t times, uint16_t delay_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
        HAL_Delay(delay_ms);
        HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
        HAL_Delay(delay_ms);
    }
}

void FW_Error_Handler(ErrorCode_t code, const char* msg)
{
    __disable_irq();

    /* Log critical error */
    if (msg != NULL) {
        uint8_t buf[128];
        int len = snprintf((char*)buf, sizeof(buf),
                           "CRITICAL ERROR [%u]: %s\r\n",
                           (unsigned)code, msg);
        if (len > 0) {
            hal_uart1_write(buf, (uint16_t)len);
        }
    }

    /* Visual indication: fast blinking */
    blink_led(3, 100);

    /* Notify system (boot manager, watchdog logic, etc.) */
    system_fatal_error_notify(code);

    __enable_irq();
}

void Error_Warning(ErrorCode_t code, const char* msg)
{
    /* Log warning */
    if (msg != NULL) {
        uint8_t buf[128];
        int len = snprintf((char*)buf, sizeof(buf),
                           "WARNING [%u]: %s\r\n",
                           (unsigned)code, msg);
        if (len > 0) {
            hal_uart1_write(buf, (uint16_t)len);
        }
    }

    /* Visual indication: single blink */
    blink_led(1, 150);
}
