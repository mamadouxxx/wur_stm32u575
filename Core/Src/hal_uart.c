/**
 * @file hal_uart.c
 * @author Mamadou
 * @date 17 Jan 2026
 * @brief Implementation of HAL UART functions
 *
 * Provides UART1 transmission using STM32 HAL functions.
 */

#include "hal_uart.h"
#include "stm32u5xx_hal.h"

extern UART_HandleTypeDef huart1;

/**
 * @brief Transmit data via UART1
 *
 * This function sends `len` bytes pointed by `data` over UART1 in blocking mode.
 *
 * @param data Pointer to the data buffer
 * @param len Number of bytes to transmit
 */
void hal_uart1_write(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, 100);
}
