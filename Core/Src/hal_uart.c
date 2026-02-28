/**
 * @file    hal_uart.c
 * @author  Mamadou
 * @date    17 jan 2026
 * @brief   Implémentation des fonctions HAL UART
 *
 * Fournit la transmission UART1 via les fonctions HAL STM32.
 */

#include "hal_uart.h"
#include "stm32u5xx_hal.h"

extern UART_HandleTypeDef huart1;

void hal_uart1_write(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, 100);
}
