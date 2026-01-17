/*
 * hal_uart.c
 *
 *  Created on: 17 janv. 2026
 *      Author: mamadou
 */


/* Includes ------------------------------------------------------------------*/
#include "hal_uart.h"
#include "stm32u5xx_hal.h"


extern UART_HandleTypeDef huart1;

/**
 *
 */
void hal_uart1_write(uint8_t* data, uint16_t len)
{
    HAL_UART_Transmit(&huart1, data, len, HAL_MAX_DELAY);
}

