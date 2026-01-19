/*
 * hal_uart.c
 *
 *  Created on: 17 janv. 2026
 *      Author: mamadou
 */

#ifndef HAL_UART_
#define HAL_UART_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

void hal_uart1_write(uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_C_ */
