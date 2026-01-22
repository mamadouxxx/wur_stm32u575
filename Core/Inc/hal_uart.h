/**
 * @file hal_uart.h
 * @author Mamadou
 * @date 17 Jan 2026
 * @brief Hardware Abstraction Layer (HAL) for UART
 *
 * Provides functions to transmit data over UART1.
 */

#ifndef HAL_UART_H_
#define HAL_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Transmit data via UART1
 *
 * Sends a buffer of bytes over UART1 using blocking HAL transmission.
 *
 * @param data Pointer to the data buffer
 * @param len Number of bytes to transmit
 */
void hal_uart1_write(uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H_ */
