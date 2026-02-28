/**
 * @file    hal_uart.h
 * @author  Mamadou
 * @date    17 jan 2026
 * @brief   Couche d'abstraction matérielle (HAL) pour l'UART
 *
 * Fournit les fonctions de transmission de données via UART1.
 */

#ifndef HAL_UART_H_
#define HAL_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Transmet des données via UART1
 *
 * Envoie un tampon d'octets sur UART1 en mode bloquant.
 *
 * @param data  Pointeur vers le tampon de données
 * @param len   Nombre d'octets à transmettre
 */
void hal_uart1_write(uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* HAL_UART_H_ */
