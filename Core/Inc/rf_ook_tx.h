/*
 * rf_ook_tx.h
 *
 *  Created on: 19 janv. 2026
 *      Author: mamadou
 */

#ifndef RF_OOK_TX_H_
#define RF_OOK_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "main.h"

// Préambule & adresse
#define PREAMBLE_BITS 16
#define ADDRESS_BITS 4

void rf_ook_tx_init(void);
void rf_ook_tx_send_bit(uint8_t bit, uint32_t bit_us);
void rf_ook_tx_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_bits, uint32_t bit_us);
void rf_ook_tx_send_test(void);


#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_TX_H_ */
