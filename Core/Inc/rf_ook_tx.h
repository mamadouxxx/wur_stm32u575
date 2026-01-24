/**
 * @file rf_ook_tx.h
 * @author Mamadou
 * @date 19 Jan 2026
 * @brief RF 433 MHz OOK Transmitter module (TX)
 *
 * Provides functions to send bits
 * using On-Off Keying (OOK) modulation.
 */

#ifndef RF_OOK_TX_H_
#define RF_OOK_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "main.h"

typedef struct {
    rf_ook_frame_t frame;  // la trame à envoyer
    uint8_t byte_idx;
    int8_t  bit_idx;
    bool    active;
} rf_ook_tx_t;

/**
 * @brief Initialize the RF OOK transmitter module.
 *
 * Configures GPIOs and prepares the timer for microsecond delays.
 * By default, the antenna is set to RX mode.
 */
void rf_ook_tx_init(void);

/**
 * @brief Send a single OOK bit.
 *
 * @param bit     Bit value to send (0 or 1)
 */
void rf_ook_tx_send_bit(uint8_t bit);

/**
 * @brief End an OOK RF transmission.
 * After this call, the RF module is fully stopped and does not emit.
 *
 */
void rf_ook_end_tx();

/**
 * @brief Start an OOK RF transmission.
 *
 * This function enables the RF transmission path and powers on
 * the RF transmitter.
 *
 * After calling this function, the RF module is ready to transmit
 * OOK-modulated bits using the TX_DATA pin.
 *
 */
void rf_ook_tx_start_tx();

/**
 * @brief Callback function to send a single bit (for use with protocol layer).
 *
 * @param bit     Bit value to send (0 or 1)
 * @param bit_us  Duration of the bit in microseconds
 */
void rf_ook_tx_bit_callback(uint8_t bit);

/**
 * @brief Callback function for precise microsecond delay (for protocol layer)
 *
 * @param us  Delay in microseconds
 */
void rf_ook_tx_delay_callback(uint32_t us);


#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_TX_H_ */
