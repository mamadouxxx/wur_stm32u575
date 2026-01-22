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
 * @param bit_us  Duration of the bit in microseconds
 */
void rf_ook_tx_send_bit(uint8_t bit, uint32_t bit_us);

/**
 * @brief Send a full OOK frame.
 *
 * The frame includes:
 * - Preamble (alternating 1s and 0s)
 * - Node address
 * - Payload
 *
 * @param address       bit node address
 * @param payload       Pointer to the payload data
 * @param payload_bits  Size of the payload in bits
 * @param bit_us        Duration of each bit in microseconds
 */
void rf_ook_tx_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_bits, uint32_t bit_us);

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
 * @brief Quick test function to send a short frame for verification.
 *
 * Can be used to check the signal on an oscilloscope.
 */
void rf_ook_tx_send_test(void);

/**
 * @brief Callback function to send a single bit (for use with protocol layer).
 *
 * @param bit     Bit value to send (0 or 1)
 * @param bit_us  Duration of the bit in microseconds
 */
void rf_ook_tx_bit_callback(uint8_t bit, uint32_t bit_us);

/**
 * @brief Callback function for precise microsecond delay (for protocol layer)
 *
 * @param us  Delay in microseconds
 */
void rf_ook_tx_delay_callback(uint32_t us);

/**
 * @brief Turn on the RF transmitter
 *
 * Sets the TX enable pin HIGH.
 */
void tx_on(void);

/**
 * @brief Turn off the RF transmitter
 *
 * Sets the TX enable pin LOW.
 */
void tx_off(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_TX_H_ */
