/**
 * @file rf_ook_tx.h
 * @author Mamadou
 * @date 19 Jan 2026
 * @brief RF 433 MHz OOK Transmitter module (TX)
 *
 * Provides functions to transmit bits using On-Off Keying (OOK)
 * modulation and manages frame-level TX states.
 */

#ifndef RF_OOK_TX_H_
#define RF_OOK_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "rf_ook_types.h"
#include <stdbool.h>
#include "main.h"

#define WAKEUP_DELAY_US   10000

/** @brief Time to wait after SYNC before sending address (in ticks of BIT_US) */
#define TX_WAIT_TICKS  (WAKEUP_DELAY_US / BIT_US)

/**
 * @brief Initialize the RF OOK transmitter module.
 *
 * Configures GPIOs and prepares the RF hardware.
 * By default, the antenna is set to RX mode.
 */
void rf_ook_tx_init(void);

/**
 * @brief Start an OOK RF transmission.
 *
 * Enables the RF TX path and powers on the transmitter.
 * After calling this, bits can be sent using rf_ook_tx_send_bit().
 */
void rf_ook_tx_start_tx(void);

/**
 * @brief End an OOK RF transmission.
 *
 * Stops any ongoing transmission and powers down the RF TX path.
 */
void rf_ook_end_tx(void);

/**
 * @brief Send a single OOK bit using the TX state machine.
 *
 * The function automatically selects the correct bit to send
 * depending on the current TX state (SYNC, WAIT, ADDRESS, PAYLOAD).
 *
 * @param frame Pointer to the TX frame context
 */
void rf_ook_tx_send_bit(rf_ook_tx_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_TX_H_ */
