/**
 * @file rf_ook_rx.h
 * @author Mamadou
 * @date 21 Jan 2026
 * @brief RF 433 MHz OOK Receiver module (RX)
 *
 * Provides a simple FSM to decode OOK-modulated frames
 * (address + payload) and store them in a circular buffer.
 */

#ifndef RF_OOK_RX_H_
#define RF_OOK_RX_H_

#include <stdint.h>
#include <stdbool.h>
#include "rf_ook_types.h"

/**
 * @brief Initialize the OOK RX module
 *
 * Resets the RX FSM state and clears the RX buffer.
 */
void rf_ook_rx_init(void);

/**
 * @brief Reset the RX FSM
 *
 * Clears internal counters and prepares the module to receive a new frame.
 */
void rf_ook_rx_reset(void);

/**
 * @brief Inject a received bit into the RX FSM
 *
 * This function should be called from an ISR or physical RX module
 * whenever a new bit is received. The FSM automatically updates
 * the address and payload buffers.
 *
 * @param bit The received bit (0 or 1)
 */
void rf_ook_rx_receive_bit(uint8_t bit);

/**
 * @brief Read a received frame from the circular RX buffer
 *
 * If a frame is available, it is copied to `frame` and removed from the buffer.
 *
 * @param frame Pointer to an `rf_ook_frame_t` struct to store the received frame
 * @return 1 if a frame was available and copied, 0 if buffer is empty
 */
uint8_t rf_ook_rx_get_frame(rf_ook_frame_t *frame);

/**
 * @brief Check if a frame is ready in the RX buffer
 *
 * @return true if at least one frame is available, false otherwise
 */
bool rf_ook_rx_is_frame_ready(void);

/**
 * @brief Clear the frame ready flag
 *
 * Call this after processing the frame to reset the notification.
 */
void rf_ook_rx_clear_frame_ready(void);

#endif /* RF_OOK_RX_H_ */
