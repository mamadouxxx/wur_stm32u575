/**
 * @file rf_ook_proto.h
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Application layer for OOK 433 MHz protocol
 *
 * Provides functions to send OOK frames (preamble + address + payload)
 * using callbacks for bit-level transmission and microsecond delays.
 */

#ifndef RF_OOK_PROTO_H_
#define RF_OOK_PROTO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Callback type to send a single OOK bit
 *
 * @param bit      Bit value to send (0 or 1)
 * @param bit_us   Duration of the bit in microseconds
 */
typedef void (*rf_ook_tx_bit_cb_t)(uint8_t bit);

/**
 * @brief Callback type for microsecond delay
 *
 * @param us Delay in microseconds
 */
typedef void (*rf_ook_delay_cb_t)(uint32_t us);

/**
 * @brief Initialization structure for rf_ook_proto
 */
typedef struct {
    rf_ook_tx_bit_cb_t tx_send_bit_cb; /**< Callback to send a single bit */
    rf_ook_delay_cb_t  delay_cb;       /**< Callback for microsecond delay */
    uint32_t bitrate;                   /**< Bitrate in bits per second (optional, 0 = default) */
} rf_ook_proto_init_t;

/**
 * @brief Initialize the protocol layer with user callbacks.
 *
 * Sets the bit and delay callbacks, and configures the bitrate.
 * Must be called before sending frames.
 *
 * @param init Pointer to initialization structure
 */
void init(rf_ook_proto_init_t *init);

/**
 * @brief Initialize the full protocol module with default TX/RX callbacks
 *
 * Calls rf_ook_tx_init() and sets default callbacks for bit sending
 * using rf_ook_tx_bit_callback.
 */
void rf_ook_proto_init(void);

/**
 * @brief Send a full OOK frame (preamble + address + payload)
 *
 * @param address       bit node address
 * @param payload       Pointer to payload data
 * @param payload_bytes  Number bytes of payload
 */
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_len_bytes);

/**
 *
 */
void rf_ook_proto_handle_received_frame(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_PROTO_H_ */
