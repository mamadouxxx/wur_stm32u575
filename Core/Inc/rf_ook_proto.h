/**
 * @file rf_ook_proto.h
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Application layer for OOK 433 MHz protocol.
 *
 * Provides functions to send and receive OOK frames (SYNC + address + payload)
 * using callbacks for bit-level transmission and microsecond delays.
 * This layer abstracts the hardware layer from the application.
 */

#ifndef RF_OOK_PROTO_H_
#define RF_OOK_PROTO_H_

#include <stdint.h>
#include "rf_ook_types.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the OOK protocol module.
 *
 * Sets up TX and RX modules and prepares the protocol layer
 * for sending and receiving frames.
 */
void rf_ook_proto_init(void);

/**
 * @brief Send a full OOK frame.
 *
 * Prepares a frame with SYNC, address, and payload, and starts
 * transmission using the TX state machine and timer interrupts.
 *
 * @param address           Node address (2 bits)
 * @param payload           Pointer to payload data array
 * @param payload_len_bytes Number of bytes in the payload
 */
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_len_bytes);

/**
 * @brief Process received frames from the RX FIFO.
 *
 * Can be used to decode, dispatch, or retransmit frames.
 */
void rf_ook_proto_handle_received_frame(void);

/**
 * @brief Get a pointer to the current TX frame structure.
 *
 * Allows checking or modifying the frame state from the application.
 *
 * @return Pointer to the active transmission context
 */
rf_ook_tx_frame_t* rf_ook_proto_get_tx_frame(void);

/**
 * @brief Check if the protocol layer is currently transmitting.
 *
 * @return true if TX is active, false otherwise
 */
bool rf_ook_proto_is_busy(void);

/**
 * @brief Get the node address of this MCU.
 *
 * @return Node address as uint8_t (limited by ADDRESS_BITS)
 */
uint8_t rf_ook_get_node_address(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_PROTO_H_ */
