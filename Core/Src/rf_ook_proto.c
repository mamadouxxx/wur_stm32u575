/**
 * @file rf_ook_proto.c
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Implementation of OOK 433 MHz protocol
 *
 * Provides bit-level and frame-level functions using callbacks
 * to decouple the hardware layer from the protocol layer.
 */

#include "rf_ook_proto.h"
#include "rf_ook_tx.h"
#include "rf_ook_rx.h"
#include "rf_ook_phy.h"

static rf_ook_tx_bit_cb_t tx_send_bit_cb; /**< Callback for sending a single bit */

static rf_ook_delay_cb_t delay_cb;        /**< Callback for microsecond delay */

/**
 * @brief Initialize the protocol layer with user-provided callbacks
 *
 * Sets the bit and delay callbacks and the bitrate.
 *
 * @param init Pointer to rf_ook_proto_init_t
 */
void init(rf_ook_proto_init_t *init)
{
    tx_send_bit_cb = init->tx_send_bit_cb;
    delay_cb = init->delay_cb;

//    if (init->bitrate == 0)
//        init->bitrate = RF_OOK_DEFAULT_BPS;
}

/**
 * @brief Initialize protocol module with default TX/RX callbacks
 *
 * Calls rf_ook_tx_init() and sets default callbacks for bit sending
 * using rf_ook_tx_bit_callback.
 */
void rf_ook_proto_init(void)
{
    rf_ook_tx_init();
    rf_ook_rx_init();

    init(&(rf_ook_proto_init_t){
        .tx_send_bit_cb = rf_ook_tx_bit_callback,
        .bitrate = RF_OOK_DEFAULT_BPS
    });
}

/**
 * @brief Send a complete OOK frame
 *
 * The frame includes:
 * - Preamble (alternating 1s and 0s)
 * - Node address
 * - Payload
 *
 * @param address       bit node address
 * @param payload       Pointer to payload data
 * @param payload_len_bytes  Number of bits in the payload
 */
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_len_bytes)
{
	// starting transmission
	rf_ook_tx_start_tx();
    delay_us_timer(TX_STABILIZATION_US); // Stabilization

    // SYNC (MSB first)
    for (int8_t i = SYNC_NB_BITS - 1; i >= 0; i--)
    {
        tx_send_bit_cb((SYNC_BITS_VALUE >> i) & 1);
    }

    // Address
    for (int8_t i = ADDRESS_BITS - 1; i >= 0; i--)
        tx_send_bit_cb((address >> i) & 1);

    // Payload (A revoir pour le payload)
    for (uint8_t bytes_index = 0; bytes_index < payload_len_bytes; bytes_index++) {
        for (int8_t bit_index = 7; bit_index >= 0; bit_index--) {
            tx_send_bit_cb((payload[bytes_index] >> bit_index) & 1);
        }
    }

    // ending transmission
    rf_ook_end_tx();
}

/**
 *
 */
void rf_ook_proto_handle_received_frame(void)
{
    rx_frame_t frame;

    while (rf_ook_rx_get_frame(&frame)) {
        // Exemple : décodage logique applicatif
        // OU dispatch selon l’adresse
        // OU retransmission conditionnelle

        // DEBUG
        // process_frame(frame);

        // Exemple retransmission (si routeur)
        // rf_ook_proto_send_frame(frame.address, frame.payload, frame.payload_len * 8);
    }
}

