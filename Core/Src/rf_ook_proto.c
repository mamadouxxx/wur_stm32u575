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
#include "rf_ook_phy.h"

static rf_ook_tx_bit_cb_t tx_send_bit_cb; /**< Callback for sending a single bit */
static rf_ook_delay_cb_t delay_cb;        /**< Callback for microsecond delay */

static rf_ook_rx_frame_cb_t rx_user_cb = 0;

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

    if (init->bitrate == 0)
        init->bitrate = RF_OOK_DEFAULT_BPS;
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
 * @param payload_bits  Number of bits in the payload
 */
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_bits)
{
	// starting transmission
	rf_ook_tx_start_tx();
    delay_us_timer(TX_STABILIZATION_US); // Stabilization

    // SYNC (MSB first)
    for (int8_t i = SYNC_NB_BITS - 1; i >= 0; i--)
    {
        tx_send_bit_cb((SYNC_BITS_VALUE >> i) & 1, BIT_US);
    }

    // Address
    for (int8_t i = ADDRESS_BITS - 1; i >= 0; i--)
        tx_send_bit_cb((address >> i) & 1, BIT_US);

    // Payload
    for (uint8_t i = 0; i < payload_bits; i++) {
        uint8_t b = payload[i / 8];
        tx_send_bit_cb((b >> (7 - (i % 8))) & 1, BIT_US);
    }

    // ending transmission
    rf_ook_end_tx();
}

/**
 *
 */
void rf_ook_proto_register_rx_callback(rf_ook_rx_frame_cb_t cb)
{
    rx_user_cb = cb;
}

/**
 * @brief Traite une frame reçue par le RX et appelle le callback utilisateur
 */
void rf_ook_proto_handle_received_frame(uint8_t address, uint8_t payload)
{
    if (rx_user_cb)
    {
        rf_ook_rx_frame_t frame;
        frame.address = address;
        frame.payload = payload;
        rx_user_cb(&frame);
    }
}

