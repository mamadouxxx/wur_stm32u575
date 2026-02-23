/**
 * @file rf_ook_proto.c
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Implementation of OOK 433 MHz protocol.
 *
 * Provides frame-level functions using callbacks to decouple
 * the hardware (GPIO, timers) from the protocol layer.
 */

#include "rf_ook_proto.h"
#include "rf_ook_tx.h"
#include "rf_ook_rx.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1;     /**< Timer used for microsecond delays and TX ISR */

static const uint8_t NODE_ADDRESS = 1;
static rf_ook_tx_frame_t tx_frame;  /**< Runtime context of current TX frame */

static uint8_t seq_num = 0;

static void rf_ook_proto_forward_frame(rf_ook_frame_t *frame);
static void process_frame(rf_ook_frame_t *frame);

/* -------------------------------------------------------------------------- */
/*                            Protocol initialization                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the OOK protocol module.
 *
 * Sets up TX and RX modules, resets states, and prepares the protocol
 * layer for sending and receiving frames.
 */
void rf_ook_proto_init(void)
{
    rf_ook_tx_init();
    rf_ook_rx_init();
}

/* -------------------------------------------------------------------------- */
/*                            Frame transmission                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send a complete OOK frame.
 *
 * The frame consists of:
 * - SYNC (preamble)
 * - Node address
 * - Payload
 *
 * The function starts the TX state machine and timer ISR to send
 * each bit at the configured bitrate.
 *
 * @param address           Node address (2 bits)
 * @param payload           Pointer to payload data
 * @param payload_len_bytes Number of bytes in the payload
 */
void rf_ook_proto_send_frame(uint8_t dest_address, uint8_t *payload, uint8_t payload_len_bytes)
{
    if (tx_frame.active) return;
    if (payload_len_bytes > MAX_PAYLOAD_SIZE) return;
    if (payload == NULL && payload_len_bytes > 0) return;

    // Initialize TX frame
    tx_frame.frame.sync_bits = SYNC_BITS_VALUE;
    tx_frame.frame.src_address  = NODE_ADDRESS;
    tx_frame.frame.dest_address = dest_address;
    tx_frame.frame.seq_ttl = MAKE_SEQ_TTL(seq_num, TTL_MAX);
    seq_num = (seq_num + 1) & 0x0F;
    memcpy(tx_frame.frame.payload, payload, payload_len_bytes);
    tx_frame.frame.payload_len = payload_len_bytes;

    // Calcul CRC
    tx_frame.frame.crc = rf_ook_compute_crc(&tx_frame.frame);


    /*-------Test sans wait (en attente wur)---------*/
    tx_frame.byte_idx = 0;
    tx_frame.bit_idx = SYNC_NB_BITS - 1;
    tx_frame.state = TX_SYNC;
    tx_frame.active = true;
    tx_frame.wait_ticks = 0;

    // Turn on TX output
    rf_ook_tx_start_tx();

    // Start timer for ISR-driven bit sending
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start_IT(&htim1);
}

static void rf_ook_proto_forward_frame(rf_ook_frame_t *frame)
{
    if (tx_frame.active) return;

    // TTL décrémenté
    tx_frame.frame = *frame;
    tx_frame.frame.seq_ttl = MAKE_SEQ_TTL(GET_SEQ(frame->seq_ttl), GET_TTL(frame->seq_ttl) - 1);

    // Recalcul CRC
    tx_frame.frame.crc = rf_ook_compute_crc(&tx_frame.frame);

    tx_frame.byte_idx   = 0;
    tx_frame.bit_idx    = SYNC_NB_BITS - 1;
    tx_frame.state      = TX_SYNC;
    tx_frame.active     = true;
    tx_frame.wait_ticks = 0;

    rf_ook_tx_start_tx();
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start_IT(&htim1);
}

/* -------------------------------------------------------------------------- */
/*                             Frame reception                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Process a received frame
 */
static void process_frame(rf_ook_frame_t *frame)
{
//     Vérifier la taille
    if (frame->payload_len != sizeof(sensor_payload_t)) {
        char msg[50];
        int len = sprintf(msg, "Invalid frame size: %d\r\n", frame->payload_len);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
        return;
    }

    // Extraire les données
    sensor_payload_t sensors;
    memcpy(&sensors, frame->payload, sizeof(sensor_payload_t));

    // Afficher les données
    char msg[80];
    int len;

    len = sprintf(msg, "=== Sensor Data ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "CO2: %d ppm\r\n", sensors.co2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Temp: %d\r\n", sensors.temp);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Humidity: %d\r\n", sensors.hum);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Lux: %d\r\n", sensors.lux);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "O2: %d %%\r\n", sensors.o2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Motion: %d\r\n", sensors.motion);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "==================\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
}

/**
 * @brief Handle received frames from the RX FIFO.
 *
 * Should be called periodically in the main loop.
 * This function fetches frames from the RX buffer and can:
 * - Decode frame content
 * - Dispatch frame based on address
 * - Optionally retransmit (if node is a router)
 */
void rf_ook_proto_handle_received_frame(void)
{
    rf_ook_frame_t frame;

    /* Fast exit if no frame is ready */
    if (!rf_ook_rx_is_frame_ready()) {
        char msg[64];
        sprintf(msg, "not ready\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

        return;
    }

    /* Drain RX FIFO */
    while (rf_ook_rx_get_frame(&frame)) {

        // Vérification CRC
        if (rf_ook_compute_crc(&frame) != frame.crc) {
            char msg[] = "CRC ERROR\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            continue;
        }

        // Trame pour nous
        if (frame.dest_address == NODE_ADDRESS) {
            process_frame(&frame);
        }
        // Retransmission
        else {
            uint8_t ttl = GET_TTL(frame.seq_ttl);
            if (ttl == 0) continue;
            rf_ook_proto_forward_frame(&frame);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                        TX frame access and status                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get a pointer to the current TX frame.
 *
 * Allows checking the transmission state or modifying the frame context.
 *
 * @return Pointer to the active transmission frame context
 */
rf_ook_tx_frame_t* rf_ook_proto_get_tx_frame(void)
{
    return &tx_frame;
}

/**
 * @brief Check if the protocol is currently transmitting.
 *
 * @return true if a transmission is active, false otherwise
 */
bool rf_ook_proto_is_busy(void)
{
    return tx_frame.active;
}

/**
 * @brief Getter for NODE_ADDRESS
 *
 * Provides access to the node address in a safe way
 */
uint8_t rf_ook_get_node_address(void)
{
    return NODE_ADDRESS & ((1 << ADDRESS_BITS) - 1);
}
