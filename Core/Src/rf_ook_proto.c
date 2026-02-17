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

static const uint8_t NODE_ADDRESS = 1;
static rf_ook_tx_frame_t tx_frame;  /**< Runtime context of current TX frame */

extern TIM_HandleTypeDef htim1;     /**< Timer used for microsecond delays and TX ISR */

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
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_len_bytes)
{
    if (tx_frame.active) {
        return; // Transmission already in progress
    }

    if ( payload_len_bytes > MAX_PAYLOAD_SIZE  ){
        return;
    }

    if (payload == NULL && payload_len_bytes > 0) {
        return;
    }

    // Initialize TX frame
    tx_frame.frame.sync_bits = SYNC_BITS_VALUE;
    tx_frame.frame.dest_address = address;
    for(uint8_t i = 0; i < payload_len_bytes; i++)
        tx_frame.frame.payload[i] = payload[i];
    tx_frame.frame.payload_len = payload_len_bytes;

    tx_frame.byte_idx = 0;

    /*-------Test sans sync et wait (en attente wur)---------*/
    tx_frame.bit_idx = ADDRESS_BITS - 1;
    tx_frame.state = TX_DEST_ADDRESS;
    /*----------------*/
//    tx_frame.bit_idx = SYNC_NB_BITS - 1;
//    tx_frame.state = TX_SYNC;
    tx_frame.active = true;
    tx_frame.wait_ticks = 0;

    // Turn on TX output
    rf_ook_tx_start_tx();

    // Start timer for ISR-driven bit sending
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
    // Vérifier la taille
//    if (frame->payload_len != sizeof(sensor_payload_t)) {
//        char msg[50];
//        int len = sprintf(msg, "Invalid frame size: %d\r\n", frame->payload_len);
//        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
//        return;
//    }

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
        return;
    }

    /* Drain RX FIFO */
    while (rf_ook_rx_get_frame(&frame)) {

        /* Example: address filtering */
        if (frame.dest_address == 0) {
            /* Frame intended for this node */
            process_frame(&frame);
        }
        else {
            char msg[50];
            /* Optional: routing / forwarding */
            // rf_ook_proto_send_frame(frame.address,
            //                          frame.payload,
            //                          frame.payload_len);
            int len = sprintf(msg, "handler Frame: addr=%d len=%d\r\n",
            frame.dest_address, frame.payload_len);
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
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
