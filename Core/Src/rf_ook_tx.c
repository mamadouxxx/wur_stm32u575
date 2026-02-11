/**
 * @file rf_ook_tx.c
 * @author Mamadou
 * @date 19 Jan 2026
 * @brief Implementation of RF 433 MHz OOK Transmitter module (TX)
 *
 * Provides bit-level and frame-level functions for OOK transmission.
 * Transmission is ISR-driven using a timer (htim1) for accurate timing.
 */

#include "rf_ook_tx.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1; /**< Timer used for microsecond delays and ISR */
extern UART_HandleTypeDef huart1;


/* -------------------------------------------------------------------------- */
/*                           Internal helper functions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Activate the RF transmitter.
 *
 * Sets the GPIO to enable TX output.
 */
static void tx_on(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_SET);
}

/**
 * @brief Deactivate the RF transmitter.
 *
 * Resets the GPIO to stop TX output.
 */
static void tx_off(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/*                        Public TX API                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the RF OOK transmitter.
 *
 * Sets GPIO pins to default state and disables TX output.
 * Prepares the RF hardware to start transmission when needed.
 */
void rf_ook_tx_init(void)
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
}

/**
 * @brief Start an OOK RF transmission.
 *
 * Prepares the hardware to send bits using OOK modulation.
 * The TX_DATA pin can then be toggled to send bits.
 */
void rf_ook_tx_start_tx(void)
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
    tx_on();
}

/**
 * @brief End an OOK RF transmission.
 *
 * Safely stops an ongoing transmission.
 * Powers down the TX path and sets TX_DATA to low.
 */
void rf_ook_end_tx(void)
{
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Send a single OOK bit based on the TX state machine.
 *
 * Handles the current TX state:
 * - TX_SYNC: sends SYNC bits
 * - TX_WAIT: waits for the wake-up delay
 * - TX_ADDRESS: sends the address bits
 * - TX_PAYLOAD: sends payload bytes
 * - TX_DONE: finishes transmission and stops timer
 *
 * @param frame Pointer to the active TX frame context
 */
void rf_ook_tx_send_bit(rf_ook_tx_frame_t *frame)
{
    uint8_t bit = 0;

    switch(frame->state) {
        case TX_SYNC:
            bit = (frame->frame.sync_bits >> frame->bit_idx) & 1;
            if(frame->bit_idx == 0) {
                frame->state = TX_WAIT;
                frame->wait_ticks = 0;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_WAIT:
            frame->wait_ticks++;
            if (frame->wait_ticks >= TX_WAIT_TICKS) {
                frame->state = TX_ADDRESS;
                frame->bit_idx = ADDRESS_BITS - 1;
            }
            return; // no bit is sent during wait

        case TX_ADDRESS:
            bit = (frame->frame.address >> frame->bit_idx) & 1;
            if(frame->bit_idx == 0) {
                frame->state = TX_LENGTH;
                frame->bit_idx = 7;
                frame->byte_idx = 0;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_LENGTH:
            bit = (frame->frame.payload_len >> frame->bit_idx) & 1;
            if(frame->bit_idx == 0) {
                if (frame->frame.payload_len > 0) {
                    frame->state = TX_PAYLOAD;
                    frame->bit_idx = 7;
                    frame->byte_idx = 0;
                } else {
                    frame->state = TX_DONE;
                }
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_PAYLOAD:
            bit = (frame->frame.payload[frame->byte_idx] >> frame->bit_idx) & 1;
            if(frame->bit_idx == 0) {
                frame->bit_idx = 7;
                frame->byte_idx++;
            } else {
                frame->bit_idx--;
            }

            if(frame->byte_idx >= frame->frame.payload_len) {
                frame->state = TX_DONE;
            }
            break;

        case TX_DONE:
            frame->active = false;
            HAL_TIM_Base_Stop_IT(&htim1);
            rf_ook_end_tx();
            return; // nothing more to send
    }

    // Write bit to TX_DATA GPIO
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
