/**
 * @file rf_ook_rx.c
 * @author Mamadou
 * @date 21 Jan 2026
 * @brief Implementation of RF 433 MHz OOK Receiver (RX)
 *
 * Provides FSM-based reception and buffering of OOK frames.
 * The RX FSM has three states: RX_IDLE, RX_ADDRESS, RX_PAYLOAD.
 * Received frames are stored in a circular buffer for later processing.
 */

#include "rf_ook_rx.h"
#include <stdio.h>
extern TIM_HandleTypeDef htim3; /**< Timer used for microsecond delays and ISR */
extern UART_HandleTypeDef huart1;

/* -------------------------------------------------------------------------- */
/*                               Internal RX FSM                               */
/* -------------------------------------------------------------------------- */
static rx_state_t rx_state = RX_IDLE; /**< Current RX FSM state */
static uint8_t rx_shift_reg = 0;      /**< Bit shift register */
static uint8_t rx_bit_count = 0;      /**< Number of bits received for current field */
static uint8_t rx_byte_count = 0;    /**< Number of datas bytes received */
static rf_ook_frame_t rx_current;     /**< Current frame being received */

/* -------------------------------------------------------------------------- */
/*                           Circular buffer storage                           */
/* -------------------------------------------------------------------------- */
static rf_ook_frame_t buffer[BUFFER_SIZE]; /**< RX FIFO buffer */
static volatile uint8_t rx_head = 0;       /**< FIFO head index */
static volatile uint8_t rx_tail = 0;       /**< FIFO tail index */
static volatile uint8_t rx_frame_count = 0;    /**< Number of frames in buffer */
static volatile uint8_t rx_overflow_count = 0; /**< Number of frames lost due to buffer overflow */

/* -------------------------------------------------------------------------- */
/*                        Public RX API                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize RX module
 *
 * Resets FSM state and buffer indices.
 */
void rf_ook_rx_init(void) {
    rx_state = RX_IDLE;
    rx_head = rx_tail = 0;
}

/**
 * @brief Reset RX FSM
 *
 * Clears internal counters and prepares FSM for new frame reception.
 */
void rf_ook_rx_reset(void) {
    rx_state = RX_IDLE;
    rx_shift_reg = 0;
    rx_bit_count = 0;
    rx_byte_count = 0;
    rx_current.address = 0;
    rx_current.payload_len = 0;
}

/**
 * @brief Receive a single bit (FSM handler)
 *
 * Updates FSM state depending on current RX phase:
 * - RX_IDLE: first bit of address
 * - RX_ADDRESS: accumulate bits until ADDRESS_BITS reached
 * - RX_PAYLOAD: accumulate bytes until PAYLOAD_BYTES reached
 *
 * When a frame is complete, it is pushed into the circular buffer.
 *
 * @param bit Received bit (0 or 1)
 */
void rf_ook_rx_receive_bit(uint8_t bit)
{
    switch (rx_state)
    {
        case RX_IDLE:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count = 1;
            rx_state = RX_ADDRESS;
            break;

        case RX_ADDRESS:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;

            if (rx_bit_count >= ADDRESS_BITS)
            {
                rx_current.address = rx_shift_reg;
                rx_state = RX_LENGTH;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_LENGTH:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;

            if (rx_bit_count >= LENGTH_BITS)
            {
                rx_current.payload_len = rx_shift_reg;
                if (rx_current.payload_len > MAX_PAYLOAD_SIZE) {
                    rf_ook_rx_reset();
                    break;
                }
                rx_state = RX_PAYLOAD;
				rx_bit_count = 0;
				rx_byte_count = 0;
				rx_shift_reg = 0;
            }
            break;

        case RX_PAYLOAD:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;

            if (rx_bit_count >= 8)
            {
                if (rx_byte_count < rx_current.payload_len &&
                    rx_byte_count < MAX_PAYLOAD_SIZE)
                {
                    rx_current.payload[rx_byte_count] = rx_shift_reg;
                    rx_byte_count++;
                }
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }

            if (rx_byte_count >= rx_current.payload_len)
            {
                if (rx_frame_count < BUFFER_SIZE)
                {
                    buffer[rx_head] = rx_current;
                    rx_head = (rx_head + 1) % BUFFER_SIZE;
                    rx_frame_count++;
                }
                else
                {
                    rx_overflow_count++;
                }
				rx_state = RX_DONE;
            }
            break;

        case RX_DONE:
//            HAL_TIM_Base_Stop_IT(&htim3);
            rf_ook_rx_reset();
        	break;
    }
}

/**
 * @brief Get a received frame from the RX FIFO
 *
 * Copies the oldest frame from the buffer and updates indices.
 *
 * @param frame Pointer to store the frame
 * @return 1 if frame was available, 0 if buffer is empty
 */
uint8_t rf_ook_rx_get_frame(rf_ook_frame_t *frame) {
    if (rx_frame_count == 0) {
        return 0; // buffer empty
    }
    __disable_irq();

    *frame = buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    rx_frame_count--;

    __enable_irq();

    return 1;
}

/**
 * @brief Check if a frame is ready in the RX buffer
 *
 * @return true if at least one frame is available, false otherwise
 */
bool rf_ook_rx_is_frame_ready(void)
{
    return (rx_frame_count > 0);;
}
