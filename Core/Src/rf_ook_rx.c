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
#include <string.h>
extern TIM_HandleTypeDef htim3; /**< Timer used for microsecond delays and ISR */
extern UART_HandleTypeDef huart1;

volatile edge_event_t edge_queue[EDGE_QUEUE_SIZE];
volatile uint8_t edge_head = 0;
volatile uint8_t edge_tail = 0;
volatile bool first_edge_received = false;

// Variables pour calculer delta
volatile uint32_t last_edge_time = 0;
volatile uint8_t last_level = 0;

static uint32_t rx_last_bit_time = 0;

/* -------------------------------------------------------------------------- */
/*                               Internal RX FSM                               */
/* -------------------------------------------------------------------------- */
static rx_state_t rx_state = RX_SYNC; /**< Current RX FSM state */
static uint8_t rx_shift_reg = 0;      /**< Bit shift register */
static uint16_t rx_sync_reg = 0;  // shift reg 16 bits uniquement pour la détection sync
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
    rx_state = RX_SYNC;
    rx_head = rx_tail = 0;
    first_edge_received = false;
    HAL_TIM_Base_Start(&htim3);
    last_edge_time = TIM3->CNT;       // position de départ
    last_level = HAL_GPIO_ReadPin(RX_DATA_GPIO_Port, RX_DATA_Pin); // niveau initial
}

/**
 * @brief Reset RX FSM
 *
 * Clears internal counters and prepares FSM for new frame reception.
 */
void rf_ook_rx_reset(void) {
    rx_state = RX_SYNC;
    rx_shift_reg = 0;
    rx_sync_reg = 0;
    rx_bit_count = 0;
    rx_byte_count = 0;
    rx_current.dest_address = 0;
    rx_current.payload_len = 0;
}

void rf_ook_rx_handle_edge(uint8_t current_level)
{
    uint32_t now = TIM3->CNT;
    uint32_t delta = now - last_edge_time;

    if(delta < EDGE_FILTER_TICKS) {
        return;
    }

    last_edge_time = now;

    if(!first_edge_received) {
        first_edge_received = true;
        last_level = current_level;
        return;
    }
    uint8_t next_head = (edge_head + 1) % EDGE_QUEUE_SIZE;
    if(next_head != edge_tail) {
//        char msg[] = "TX: Reception en cours 11...\r\n";
//        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

        edge_queue[edge_head].delta_ticks = delta;
        edge_queue[edge_head].level = last_level;
        edge_head = next_head;
//        rf_ook_rx_process_queue();
    }

    last_level = current_level;
}

void rf_ook_rx_process_queue(void)
{
	if (rx_state != RX_SYNC && rx_state != RX_DONE) {
	    uint32_t now = TIM3->CNT;
	    if ((now - rx_last_bit_time) > RX_TIMEOUT_TICKS) {
	        rf_ook_rx_reset();
	    }
	}

    while (edge_tail != edge_head)
    {
        edge_event_t e = edge_queue[edge_tail];
        edge_tail = (edge_tail + 1) % EDGE_QUEUE_SIZE;

        uint32_t nb_bits = (e.delta_ticks + BIT_TICKS/2) / BIT_TICKS;

        for (uint32_t i = 0; i < nb_bits; i++)
        {
            rf_ook_rx_receive_bit(e.level);
        }
    }
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
	rx_last_bit_time = TIM3->CNT;

    switch (rx_state)
    {
        case RX_SYNC:
            rx_sync_reg = (rx_sync_reg << 1) | (bit & 0x01);
            if(rx_sync_reg == 0xAAAA)
            {
                rx_state = RX_SRC_ADDRESS;
                rx_bit_count = 0;
                rx_sync_reg = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_SRC_ADDRESS:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= ADDRESS_BITS)
            {
                rx_current.src_address = rx_shift_reg & ((1 << ADDRESS_BITS) - 1);
                rx_state     = RX_DEST_ADDRESS;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_DEST_ADDRESS:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;

            if (rx_bit_count >= ADDRESS_BITS)
            {
                rx_current.dest_address = rx_shift_reg & ((1 << ADDRESS_BITS) - 1);
                rx_state = RX_SEQ_TTL;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_SEQ_TTL:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8)
            {
                rx_current.seq_ttl = rx_shift_reg;
                rx_state     = RX_LENGTH;
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

                if (rx_byte_count >= rx_current.payload_len)
                {
                    rx_state     = RX_CRC;
                }
            }
            break;

        case RX_CRC:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8)
            {
                rx_current.crc = rx_shift_reg;

                if (rx_frame_count < BUFFER_SIZE) {
                    buffer[rx_head] = rx_current;
                    rx_head = (rx_head + 1) % BUFFER_SIZE;
                    rx_frame_count++;
                } else {
                    rx_overflow_count++;
                }
                rx_state = RX_DONE;
            }
            break;

        case RX_DONE:
//            HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
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
