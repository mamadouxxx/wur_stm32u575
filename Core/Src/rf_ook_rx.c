/**
 * @file rf_ook_rx.c
 * @author Mamadou
 * @date 21 Jan 2026
 * @brief RF 433 MHz OOK Receiver – FSM implementation
 */

#include "rf_ook_rx.h"

typedef enum { RX_IDLE=0, RX_ADDRESS, RX_PAYLOAD } rx_state_t;


/* FSM interne */
static rx_state_t rx_state = RX_IDLE;
static uint8_t rx_shift_reg = 0;
static uint8_t rx_bit_count = 0;
static rx_frame_t rx_current;

/* Buffer circulaire */
static rx_frame_t buffer[BUFFER_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;
static volatile uint8_t rx_frame_count = 0;
static volatile uint8_t rx_overflow_count = 0;

void rf_ook_rx_init(void) {
    rx_state = RX_IDLE;
    rx_head = rx_tail = 0;
}

void rf_ook_rx_reset(void) {
    rx_state = RX_IDLE;
    rx_shift_reg = 0;
    rx_bit_count = 0;
    rx_current.address = 0;
    rx_current.payload_len = 0;
}

/* FSM + Buffer */
void rf_ook_rx_bit_callback(uint8_t bit) {
    rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
    rx_bit_count++;

    switch (rx_state) {
        case RX_IDLE:
            rx_current.address = bit; // 1er bit adresse
            rx_bit_count = 1;
            rx_state = RX_ADDRESS;
            rx_shift_reg = bit;
            break;

        case RX_ADDRESS:
            if (rx_bit_count >= ADDRESS_BITS) {
                rx_current.address = rx_shift_reg & ((1<<ADDRESS_BITS)-1);
                rx_state = RX_PAYLOAD;
                rx_bit_count = 0;
                rx_shift_reg = 0;
                rx_current.payload_len = 0;
            }
            break;

        case RX_PAYLOAD:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8) {
                if (rx_current.payload_len < MAX_PAYLOAD_SIZE) {
                    rx_current.payload[rx_current.payload_len++] = rx_shift_reg;
                }
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            if (rx_current.payload_len >= PAYLOAD_BYTES) { // frame complète
            	if (rx_frame_count < BUFFER_SIZE) {
            	    buffer[rx_head] = rx_current;
            	    rx_head = (rx_head + 1) % BUFFER_SIZE;
            	    rx_frame_count++;
            	} else {
            	    // Overflow → on drop la frame
                    rx_overflow_count++; // buffer plein → trame perdue
            	}
                rf_ook_rx_reset();
            }
            break;
    }
}

/* Lecture FIFO */
uint8_t rf_ook_rx_get_frame(rx_frame_t *frame) {
    if (rx_frame_count == 0)
        return 0; // vide

    *frame = buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BUFFER_SIZE;
    rx_frame_count--;

    return 1;
}
