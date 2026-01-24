#ifndef RF_OOK_RX_H_
#define RF_OOK_RX_H_

#include <stdint.h>
#include <stdbool.h>
#include "rf_ook_phy.h"

typedef struct {
    uint8_t address;
    uint8_t payload_len;
    uint8_t payload[MAX_PAYLOAD_SIZE];
} rx_frame_t;

/* Initialisation */
void rf_ook_rx_init(void);

/* Réinitialisation FSM */
void rf_ook_rx_reset(void);

/* Injection d’un bit (ISR ou module physique) */
void rf_ook_rx_bit_callback(uint8_t bit);

/* Lecture du buffer */
uint8_t rf_ook_rx_get_frame(rx_frame_t *frame);

#endif
