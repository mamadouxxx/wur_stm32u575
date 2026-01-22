/*
 * rf_ook_rx.c
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
* @file rf_ook_rx.c
*
 * @brief Réception OOK 433 MHz (WuR / RX) avec FSM
 *
 */

#include "rf_ook_rx.h"

volatile rf_ook_rx_state_t rx_state = RX_IDLE;
volatile uint8_t rx_shift_reg = 0;
volatile uint8_t rx_bit_count = 0;
volatile uint8_t rx_address = 0;
volatile uint8_t rx_payload = 0;
volatile uint8_t rx_frame_ready = 0;

void rf_ook_rx_init(void)
{
    rx_frame_ready = 0;
    rf_ook_rx_reset();
}

void rf_ook_rx_reset(void)
{
    rx_state = RX_IDLE;
    rx_shift_reg = 0;
    rx_bit_count = 0;
    rx_address = 0;
    rx_payload = 0;
}

/**
 * @brief Gestion d'un bit reçu depuis TIM3 ISR
 *
 * Appelée depuis HAL_TIM_PeriodElapsedCallback(TIM3)
 *
 * @param bit Bit reçu (0 ou 1)
 */
void rf_ook_rx_bit_handler(uint8_t bit)
{
    // Décalage du bit dans le registre
    rx_shift_reg = ((rx_shift_reg << 1) | (bit & 0x01));

    switch (rx_state)
    {
        case RX_IDLE:
            // On attend un préambule (alternating 1010…)
            rx_bit_count++;
            if (rx_bit_count >= PREAMBLE_BITS)
            {
                rx_state = RX_SYNC;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_SYNC:
            rx_bit_count++;
            if (rx_bit_count >= SYNC_NB_BITS)
            {
                // Vérifie si les bits reçus correspondent au SYNC
                if ((rx_shift_reg & ((1 << SYNC_NB_BITS) - 1)) == SYNC_BITS_VALUE)
                {
                    rx_state = RX_ADDRESS;
                    rx_bit_count = 0;
                    rx_shift_reg = 0;
                }
                else
                {
                    // Erreur SYNC, retour à IDLE
                    rf_ook_rx_reset();
                }
            }
            break;

        case RX_ADDRESS:
            rx_bit_count++;
            if (rx_bit_count >= ADDRESS_BITS)
            {
                rx_address = rx_shift_reg & ((1 << ADDRESS_BITS) - 1);
                rx_state = RX_PAYLOAD;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_PAYLOAD:
            rx_bit_count++;
            if (rx_bit_count >= 8) // 1 octet payload, adapter si plus
            {
                rx_payload = rx_shift_reg & 0xFF;
                rx_frame_ready = 1; // frame complet
                rf_ook_rx_reset();
            }
            break;

        default:
            rf_ook_rx_reset();
            break;
    }
}
