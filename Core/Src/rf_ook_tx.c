/**
 * @file    rf_ook_tx.c
 * @author  Mamadou
 * @date    19 jan 2026
 * @brief   Implémentation du module émetteur OOK 433 MHz (TX)
 *
 * Fournit les fonctions de transmission bit à bit et trame par trame en OOK.
 * La transmission est pilotée par interruption timer (TIM1) pour un timing précis.
 */

#include "rf_ook_tx.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim1;  /**< Timer utilisé pour l'ISR de transmission TX */
extern UART_HandleTypeDef huart1;

/* -------------------------------------------------------------------------- */
/*                           Fonctions internes                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Active l'émetteur RF
 */
static void tx_on(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_SET);
}

/**
 * @brief Désactive l'émetteur RF
 */
static void tx_off(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------- */
/*                              API publique TX                               */
/* -------------------------------------------------------------------------- */

void rf_ook_tx_init(void)
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
}

void rf_ook_tx_start_tx(void)
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
    tx_on();
}

void rf_ook_end_tx(void)
{
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
}

void rf_ook_tx_send_bit(rf_ook_tx_frame_t *frame)
{
    uint8_t bit = 0;

    switch (frame->state)
    {
        case TX_SYNC:
            bit = (frame->frame.sync_bits >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                frame->state   = TX_SRC_ADDRESS;
                frame->bit_idx = ADDRESS_BITS - 1;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_SRC_ADDRESS:
            bit = (frame->frame.src_address >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                frame->state   = TX_DEST_ADDRESS;
                frame->bit_idx = ADDRESS_BITS - 1;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_DEST_ADDRESS:
            bit = (frame->frame.dest_address >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                frame->state    = TX_SEQ_TTL;
                frame->bit_idx  = 7;
                frame->byte_idx = 0;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_SEQ_TTL:
            bit = (frame->frame.seq_ttl >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                frame->state   = TX_LENGTH;
                frame->bit_idx = 7;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_LENGTH:
            bit = (frame->frame.payload_len >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                if (frame->frame.payload_len > 0) {
                    frame->state    = TX_PAYLOAD;
                    frame->bit_idx  = 7;
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
            if (frame->bit_idx == 0) {
                frame->bit_idx = 7;
                frame->byte_idx++;
            } else {
                frame->bit_idx--;
            }
            if (frame->byte_idx >= frame->frame.payload_len) {
                frame->state = TX_CRC;
            }
            break;

        case TX_CRC:
            bit = (frame->frame.crc >> frame->bit_idx) & 1;
            if (frame->bit_idx == 0) {
                frame->state = TX_DONE;
            } else {
                frame->bit_idx--;
            }
            break;

        case TX_DONE:
            // Fin de transmission : arrêt du timer et de l'émetteur
            frame->active = false;
            HAL_TIM_Base_Stop_IT(&htim1);
            rf_ook_end_tx();
            return;
    }

    // Écriture du bit sur la broche TX_DATA
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
