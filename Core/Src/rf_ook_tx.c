/*
 * rf_ook_tx.c
 *
 *  Created on: 19 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "rf_ook_tx.h"
extern TIM_HandleTypeDef htim1;

void delay_us_timer(uint32_t us);

void tx_on(void) {
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_SET);
}

void tx_off(void) {
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_RESET);
}

// Envoie un bit OOK
void rf_ook_tx_send_bit(uint8_t bit, uint32_t bit_us)
{
    if (bit) {
        HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    }

    // Attente de la durée du bit
    delay_us_timer(bit_us);
}



// Envoie un frame: préambule + adresse + payload
void rf_ook_tx_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_bits, uint32_t bit_us)
{
    // Mettre l'antenne sur TX
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);

    // Allumer le module TX
    tx_on();

    delay_us_timer(1000);  // 1s de stabilisation

    for (uint8_t i = 0; i < PREAMBLE_BITS; i++) {
        uint8_t bit = (i % 2) ? 1 : 0; // 101010...
        rf_ook_tx_send_bit(bit, bit_us);
    }

    for (int8_t i = ADDRESS_BITS-1; i >= 0; i--) {
        uint8_t bit = (address >> i) & 1;
        rf_ook_tx_send_bit(bit, bit_us);
    }

    for (uint8_t i = 0; i < payload_bits; i++) {
        uint8_t byte_idx = i / 8;
        uint8_t bit_idx  = 7 - (i % 8);
        uint8_t bit = (payload[byte_idx] >> bit_idx) & 1;
        rf_ook_tx_send_bit(bit, bit_us);
    }

    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    // Retour à RX
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);

}

void rf_ook_tx_send_test(void)
{
    uint8_t frame[] = {1,0,1,1,0,0};
    uint32_t bit_us = 769; // microsecondes pour 1300 bps

    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);

    tx_on();

    delay_us_timer(1000);  // 1s de stabilisation

    for(int i=0; i<6; i++)
    {
        rf_ook_tx_send_bit(frame[i], bit_us);
    }

    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
}

void delay_us_timer(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start(&htim1);
    while(__HAL_TIM_GET_COUNTER(&htim1) < us);
    HAL_TIM_Base_Stop(&htim1);
}


