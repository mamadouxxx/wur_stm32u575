/**
 * @file rf_ook_tx.c
 * @author Mamadou
 * @date 19 Jan 2026
 * @brief Implementation of RF 433 MHz OOK Transmitter module (TX)
 *
 * Provides bit-level and frame-level functions for OOK transmission.
 */

#include "rf_ook_tx.h"
#include "rf_ook_phy.h"

extern TIM_HandleTypeDef htim1; /**< Timer used for microsecond delays */

/**
 * @brief Initialize the RF OOK transmitter.
 *
 * Prepares GPIOs and sets the antenna to RX mode by default.
 */
void rf_ook_tx_init(void)
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
}

/**
 * @brief Turn on the RF transmitter
 *
 * Activates the TX by setting the corresponding GPIO pin.
 */
void tx_on(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_SET);
}

/**
 * @brief Turn off the RF transmitter
 *
 * Deactivates the TX by resetting the corresponding GPIO pin.
 */
void tx_off(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Start an OOK RF transmission.
 *
 * This function prepares the RF hardware to start transmitting data using
 * On-Off Keying (OOK) modulation.
 *
 */
void rf_ook_tx_start_tx()
{
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
    tx_on();
}

/**
 * @brief End an OOK RF transmission.
 *
 * This function safely stops an ongoing RF transmission.
 */
void rf_ook_end_tx()
{
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Send a single OOK bit.
 *
 * @param bit      Bit value (0 or 1)
 * @param bit_us   Duration of the bit in microseconds
 */
void rf_ook_tx_send_bit(uint8_t bit, uint32_t bit_us)
{
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_us_timer(bit_us);
}

/**
 * @brief Bit-level callback for protocol layer.
 *
 * Wraps rf_ook_tx_send_bit() for use with rf_ook_proto.
 *
 * @param bit     Bit value (0 or 1)
 * @param bit_us  Duration of the bit in microseconds
 */
void rf_ook_tx_bit_callback(uint8_t bit, uint32_t bit_us)
{
    rf_ook_tx_send_bit(bit, bit_us);
}

/**
 * @brief Microsecond delay callback for protocol layer.
 *
 * @param us  Delay in microseconds
 */
void rf_ook_tx_delay_callback(uint32_t us)
{
    delay_us_timer(us);
}

/**
 * @brief Quick test function to send a short frame.
 *
 * Sends a 6-bit frame to quickly verify the transmission on an oscilloscope.
 */
void rf_ook_tx_send_test(void)
{
    uint8_t frame[] = {1,0,1,1,0,0};
    uint32_t bit_us = 769; // ~1300 bps

    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
    tx_on();
    delay_us_timer(1000); // Stabilization

    for (uint8_t i = 0; i < sizeof(frame); i++) {
        rf_ook_tx_send_bit(frame[i], bit_us);
    }

    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
    tx_off();
    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
}


///**
// * @brief Envoie une trame complète: préambule + adresse + payload
// * @param address        Adresse du noeud (4 bits)
// * @param payload        Tableau de données
// * @param payload_bits   Taille du payload en bits
// * @param bit_us         Durée d’un bit en microsecondes
// */
//void rf_ook_tx_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_bits, uint32_t bit_us)
//{
//    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_SET);
//    tx_on();
//    delay_us_timer(1000);  // stabilisation
//
//    // Préambule 101010...
//    for (uint8_t i = 0; i < PREAMBLE_BITS; i++) {
//        rf_ook_tx_send_bit((i % 2) ? 1 : 0, bit_us);
//    }
//
//    // Adresse
//    for (int8_t i = ADDRESS_BITS - 1; i >= 0; i--) {
//        rf_ook_tx_send_bit((address >> i) & 1, bit_us);
//    }
//
//    // Payload
//    for (uint8_t i = 0; i < payload_bits; i++) {
//        uint8_t byte_idx = i / 8;
//        uint8_t bit_idx  = 7 - (i % 8);
//        rf_ook_tx_send_bit((payload[byte_idx] >> bit_idx) & 1, bit_us);
//    }
//
//    // Fin de transmission
//    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, GPIO_PIN_RESET);
//    tx_off();
//    HAL_GPIO_WritePin(Switch_RF_GPIO_Port, Switch_RF_Pin, GPIO_PIN_RESET);
//}

//void rf_ook_tx_send_test(void)
//{
//    uint8_t frame[] = {1,0,1,1,0,0};
//    rf_ook_tx_send_frame(0xA, frame, 6, BIT_1_US);
//}

