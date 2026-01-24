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
 * @brief Turn on the RF transmitter
 *
 * Activates the TX by setting the corresponding GPIO pin.
 */
static void tx_on(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_SET);
}

/**
 * @brief Turn off the RF transmitter
 *
 * Deactivates the TX by resetting the corresponding GPIO pin.
 */
static void tx_off(void)
{
    HAL_GPIO_WritePin(TX_ONOFF_GPIO_Port, TX_ONOFF_Pin, GPIO_PIN_RESET);
}

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
 */
void rf_ook_tx_send_bit(uint8_t bit)
{
    HAL_GPIO_WritePin(TX_DATA_GPIO_Port, TX_DATA_Pin, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);
    delay_us_timer(BIT_US);
}

/**
 * @brief Bit-level callback for protocol layer.
 *
 * Wraps rf_ook_tx_send_bit() for use with rf_ook_proto.
 *
 * @param bit     Bit value (0 or 1)
 * @param bit_us  Duration of the bit in microseconds
 */
void rf_ook_tx_bit_callback(uint8_t bit)
{
    rf_ook_tx_send_bit(bit);
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
