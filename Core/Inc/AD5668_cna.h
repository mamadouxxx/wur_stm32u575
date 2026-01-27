/**
 * @file    ad5668_cna.h
 * @brief   AD5668 8-channel DAC (CNA) driver
 * @author  Mamadou
 * @date    January 25, 2026
 *
 * This driver provides a high-level API to control the AD5668 DAC via SPI.
 *
 * Features:
 *  - SPI communication with AD5668
 *  - Voltage-based and raw code DAC control
 *  - LDAC, CLR, power-down and reset management
 *  - Internal reference control
 *
 * Designed for low-power embedded systems (e.g. Wake-up Radio projects).
 */

#ifndef AD5668_CNA_H_
#define AD5668_CNA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief SPI handle used by the AD5668 driver
 */
extern SPI_HandleTypeDef hspi1;

/** @name AD5668 command codes
 * @{
 */
#define CNA_CMD_WRITE_INPUT_REG           0 /**< Write to input register */
#define CNA_CMD_UPDATE_OUTPUT_REG         1 /**< Update DAC output register */
#define CNA_CMD_WRITE_INPUT_UPDATE_ALL    2 /**< Write and update all channels */
#define CNA_CMD_WRITE_INPUT_UPDATE_N      3 /**< Write and update selected channel */
#define CNA_CMD_POWER_DOWN_UP             4 /**< Power-down / power-up command */
#define CNA_CMD_LOAD_CLEAR_CODE           5 /**< Load clear code register */
#define CNA_CMD_LOAD_LDAC                 6 /**< Load LDAC register */
#define CNA_CMD_RESET_POWER_ON            7 /**< Full DAC reset */
#define CNA_CMD_SETUP_INTERNAL_REF        8 /**< Enable / disable internal reference */
/** @} */

/**
 * @brief Initialize the AD5668 DAC
 *
 * This function:
 *  - Resets the DAC using the CLR pin
 *  - Sets LDAC to inactive state
 *  - Loads default voltages on all channels
 */
void cna_init(void);

/**
 * @brief Clear all DAC outputs using the CLR pin
 */
void cna_clear(void);

/**
 * @brief Set output voltage on a DAC channel (in millivolts)
 *
 * @param channel Channel index (0 to 7)
 * @param voltage_mV Desired output voltage in millivolts
 */
void cna_set_voltage(uint8_t channel, float voltage_mV);

/**
 * @brief Write a raw 16-bit DAC code to the input register
 *
 * @param channel Channel index (0 to 7)
 * @param code 16-bit DAC value
 */
void cna_write_channel(uint8_t channel, uint16_t code);

/**
 * @brief Update the DAC output register of a channel
 *
 * @param channel Channel index (0 to 7)
 */
void cna_update_channel(uint8_t channel);

/**
 * @brief Write and immediately update a DAC channel
 *
 * @param channel Channel index (0 to 7)
 * @param code 16-bit DAC value
 */
void cna_write_update_channel(uint8_t channel, uint16_t code);

/**
 * @brief Write a value and update all DAC outputs
 *
 * @param channel Channel index (ignored by hardware, kept for consistency)
 * @param code 16-bit DAC value
 */
void cna_write_update_all(uint8_t channel, uint16_t code);

/**
 * @brief Power up selected DAC channels
 *
 * @param channels Bitmask of channels (bit 0 = channel A, bit 7 = channel H)
 */
void cna_power_normal(uint8_t channels);

/**
 * @brief Power down selected DAC channels
 *
 * @param channels Bitmask of channels (bit 0 = channel A, bit 7 = channel H)
 * @param mode Power-down mode:
 *             - 0: 1kΩ to GND
 *             - 1: 100kΩ to GND
 *             - 2: High impedance (tri-state)
 */
void cna_power_down(uint8_t channels, uint8_t mode);

/**
 * @brief Enable the internal voltage reference of the DAC
 */
void cna_enable_internal_ref(void);

/**
 * @brief Disable the internal voltage reference of the DAC
 */
void cna_disable_internal_ref(void);

/**
 * @brief Perform a full DAC reset (power-on reset state)
 */
void cna_reset(void);

/**
 * @brief Toggle the LDAC pin to apply pending DAC values
 */
void cna_toggle_ldac(void);

#ifdef __cplusplus
}
#endif

#endif /* AD5668_CNA_H_ */
