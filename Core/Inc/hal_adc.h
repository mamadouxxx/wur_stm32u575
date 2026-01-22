/**
 * @file hal_adc.h
 * @author Mamadou
 * @date 17 Jan 2026
 * @brief Hardware Abstraction Layer (HAL) for ADC readings
 *
 * Provides functions to read ADC channels, perform calibration,
 * and convert ADC values to illuminance (lux).
 */

#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Read raw value from ADC4
 *
 * Starts the ADC conversion, waits for it to complete, and returns the 12-bit result.
 * @return 12-bit ADC value (0..4095)
 */
uint32_t hal_adc4_read(void);

/**
 * @brief Read raw value from ADC1
 *
 * Similar to hal_adc4_read but for ADC1.
 * @return 12-bit ADC value (0..4095)
 */
uint32_t hal_adc1_read(void);

/**
 * @brief Calibrate ADC4
 *
 * Starts the offset calibration for ADC4 in single-ended mode.
 * @return Calibration status (0 if successful, otherwise error code)
 */
uint32_t hal_adc4_calibration(void);

/**
 * @brief Calibrate ADC1
 *
 * Starts the offset calibration for ADC1 in single-ended mode.
 * @return Calibration status (0 if successful, otherwise error code)
 */
uint32_t hal_adc1_calibration(void);

/**
 * @brief Convert ADC value to illuminance (lux)
 *
 * Uses a phototransistor model to convert a raw ADC reading into
 * approximate lux value.
 * @param adc_val Raw ADC value
 * @return Calculated illuminance in lux
 */
float lux_from_adc(uint32_t adc_val);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H_ */
