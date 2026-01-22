/**
 * @file hal_adc.c
 * @author Mamadou
 * @date 17 Jan 2026
 * @brief Implementation of HAL ADC functions
 *
 * Provides reading, calibration, and conversion functions for ADC1 and ADC4.
 */

#include "hal_adc.h"
#include "stm32u5xx_hal.h"
#include <math.h>

/* Phototransistor parameters for lux calculation */
#define PHT_UP_R    10000.0f   /**< Pull-up resistor */
#define PHT_10LX_R  50000.0f   /**< Resistor corresponding to 10 lux */
#define PHT_GAMMA   0.8f       /**< Gamma correction factor */

extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc1;

/**
 * @brief Read raw value from ADC4
 */
uint32_t hal_adc4_read(void)
{
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc4);
    HAL_ADC_Stop(&hadc4);

    if (val == 0) val = 1;  // avoid division by zero
    return val;
}

/**
 * @brief Read raw value from ADC1
 */
uint32_t hal_adc1_read(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

/**
 * @brief Calibrate ADC4
 */
uint32_t hal_adc4_calibration(void)
{
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    return 0; // Return 0 for success (can modify if needed)
}

/**
 * @brief Calibrate ADC1
 */
uint32_t hal_adc1_calibration(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    return 0;
}

/**
 * @brief Convert ADC value to lux
 */
float lux_from_adc(uint32_t adc_val)
{
    float r_photo = PHT_UP_R / ((4095.0f / (float)adc_val) - 1.0f);
    float ratio   = PHT_10LX_R / r_photo;
    float temp    = (0.42f * logf(ratio)) / PHT_GAMMA + 1.0f;
    float lux     = powf(10.0f, temp);
    return lux;
}
