/**
 * @file    hal_adc.c
 * @author  Mamadou
 * @date    17 jan 2026
 * @brief   Implémentation des fonctions HAL ADC
 *
 * Fournit les fonctions de lecture, calibration et conversion pour ADC1 et ADC4.
 */

#include "hal_adc.h"
#include "stm32u5xx_hal.h"
#include <math.h>

extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc1;

uint32_t hal_adc4_read(void)
{
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc4);
    HAL_ADC_Stop(&hadc4);

    if (val == 0) val = 1;  // évite la division par zéro
    return val;
}

uint32_t hal_adc1_read(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

uint32_t hal_adc4_calibration(void)
{
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    return 0;
}

uint32_t hal_adc1_calibration(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    return 0;
}
