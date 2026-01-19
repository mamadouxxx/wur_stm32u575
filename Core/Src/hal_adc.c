/*
 * hal_adc.c
 *
 *  Created on: 17 janv. 2026
 *      Author: mamadou
 */


/* Includes ------------------------------------------------------------------*/
#include "hal_adc.h"
#include "stm32u5xx_hal.h"
#include <math.h>

#define PHT_UP_R    10000.0f
#define PHT_10LX_R  50000.0f
#define PHT_GAMMA   0.8f

extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc1;

uint32_t hal_adc4_read(void)
{
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc4);
    HAL_ADC_Stop(&hadc4);

    if (val == 0) val = 1;                       // éviter division par zéro
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

uint32_t hal_adc4_calibration()
{
	HAL_ADCEx_Calibration_Start(&hadc4, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

uint32_t hal_adc1_calibration()
{
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
}

/**
 * non utilisé pour l'instant
 */
float lux_from_adc(uint32_t adc_val)
{
    float r_photo = PHT_UP_R / ((4095.0f / (float)adc_val) - 1.0f);
    float ratio   = PHT_10LX_R / r_photo;
    float temp    = (0.42f * logf(ratio)) / PHT_GAMMA + 1.0f;
    float lux     = powf(10.0f, temp);
    return lux;
}



