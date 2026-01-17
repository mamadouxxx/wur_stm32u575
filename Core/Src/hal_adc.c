/*
 * hal_adc.c
 *
 *  Created on: 17 janv. 2026
 *      Author: mamadou
 */


/* Includes ------------------------------------------------------------------*/
#include "hal_adc.h"
#include "stm32u5xx_hal.h"

extern ADC_HandleTypeDef hadc4;
extern ADC_HandleTypeDef hadc1;

uint32_t hal_adc4_read(void)
{
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    uint32_t val = HAL_ADC_GetValue(&hadc4);
    HAL_ADC_Stop(&hadc4);
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
