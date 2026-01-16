/*
 * lux_app.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "lux_app.h"
#include "sensor_data.h"

extern ADC_HandleTypeDef hadc4;

void lux_app_init(void)
{
	}

void lux_app_task(void)
{
    HAL_ADC_Start(&hadc4);
    HAL_ADC_PollForConversion(&hadc4, HAL_MAX_DELAY);
    sensor_data_update_lux(HAL_ADC_GetValue(&hadc4));
    HAL_ADC_Stop(&hadc4);
}
