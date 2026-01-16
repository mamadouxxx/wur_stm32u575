/*
 * o2_app.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "o2_app.h"
#include "sensor_data.h"

extern ADC_HandleTypeDef hadc1;

void o2_app_init(void)
{
    // Rien à faire
}

void o2_app_task(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    sensor_data_update_o2(HAL_ADC_GetValue(&hadc1));
    HAL_ADC_Stop(&hadc1);
}
