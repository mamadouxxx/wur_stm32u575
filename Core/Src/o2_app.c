/*
 * o2_app.c
 *
 *  Created on: 15 Jan 2026
 *      Author: Mamadou
 *
 *  Implementation of the O2 sensor module.
 *  Reads ADC1 and updates the global sensor data.
 */

/* Includes ------------------------------------------------------------------*/
#include "o2_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

/**
 * @brief Initialize the O2 sensor module.
 *        Calibrates ADC1 for accurate readings.
 */
void o2_app_init(void)
{
    hal_adc1_calibration();
}

/**
 * @brief Periodic task for O2 measurement.
 *        Reads the ADC1 value and updates the O2 measurement in the sensor data.
 */
void o2_app_task(void)
{
    uint32_t adc_val = hal_adc1_read();
    sensor_data_update_o2(adc_val);
}
