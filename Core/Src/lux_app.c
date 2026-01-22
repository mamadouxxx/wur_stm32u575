/*
 * lux_app.c
 *
 *  Created on: 15 Jan 2026
 *      Author: Mamadou
 *
 *  Implementation of the light sensor module.
 *  Reads ADC4 and updates the global sensor data.
 */

/* Includes ------------------------------------------------------------------*/
#include "lux_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

/**
 * @brief Initialize the light sensor module.
 *        Calibrates ADC4 for accurate readings.
 */
void lux_app_init(void)
{
    hal_adc4_calibration();
}

/**
 * @brief Periodic task for light measurement.
 *        Reads the ADC4 value and updates the lux measurement in the sensor data.
 */
void lux_app_task(void)
{
    uint32_t adc_val = hal_adc4_read();
    sensor_data_update_lux(adc_val);
}
