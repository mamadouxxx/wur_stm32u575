/*
 * lux_app.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "lux_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

/**
 *
 */
void lux_app_init(void)
{
	hal_adc4_calibration();
}

/**
 *
 */
void lux_app_task(void)
{
    sensor_data_update_lux(
    		hal_adc4_read()
	);
}
