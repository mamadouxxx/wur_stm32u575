/*
 * o2_app.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "o2_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

void o2_app_init(void)
{
    // Rien à faire
}

void o2_app_task(void)
{
    sensor_data_update_o2(hal_adc1_read());
}
