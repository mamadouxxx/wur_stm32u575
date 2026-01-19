/*
 * app_sensors.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "app_sensors.h"
#include "scd30_app.h"
#include "lux_app.h"
#include "o2_app.h"

void sensors_init(void)
{
    scd30_app_init();
//    lux_app_init();
//    o2_app_init();
}

void sensors_task(void)
{
    scd30_app_task();
    lux_app_task();
    o2_app_task();
}

