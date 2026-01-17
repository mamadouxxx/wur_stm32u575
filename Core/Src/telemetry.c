/*
 * telemetry.c
 *
 *  Created on: 16 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "telemetry.h"
#include "hal_uart.h"
#include "sensor_data.h"
#include <stdio.h>
#include <string.h>


void telemetry_send_uart(void)
{
    char msg[160];

    sprintf(msg,
    	"===========================================\r\n"
    	"CO2: %.1f ppm | T: %.2f C | RH: %.2f %%\r\n"
        "Lux :%lu | O2 :%lu\r\n",
        sensor_data_get_co2(),
        sensor_data_get_temperature(),
        sensor_data_get_humidity(),
        sensor_data_get_lux(),
        sensor_data_get_o2()
    );

    hal_uart1_write((uint8_t*)msg, strlen(msg));
}
