/*
 * sensor_data.c
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

/* Includes ------------------------------------------------------------------*/
#include "sensor_data.h"

sensor_data_t g_sensor_data = {0};

void sensor_data_update_scd30(float co2, float temp, float hum)
{
	g_sensor_data.co2_ppm = co2;
	g_sensor_data.temperature_c = temp;
	g_sensor_data.humidity_pct = hum;
}

void sensor_data_update_o2 (uint32_t o2)
{
	g_sensor_data.o2_raw = o2;
}

void sensor_data_update_lux(uint32_t lux)
{
	g_sensor_data.lux_raw = lux;
}

void sensor_data_update_motion(uint8_t motion) {
	g_sensor_data.motion_raw = motion;
}

float sensor_data_get_co2(void)
{
    return g_sensor_data.co2_ppm;
}

float sensor_data_get_temperature(void)
{
    return g_sensor_data.temperature_c;
}

float sensor_data_get_humidity(void)
{
    return g_sensor_data.humidity_pct;
}

uint32_t sensor_data_get_o2(void)
{
    return g_sensor_data.o2_raw;
}

uint32_t sensor_data_get_lux(void)
{
    return g_sensor_data.lux_raw;
}

uint8_t sensor_data_get_motion(void) {
	return g_sensor_data.motion_raw;
}

