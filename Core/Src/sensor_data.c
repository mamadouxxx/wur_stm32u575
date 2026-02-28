/**
 * @file    sensor_data.c
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Implémentation du module de stockage des données capteurs
 *
 * Maintient une structure globale des dernières valeurs mesurées
 * et fournit les accesseurs pour lecture et mise à jour.
 */

#include "sensor_data.h"
#include "stm32u5xx_hal.h"

sensor_data_t g_sensor_data = {0}; /**< Instance globale des données capteurs */

void sensor_data_update_scd30(float co2, float temp, float hum)
{
    __disable_irq();
    g_sensor_data.co2_ppm       = co2;
    g_sensor_data.temperature_c = temp;
    g_sensor_data.humidity_pct  = hum;
    __enable_irq();
}

void sensor_data_update_o2(uint32_t o2)
{
    __disable_irq();
    g_sensor_data.o2_raw = o2;
    __enable_irq();
}

void sensor_data_update_lux(uint32_t lux)
{
    __disable_irq();
    g_sensor_data.lux_raw = lux;
    __enable_irq();
}

void sensor_data_update_motion(uint8_t motion)
{
    __disable_irq();
    g_sensor_data.motion_raw = motion;
    __enable_irq();
}

float sensor_data_get_co2(void)
{
    __disable_irq();
    float val = g_sensor_data.co2_ppm;
    __enable_irq();
    return val;
}

float sensor_data_get_temperature(void)
{
    __disable_irq();
    float val = g_sensor_data.temperature_c;
    __enable_irq();
    return val;
}

float sensor_data_get_humidity(void)
{
    __disable_irq();
    float val = g_sensor_data.humidity_pct;
    __enable_irq();
    return val;
}

uint32_t sensor_data_get_o2(void)
{
    __disable_irq();
    uint32_t val = g_sensor_data.o2_raw;
    __enable_irq();
    return val;
}

uint32_t sensor_data_get_lux(void)
{
    __disable_irq();
    uint32_t val = g_sensor_data.lux_raw;
    __enable_irq();
    return val;
}

uint8_t sensor_data_get_motion(void)
{
    __disable_irq();
    uint8_t val = g_sensor_data.motion_raw;
    __enable_irq();
    return val;
}
