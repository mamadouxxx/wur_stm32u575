/**
 * @file    scd30_app.c
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Implémentation du module applicatif SCD30
 *
 * Initialise le capteur SCD30 et lit périodiquement les mesures
 * de CO2, température et humidité via I2C.
 */

#include "scd30_app.h"
#include "sensor_data.h"
#include "scd30_i2c.h"

static uint16_t interval_in_seconds = 2; /**< Intervalle de mesure en secondes */

void scd30_app_init(void)
{
    scd30_init(SCD30_I2C_ADDR_61);
    scd30_set_measurement_interval(interval_in_seconds);
    scd30_start_periodic_measurement(0);
}

void scd30_app_task(void)
{
    uint16_t data_ready = 0;

    if (scd30_get_data_ready(&data_ready) != NO_ERROR) return;
    if (!data_ready) return;

    float co2, temp, hum;
    if (scd30_read_measurement_data(&co2, &temp, &hum) != NO_ERROR) return;

    sensor_data_update_scd30(co2, temp, hum);
}
