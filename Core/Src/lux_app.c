/**
 * @file    lux_app.c
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Implémentation du module capteur de luminosité
 *
 * Lit l'ADC4 et met à jour les données globales du capteur.
 */

/* Includes ------------------------------------------------------------------*/
#include "lux_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

void lux_app_init(void)
{
    hal_adc4_calibration();
}

void lux_app_task(void)
{
    uint32_t adc_val = hal_adc4_read();
    sensor_data_update_lux(adc_val);
}
