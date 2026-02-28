/**
 * @file    o2_app.c
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Implémentation du module capteur O2
 *
 * Lit l'ADC1 et met à jour les données globales du capteur.
 */

#include "o2_app.h"
#include "sensor_data.h"
#include "hal_adc.h"

void o2_app_init(void)
{
    hal_adc1_calibration();
}

void o2_app_task(void)
{
    uint32_t adc_val = hal_adc1_read();
    sensor_data_update_o2(adc_val);
}
