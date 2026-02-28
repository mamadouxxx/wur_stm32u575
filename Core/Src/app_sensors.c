/**
 * @file    app_sensors.c
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Implémentation du module applicatif de gestion des capteurs
 *
 * Orchestre l'initialisation et l'exécution périodique des tâches
 * pour tous les capteurs : SCD30, capteur de luminosité et capteur O2.
 */

#include "app_sensors.h"
#include "scd30_app.h"
#include "lux_app.h"
#include "o2_app.h"

void sensors_init(void)
{
    scd30_app_init();
//    lux_app_init();  // Décommenter quand la calibration du capteur de lumière est disponible
//    o2_app_init();   // Décommenter quand la calibration du capteur O2 est disponible
}

void sensors_task(void)
{
    scd30_app_task();
    lux_app_task();
    o2_app_task();
}
