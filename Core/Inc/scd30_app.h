/**
 * @file    scd30_app.h
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Module applicatif pour le capteur SCD30 (CO2, température, humidité)
 *
 * Fournit l'initialisation et la tâche périodique de lecture du capteur SCD30
 * via I2C. Les données lues sont stockées via le module sensor_data.
 */

#ifndef SCD30_APP_H_
#define SCD30_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NO_ERROR 0 /**< Code de retour indiquant l'absence d'erreur */

#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdint.h>

/**
 * @brief Initialise le module SCD30
 *
 * Configure la communication I2C et démarre les mesures continues du capteur.
 */
void scd30_app_init(void);

/**
 * @brief Tâche périodique de lecture du SCD30
 *
 * Vérifie la disponibilité d'une nouvelle mesure et met à jour
 * les données de CO2, température et humidité dans sensor_data.
 */
void scd30_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* SCD30_APP_H_ */
