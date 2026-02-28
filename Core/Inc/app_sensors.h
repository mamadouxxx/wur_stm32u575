/**
 * @file    app_sensors.h
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Module applicatif de gestion des capteurs
 *
 * Module de niveau applicatif pour la gestion de l'ensemble des capteurs.
 * Fournit l'initialisation et les tâches de lecture périodique.
 *
 * Capteurs inclus :
 *   - SCD30 (CO2, température, humidité)
 *   - Capteur de luminosité (via ADC4)
 *   - Capteur O2 (via ADC1)
 *
 * Ce module abstrait les pilotes individuels et offre une interface unique
 * à l'application pour l'initialisation et la lecture périodique.
 */

#ifndef APP_SENSORS_H_
#define APP_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

/**
 * @brief Initialise tous les capteurs
 *
 * Initialise chaque module capteur.
 * À appeler une seule fois au démarrage du système.
 */
void sensors_init(void);

/**
 * @brief Exécute les tâches de lecture des capteurs
 *
 * À appeler périodiquement (ex. dans la boucle principale ou un ordonnanceur)
 * pour lire les données des capteurs et mettre à jour l'état interne.
 */
void sensors_task(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_H_ */
