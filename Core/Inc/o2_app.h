/**
 * @file    o2_app.h
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Module applicatif pour la lecture du capteur d'oxygène
 *
 * Module de niveau applicatif pour la lecture du capteur O2 via ADC1.
 * Fournit l'initialisation et la mise à jour périodique des données du capteur O2.
 */

#ifndef O2_APP_H_
#define O2_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdint.h>

/**
 * @brief Initialise le module capteur O2
 *
 * Calibre l'ADC1 et prépare le capteur pour les lectures.
 */
void o2_app_init(void);

/**
 * @brief Tâche périodique de lecture du capteur O2
 *
 * Lit la valeur ADC1 et met à jour la mesure O2 dans les données capteur.
 */
void o2_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* O2_APP_H_ */
