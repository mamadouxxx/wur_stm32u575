/**
 * @file    lux_app.h
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Module applicatif pour la lecture de la luminosité ambiante
 *
 * Module de niveau applicatif pour la lecture de la luminosité via ADC4.
 * Fournit l'initialisation et la mise à jour périodique des données du capteur de lux.
 */

#ifndef LUX_APP_H_
#define LUX_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le module capteur de luminosité
 *
 * Effectue la calibration ADC et prépare le capteur pour les lectures.
 */
void lux_app_init(void);

/**
 * @brief Tâche périodique de lecture du capteur de luminosité
 *
 * Lit l'ADC, convertit la valeur en lux et met à jour le stockage des données capteur.
 */
void lux_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* LUX_APP_H_ */
