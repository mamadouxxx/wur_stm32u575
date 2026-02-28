/**
 * @file    hal_adc.h
 * @author  Mamadou
 * @date    17 jan 2026
 * @brief   Couche d'abstraction matérielle (HAL) pour les lectures ADC
 *
 * Fournit des fonctions pour lire les canaux ADC, effectuer la calibration
 * et convertir les valeurs ADC en éclairement (lux).
 */

#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Lit la valeur brute de l'ADC4
 *
 * Démarre la conversion ADC, attend la fin et retourne le résultat sur 12 bits.
 * @return Valeur ADC sur 12 bits (0..4095)
 */
uint32_t hal_adc4_read(void);

/**
 * @brief Lit la valeur brute de l'ADC1
 *
 * Identique à hal_adc4_read mais pour l'ADC1.
 * @return Valeur ADC sur 12 bits (0..4095)
 */
uint32_t hal_adc1_read(void);

/**
 * @brief Calibre l'ADC4
 *
 * Lance la calibration d'offset de l'ADC4 en mode single-ended.
 * @return Statut de calibration (0 si succès, code d'erreur sinon)
 */
uint32_t hal_adc4_calibration(void);

/**
 * @brief Calibre l'ADC1
 *
 * Lance la calibration d'offset de l'ADC1 en mode single-ended.
 * @return Statut de calibration (0 si succès, code d'erreur sinon)
 */
uint32_t hal_adc1_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H_ */
