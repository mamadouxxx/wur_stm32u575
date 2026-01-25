/**
 * @file    ad5668_cna.h
 * @brief   Driver pour le DAC 8 canaux AD5668 (CNA)
 * @author  Mamadou
 * @date    25 janvier 2026
 *
 * Fournit l'API pour :
 *  - Initialisation SPI et GPIO
 *  - Lecture/écriture de tensions ou codes DAC
 *  - Gestion LDAC, CLR, power-down, reset et référence interne
 */

#ifndef AD5668_CNA_H_
#define AD5668_CNA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/** @brief Handle SPI utilisé pour le DAC */
extern SPI_HandleTypeDef hspi1;

/** @name Commandes du DAC AD5668
 * @{
 */
#define CNA_CMD_WRITE_INPUT_REG           0 /**< Écrit registre d'entrée d'un canal */
#define CNA_CMD_UPDATE_OUTPUT_REG         1 /**< Met à jour la sortie d'un canal */
#define CNA_CMD_WRITE_INPUT_UPDATE_ALL    2 /**< Écrit et met à jour tous les canaux */
#define CNA_CMD_WRITE_INPUT_UPDATE_N      3 /**< Écrit et met à jour un canal */
#define CNA_CMD_POWER_DOWN_UP             4 /**< Power up / power down */
#define CNA_CMD_LOAD_CLEAR_CODE           5 /**< Charge registre Clear Code */
#define CNA_CMD_LOAD_LDAC                 6 /**< Charge registre LDAC */
#define CNA_CMD_RESET_POWER_ON            7 /**< Reset complet */
#define CNA_CMD_SETUP_INTERNAL_REF        8 /**< Activer / désactiver référence interne */
/** @} */

/*=====================
  API publique
======================*/

/**
 * @brief Initialise le CNA / AD5668
 *
 * Configure SPI, GPIO et effectue :
 *  - Reset via CLR
 *  - Initialisation des tensions par défaut
 */
void cna_init(void);

/**
 * @brief Clear les sorties du DAC via le signal CLR
 */
void cna_clear(void);

/**
 * @brief Écrit une tension en mV sur un canal et applique via LDAC
 * @param channel Canal à mettre à jour (0 à 7)
 * @param voltage_mV Tension en millivolts
 */
void cna_set_voltage(uint8_t channel, float voltage_mV);

/**
 * @brief Écrit un code 16 bits dans le registre d'entrée d'un canal
 * @param channel Canal (0-7)
 * @param code Valeur DAC 16 bits
 */
void cna_write_channel(uint8_t channel, uint16_t code);

/**
 * @brief Met à jour la sortie d'un canal avec la valeur du registre d'entrée
 * @param channel Canal (0-7)
 */
void cna_update_channel(uint8_t channel);

/**
 * @brief Écrit un code dans le registre d'entrée et applique sur le canal
 * @param channel Canal (0-7)
 * @param code Valeur DAC 16 bits
 */
void cna_write_update_channel(uint8_t channel, uint16_t code);

/**
 * @brief Écrit un code sur un canal et applique sur tous les canaux
 * @param channel Canal (0-7)
 * @param code Valeur DAC 16 bits
 */
void cna_write_update_all(uint8_t channel, uint16_t code);

/**
 * @brief Met les canaux spécifiés en mode normal (powered up)
 * @param channels Bits 0-7 correspondant aux canaux A-H
 */
void cna_power_normal(uint8_t channels);

/**
 * @brief Met les canaux spécifiés en power-down
 * @param channels Bits 0-7 correspondant aux canaux A-H
 * @param mode 0 = 1kΩ, 1 = 100kΩ, 2 = tri-state
 */
void cna_power_down(uint8_t channels, uint8_t mode);

/**
 * @brief Active la référence interne du DAC
 */
void cna_enable_internal_ref(void);

/**
 * @brief Désactive la référence interne du DAC
 */
void cna_disable_internal_ref(void);

/**
 * @brief Reset complet du DAC
 */
void cna_reset(void);

/**
 * @brief Pulse le signal LDAC pour appliquer les valeurs
 */
void cna_toggle_ldac(void);

#ifdef __cplusplus
}
#endif

#endif /* AD5668_CNA_H_ */
