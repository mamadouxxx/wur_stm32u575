/**
 * @file    ad5668_cna.h
 * @brief   Pilote CNA (DAC) AD5668 8 canaux
 * @author  Mamadou
 * @date    25 janvier 2026
 *
 * Ce pilote fournit une API haut niveau pour contrôler le DAC AD5668 via SPI.
 *
 * Fonctionnalités :
 *  - Communication SPI avec l'AD5668
 *  - Contrôle du DAC par tension (mV) ou par code brut
 *  - Gestion du LDAC, CLR, mise en veille et reset
 *  - Contrôle de la référence de tension interne
 *
 * Conçu pour les systèmes embarqués basse consommation (ex. projets Wake-up Radio).
 */

#ifndef AD5668_CNA_H_
#define AD5668_CNA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Handle SPI utilisé par le pilote AD5668
 */
extern SPI_HandleTypeDef hspi1;

/** @name Codes de commande AD5668
 * @{
 */
#define CNA_CMD_WRITE_INPUT_REG           0 /**< Écriture dans le registre d'entrée */
#define CNA_CMD_UPDATE_OUTPUT_REG         1 /**< Mise à jour du registre de sortie DAC */
#define CNA_CMD_WRITE_INPUT_UPDATE_ALL    2 /**< Écriture et mise à jour de tous les canaux */
#define CNA_CMD_WRITE_INPUT_UPDATE_N      3 /**< Écriture et mise à jour du canal sélectionné */
#define CNA_CMD_POWER_DOWN_UP             4 /**< Commande mise en veille / réveil */
#define CNA_CMD_LOAD_CLEAR_CODE           5 /**< Chargement du registre de code de remise à zéro */
#define CNA_CMD_LOAD_LDAC                 6 /**< Chargement du registre LDAC */
#define CNA_CMD_RESET_POWER_ON            7 /**< Reset complet du DAC */
#define CNA_CMD_SETUP_INTERNAL_REF        8 /**< Activation / désactivation de la référence interne */
/** @} */

/**
 * @brief Initialise le DAC AD5668
 *
 * Cette fonction :
 *  - Réinitialise le DAC via la broche CLR
 *  - Met le LDAC en état inactif
 *  - Charge les tensions par défaut sur tous les canaux
 */
void cna_init(void);

/**
 * @brief Remet à zéro toutes les sorties DAC via la broche CLR
 */
void cna_clear(void);

/**
 * @brief Définit la tension de sortie d'un canal DAC (en millivolts)
 *
 * @param channel     Index du canal (0 à 7)
 * @param voltage_mV  Tension de sortie souhaitée en millivolts
 */
void cna_set_voltage(uint8_t channel, float voltage_mV);

/**
 * @brief Écrit un code DAC 16 bits brut dans le registre d'entrée
 *
 * @param channel  Index du canal (0 à 7)
 * @param code     Valeur DAC sur 16 bits
 */
void cna_write_channel(uint8_t channel, uint16_t code);

/**
 * @brief Met à jour le registre de sortie DAC d'un canal
 *
 * @param channel  Index du canal (0 à 7)
 */
void cna_update_channel(uint8_t channel);

/**
 * @brief Écrit et met à jour immédiatement un canal DAC
 *
 * @param channel  Index du canal (0 à 7)
 * @param code     Valeur DAC sur 16 bits
 */
void cna_write_update_channel(uint8_t channel, uint16_t code);

/**
 * @brief Écrit une valeur et met à jour toutes les sorties DAC
 *
 * @param channel  Index du canal (ignoré par le matériel, conservé par cohérence)
 * @param code     Valeur DAC sur 16 bits
 */
void cna_write_update_all(uint8_t channel, uint16_t code);

/**
 * @brief Remet en fonctionnement normal les canaux DAC sélectionnés
 *
 * @param channels  Masque de bits des canaux (bit 0 = canal A, bit 7 = canal H)
 */
void cna_power_normal(uint8_t channels);

/**
 * @brief Met en veille les canaux DAC sélectionnés
 *
 * @param channels  Masque de bits des canaux (bit 0 = canal A, bit 7 = canal H)
 * @param mode      Mode de mise en veille :
 *                   - 0 : 1kΩ vers GND
 *                   - 1 : 100kΩ vers GND
 *                   - 2 : Haute impédance (tri-state)
 */
void cna_power_down(uint8_t channels, uint8_t mode);

/**
 * @brief Active la référence de tension interne du DAC
 */
void cna_enable_internal_ref(void);

/**
 * @brief Désactive la référence de tension interne du DAC
 */
void cna_disable_internal_ref(void);

/**
 * @brief Effectue un reset complet du DAC (état de mise sous tension)
 */
void cna_reset(void);

/**
 * @brief Bascule la broche LDAC pour appliquer les valeurs DAC en attente
 */
void cna_toggle_ldac(void);

#ifdef __cplusplus
}
#endif

#endif /* AD5668_CNA_H_ */
