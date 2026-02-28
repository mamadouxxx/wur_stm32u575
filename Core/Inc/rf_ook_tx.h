/**
 * @file    rf_ook_tx.h
 * @author  Mamadou
 * @date    19 jan 2026
 * @brief   Module émetteur OOK 433 MHz (TX)
 *
 * Fournit les fonctions de transmission de bits en modulation OOK
 * et gère les états de transmission au niveau trame.
 */

#ifndef RF_OOK_TX_H_
#define RF_OOK_TX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <rf_ook_utils.h>
#include <stdbool.h>
#include "main.h"

/**
 * @brief Initialise le module émetteur OOK
 *
 * Configure les GPIO et prépare le matériel RF.
 * Par défaut, l'antenne est placée en mode réception.
 */
void rf_ook_tx_init(void);

/**
 * @brief Démarre une transmission RF OOK
 *
 * Active la chaîne d'émission RF et met l'émetteur sous tension.
 * Après cet appel, les bits peuvent être envoyés via rf_ook_tx_send_bit().
 */
void rf_ook_tx_start_tx(void);

/**
 * @brief Termine une transmission RF OOK
 *
 * Arrête toute transmission en cours et met hors tension la chaîne d'émission RF.
 */
void rf_ook_end_tx(void);

/**
 * @brief Envoie un bit OOK via la machine d'état TX
 *
 * Sélectionne automatiquement le bit à envoyer selon l'état TX courant
 * (SYNC, ATTENTE, SRC_ADRESSE, DEST_ADRESSE, PAYLOAD, CRC).
 *
 * @param frame  Pointeur vers le contexte de trame TX
 */
void rf_ook_tx_send_bit(rf_ook_tx_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_TX_H_ */
