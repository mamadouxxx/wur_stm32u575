/**
 * @file    rf_ook_proto.h
 * @author  Mamadou
 * @date    20 jan 2026
 * @brief   Couche applicative du protocole OOK 433 MHz
 *
 * Fournit les fonctions d'envoi et de réception de trames OOK (SYNC + adresse + payload)
 * via la machine d'état TX et les interruptions timer.
 * Cette couche abstrait le matériel de l'application.
 */

#ifndef RF_OOK_PROTO_H_
#define RF_OOK_PROTO_H_

#include <rf_ook_utils.h>
#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise le module protocole OOK
 *
 * Configure les modules TX et RX et prépare la couche protocole
 * pour l'envoi et la réception de trames.
 */
void rf_ook_proto_init(void);

/**
 * @brief Envoie une trame OOK complète
 *
 * Prépare une trame avec SYNC, adresse et payload, et démarre
 * la transmission via la machine d'état TX et les interruptions timer.
 *
 * @param address            Adresse du nœud destinataire (2 bits)
 * @param payload            Pointeur vers le tableau de données payload
 * @param payload_len_bytes  Nombre d'octets du payload
 */
void rf_ook_proto_send_frame(uint8_t address, uint8_t *payload, uint8_t payload_len_bytes);

/**
 * @brief Traite les trames reçues depuis le FIFO RX
 *
 * Permet de décoder, dispatcher ou retransmettre les trames reçues.
 */
void rf_ook_proto_handle_received_frame(void);

/**
 * @brief Retourne un pointeur vers la structure de trame TX courante
 *
 * Permet de consulter ou modifier l'état de la trame depuis l'application.
 *
 * @return Pointeur vers le contexte de transmission actif
 */
rf_ook_tx_frame_t* rf_ook_proto_get_tx_frame(void);

/**
 * @brief Indique si la couche protocole est en cours de transmission
 *
 * @return true si une transmission est active, false sinon
 */
bool rf_ook_proto_is_busy(void);

/**
 * @brief Retourne l'adresse du nœud de ce MCU
 *
 * @return Adresse du nœud en uint8_t (limitée par ADDRESS_BITS)
 */
uint8_t rf_ook_get_node_address(void);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_PROTO_H_ */
