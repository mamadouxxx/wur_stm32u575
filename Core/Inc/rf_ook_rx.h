/**
 * @file    rf_ook_rx.h
 * @author  Mamadou
 * @date    21 jan 2026
 * @brief   Module récepteur OOK 433 MHz (RX)
 *
 * Fournit une machine d'état (FSM) pour décoder les trames OOK
 * (adresse + payload) et les stocker dans un buffer circulaire.
 */

#ifndef RF_OOK_RX_H_
#define RF_OOK_RX_H_

#include <rf_ook_utils.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

extern volatile uint32_t last_edge_time; /**< Timestamp (ticks TIM3) du dernier front détecté sur RX_DATA.
                                              Mis à jour en IRQ EXTI via rf_ook_rx_handle_edge().
                                              Recaler après Stop2 : last_edge_time = TIM3->CNT. */

/**
 * @brief Initialise le module RX OOK
 *
 * Réinitialise la FSM RX et vide le buffer de réception.
 */
void rf_ook_rx_init(void);

/**
 * @brief Réinitialise la FSM RX
 *
 * Remet les compteurs internes à zéro et prépare le module à recevoir une nouvelle trame.
 */
void rf_ook_rx_reset(void);

/**
 * @brief Injecte un bit reçu dans la FSM RX
 *
 * À appeler depuis une ISR ou le module RX physique à chaque nouveau bit reçu.
 * La FSM met automatiquement à jour les buffers d'adresse et de payload.
 *
 * @param bit  Bit reçu (0 ou 1)
 */
void rf_ook_rx_receive_bit(uint8_t bit);

/**
 * @brief Lit une trame reçue depuis le buffer circulaire RX
 *
 * Si une trame est disponible, elle est copiée dans @p frame et retirée du buffer.
 *
 * @param frame  Pointeur vers une structure rf_ook_frame_t pour stocker la trame reçue
 * @return 1 si une trame était disponible et a été copiée, 0 si le buffer est vide
 */
uint8_t rf_ook_rx_get_frame(rf_ook_frame_t *frame);

/**
 * @brief Vérifie si une trame est disponible dans le buffer RX
 *
 * @return true si au moins une trame est disponible, false sinon
 */
bool rf_ook_rx_is_frame_ready(void);

/**
 * @brief Enregistre un front détecté sur RX_DATA
 *
 * À appeler depuis le callback EXTI à chaque front montant ou descendant.
 *
 * @param current_level  Niveau logique actuel de la pin RX_DATA (0 ou 1)
 */
void rf_ook_rx_handle_edge(uint8_t current_level);

/**
 * @brief Vérifie si des fronts sont en attente dans la file
 *
 * @return true si la file de fronts contient des événements, false sinon
 */
bool rf_ook_rx_has_edges(void);

/**
 * @brief Traite les fronts en attente dans la file et alimente la FSM RX
 *
 * À appeler périodiquement dans la boucle principale pour décoder les bits reçus.
 */
void rf_ook_rx_process_queue(void);

#endif /* RF_OOK_RX_H_ */
