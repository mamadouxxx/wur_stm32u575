/*
 *
 *  Created on: 21 janv. 2026
 *      Author: mamadou
 *
/**
 * @file rf_ook_rx.h
 * @brief Réception OOK 433 MHz (WuR / RX) avec FSM
 *
 * Détecte :
 *  - Preamble (alternating 1 and 0)
 *  - SYNC (101100)
 *  - Adresse (2 bits)
 *  - Payload
 *
 * Utilise TIM3 pour déclencher une ISR toutes les BIT_US microsecondes.
 */

#ifndef RF_OOK_RX_H_
#define RF_OOK_RX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include "rf_ook_phy.h"

/* Adresse du nœud courant */
#define NODE_ADDRESS          0b11         /**< adresse du nœud */

typedef enum {
    RX_IDLE = 0,
    RX_PREAMBLE,
    RX_SYNC,
    RX_ADDRESS,
    RX_PAYLOAD
} rf_ook_rx_state_t;

extern volatile rf_ook_rx_state_t rx_state; /**< état actuel de la FSM */
extern volatile uint8_t rx_shift_reg;       /**< registre à décalage pour la détection */
extern volatile uint8_t rx_bit_count;       /**< compteur de bits reçus dans l'état courant */
extern volatile uint8_t rx_address;         /**< adresse reçue */
extern volatile uint8_t rx_payload;         /**< payload reçu (1 octet ici, modifier si nécessaire) */
extern volatile uint8_t rx_frame_ready;     /**< flag frame complète reçue */

/**
 * @brief Initialisation du module RX OOK
 *
 * Configure variables, reset FSM.
 */
void rf_ook_rx_init(void);

/**
 * @brief Reset de la FSM RX
 */
void rf_ook_rx_reset(void);

/**
 * @brief Fonction à appeler dans HAL_TIM_PeriodElapsedCallback pour TIM3
 *
 * Lit le pin RX_DATA, décode préambule, SYNC, adresse et payload.
 */
void rf_ook_rx_bit_handler(uint8_t bit);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_RX_H_ */
