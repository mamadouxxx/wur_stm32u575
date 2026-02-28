/**
 * @file    rf_ook_proto.c
 * @author  Mamadou
 * @date    20 jan 2026
 * @brief   Implémentation de la couche protocole OOK 433 MHz
 *
 * Fournit les fonctions de gestion des trames en découplant
 * le matériel (GPIO, timers) de la couche protocole.
 */

#include "rf_ook_proto.h"
#include "rf_ook_tx.h"
#include "rf_ook_rx.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim1; /**< Timer utilisé pour l'ISR de transmission TX */

static const uint8_t NODE_ADDRESS = 1;         /**< Adresse de ce nœud */
static rf_ook_tx_frame_t tx_frame;             /**< Contexte d'exécution de la trame TX courante */
static uint8_t seq_num = 0;                    /**< Numéro de séquence courant (0-15) */

static void rf_ook_proto_forward_frame(rf_ook_frame_t *frame);
static void process_frame(rf_ook_frame_t *frame);

/* -------------------------------------------------------------------------- */
/*                          Initialisation du protocole                       */
/* -------------------------------------------------------------------------- */

void rf_ook_proto_init(void)
{
    rf_ook_tx_init();
    rf_ook_rx_init();
}

/* -------------------------------------------------------------------------- */
/*                            Émission de trames                              */
/* -------------------------------------------------------------------------- */

void rf_ook_proto_send_frame(uint8_t dest_address, uint8_t *payload, uint8_t payload_len_bytes)
{
    if (tx_frame.active) return;
    if (payload_len_bytes > MAX_PAYLOAD_SIZE) return;
    if (payload == NULL && payload_len_bytes > 0) return;

    // Construction de la trame
    tx_frame.frame.sync_bits    = SYNC_BITS_VALUE;
    tx_frame.frame.src_address  = NODE_ADDRESS;
    tx_frame.frame.dest_address = dest_address;
    tx_frame.frame.seq_ttl      = MAKE_SEQ_TTL(seq_num, TTL_MAX);
    seq_num = (seq_num + 1) & 0x0F;
    memcpy(tx_frame.frame.payload, payload, payload_len_bytes);
    tx_frame.frame.payload_len  = payload_len_bytes;
    tx_frame.frame.crc          = rf_ook_compute_crc(&tx_frame.frame);

    // Initialisation du contexte TX
    tx_frame.byte_idx   = 0;
    tx_frame.bit_idx    = SYNC_NB_BITS - 1;
    tx_frame.state      = TX_SYNC;
    tx_frame.active     = true;
    tx_frame.wait_ticks = 0;

    // Démarrage de l'émission
    rf_ook_tx_start_tx();
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start_IT(&htim1);
}

/**
 * @brief Retransmet une trame avec TTL décrémenté
 *
 * Utilisé pour le routage multi-saut. Ignore la trame si TTL == 0.
 *
 * @param frame  Pointeur vers la trame à retransmettre
 */
static void rf_ook_proto_forward_frame(rf_ook_frame_t *frame)
{
    if (tx_frame.active) return;

    // Copie de la trame avec TTL décrémenté
    tx_frame.frame        = *frame;
    tx_frame.frame.seq_ttl = MAKE_SEQ_TTL(GET_SEQ(frame->seq_ttl), GET_TTL(frame->seq_ttl) - 1);
    tx_frame.frame.crc    = rf_ook_compute_crc(&tx_frame.frame);

    tx_frame.byte_idx   = 0;
    tx_frame.bit_idx    = SYNC_NB_BITS - 1;
    tx_frame.state      = TX_SYNC;
    tx_frame.active     = true;
    tx_frame.wait_ticks = 0;

    rf_ook_tx_start_tx();
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    HAL_TIM_Base_Start_IT(&htim1);
}

/* -------------------------------------------------------------------------- */
/*                            Réception de trames                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Traite une trame reçue et affiche les données capteurs sur UART
 *
 * Vérifie la taille du payload, extrait les données et les affiche.
 *
 * @param frame  Pointeur vers la trame reçue
 */
static void process_frame(rf_ook_frame_t *frame)
{
    if (frame->payload_len != sizeof(sensor_payload_t)) {
        char msg[50];
        int len = sprintf(msg, "Taille de trame invalide : %d\r\n", frame->payload_len);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
        return;
    }

    sensor_payload_t sensors;
    memcpy(&sensors, frame->payload, sizeof(sensor_payload_t));

    char msg[80];
    int len;

    len = sprintf(msg, "=== Données capteurs ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "CO2: %d ppm\r\n", sensors.co2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Température : %d\r\n", sensors.temp);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Humidité : %d\r\n", sensors.hum);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Lux : %d\r\n", sensors.lux);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "O2 : %d %%\r\n", sensors.o2);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "Mouvement : %d\r\n", sensors.motion);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);

    len = sprintf(msg, "========================\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
}

void rf_ook_proto_handle_received_frame(void)
{
    rf_ook_frame_t frame;

    if (!rf_ook_rx_is_frame_ready()) return;

    while (rf_ook_rx_get_frame(&frame)) {

        // Vérification CRC
        if (rf_ook_compute_crc(&frame) != frame.crc) {
            char msg[] = "ERREUR CRC\r\n";
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            continue;
        }

        if (frame.dest_address == NODE_ADDRESS) {
            // Trame destinée à ce nœud
            process_frame(&frame);
        } else {
            // Retransmission si TTL > 0
            uint8_t ttl = GET_TTL(frame.seq_ttl);
            if (ttl == 0) continue;
            rf_ook_proto_forward_frame(&frame);
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                         Accès et statut de la trame TX                     */
/* -------------------------------------------------------------------------- */

rf_ook_tx_frame_t* rf_ook_proto_get_tx_frame(void)
{
    return &tx_frame;
}

bool rf_ook_proto_is_busy(void)
{
    return tx_frame.active;
}

uint8_t rf_ook_get_node_address(void)
{
    return NODE_ADDRESS & ((1 << ADDRESS_BITS) - 1);
}
