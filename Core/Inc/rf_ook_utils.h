/**
 * @file    rf_ook_types.h
 * @author  Mamadou
 * @date    24 jan 2026
 * @brief   Types communs et constantes du protocole OOK 433 MHz
 *
 * Définit toutes les constantes, structures et énumérations partagées
 * par la pile protocole OOK (couches TX, RX et protocole).
 *
 * Le protocole est conçu pour la communication capteur basse consommation
 * à faible débit, en modulation OOK avec radio de réveil (WuR).
 */

#ifndef RF_OOK_TYPES_H_
#define RF_OOK_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                            Constantes du protocole                         */
/* -------------------------------------------------------------------------- */

#define TTL_MAX 3 /**< Nombre maximum de sauts (Time To Live) */

/** @brief Extrait le numéro de séquence du champ seq_ttl */
#define GET_SEQ(seq_ttl)       (((seq_ttl) >> 4) & 0x0F)

/** @brief Extrait le TTL du champ seq_ttl */
#define GET_TTL(seq_ttl)       ((seq_ttl) & 0x0F)

/** @brief Construit le champ seq_ttl à partir du numéro de séquence et du TTL */
#define MAKE_SEQ_TTL(seq, ttl) ((((seq) & 0x0F) << 4) | ((ttl) & 0x0F))

/** @brief Débit binaire par défaut pour la communication OOK (bits par seconde) */
#define RF_OOK_DEFAULT_BPS    1300UL

/** @brief Nombre de bits de synchronisation par trame */
#define SYNC_NB_BITS          16

/** @brief Motif de synchronisation envoyé avant chaque trame */
#define SYNC_BITS_VALUE       0xAAAA

/** @brief Nombre de bits utilisés pour l'adressage des nœuds */
#define ADDRESS_BITS          2

/** @brief Durée d'un bit en microsecondes (calculée à partir du débit) */
#define BIT_US                (1000000UL / RF_OOK_DEFAULT_BPS)

/** @brief Délai après le SYNC pour permettre le réveil du récepteur WuR (en microsecondes) */
#define RF_WAKEUP_DELAY_US    20000

/** @brief Nombre de bits du champ longueur de payload */
#define LENGTH_BITS           8

/** @brief Nombre de trames pouvant être mises en attente dans le FIFO RX */
#define BUFFER_SIZE           4

/** @brief Taille maximale du payload en octets */
#define MAX_PAYLOAD_SIZE      32

/** @brief Taille de la file de fronts EXTI */
#define EDGE_QUEUE_SIZE       32

/** @brief Fréquence de l'horloge TIM3 (Hz) */
#define TIMER_FREQ_HZ         16000000UL

/** @brief Durée d'un bit en ticks TIM3 */
#define BIT_TICKS             (TIMER_FREQ_HZ / RF_OOK_DEFAULT_BPS)

/** @brief Filtre anti-rebond : durée minimale d'un front valide (1/4 de bit) */
#define EDGE_FILTER_TICKS     (BIT_TICKS / 4)

/** @brief Timeout FSM RX : durée de silence équivalente à 20 bits sans activité */
#define RX_TIMEOUT_TICKS      (BIT_TICKS * 20)

#define NODE_IS_TRANSMITTER   // Décommenter pour configurer ce MCU en émetteur

/* -------------------------------------------------------------------------- */
/*                            Structures de données                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Événement de front détecté sur RX_DATA
 */
typedef struct {
    uint32_t delta_ticks; /**< Durée écoulée depuis le front précédent (en ticks TIM3) */
    uint8_t  level;       /**< Niveau logique du plateau précédent (0 ou 1) */
} edge_event_t;

/**
 * @brief Payload capteur transmis dans une trame OOK
 */
typedef struct __attribute__((packed)) {
    uint16_t co2;    /**< Concentration CO2 (ppm) */
    uint16_t temp;   /**< Température */
    uint16_t hum;    /**< Humidité relative */
    uint16_t lux;    /**< Éclairement (lux) */
    uint16_t o2;     /**< Concentration O2 */
    uint8_t  motion; /**< Détection de mouvement (0 = aucun) */
} sensor_payload_t;

/**
 * @brief Structure d'une trame OOK logique
 *
 * Représente une trame protocole complète après décodage (ou avant encodage),
 * sans les informations de timing ni de couche physique.
 */
typedef struct {
    uint16_t sync_bits;                  /**< Motif de synchronisation */
    uint8_t  src_address;                /**< Adresse du nœud source */
    uint8_t  dest_address;               /**< Adresse du nœud destinataire */
    uint8_t  seq_ttl;                    /**< Numéro de séquence (4 bits) + TTL (4 bits) */
    uint8_t  payload_len;                /**< Longueur du payload en octets */
    uint8_t  payload[MAX_PAYLOAD_SIZE];  /**< Données du payload */
    uint8_t  crc;                        /**< CRC de contrôle d'intégrité */
} rf_ook_frame_t;

/* -------------------------------------------------------------------------- */
/*                            Machines d'état                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief États de la machine d'état d'émission (TX)
 */
typedef enum {
    TX_SYNC         = 0, /**< Envoi des bits de synchronisation */
    TX_LENGTH,           /**< Envoi de la longueur du payload */
    TX_SRC_ADDRESS,      /**< Envoi de l'adresse source */
    TX_DEST_ADDRESS,     /**< Envoi de l'adresse destination */
    TX_SEQ_TTL,          /**< Envoi du champ séquence + TTL */
    TX_PAYLOAD,          /**< Envoi des octets de payload */
    TX_CRC,              /**< Envoi du CRC */
    TX_DONE              /**< Transmission terminée */
} tx_state_t;

/**
 * @brief États de la machine d'état de réception (RX)
 */
typedef enum {
    RX_SYNC         = 0, /**< Attente du motif de synchronisation */
    RX_LENGTH,           /**< Réception de la longueur du payload */
    RX_SRC_ADDRESS,      /**< Réception de l'adresse source */
    RX_DEST_ADDRESS,     /**< Réception de l'adresse destination */
    RX_SEQ_TTL,          /**< Réception du champ séquence + TTL */
    RX_PAYLOAD,          /**< Réception des octets de payload */
    RX_CRC,              /**< Réception du CRC */
    RX_DONE              /**< Trame complète reçue */
} rx_state_t;

/* -------------------------------------------------------------------------- */
/*                         Contexte d'émission TX                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Contexte d'exécution de la transmission TX
 *
 * Contient l'état courant d'une transmission en cours,
 * mis à jour à chaque interruption timer.
 */
typedef struct {
    rf_ook_frame_t frame;    /**< Trame en cours de transmission */
    uint8_t        byte_idx; /**< Index de l'octet courant dans le payload */
    int8_t         bit_idx;  /**< Index du bit courant dans l'octet */
    tx_state_t     state;    /**< État courant de la machine d'état TX */
    volatile bool  active;   /**< Indique si une transmission est en cours */
    uint16_t       wait_ticks; /**< Compteur utilisé pendant l'état d'attente TX */
} rf_ook_tx_frame_t;

/* -------------------------------------------------------------------------- */
/*                              Fonctions utilitaires                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Calcule le CRC d'une trame OOK
 *
 * XOR de l'adresse destination, source, seq_ttl, longueur et tous les octets du payload.
 *
 * @param frame  Pointeur vers la trame dont calculer le CRC
 * @return       Valeur CRC calculée sur 8 bits
 */
static inline uint8_t rf_ook_compute_crc(rf_ook_frame_t *frame)
{
    uint8_t crc = 0;
    crc ^= frame->dest_address;
    crc ^= frame->src_address;
    crc ^= frame->seq_ttl;
    crc ^= frame->payload_len;
    for (uint8_t i = 0; i < frame->payload_len; i++) crc ^= frame->payload[i];
    return crc;
}

#endif /* RF_OOK_TYPES_H_ */
