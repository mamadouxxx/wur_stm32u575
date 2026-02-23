/**
 * @file rf_ook_types.h
 * @author Mamadou
 * @date 24 Jan 2026
 * @brief Common data types and protocol constants for OOK 433 MHz communication.
 *
 * This file defines all shared constants, structures, and enumerations
 * used by the OOK protocol stack (TX, RX, and protocol layers).
 *
 * The protocol is designed for low-power, low-bitrate sensor communication
 * using On-Off Keying (OOK) modulation and Wake-up Radio (WuR).
 */

#ifndef RF_OOK_TYPES_H_
#define RF_OOK_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                               Protocol constants                           */
/* -------------------------------------------------------------------------- */

#define TTL_MAX 3

// Macros utilitaires
#define GET_SEQ(seq_ttl)  (((seq_ttl) >> 4) & 0x0F)
#define GET_TTL(seq_ttl)  ((seq_ttl) & 0x0F)
#define MAKE_SEQ_TTL(seq, ttl) ((((seq) & 0x0F) << 4) | ((ttl) & 0x0F))

/** @brief Default bitrate for OOK communication (bits per second) */
#define RF_OOK_DEFAULT_BPS    1300UL

/** @brief Number of synchronization bits per frame */
#define SYNC_NB_BITS          16

/** @brief Synchronization pattern sent before each frame */
#define SYNC_BITS_VALUE       0xAAAA

/** @brief Number of bits used for node addressing */
#define ADDRESS_BITS          2

/** @brief Duration of a single bit in microseconds (based on bitrate) */
#define BIT_US                (1000000UL / RF_OOK_DEFAULT_BPS)

/** @brief Transmitter stabilization time before sending data (microseconds) */
#define TX_STABILIZATION_US   1000

/** @brief Delay after SYNC to allow Wake-up Receiver to wake the MCU (microseconds) */
#define RF_WAKEUP_DELAY_US    20000   /**< 20 ms */

/** @brief Number of bits for payload length field */
#define LENGTH_BITS           8

/** @brief Number of frames that can be buffered in RX FIFO */
#define BUFFER_SIZE           4

/** @brief Maximum allowed payload size in bytes */
#define MAX_PAYLOAD_SIZE      32

#define EDGE_QUEUE_SIZE 32

// Fréquence horloge
#define TIMER_FREQ_HZ           16000000UL      // 16 MHz

// Durée d'un bit en ticks TIM3
#define BIT_TICKS               (TIMER_FREQ_HZ / RF_OOK_DEFAULT_BPS)     // 12307 ticks

// Filtre anti-rebond : 1/4 de bit
#define EDGE_FILTER_TICKS       (BIT_TICKS / 4)

// Timeout FSM RX : 20 bits de silence
#define RX_TIMEOUT_TICKS        (BIT_TICKS * 20)

typedef struct {
    uint32_t delta_ticks;   // Durée depuis le front précédent
    uint8_t level;          // Niveau du plateau précédent (0 ou 1)
} edge_event_t;

/* -------------------------------------------------------------------------- */
/*                               Frame structures                             */
/* -------------------------------------------------------------------------- */

typedef struct __attribute__((packed)) {
    uint16_t co2;
    uint16_t temp;
    uint16_t hum;
    uint16_t lux;
    uint16_t o2;
    uint8_t  motion;
} sensor_payload_t;

/**
 * @brief Logical OOK frame structure.
 *
 * This structure represents a complete protocol frame after decoding
 * (or before encoding), excluding timing and physical layer information.
 */
typedef struct {
    uint16_t sync_bits;                      /**< Synchronization pattern */
    uint8_t  src_address;
    uint8_t dest_address;                        /**< Node address */
    uint8_t  seq_ttl;
    uint8_t payload_len;                    /**< Payload length in bytes */
    uint8_t payload[MAX_PAYLOAD_SIZE];      /**< Payload data */
    uint8_t  crc;
} rf_ook_frame_t;

/* -------------------------------------------------------------------------- */
/*                               State machines                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Transmission state machine states.
 *
 * Defines the different stages of frame transmission.
 */
typedef enum {
    TX_SYNC = 0,    /**< Transmitting SYNC bits */
//    TX_WAIT,        /**< Waiting for receiver wake-up delay */
    TX_LENGTH,     	/**< Payload Length */
	TX_SRC_ADDRESS,
    TX_DEST_ADDRESS,     /**< Transmitting address bits */
	TX_SEQ_TTL,
    TX_PAYLOAD,     /**< Transmitting payload bits */
	TX_CRC,
    TX_DONE         /**< Transmission completed */
} tx_state_t;

/**
 * @brief Reception state machine states.
 *
 * Defines the different stages of frame reception.
 */
typedef enum {
    RX_SYNC = 0,    /**< Waiting for address bits */
    RX_LENGTH,     	/**< Payload length */
	RX_SRC_ADDRESS,
    RX_DEST_ADDRESS,     /**< Receiving address bits */
	RX_SEQ_TTL,
    RX_PAYLOAD,      /**< Receiving payload bits */
	RX_CRC,
	RX_DONE			/**< Transmission completed */
} rx_state_t;

/* -------------------------------------------------------------------------- */
/*                           Transmission runtime context                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Transmission context structure.
 *
 * This structure holds the runtime state of an ongoing transmission
 * and is updated at each timer interrupt.
 */
typedef struct {
    rf_ook_frame_t frame;   /**< Frame currently being transmitted */
    uint8_t byte_idx;       /**< Current payload byte index */
    int8_t bit_idx;         /**< Current bit index within byte */
    tx_state_t state;       /**< Current TX state */
    volatile bool active;            /**< Indicates if a transmission is active */
    uint16_t wait_ticks;    /**< Counter used during TX_WAIT state */
} rf_ook_tx_frame_t;


static inline uint8_t rf_ook_compute_crc(rf_ook_frame_t *frame)
{
    uint8_t crc = 0;
    crc ^= frame->dest_address;
    crc ^= frame->src_address;
    crc ^= frame->seq_ttl;
    crc ^= frame->payload_len;
    for(uint8_t i = 0; i < frame->payload_len; i++) crc ^= frame->payload[i];
    return crc;
}

#endif /* RF_OOK_TYPES_H_ */
