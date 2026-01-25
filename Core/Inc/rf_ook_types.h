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

/** @brief Default bitrate for OOK communication (bits per second) */
#define RF_OOK_DEFAULT_BPS    1300

/** @brief Number of synchronization bits per frame */
#define SYNC_NB_BITS          6

/** @brief Synchronization pattern sent before each frame */
#define SYNC_BITS_VALUE       0b101100

/** @brief Number of bits used for node addressing */
#define ADDRESS_BITS          2

/** @brief Duration of a single bit in microseconds (based on bitrate) */
#define BIT_US                (1000000 / RF_OOK_DEFAULT_BPS)

/** @brief Transmitter stabilization time before sending data (microseconds) */
#define TX_STABILIZATION_US   1000

/** @brief Delay after SYNC to allow Wake-up Receiver to wake the MCU (microseconds) */
#define RF_WAKEUP_DELAY_US    20000   /**< 20 ms */

/** @brief Expected payload size in bytes for a complete frame */
#define PAYLOAD_BYTES         8

/** @brief Number of frames that can be buffered in RX FIFO */
#define BUFFER_SIZE           8

/** @brief Maximum allowed payload size in bytes */
#define MAX_PAYLOAD_SIZE      32

/* -------------------------------------------------------------------------- */
/*                               Frame structures                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Logical OOK frame structure.
 *
 * This structure represents a complete protocol frame after decoding
 * (or before encoding), excluding timing and physical layer information.
 */
typedef struct {
    uint8_t sync_bits;                      /**< Synchronization pattern */
    uint8_t address;                        /**< Node address */
    uint8_t payload[MAX_PAYLOAD_SIZE];      /**< Payload data */
    uint8_t payload_len;                    /**< Payload length in bytes */
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
    TX_WAIT,        /**< Waiting for receiver wake-up delay */
    TX_ADDRESS,     /**< Transmitting address bits */
    TX_PAYLOAD,     /**< Transmitting payload bits */
    TX_DONE         /**< Transmission completed */
} tx_state_t;

/**
 * @brief Reception state machine states.
 *
 * Defines the different stages of frame reception.
 */
typedef enum {
    RX_IDLE = 0,    /**< Waiting for address bits */
    RX_ADDRESS,     /**< Receiving address bits */
    RX_PAYLOAD      /**< Receiving payload bits */
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
    bool active;            /**< Indicates if a transmission is active */
    uint16_t wait_ticks;    /**< Counter used during TX_WAIT state */
} rf_ook_tx_frame_t;

#endif /* RF_OOK_TYPES_H_ */
