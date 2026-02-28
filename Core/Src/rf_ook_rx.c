/**
 * @file    rf_ook_rx.c
 * @author  Mamadou
 * @date    21 jan 2026
 * @brief   Implémentation du récepteur OOK 433 MHz (RX)
 *
 * Fournit la réception et la mise en buffer de trames OOK via une FSM.
 * Les trames reçues sont stockées dans un buffer circulaire pour traitement ultérieur.
 */

#include "rf_ook_rx.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim3;  /**< Timer utilisé pour l'horodatage des fronts RX */
extern UART_HandleTypeDef huart1;

/* -------------------------------------------------------------------------- */
/*                            File de fronts EXTI                             */
/* -------------------------------------------------------------------------- */

volatile edge_event_t edge_queue[EDGE_QUEUE_SIZE]; /**< File circulaire des événements de front */
volatile uint8_t edge_head = 0;                    /**< Indice d'écriture dans la file */
volatile uint8_t edge_tail = 0;                    /**< Indice de lecture dans la file */
volatile bool first_edge_received = false;         /**< Indique si le premier front a été reçu */

volatile uint32_t last_edge_time = 0; /**< Timestamp du dernier front (ticks TIM3) */
volatile uint8_t  last_level = 0;     /**< Niveau logique du dernier plateau détecté */

static uint32_t rx_last_bit_time = 0; /**< Timestamp du dernier bit traité (pour timeout FSM) */

/* -------------------------------------------------------------------------- */
/*                              FSM RX interne                                */
/* -------------------------------------------------------------------------- */

static rx_state_t   rx_state      = RX_SYNC; /**< État courant de la FSM RX */
static uint8_t      rx_shift_reg  = 0;        /**< Registre à décalage pour l'accumulation des bits */
static uint16_t     rx_sync_reg   = 0;        /**< Registre 16 bits dédié à la détection du SYNC */
static uint8_t      rx_bit_count  = 0;        /**< Nombre de bits reçus pour le champ courant */
static uint8_t      rx_byte_count = 0;        /**< Nombre d'octets de payload reçus */
static rf_ook_frame_t rx_current;             /**< Trame en cours de réception */

/* -------------------------------------------------------------------------- */
/*                           Buffer circulaire RX                             */
/* -------------------------------------------------------------------------- */

static rf_ook_frame_t   buffer[BUFFER_SIZE];      /**< Buffer FIFO des trames reçues */
static volatile uint8_t rx_head          = 0;     /**< Indice d'écriture du FIFO */
static volatile uint8_t rx_tail          = 0;     /**< Indice de lecture du FIFO */
static volatile uint8_t rx_frame_count   = 0;     /**< Nombre de trames disponibles dans le buffer */
static volatile uint8_t rx_overflow_count = 0;    /**< Nombre de trames perdues par débordement */

/* -------------------------------------------------------------------------- */
/*                              API publique RX                               */
/* -------------------------------------------------------------------------- */

void rf_ook_rx_init(void)
{
    rx_state = RX_SYNC;
    rx_head = rx_tail = 0;
    first_edge_received = false;
    HAL_TIM_Base_Start(&htim3);
    last_edge_time = TIM3->CNT;
    last_level = HAL_GPIO_ReadPin(RX_DATA_GPIO_Port, RX_DATA_Pin);
}

void rf_ook_rx_reset(void)
{
    rx_state      = RX_SYNC;
    rx_shift_reg  = 0;
    rx_sync_reg   = 0;
    rx_bit_count  = 0;
    rx_byte_count = 0;
    rx_current.dest_address = 0;
    rx_current.payload_len  = 0;

    uint32_t now     = TIM3->CNT;
    rx_last_bit_time = now;
    last_edge_time   = now;
    first_edge_received = false;
}

void rf_ook_rx_handle_edge(uint8_t current_level)
{
    uint32_t now   = TIM3->CNT;
    uint32_t delta = now - last_edge_time;

    // Filtre anti-rebond : ignore les fronts trop rapprochés
    if (delta < EDGE_FILTER_TICKS) return;

    last_edge_time = now;

    if (!first_edge_received) {
        // Premier front : initialisation de la référence, pas encore de delta valide
        first_edge_received = true;
        last_level = current_level;
        return;
    }

    // Enfile l'événement si la file n'est pas pleine
    uint8_t next_head = (edge_head + 1) % EDGE_QUEUE_SIZE;
    if (next_head != edge_tail) {
        edge_queue[edge_head].delta_ticks = delta;
        edge_queue[edge_head].level       = last_level;
        edge_head = next_head;
    }

    last_level = current_level;
}

void rf_ook_rx_process_queue(void)
{
    // Timeout FSM : réinitialise si silence trop long hors état SYNC
    if (rx_state != RX_SYNC) {
        uint32_t now = TIM3->CNT;
        if ((now - rx_last_bit_time) > RX_TIMEOUT_TICKS) {
            rf_ook_rx_reset();
        }
    }

    // Traitement de tous les fronts en attente
    while (edge_tail != edge_head)
    {
        edge_event_t e = edge_queue[edge_tail];
        edge_tail = (edge_tail + 1) % EDGE_QUEUE_SIZE;

        // Conversion de la durée en nombre de bits (arrondi au plus proche)
        uint32_t nb_bits = (e.delta_ticks + BIT_TICKS / 2) / BIT_TICKS;

        for (uint32_t i = 0; i < nb_bits; i++) {
            rf_ook_rx_receive_bit(e.level);
        }
    }
}

void rf_ook_rx_receive_bit(uint8_t bit)
{
    rx_last_bit_time = TIM3->CNT;

    switch (rx_state)
    {
        case RX_SYNC:
            rx_sync_reg = (rx_sync_reg << 1) | (bit & 0x01);
            if (rx_sync_reg == SYNC_BITS_VALUE) {
                // Motif de synchronisation détecté
                rx_state     = RX_SRC_ADDRESS;
                rx_bit_count = 0;
                rx_sync_reg  = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_SRC_ADDRESS:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= ADDRESS_BITS) {
                rx_current.src_address = rx_shift_reg & ((1 << ADDRESS_BITS) - 1);
                rx_state     = RX_DEST_ADDRESS;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_DEST_ADDRESS:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= ADDRESS_BITS) {
                rx_current.dest_address = rx_shift_reg & ((1 << ADDRESS_BITS) - 1);
                rx_state     = RX_SEQ_TTL;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_SEQ_TTL:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8) {
                rx_current.seq_ttl = rx_shift_reg;
                rx_state     = RX_LENGTH;
                rx_bit_count = 0;
                rx_shift_reg = 0;
            }
            break;

        case RX_LENGTH:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= LENGTH_BITS) {
                rx_current.payload_len = rx_shift_reg;
                if (rx_current.payload_len > MAX_PAYLOAD_SIZE) {
                    // Longueur invalide : réinitialisation de la FSM
                    rf_ook_rx_reset();
                    break;
                }
                rx_state      = RX_PAYLOAD;
                rx_bit_count  = 0;
                rx_byte_count = 0;
                rx_shift_reg  = 0;
            }
            break;

        case RX_PAYLOAD:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8) {
                if (rx_byte_count < rx_current.payload_len &&
                    rx_byte_count < MAX_PAYLOAD_SIZE) {
                    rx_current.payload[rx_byte_count] = rx_shift_reg;
                    rx_byte_count++;
                }
                rx_bit_count = 0;
                rx_shift_reg = 0;

                if (rx_byte_count >= rx_current.payload_len) {
                    rx_state = RX_CRC;
                }
            }
            break;

        case RX_CRC:
            rx_shift_reg = (rx_shift_reg << 1) | (bit & 0x01);
            rx_bit_count++;
            if (rx_bit_count >= 8) {
                rx_current.crc = rx_shift_reg;

                // Stockage dans le buffer si de la place est disponible
                if (rx_frame_count < BUFFER_SIZE) {
                    buffer[rx_head] = rx_current;
                    rx_head = (rx_head + 1) % BUFFER_SIZE;
                    rx_frame_count++;
                } else {
                    rx_overflow_count++;
                }
                rx_state = RX_DONE;
            }
            break;

        case RX_DONE:
            rf_ook_rx_reset();
            break;
    }
}

uint8_t rf_ook_rx_get_frame(rf_ook_frame_t *frame)
{
    if (rx_frame_count == 0) return 0;

    __disable_irq();
    *frame   = buffer[rx_tail];
    rx_tail  = (rx_tail + 1) % BUFFER_SIZE;
    rx_frame_count--;
    __enable_irq();

    return 1;
}

bool rf_ook_rx_is_frame_ready(void)
{
    return (rx_frame_count > 0);
}
