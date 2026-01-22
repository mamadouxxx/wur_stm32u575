/**
 * @file rf_ook_phy.h
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Physical layer for OOK 433 MHz (TX/RX control and precise timing)
 *
 * Provides low-level functions to control the RF transmitter
 * and precise microsecond delays using a hardware timer.
 */

#ifndef RF_OOK_PHY_H_
#define RF_OOK_PHY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Default bitrate for OOK communication in bits per second */
#define RF_OOK_DEFAULT_BPS	1300

/** @brief Number of preamble bits per frame */
#define PREAMBLE_BITS		8

/** @brief Number of sync bits per frame */
#define SYNC_NB_BITS		6

/** @brief SYNC BITS */
#define SYNC_BITS_VALUE			0b101100

/** @brief Number of address bits per frame */
#define ADDRESS_BITS         2

/** @brief Duration of a single bit in microseconds (at default bitrate) */
#define BIT_US  (1000000 / RF_OOK_DEFAULT_BPS)

/** @brief Time to wait for TX stabilization before sending bits (in microseconds) */
#define TX_STABILIZATION_US 1000

/**
 * @brief Delay execution for a precise number of microseconds
 *
 * Uses a hardware timer (TIM1) to implement an accurate microsecond delay.
 *
 * @param us Delay in microseconds
 */
void delay_us_timer(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* RF_OOK_PHY_H_ */
