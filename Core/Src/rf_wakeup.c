/*
 * rf_wakeup.c
 *
 *  Created on: 23 janv. 2026
 *      Author: mamadou
 */


#include "rf_wakeup.h"
#include "rf_ook_phy.h"
#include "rf_ook_rx.h"

static volatile bool rx_active = false;

void rf_wakeup_init(void)
{
    rx_active = false;
}

void rf_wakeup_irq_handler(void)
{
    /* Appelé depuis l'IRQ WuR */
    rx_active = false;

    /* Laisser le MCU se stabiliser */
    delay_us_timer(RF_WAKEUP_DELAY_US);

    /* Reset RX FSM */
    rf_ook_rx_reset();

    /* Activer la réception */
    rx_active = true;
}

bool rf_wakeup_rx_active(void)
{
    return rx_active;
}
