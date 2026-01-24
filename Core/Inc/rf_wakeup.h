/*
 * rf_wakeup.h
 *
 *  Created on: 23 janv. 2026
 *      Author: mamadou
 */

#ifndef RF_WAKEUP_H_
#define RF_WAKEUP_H_

#include <stdbool.h>

void rf_wakeup_init(void);
void rf_wakeup_irq_handler(void);
bool rf_wakeup_rx_active(void);

#endif
