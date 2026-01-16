/*
 * lux_app.h
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

#ifndef LUX_APP_H_
#define LUX_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdint.h>

void lux_app_init(void);
void lux_app_task(void);

#endif /* LUX_APP_H_ */
