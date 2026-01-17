/*
 * o2_app.h
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

#ifndef O2_APP_H_
#define O2_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdint.h>

void o2_app_init(void);
void o2_app_task(void);



#ifdef __cplusplus
}
#endif

#endif /* O2_APP_H_ */
