/*
 * scd30_app.h
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

#ifndef SCD30_APP_H_
#define SCD30_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

#define NO_ERROR 0

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"
#include "stm32u5xx_nucleo.h"
#include <stdint.h>

void scd30_app_init(void);
void scd30_app_task(void);




#ifdef __cplusplus
}
#endif
#endif /* SCD30_APP_H_ */
