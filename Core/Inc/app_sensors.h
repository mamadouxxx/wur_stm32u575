/*
 * app_sensors.h
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

#ifndef APP_SENSORS_H_
#define APP_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32u5xx_hal.h"

void sensors_init(void);
void sensors_task(void);


#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_H_ */
