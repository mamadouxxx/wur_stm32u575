/*
 * o2_app.h
 *
 *  Created on: 15 Jan 2026
 *      Author: Mamadou
 *
 *  Application-level module for reading oxygen sensor via ADC1.
 *  Provides initialization and periodic update of O2 sensor data.
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

/**
 * @brief Initialize the O2 sensor module.
 *        Calibrates ADC1 and prepares the sensor for readings.
 */
void o2_app_init(void);

/**
 * @brief Periodic task to read O2 sensor values.
 *        Reads the ADC1 value and updates the O2 measurement in sensor data.
 */
void o2_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* O2_APP_H_ */
