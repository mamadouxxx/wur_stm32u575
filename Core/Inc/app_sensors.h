/**
 * @file app_sensors.h
 * @author Mamadou
 * @date 15 Jan 2026
 * @brief High-level sensor application module
 *
*  Application-level module for managing all sensors.
 *  Provides initialization and periodic update tasks for all sensors.
 *
 *  Sensors included:
 *    - SCD30 (CO2, temperature, humidity)
 *    - Lux sensor (via ADC4)
 *    - O2 sensor (via ADC1)
 *
 *  This module abstracts individual sensor drivers and offers a single interface
 *  to the application for initialization and periodic reading. */

#ifndef APP_SENSORS_H_
#define APP_SENSORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"

/**
 * @brief Initialize all sensors
 *
 * This function initializes each sensor module.
 * Call this once at system startup.
 */
void sensors_init(void);

/**
 * @brief Execute sensor tasks
 *
 * This function should be called periodically (e.g., in main loop or scheduler)
 * to read data from sensors and update internal state.
 */
void sensors_task(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_H_ */
