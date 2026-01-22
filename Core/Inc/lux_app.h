/*
 * lux_app.h
 *
 *  Created on: 15 Jan 2026
 *      Author: Mamadou
 *
 *  Application-level module for reading ambient light via ADC4.
 *  Provides initialization and periodic update of lux sensor data.
 */

#ifndef LUX_APP_H_
#define LUX_APP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
 * @brief Initialize the light sensor module.
 *        Performs ADC calibration and prepares the sensor for readings.
 */
void lux_app_init(void);

/**
 * @brief Periodic task to read light sensor values.
 *        Reads the ADC, converts if necessary, and updates sensor data storage.
 */
void lux_app_task(void);

#ifdef __cplusplus
}
#endif

#endif /* LUX_APP_H_ */
