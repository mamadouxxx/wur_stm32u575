/*
 * hal_adc.h
 *
 *  Created on: 17 janv. 2026
 *      Author: mamadou
 */

#ifndef HAL_ADC_H_
#define HAL_ADC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>


uint32_t hal_adc4_read(void);
uint32_t hal_adc1_read(void);
uint32_t hal_adc4_calibration();
uint32_t hal_adc1_calibration();
float lux_from_adc(uint32_t adc_val);


#ifdef __cplusplus
}
#endif

#endif /* HAL_ADC_H_ */
