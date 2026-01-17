/*
 * sensor_data.h
 *
 *  Created on: 15 janv. 2026
 *      Author: mamadou
 */

#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    float co2_ppm;
    float temperature_c;
    float humidity_pct;

    uint32_t lux_raw;
    uint32_t o2_raw;

    uint32_t timestamp_ms;
} sensor_data_t;

void sensor_data_update_scd30(float co2, float temp, float hum);
void sensor_data_update_o2(uint32_t o2);
void sensor_data_update_lux(uint32_t lux);

float    sensor_data_get_co2(void);
float    sensor_data_get_temperature(void);
float    sensor_data_get_humidity(void);
uint32_t sensor_data_get_o2(void);
uint32_t sensor_data_get_lux(void);


#ifdef __cplusplus
}
#endif

#endif /* INC_SENSOR_DATA_H_ */
