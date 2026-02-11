/**
 * @file app_sensors.c
 * @author Mamadou
 * @date 15 Jan 2026
 * @brief Implementation of high-level sensor application module
 *
 * Orchestrates the initialization and periodic task execution
 * for all actual sensors: SCD30, light sensor, and O2 sensor.
 */

#include "app_sensors.h"
#include "scd30_app.h"
#include "lux_app.h"
#include "o2_app.h"

/**
 * @brief Initialize all sensors
 *
 * Calls the initialization routine of each sensor module.
 */
void sensors_init(void)
{
    scd30_app_init();
    // lux_app_init();  // Uncomment for calibrate light sensor is available
//     o2_app_init();   // Uncomment to calibrate O2 sensor is available
}

/**
 * @brief Execute periodic sensor tasks
 *
 * Calls the task function of each sensor module to read data
 * and perform internal updates.
 */
void sensors_task(void)
{
    scd30_app_task();
    lux_app_task();
    o2_app_task();
}
