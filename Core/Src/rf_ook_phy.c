/**
 * @file rf_ook_phy.c
 * @author Mamadou
 * @date 20 Jan 2026
 * @brief Implementation of the OOK physical layer (TX/RX control and precise timing)
 */

#include "rf_ook_phy.h"
#include "main.h"

extern TIM_HandleTypeDef htim1; /**< Timer used for microsecond delays */

/**
 * @brief Delay execution for a precise number of microseconds
 *
 * Uses TIM1 to generate a blocking delay.
 *
 * @param us Delay in microseconds
 */
void delay_us_timer(uint32_t us)
{
    while(us > 0)
    {
        uint32_t step = (us > 60000) ? 60000 : us; // limiter à 60 ms pour sécurité
        __HAL_TIM_SET_COUNTER(&htim1, 0);
        HAL_TIM_Base_Start(&htim1);
        while(__HAL_TIM_GET_COUNTER(&htim1) < step);
        HAL_TIM_Base_Stop(&htim1);
        us -= step;
    }
}
