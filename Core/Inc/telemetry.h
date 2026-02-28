/**
 * @file    telemetry.h
 * @author  Mamadou
 * @date    16 jan 2026
 * @brief   Module de télémétrie via UART
 *
 * Fournit l'envoi périodique des données capteurs formatées sur UART1
 * à des fins de monitoring et de débogage.
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Envoie les données capteurs sur UART1
 *
 * Formate et transmet les dernières valeurs lues depuis sensor_data
 * (CO2, température, humidité, lux, O2, mouvement).
 */
void telemetry_send_uart(void);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H_ */
