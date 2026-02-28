/**
 * @file    sensor_data.h
 * @author  Mamadou
 * @date    15 jan 2026
 * @brief   Module de stockage centralisé des données capteurs
 *
 * Fournit une interface unifiée pour mettre à jour et lire
 * les dernières valeurs mesurées par l'ensemble des capteurs.
 */

#ifndef SENSOR_DATA_H_
#define SENSOR_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Structure de stockage des données capteurs
 */
typedef struct {
    float    co2_ppm;       /**< Concentration CO2 en ppm */
    float    temperature_c; /**< Température en degrés Celsius */
    float    humidity_pct;  /**< Humidité relative en % */
    uint32_t lux_raw;       /**< Valeur brute ADC du capteur de luminosité */
    uint32_t o2_raw;        /**< Valeur brute ADC du capteur O2 */
    uint8_t  motion_raw;    /**< État du capteur de mouvement (0 = aucun) */
} sensor_data_t;

/**
 * @brief Met à jour les données du capteur SCD30
 *
 * @param co2   Concentration CO2 (ppm)
 * @param temp  Température (°C)
 * @param hum   Humidité relative (%)
 */
void sensor_data_update_scd30(float co2, float temp, float hum);

/**
 * @brief Met à jour la valeur brute du capteur O2
 *
 * @param o2  Valeur ADC brute
 */
void sensor_data_update_o2(uint32_t o2);

/**
 * @brief Met à jour la valeur brute du capteur de luminosité
 *
 * @param lux  Valeur ADC brute
 */
void sensor_data_update_lux(uint32_t lux);

/**
 * @brief Met à jour l'état du capteur de mouvement
 *
 * @param motion  État du capteur (0 = aucun mouvement)
 */
void sensor_data_update_motion(uint8_t motion);

/**
 * @brief Retourne la dernière concentration CO2 mesurée (ppm)
 */
float sensor_data_get_co2(void);

/**
 * @brief Retourne la dernière température mesurée (°C)
 */
float sensor_data_get_temperature(void);

/**
 * @brief Retourne la dernière humidité relative mesurée (%)
 */
float sensor_data_get_humidity(void);

/**
 * @brief Retourne la dernière valeur brute ADC du capteur O2
 */
uint32_t sensor_data_get_o2(void);

/**
 * @brief Retourne la dernière valeur brute ADC du capteur de luminosité
 */
uint32_t sensor_data_get_lux(void);

/**
 * @brief Retourne le dernier état du capteur de mouvement
 */
uint8_t sensor_data_get_motion(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_DATA_H_ */
