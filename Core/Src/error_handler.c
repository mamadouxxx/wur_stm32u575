/**
 * @file    error_handler.c
 * @author  Mamadou
 * @date    21 jan 2026
 * @brief   Implémentation de la gestion des erreurs firmware
 *
 * Fournit les fonctions de signalement d'erreurs critiques et d'avertissements
 * via UART et LED. Les erreurs critiques désactivent les interruptions et
 * notifient le système (gestionnaire de démarrage, watchdog, etc.).
 */

#include "error_handler.h"
#include "hal_uart.h"
#include "main.h"
#include <stdio.h>

/* Hook faible pour la gestion des erreurs fatales au niveau système (boot manager) */
__attribute__((weak))
void system_fatal_error_notify(ErrorCode_t code)
{
    (void)code;
}

/**
 * @brief Fait clignoter la LED d'erreur un nombre de fois donné
 *
 * @param times     Nombre de clignotements
 * @param delay_ms  Délai entre chaque basculement en millisecondes
 */
static void blink_led(uint8_t times, uint16_t delay_ms)
{
    for (uint8_t i = 0; i < times; i++) {
        HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
        HAL_Delay(delay_ms);
        HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
        HAL_Delay(delay_ms);
    }
}

void FW_Error_Handler(ErrorCode_t code, const char* msg)
{
    __disable_irq();

    /* Journalisation de l'erreur critique */
    if (msg != NULL) {
        uint8_t buf[128];
        int len = snprintf((char*)buf, sizeof(buf),
                           "ERREUR CRITIQUE [%u]: %s\r\n",
                           (unsigned)code, msg);
        if (len > 0) {
            hal_uart1_write(buf, (uint16_t)len);
        }
    }

    /* Indication visuelle : clignotement rapide */
    blink_led(3, 100);

    /* Notification système (boot manager, watchdog, etc.) */
    system_fatal_error_notify(code);

    __enable_irq();
}

void Error_Warning(ErrorCode_t code, const char* msg)
{
    /* Journalisation de l'avertissement */
    if (msg != NULL) {
        uint8_t buf[128];
        int len = snprintf((char*)buf, sizeof(buf),
                           "AVERTISSEMENT [%u]: %s\r\n",
                           (unsigned)code, msg);
        if (len > 0) {
            hal_uart1_write(buf, (uint16_t)len);
        }
    }

    /* Indication visuelle : clignotement unique */
    blink_led(1, 150);
}
