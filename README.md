# Système de réveil radiofréquence et de communication pour un objet connecté d'instrumentation à faible consommation

## Résumé

Ce projet consiste à mettre en place un objet connecté intelligent capable d'être en veille profonde pour économiser l'énergie, de se réveiller sur événement radiofréquence, d'identifier rapidement les messages qui le concernent, de traiter ces messages dans la fenêtre temporelle la plus courte possible, et de gérer la fréquence et la réactivité pour garantir la fiabilité du système.

Mots clés : IoT, Systèmes embarqués, STM32, OOK 433 MHz, Wake-up Radio, Basse consommation

---

## Contexte

Les catiches ou chambres et piliers sont des carrières souterraines pouvant atteindre 30 mètres de profondeur, connectées entre elles par des galeries. On recense environ 160 carrières sur le territoire lillois. Ces vides souterrains, soutenus par des voûtes en moellons de craie non cimentés, doivent être surveillés en permanence afin d'anticiper de possibles effondrements.

Dans ce contexte, un réseau de capteurs communicants souterrains a été conçu et déployé afin de permettre aux responsables du site et aux géologues d'accéder directement à des données physiques (température, humidité, concentration en gaz, signaux d'alerte, etc.) en différents points des galeries. Certains noeuds de ce réseau maillé peuvent également réagir à des commandes reçues et agir en conséquence.

---

## Problématique

L'objectif principal est de réduire la consommation énergétique d'un noeud instrumenté tout en garantissant un traitement rapide et sélectif des messages reçus. Il s'agit de développer un système embarqué intégrant la fonction de réveil radiofréquence de l'IEMN, capable d'activer un microcontrôleur STM32, d'interpréter le message reçu et de décider soit d'acquérir et transmettre les données des capteurs vers l'émetteur, soit, si le message ne lui est pas destiné, de le relayer vers le noeud suivant.

---

## Comportement attendu

```
[Veille profonde / Stop2]
          |
[Reception d'un signal RF OOK 433 MHz]
          |
[Reveil du MCU via WuR ou LPTIM1]
          |
[Comparaison de l'adresse destination du message]
          |
     _____|_____
    |           |
   Oui          Non
    |           |
[Acquisition   [Relais si TTL > 0
 des donnees]   ou ignorer]
    |
[Traitement + Transmission vers l'emetteur]
    |
[Retour en veille profonde]
```

---

## Materiel

| Composant | Role |
|---|---|
| STM32U575 (Nucleo) | MCU principal |
| Carte WuR (Wake-up Radio) IEMN | Demodulation OOK, reveil MCU |
| SCD30 | Capteur CO2, temperature, humidite (I2C) |
| Phototransistor | Capteur de luminosite (ADC4) |
| Capteur electrochimique | Capteur O2 (ADC1) |
| Module RF 433 MHz OOK | Emetteur radio |
| AD5668 | DAC 8 canaux SPI (controle tension WuR) |

---

## Architecture logicielle

```
Application      : app_sensors, scd30_app, lux_app, o2_app, telemetry
Protocole        : rf_ook_proto
Transport        : rf_ook_tx, rf_ook_rx
HAL personnalise : hal_adc, hal_uart
Donnees          : sensor_data
Pilotes          : AD5668_cna, scd30_i2c
```

---

## Structure des fichiers

```
Core/
├── Src/
│   ├── main.c              Point d'entree, boucle principale, gestion Stop2
│   ├── app_sensors.c       Orchestration des capteurs
│   ├── scd30_app.c         Lecture SCD30 (CO2, temperature, humidite)
│   ├── lux_app.c           Lecture capteur luminosite
│   ├── o2_app.c            Lecture capteur O2
│   ├── sensor_data.c       Stockage centralise des mesures
│   ├── telemetry.c         Envoi UART des donnees
│   ├── rf_ook_proto.c      Couche protocole OOK
│   ├── rf_ook_tx.c         Emetteur OOK (ISR TIM1)
│   ├── rf_ook_rx.c         Recepteur OOK (FSM + EXTI)
│   ├── AD5668_cna.c        Pilote DAC AD5668 SPI
│   ├── hal_adc.c           Abstraction ADC1/ADC4
│   ├── hal_uart.c          Abstraction UART1
│   └── error_handler.c     Gestion des erreurs firmware
└── Inc/
    ├── rf_ook_types.h      Types, constantes et structures communes
    └── ...
```

---

## Format de trame OOK

```
[ SYNC 16b ][ SRC 2b ][ DST 2b ][ SEQ_TTL 8b ][ LEN 8b ][ PAYLOAD Nb ][ CRC 8b ]
```

| Champ | Taille | Description |
|---|---|---|
| SYNC | 16 bits | Motif 0xAAAA de synchronisation |
| SRC | 2 bits | Adresse noeud source |
| DST | 2 bits | Adresse noeud destination |
| SEQ_TTL | 8 bits | Numero de sequence (4b) + TTL (4b) |
| LEN | 8 bits | Longueur payload en octets |
| PAYLOAD | N octets | Donnees capteurs |
| CRC | 8 bits | XOR de tous les champs |

### Payload capteurs

```c
typedef struct __attribute__((packed)) {
    uint16_t co2;     // CO2 en ppm
    uint16_t temp;    // Temperature
    uint16_t hum;     // Humidite relative
    uint16_t lux;     // Luminosite
    uint16_t o2;      // Oxygene
    uint8_t  motion;  // Mouvement
} sensor_payload_t;   // 11 octets
```

---

## Configuration des noeuds

Dans `rf_ook_types.h`, decommenter selon le role du MCU :

```c
#define NODE_IS_TRANSMITTER   // Decommenter pour le noeud emetteur
                              // Commenter pour le noeud recepteur
```

---

## Parametres protocole

| Parametre | Valeur |
|---|---|
| Debit | 1300 bps |
| Frequence | 433 MHz |
| Duree d'un bit | ~769 µs |
| Motif SYNC | 16 bits 0xAAAA |
| Adressage | 2 bits (4 noeuds max) |
| TTL max | 3 sauts |
| Buffer RX | 4 trames |
| Payload max | 32 octets |

---

## Timers utilises

| Timer | Role |
|---|---|
| TIM1 | ISR emission OOK (1 bit par interruption) |
| TIM3 | Horodatage des fronts RX (compteur libre 32 bits) |
| TIM17 | Timebase HAL (1 ms, HAL_IncTick) |
| LPTIM1 | Reveil periodique Stop2 toutes les 10 secondes |

---

## Gestion basse consommation

Le MCU entre en mode Stop2 entre chaque cycle de mesure et de transmission.

### Reveil TX — LPTIM1

LPTIM1 est cadence sur LSE (32768 Hz) avec un prescaler DIV128. La periode est de 10 secondes : (2559 + 1) / 256 Hz. Le callback `HAL_LPTIM_AutoReloadMatchCallback` leve le flag `tx_periodic_flag`.

### Reveil RX — EXTI

La pin RX_DATA est configuree en interruption RISING_FALLING. Le callback EXTI leve `rx_wakeup_flag` et appelle `rf_ook_rx_handle_edge()` pour enregistrer le front dans la file.

### Restauration apres Stop2

La fonction `system_clock_restore()` est appelee apres chaque reveil. Elle relance HSI16 comme SYSCLK, reinitialise I2C3, UART1, SPI1, ADC1 et ADC4, recalibre les deux ADC (calibration perdue en Stop2 sur STM32U5), et recale la reference temporelle RX avec `last_edge_time = TIM3->CNT`.

---

## Point bloquant — Stop2 en cours de debogage

L'entree en mode Stop2 ne fonctionne pas encore correctement. C'est le seul point bloquant fonctionnel — tout le reste du systeme est operationnel.

### Symptomes

Le code atteint bien `HAL_PWREx_EnterSTOP2Mode()`. Les registres `NVIC->ISPR` sont a zero avant l'appel, confirmant l'absence d'IRQ pending. Le MCU ne semble neanmoins pas entrer en sommeil.

### Causes investiguees

| Cause | Statut |
|---|---|
| IRQ NVIC pending avant WFI | Elimine — ISPR = 0 confirme |
| LPTIM1 sur mauvaise source clock | Elimine — LSE confirme dans MspInit |
| Fonction HAL_LPTIM_Counter_Start_IT incorrecte | Elimine — bit ARRMIE bien active |

---

## Prerequis

- STM32CubeIDE
- STM32CubeU5 HAL
- Pilote SCD30 Sensirion

---

## Auteur

Mamadou — Janvier / Fevrier 2026