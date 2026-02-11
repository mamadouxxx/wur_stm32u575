/**
 * @file    ad5668_cna.c
 * @brief   AD5668 DAC driver implementation
 * @author  Mamadou
 * @date    January 25, 2026
 */

#include "AD5668_cna.h"
#include <math.h>

/** Maximum 16-bit DAC code */
#define CNA_DAC_MAX_CODE 65535

/** DAC reference voltage in millivolts */
#define CNA_VREF_mV 3300.0f

/**
 * @brief Default output voltages for all DAC channels (mV)
 */
static const float cna_init_voltages[8] = {500, 500, 1200, 250, 250, 800, 500, 400};

/**
 * @brief Convert a voltage value (mV) to a 16-bit DAC code
 *
 * @param voltage_mV Input voltage in millivolts
 * @return Corresponding DAC code
 */
static uint16_t voltage_to_code(float voltage_mV)
{
    if (voltage_mV < 0.0f) {
        voltage_mV = 0.0f;
    }
    if (voltage_mV > CNA_VREF_mV) {
        voltage_mV = CNA_VREF_mV;
    }

    return (uint16_t)roundf((voltage_mV / CNA_VREF_mV) * CNA_DAC_MAX_CODE);
}

/**
 * @brief Send a 32-bit SPI command frame to the DAC
 *
 * @param command AD5668 command code
 * @param channel DAC channel
 * @param data 16-bit data payload
 * @param function Lower 4-bit function field
 */
static void cna_write_dac(uint8_t command, uint8_t channel, uint16_t data, uint8_t function)
{
    uint8_t tx[4];

    tx[0] = command;
    tx[1] = (channel << 4) | (data >> 12);
    tx[2] = (data >> 4) & 0xFF;
    tx[3] = ((data & 0x0F) << 4) | (function & 0x0F);

    HAL_GPIO_WritePin(AD5668_SYNC_GPIO_Port, AD5668_SYNC_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, tx, 4, 10);
    HAL_GPIO_WritePin(AD5668_SYNC_GPIO_Port, AD5668_SYNC_Pin, GPIO_PIN_SET);
}

void cna_init(void)
{
    // Reset DAC via CLR
    HAL_GPIO_WritePin(AD5668_CLR_GPIO_Port, AD5668_CLR_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD5668_CLR_GPIO_Port, AD5668_CLR_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    // Init LDAC high
    HAL_GPIO_WritePin(AD5668_LDAC_GPIO_Port, AD5668_LDAC_Pin, GPIO_PIN_SET);

    // Load the initial voltages on all channels
    for(uint8_t ch=0; ch<8; ch++)
    {
        cna_set_voltage(ch, cna_init_voltages[ch]);
    }
}

void cna_clear(void)
{
    HAL_GPIO_WritePin(AD5668_CLR_GPIO_Port, AD5668_CLR_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD5668_CLR_GPIO_Port, AD5668_CLR_Pin, GPIO_PIN_SET);
}

void cna_set_voltage(uint8_t channel, float voltage_mV)
{
    uint16_t code = voltage_to_code(voltage_mV);
    cna_write_dac(CNA_CMD_WRITE_INPUT_UPDATE_N, channel, code, 15);
    cna_toggle_ldac(); // ensures that voltage is applied
}

void cna_write_channel(uint8_t channel, uint16_t code)
{
    cna_write_dac(CNA_CMD_WRITE_INPUT_REG, channel, code, 15);
}

void cna_update_channel(uint8_t channel)
{
    cna_write_dac(CNA_CMD_UPDATE_OUTPUT_REG, channel, 0, 15);
}

void cna_write_update_channel(uint8_t channel, uint16_t code)
{
    cna_write_dac(CNA_CMD_WRITE_INPUT_UPDATE_N, channel, code, 15);
}

void cna_write_update_all(uint8_t channel, uint16_t code)
{
    cna_write_dac(CNA_CMD_WRITE_INPUT_UPDATE_ALL, channel, code, 15);
}

void cna_power_normal(uint8_t channels)
{
    uint16_t modeChE_H = channels >> 4;
    uint8_t chA_D = channels & 0x0F;
    cna_write_dac(CNA_CMD_POWER_DOWN_UP, 0, modeChE_H, chA_D);
}

void cna_power_down(uint8_t channels, uint8_t mode)
{
    uint16_t modeChE_H;
    switch(mode)
    {
        case 0: modeChE_H = 0x10 | (channels >> 4); break; // 1kΩ
        case 1: modeChE_H = 0x20 | (channels >> 4); break; // 100kΩ
        case 2: modeChE_H = 0x30 | (channels >> 4); break; // tri-state
        default: modeChE_H = 0; break;
    }
    uint8_t chA_D = channels & 0x0F;
    cna_write_dac(CNA_CMD_POWER_DOWN_UP, 0, modeChE_H, chA_D);
}

void cna_enable_internal_ref(void)
{
    cna_write_dac(CNA_CMD_SETUP_INTERNAL_REF, 0, 0, 1);
}

void cna_disable_internal_ref(void)
{
    cna_write_dac(CNA_CMD_SETUP_INTERNAL_REF, 0, 0, 0);
}

void cna_reset(void)
{
    cna_write_dac(CNA_CMD_RESET_POWER_ON, 0, 0, 0);
}

void cna_toggle_ldac(void)
{
    HAL_GPIO_WritePin(AD5668_LDAC_GPIO_Port, AD5668_LDAC_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(AD5668_LDAC_GPIO_Port, AD5668_LDAC_Pin, GPIO_PIN_SET);
}
