/*
 * adxl345.c
 *
 *  Created on: Jan 6, 2026
 *      Author: eliof
 */

#include "adxl345.h"

// === Fonction qui ne fonctionnait pas, On obtenait un adresse de DEV_id = 0xCB ===
//HAL_StatusTypeDef adxl345_read_reg(SPI_HandleTypeDef *hspi,
//                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
//                                  uint8_t reg, uint8_t *val)
//{
//    uint8_t addr = reg | 0x80; // READ bit
//    HAL_StatusTypeDef st;
//
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
//
//    st = HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY);
//    if (st == HAL_OK)
//        st = HAL_SPI_Receive(hspi, val, 1, HAL_MAX_DELAY);
//
//    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
//    return st;
//}

HAL_StatusTypeDef adxl345_read_reg(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0, 0 };
    HAL_StatusTypeDef st;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    st = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    if (st == HAL_OK) {
        *val = rx[1];
    }
    return st;
}

HAL_StatusTypeDef adxl345_read_multi(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t addr = (uint8_t)(start_reg | 0x80 | 0x40); // READ + MULTI
    uint8_t dummy = 0x00;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);

    st = HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY);
    if (st == HAL_OK)
    {
        for (uint16_t i = 0; i < len; i++)
        {
            st = HAL_SPI_TransmitReceive(hspi, &dummy, &buf[i], 1, HAL_MAX_DELAY);
            if (st != HAL_OK) break;
        }
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return st;
}

HAL_StatusTypeDef adxl345_write_reg(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                   uint8_t reg, uint8_t val)
{
    // En écriture SPI ADXL345 : bit7=0 (WRITE), bit6=0 (single byte)
    uint8_t tx[2] = { reg & 0x3F, val };   // on force read=0 et multi=0 par sécurité
    HAL_StatusTypeDef st;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    st = HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return st;
}

