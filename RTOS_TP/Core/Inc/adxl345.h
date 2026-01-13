/*
 * adxl345.h
 *
 *  Created on: Jan 6, 2026
 *      Author: eliof
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include <stdint.h>
#include "main.h"
#include "spi.h"

HAL_StatusTypeDef adxl345_read_reg(SPI_HandleTypeDef *hspi,
                                  GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                  uint8_t reg, uint8_t *val);

HAL_StatusTypeDef adxl345_read_multi(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                     uint8_t start_reg, uint8_t *buf, uint16_t len);

HAL_StatusTypeDef adxl345_write_reg(SPI_HandleTypeDef *hspi,
                                   GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                   uint8_t reg, uint8_t val);


#endif /* INC_ADXL345_H_ */
