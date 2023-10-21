/*
 * MAGNETOMETRO.h
 *
 *  Created on: Oct 19, 2023
 *      Authors: Stephanie, Lucas e Jackson
 */

#ifndef INC_MAGNETOMETRO_H_
#define INC_MAGNETOMETRO_H_


#include "main.h"
#include <math.h>
#include <stm32f4xx_hal_i2c.h>

// HMC5883l - ADDRESS
// 7-bit address (0x1E) plus 1 bit read/write identifier, i.e. 0x3D for read and 0x3C for write.
#define HMC5883l_ADDRESS (0x1E << 1)

float getAngulo(I2C_HandleTypeDef i2c);


#endif /* INC_MAGNETOMETRO_H_ */
