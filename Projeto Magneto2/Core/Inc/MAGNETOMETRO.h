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

float getAngulo(I2C_HandleTypeDef i2c);


#endif /* INC_MAGNETOMETRO_H_ */
