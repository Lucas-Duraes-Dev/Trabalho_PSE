/*
 * SERVOMOTOR.h
 *
 *  Created on: Oct 6, 2023
 *      Authors: Stephanie, Lucas e Jackson
 */

#ifndef SRC_SERVOMOTOR_H_
#define SRC_SERVOMOTOR_H_

#include "main.h"
#include "stm32f4xx_hal_conf.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

void setPWMAngulo(TIM_HandleTypeDef* timer, uint32_t channel, uint16_t period, uint16_t angulo);


#endif /* SRC_SERVOMOTOR_H_ */
