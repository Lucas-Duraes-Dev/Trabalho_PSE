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

// HMC5883l - Endereço I2C de 7 bits
#define HMC5883l_ADDRESS (0x1E << 1)

// Endereços de registradores para o HMC5883l
#define CONFIG_A_REGISTER 0x00
#define CONFIG_B_REGISTER 0x01
#define MODE_REGISTER 0x02
#define DATA_X_MSB_REGISTER 0x03
#define DATA_Z_MSB_REGISTER 0x05
#define DATA_Y_MSB_REGISTER 0x07

typedef struct{
	float x;
	float y;
} Coordenadas;

float getAngulo(I2C_HandleTypeDef i2c, Coordenadas XY);
void configuraMagnetometro(I2C_HandleTypeDef i2c, uint8_t taxaAquisicao, uint8_t ganho, uint8_t modoOperacao);

Coordenadas XY;

#endif /* INC_MAGNETOMETRO_H_ */
