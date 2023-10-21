/*
 * MAGNETOMETRO.c
 *
 *  Created on: Oct 19, 2023
 *      Authors: Stephanie, Lucas e Jackson
 */

#include "MAGNETOMETRO.h"

/*
 * O código abaixo mostra como é feita a leitura de cada registrador
 * responsável por cada eixo. De acordo com o datasheet do HMC5883L, os
 * registradores de leitura são:
 * 04 Data Output X LSB Register Read
 * 06 Data Output Z LSB Register Read
 * 08 Data Output Y LSB Register Read
 *
 * O endereço do registrador do magnetômetro do modelo HMC5883L é o 0x1E.
 * De acordo com o Datasheet,  as localizações de ponteiro são enviadas do mestre
 * para este dispositivo escravo e sucede o endereço de 7 bits (0x1E) mais o
 * identificador de leitura/gravação de 1 bit, ou seja, 0x3D para leitura e 0x3C para gravação.
 *
 * Usando a função atan2f calcula-se o arco tangente entre os pontos X e Y
 * obtendo assim o ângulo entre eles e que será repassado para o Servo Motor
 * fazer o controle do leme do barco.
 *
 * Como atan2f retorna um ângulo em radianos, a operação 180/pi foi necessária
 * para converter o ângulo para graus.
 *
*/

float getAngulo(I2C_HandleTypeDef i2c){
	uint8_t leitura[6];
	int16_t X = 0, Y = 0, Z = 0;
	float leituraBussola = 0.0, bussola = 0.0;

	// RECEIVE X_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, 0x04, 1, leitura, 2, 100);
	X = (leitura[1]<<8) | leitura[0];
	// RECEIVE Y_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, 0x06, 1, leitura, 2, 100);
	Y = (leitura[3]<<8) | leitura[2];
	// RECEIVE Z_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, 0x08, 1, leitura, 2, 100);
	Z = (leitura[5]<<8) | leitura[4];

	bussola = atan2f(Y,X)*180/3.14;

	if(bussola > 0){
		leituraBussola = bussola;
	}
	else{
		leituraBussola = 360 + bussola;
	}

	return leituraBussola;
}
