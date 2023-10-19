/*
 * MAGNETOMETRO.c
 *
 *  Created on: Oct 19, 2023
 *      Authors: Stephanie, Lucas e Jackson
 */

#include "MAGNETOMETRO.h"

/*
 * O código abaixo mostra como é feita a leitura de cada registrador
 * responsável por cada eixo. De acordo com o datasheet do HMC5883, os
 * registradores de leitura são:
 * Eixo X é o 00H Data Output X LSB Register   XOUT[7:0]
 * Eixo Y é o 02H Data Output X LSB Register   YOUT[7:0]
 * Eixo Z é o 04H Data Output X LSB Register   ZOUT[7:0]
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
	int16_t X, Y, Z;
	float leituraBussola = 0.0, bussola = 0.0;

	HAL_I2C_Mem_Read(&i2c, 0X1A, 0X06, 1, leitura, 1, 100);
	if((leitura[0]&0x01)==1){
		HAL_I2C_Mem_Read(&i2c, 0X1A, 0X00, 1, leitura, 6, 100);
		X = (leitura[1]<<8) | leitura[0];
		Y = (leitura[3]<<8) | leitura[2];
		Z = (leitura[5]<<8) | leitura[4];
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		bussola = atan2f(Y,X)*180/3.14;

		if(bussola > 0){
			leituraBussola = bussola;
			return leituraBussola;
		}
		else{
			leituraBussola = 360 + bussola;
			return leituraBussola;
		}

	}
	else{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
	return leituraBussola;
}
