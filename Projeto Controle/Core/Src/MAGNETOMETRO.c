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
 * Como atan2f retorna um ângulo em radianos, a multiplicação por 180/pi foi necessária
 * para converter o ângulo para graus.
 *
*/

float getAngulo(I2C_HandleTypeDef i2c, Coordenadas XY){
	uint8_t leitura[6];
	int16_t X = 0, Y = 0, Z = 0;
	float leituraBussola = 0.0, bussola = 0.0;

	// RECEIVE X_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, DATA_X_MSB_REGISTER, 1, leitura, 2, 100);
	X = (leitura[1]<<8) | leitura[0];
	// RECEIVE Y_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, DATA_Y_MSB_REGISTER, 1, leitura, 2, 100);
	Y = (leitura[3]<<8) | leitura[2];
	// RECEIVE Z_axis
	HAL_I2C_Mem_Read(&i2c, HMC5883l_ADDRESS, DATA_Z_MSB_REGISTER, 1, leitura, 2, 100);
	Z = (leitura[5]<<8) | leitura[4];

	// Armazena os valores na struct
	XY.x = (float)X;
	XY.y = (float)Y;

	bussola = atan2(Y,X);

	/* Para corrigir o ângulo de modo que o magnetômetro aponte para o
	 * norte geográfico é preciso encontrar qual a declinação.
	 * Utilizando os sites: http://www.magnetic-declination.com/
	 * e https://www.ngdc.noaa.gov/geomag/calculators/ para encontrar
	 * e o site https://planetcalc.com/71/ para realizar a conversão.
	 * Encontramos que a nossa é -23* 6' 10" W. Que convertidas em radianos
	 * equivale a 0.4032.
	 *
	 */
	float declinacaoAngle = 0.4032;
	bussola += declinacaoAngle;


	// Converter radianos para graus
	bussola = bussola * 180/3.14;


	if(bussola > 0){
		leituraBussola = bussola;
		return leituraBussola;
	}
	else{
		leituraBussola = 360 + bussola;
		return leituraBussola;
	}
}

/*
 * Configura os registradores de taxa de aquisição, ganho e modo de operação.
 *
*/
void configuraMagnetometro(I2C_HandleTypeDef i2c, uint8_t taxaAquisicao, uint8_t ganho, uint8_t modoOperacao)
{
	  // Configuração do ganho, taxa de aquisição e modo de operação do magnetômetro
	  HAL_I2C_Mem_Write(&i2c, HMC5883l_ADDRESS, CONFIG_A_REGISTER , 1, &taxaAquisicao , 1, 100);
	  HAL_I2C_Mem_Write(&i2c, HMC5883l_ADDRESS, CONFIG_B_REGISTER , 1, &ganho, 1, 100);
	  HAL_I2C_Mem_Write(&i2c, HMC5883l_ADDRESS, MODE_REGISTER, 1, &modoOperacao, 1, 100);
}
