/*
 * bluetooth.c
 *
 *  Created on: Nov 4, 2023
 *      Author: jac-l
 */

#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"

void reset(UART_HandleTypeDef huart){
	char envio[12] = {0};
	sprintf(envio,"AT+RESET\r\n");
	HAL_UART_Transmit(&huart, (uint8_t *) envio, strlen(envio), 100);
}

void start(UART_HandleTypeDef huart){
	char envio[12] = {0};
	sprintf(envio,"AT+NAME\r\n");
	HAL_UART_Transmit(&huart, (uint8_t *) envio, strlen(envio), 100);
}

void getResponse(UART_HandleTypeDef huart, char * resposta){
	char envio[32] = {0};
	//char resposta[32] = {0};
	sprintf(envio,"AT+BAUD\r\n");

	HAL_UART_Transmit(&huart, (uint8_t *)envio, strlen(envio), 100);
	HAL_UART_Receive(&huart, (uint8_t *)resposta, 8, 1000);
	HAL_Delay(1000);

	//return resposta;
}

void programarBluetooth(UART_HandleTypeDef huart){
	char envio[20] = {0};
	char envio1[20] = {0};
	char envio2[20] = {0};
	char envio3[20] = {0};
	sprintf(envio, "AT+BAUD9\r\n");
	sprintf(envio1, "AT+PARITY0\r\n");
	sprintf(envio2, "AT+USTP0\r\n");
	sprintf(envio2, "AT+ROLE1\r\n");

	HAL_UART_Transmit(&huart, (uint8_t *) envio, strlen(envio), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart, (uint8_t *) envio1, strlen(envio1), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart, (uint8_t *) envio2, strlen(envio2), 100);
	HAL_Delay(1000);
	HAL_UART_Transmit(&huart, (uint8_t *) envio3, strlen(envio3), 100);
	HAL_Delay(1000);
}

void localizarBeacons(UART_HandleTypeDef huart, char * resposta){
		char envio[32] = {0};
		//char resposta[32] = {0};
		sprintf(envio,"AT+INQ\r\n");

		HAL_UART_Transmit(&huart, (uint8_t *)envio, strlen(envio), 100);
		HAL_UART_Receive(&huart, (uint8_t *)resposta, 128, 1000);

		HAL_Delay(1000);

}
