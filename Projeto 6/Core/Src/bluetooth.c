/*
 * bluetooth.c
 *
 *  Created on: Nov 4, 2023
 *      Author: jac-l
 */

#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"

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

	HAL_UART_Transmit(&huart, (uint8_t *) envio, strlen(envio), 100);
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


void processarInformacoesBluetooth(const char* informacoes, char* numerosEncontrados) {
//void processarInformacoesBluetooth(char* numerosEncontrados) {
    // Verificar se as referências são válidas
    if (informacoes == NULL) {
        printf("Parâmetros inválidos.\n");
        return;
    }

    // Inicializar a string de números encontrados
    numerosEncontrados[0] = '\0';

    // Iterar sobre a string de informações
    char* informacoesCopia = strdup(informacoes); // Faz uma cópia da string para evitar modificar a original
    //char* informacoesCopia;
    //informacoesCopia = "+OK\r\n+DEV:1=EFC0F0845DBA,-60,Baseus E9\r\n+DEV:2=C0238D1B26A9,-82,[TV] Samsung AU7700 65 TV\r\n\r\n+DEV:3=ABC123456789,-75,Example Device\r\n+DEV:4=XZY987654321,-68,Sample Gadget\r\n";
    char* token = strtok(informacoesCopia, "\r\n");

    while (token != NULL) {
    	printf( " %s\n", token );
         //Verificar se a substring contém nomes específicos
        if (strstr(token, "PSE2022_B1") || strstr(token, "PSE2022_B2") || strstr(token, "PSE2022_B3")) {
    	//if (strstr(token, "-")) {
            // Encontrar o número entre '-' e ','
            char* inicioNumero = strchr(token, ',') + 2;
            char* fimNumero = strchr(token, '-') + 3;

            if (inicioNumero != NULL && fimNumero != NULL && inicioNumero < fimNumero) {
                // Copiar o número para a string de números encontrados
                strncat(numerosEncontrados, inicioNumero, fimNumero - inicioNumero);
                strncat(numerosEncontrados, "\r\n", 2);
            }

        }

        token = strtok(NULL, "\r\n");
    }

    free(informacoesCopia); // Liberar a memória alocada
}
