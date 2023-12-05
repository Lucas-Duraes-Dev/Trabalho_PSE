/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "SERVOMOTOR.h"
#include "L293D.h"
#include "bluetooth.h"
#include "MAGNETOMETRO.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEMPLO_CICLO_CONTROLE 1000
#define NUMERO_DE_LEITURAS_MAGNETOMETRO 5

#define BIT_MAGNETOMETRO 0x01
#define BIT_BLUETOOTH 0x02
#define BIT_SERVO_MOTOR 0x04
#define BIT_MOTOR_DC 0x08


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart3;

/* Definitions for controlador */
osThreadId_t controladorHandle;
const osThreadAttr_t controlador_attributes = {
  .name = "controlador",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for magnetometro */
osThreadId_t magnetometroHandle;
const osThreadAttr_t magnetometro_attributes = {
  .name = "magnetometro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servoMotor */
osThreadId_t servoMotorHandle;
const osThreadAttr_t servoMotor_attributes = {
  .name = "servoMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bluetooth */
osThreadId_t bluetoothHandle;
const osThreadAttr_t bluetooth_attributes = {
  .name = "bluetooth",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorDC */
osThreadId_t motorDCHandle;
const osThreadAttr_t motorDC_attributes = {
  .name = "motorDC",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for queueMagnetometro */
osMessageQueueId_t queueMagnetometroHandle;
const osMessageQueueAttr_t queueMagnetometro_attributes = {
  .name = "queueMagnetometro"
};
/* Definitions for queueBluetooth */
osMessageQueueId_t queueBluetoothHandle;
const osMessageQueueAttr_t queueBluetooth_attributes = {
  .name = "queueBluetooth"
};
/* Definitions for queueMotorDC */
osMessageQueueId_t queueMotorDCHandle;
const osMessageQueueAttr_t queueMotorDC_attributes = {
  .name = "queueMotorDC"
};
/* Definitions for queueServoMotor */
osMessageQueueId_t queueServoMotorHandle;
const osMessageQueueAttr_t queueServoMotor_attributes = {
  .name = "queueServoMotor"
};
/* USER CODE BEGIN PV */

osEventFlagsId_t grupoEventosBarco;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART3_UART_Init(void);
void startControlador(void *argument);
void startMagnetometro(void *argument);
void startServoMotor(void *argument);
void startBluetooth(void *argument);
void startMotorDC(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queueMagnetometro */
  queueMagnetometroHandle = osMessageQueueNew (1, sizeof(float), &queueMagnetometro_attributes);

  /* creation of queueBluetooth */
  queueBluetoothHandle = osMessageQueueNew (16, sizeof(char), &queueBluetooth_attributes);

  /* creation of queueMotorDC */
  queueMotorDCHandle = osMessageQueueNew (1, sizeof(uint8_t), &queueMotorDC_attributes);

  /* creation of queueServoMotor */
  queueServoMotorHandle = osMessageQueueNew (1, sizeof(uint16_t), &queueServoMotor_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controlador */
  controladorHandle = osThreadNew(startControlador, NULL, &controlador_attributes);

  /* creation of magnetometro */
  magnetometroHandle = osThreadNew(startMagnetometro, NULL, &magnetometro_attributes);

  /* creation of servoMotor */
  servoMotorHandle = osThreadNew(startServoMotor, NULL, &servoMotor_attributes);

  /* creation of bluetooth */
  bluetoothHandle = osThreadNew(startBluetooth, NULL, &bluetooth_attributes);

  /* creation of motorDC */
  motorDCHandle = osThreadNew(startMotorDC, NULL, &motorDC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  // Declara grupo de eventos utilizado
  grupoEventosBarco = osEventFlagsNew(NULL);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 320-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1250 -1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 75;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L293D_LATCH_Pin|L293D_EN_Pin|L293D_SER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L293D_CLK_GPIO_Port, L293D_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin L293D_LATCH_Pin L293D_EN_Pin L293D_SER_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|L293D_LATCH_Pin|L293D_EN_Pin|L293D_SER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L293D_CLK_Pin */
  GPIO_InitStruct.Pin = L293D_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L293D_CLK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startControlador */
/**
  * @brief  Function implementing the controlador thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startControlador */
void startControlador(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint32_t eventosBarco;

  /* Infinite loop */
  for(;;)
  {
	// Delay entre os ciclos de controle
	osDelay(pdMS_TO_TICKS(TEMPLO_CICLO_CONTROLE));

	// Seta as flags do bluetooth e magnetômetro
	osEventFlagsSet(grupoEventosBarco, BIT_MAGNETOMETRO | BIT_BLUETOOTH);

	// Espera até que os bits do bluetooth e magnetômetro sejam iguais a 0
	eventosBarco = osEventFlagsGet(grupoEventosBarco);
	while( (eventosBarco & BIT_BLUETOOTH) | (eventosBarco & BIT_MAGNETOMETRO) )
	{
		eventosBarco = osEventFlagsGet(grupoEventosBarco);
		osDelay(pdMS_TO_TICKS(10));
	}

	// TODO: Pegar as informações das queues do magnetômetro e bluetooth


	// TODO: Realiza as equações da estratégia de controle


	// TODO: Adiciona a nova velocidade e ângulo para as filas correspondentes


	// Seta os bits de evento do motor dc e do servomotor após a adição dos elementos à fila
	osEventFlagsSet(grupoEventosBarco, BIT_SERVO_MOTOR | BIT_MOTOR_DC);

	// Espera até que os bits do servomotor e motor DC sejam iguais a 0
	eventosBarco = osEventFlagsGet(grupoEventosBarco);
	while( (eventosBarco & BIT_SERVO_MOTOR) | (eventosBarco & BIT_MOTOR_DC) )
	{
		eventosBarco = osEventFlagsGet(grupoEventosBarco);
		osDelay(pdMS_TO_TICKS(10));
	}

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startMagnetometro */
/**
* @brief Function implementing the magnetometro thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMagnetometro */
void startMagnetometro(void *argument)
{
  /* USER CODE BEGIN startMagnetometro */

  // Parâmetros magnetômetro
  uint8_t config[3] = {0x70, 0xA0, 0x00}; // Valores de exemplo para CONTROL REG A, CONTROL REG B e MODE REGISTER
  // config[0] = 0x70 = 01110000 -> Configuração de medição normal, output de dados de 15 Hz e média de 8 amostras por medição
  // config[1] = 0xA0 = 10100000 -> Ganho de 4.7 GA
  // config[2] = 0x00 = 00000000 -> Modo de leitura contínua
  configuraMagnetometro(hi2c1, config[0], config[1], config[2]);

  float anguloMagnetometro = 0;
  float leiturasMagnetometro[NUMERO_DE_LEITURAS_MAGNETOMETRO];


  /* Infinite loop */
  for(;;)
  {
	// Espera até que a task controlador solicite uma leitura
	osEventFlagsWait(grupoEventosBarco, BIT_MAGNETOMETRO, osFlagsNoClear, osWaitForever);

	// Realiza leituras
	for(int i = 0; i < NUMERO_DE_LEITURAS_MAGNETOMETRO; i ++)
	{
		leiturasMagnetometro[i] = getAngulo(hi2c1);
		anguloMagnetometro += leiturasMagnetometro[i];
		leiturasMagnetometro[i] = 0;
	}
	anguloMagnetometro = anguloMagnetometro / NUMERO_DE_LEITURAS_MAGNETOMETRO;

	// Envia informações para a fila
	xQueueSend(queueMagnetometroHandle, &anguloMagnetometro, 100);
	// Seta o valor do ângulo do magnetômetro para 0
	anguloMagnetometro = 0;

	// Realiza um clear no grupo de eventos após enviar suas informações
    osEventFlagsClear(grupoEventosBarco, BIT_MAGNETOMETRO);
  }
  /* USER CODE END startMagnetometro */
}

/* USER CODE BEGIN Header_startServoMotor */
/**
* @brief Function implementing the servoMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startServoMotor */
void startServoMotor(void *argument)
{
  /* USER CODE BEGIN startServoMotor */

  // Variáveis de controle
  UBaseType_t quantidadeElementosQueue = 0;
  uint16_t anguloServoMotor = 0; // Ângulo recebido pela queue

  uint16_t testAngle = 0;


  /* Infinite loop */
  for(;;)
  {
    // Espera até que a task controlador solicite uma mudança de velocidade
	osEventFlagsWait(grupoEventosBarco, BIT_SERVO_MOTOR, osFlagsNoClear, osWaitForever);

	quantidadeElementosQueue =  uxQueueMessagesWaiting(queueServoMotorHandle);

	if(quantidadeElementosQueue > 0)
	{
		xQueueReceive(queueServoMotorHandle, &anguloServoMotor, pdMS_TO_TICKS(100));

		// Altera o ângulo do servomotor
		setPWMAngulo(&htim4, TIM_CHANNEL_1, 1250 , anguloServoMotor);

	}
	testAngle += 20;
	if(testAngle >= 180)
	{
		testAngle = 0;
	}
	//xQueueReceive(queueServoMotorHandle, &anguloServoMotor, pdMS_TO_TICKS(100));

	// Altera o ângulo do servomotor
	setPWMAngulo(&htim4, TIM_CHANNEL_1, 1250 , testAngle);

	HAL_Delay(500);

	//setPWMAngulo(&htim4, TIM_CHANNEL_1, 1250 , 180);

	//setPWMAngulo(htim4, TIM_CHANNEL_1, 1250 , 135);

	//HAL_Delay(5000);


	// Limpa a flag após mudar o ângulo do barco
    osEventFlagsClear(grupoEventosBarco, BIT_SERVO_MOTOR);
  }
  /* USER CODE END startServoMotor */
}

/* USER CODE BEGIN Header_startBluetooth */
/**
* @brief Function implementing the bluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startBluetooth */
void startBluetooth(void *argument)
{
  /* USER CODE BEGIN startBluetooth */

	// String que armazena os resultados lidos pelo módulo bluetooth
	char respostaBluetooth[128] = {0};
	// Verifica se o módulo bluetooth está respondendo aos comandos
	//getResponse(huart3, respostaBluetooth);
	//start(huart3);
	//getResponse(huart3, respostaBluetooth);

	char envio[32] = {0};
	char resposta[32] = {0};
	sprintf(envio,"AT+BAUD\r\n");


  /* Infinite loop */
  for(;;)
  {
	// Espera até que a task controlador solicite uma leitura
	osEventFlagsWait(grupoEventosBarco, BIT_BLUETOOTH, osFlagsNoClear, osWaitForever);

	//HAL_UART_Transmit(&huart3, (uint8_t *) envio, strlen(envio), 100);
	//HAL_UART_Receive(&huart3, (uint8_t *)resposta, 8, 1000);
	// TODO: Realizar leituras filtradas
	//localizarBeacons(huart3, respostaBluetooth);

	// TODO: Enviar informações para a fila

	// Após enviar informações para a fila, dá um clear no bit do magnetômetro
	osEventFlagsClear(grupoEventosBarco, BIT_BLUETOOTH);
  }
  /* USER CODE END startBluetooth */
}

/* USER CODE BEGIN Header_startMotorDC */
/**
* @brief Function implementing the motorDC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorDC */
void startMotorDC(void *argument)
{
  /* USER CODE BEGIN startMotorDC */

  UBaseType_t quantidadeElementosQueue = 0;
  uint8_t velocidadeMotorDC = 0; // Velocidade recebida por queueMotorDC

  // Structs de configuração para o PWM e o HC595
  motor_dc motorTeste;
  HC595 hc595;

  // Configuração das structs de interação com o motor e com o HC595
  configHC595(&hc595, L293D_EN_GPIO_Port, L293D_EN_Pin, L293D_CLK_GPIO_Port, L293D_CLK_Pin, L293D_SER_GPIO_Port, L293D_SER_Pin);
  configMotor(&motorTeste, htim14, TIM_CHANNEL_1);

  // TODO: Adicionar mensagens de debug

  /* Infinite loop */
  for(;;)
  {
	// Espera até que a task controlador solicite uma mudança de velocidade
	osEventFlagsWait(grupoEventosBarco, BIT_MOTOR_DC, osFlagsNoClear, osWaitForever);

	// Verifica quantidade de elementos na queue
	quantidadeElementosQueue =  uxQueueMessagesWaiting(queueMotorDCHandle);
	if(quantidadeElementosQueue > 0)
	{
		xQueueReceive(queueMotorDCHandle, &velocidadeMotorDC, pdMS_TO_TICKS(100));

		// Altera a velocidade do motor
		changeMotorSpeed(&motorTeste, velocidadeMotorDC);

	}

    // Limpa a flag após mudar a velocidade do barco
    osEventFlagsClear(grupoEventosBarco, BIT_MOTOR_DC);
  }
  /* USER CODE END startMotorDC */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
