/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	ESPERA,
	MANUAL,
	HORARIO,
	AUTOMATICO
} State_Type;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

volatile int minutos = 0;
volatile int horas = 0;

uint8_t buffer[64]; //Para recibir lo que el terminal envia al micro

char* str_CAMBIAR_A_MODO = "CAMBIAR A MODO";
char* str_ESPERA = "ESPERA";
char* str_MANUAL = "MANUAL";
char* str_HORARIO = "HORARIO";
char* str_AUTOMATICO = "AUTOMATICO";

char* str_ENCENDER_POR = "ENCENDER POR";
char* str_APAGAR = "APAGAR";

volatile int last_interrupt_time = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void state_machine_init(void);
void ESPERA_function(void);
void MANUAL_function(void);
void HORARIO_function(void);
void AUTOMATICO_function(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/************************************* TIEMPO ********************************************/

int tiempoTranscurridoMinutos(int tiempo){
	int retorno = 0;
	HAL_TIM_Base_Start_IT(&htim3);
	if (minutos < tiempo){
		retorno = 0;
	}
	else{
		retorno = 1;
		HAL_TIM_Base_Stop_IT(&htim3);
		minutos = 0;
	}
	return retorno;
}

int tiempoTranscurridoHoras(int tiempo){
	int retorno = 0;
	HAL_TIM_Base_Start_IT(&htim2);
	if (horas < tiempo){
		retorno = 0;
	}
	else{
		retorno = 1;
		HAL_TIM_Base_Stop_IT(&htim3);
		horas = 0;
	}
	return retorno;
}

/************************************* FSM ********************************************/

static void (*state_table []) (void) ={ESPERA_function,MANUAL_function,HORARIO_function,AUTOMATICO_function};
static State_Type Current_State;
volatile int btn_pressed;

/*********************************** GENERAL ********************************************/

void displayModo(int k){
	int i;
	char msg[19];
	switch(k){
	case 0:
		strcpy(msg,"MODO: ESPERA    \r\n");
		break;
	case 1:
		strcpy(msg,"MODO: MANUAL    \r\n");
		break;
	case 2:
		strcpy(msg,"MODO: HORARIO   \r\n");
		break;
	case 3:
		strcpy(msg,"MODO: AUTOMATICO\r\n");
		break;
	}
	uint8_t data[sizeof(msg)];
	for(i = 0;i<=sizeof(msg);i++){
	  data[i] = msg[i];
	};
	CDC_Transmit_FS(data, sizeof(data));
}



void Delay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim5,0);
	while ((__HAL_TIM_GET_COUNTER(&htim5))<time);
}

void Display_Temp(float Temp)
{
//	char str[20] = {0};
//	lcd_put_cur(0, 0);
//
//	sprintf (str, "TEMP:- %.2f ", Temp);
//	lcd_send_string(str);
//	lcd_send_data('C');
}

void Display_Rh (float Rh)
{
//	char str[20] = {0};
//	lcd_put_cur(1, 0);
//
//	sprintf (str, "RH:- %.2f ", Rh);
//	lcd_send_string(str);
//	lcd_send_data('%');
}

uint8_t Rh_byte1, Rh_byte2, Temp_byte1, Temp_byte2;
uint16_t SUM, RH, TEMP;

float Temperature = 0;
float Humidity = 0;
uint8_t Presence = 0;

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*********************************** DHT11 FUNCTIONS ********************************************/

#define DHT11_PORT GPIOD
#define DHT11_PIN  GPIO_PIN_13

void DHT11_Start (void)
{
	Set_Pin_Output (DHT11_PORT, DHT11_PIN);  // set the pin as output
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);   // pull the pin low
	Delay(18000);   // wait for 18ms
    HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);   // pull the pin high
	Delay (20);   // wait for 20us
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);    // set as input
}

uint8_t DHT11_Check_Response (void)
{
	uint8_t Response = 0;
	Delay(40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		Delay(80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1; // 255
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go low

	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));   // wait for the pin to go high
		Delay(40);   // wait for 40 us
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))   // if the pin is low
		{
			i&= ~(1<<(7-j));   // write 0
		}
		else i|= (1<<(7-j));  // if the pin is high, write 1
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));  // wait for the pin to go low
	}
	return i;
}

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  state_machine_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  state_table[Current_State]();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3599999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 48-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xffff-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

void state_machine_init(void){
	displayModo(ESPERA);
	Current_State = ESPERA;
	btn_pressed = 0;
}
void ESPERA_function(void){
	char *s  = strstr((char*)buffer,str_CAMBIAR_A_MODO);
	char *sH = strstr((char*)buffer,str_HORARIO);
	char *sM = strstr((char*)buffer,str_MANUAL);
	char *sA = strstr((char*)buffer,str_AUTOMATICO);
	//Funcionamiento del modo ESPERA
	//
	if(btn_pressed == 1){
		btn_pressed = 0;
		displayModo(MANUAL);
		buffer[0] = '\0';
		Current_State = MANUAL;
	}
	else if(s != NULL){
		if(sH != NULL){
			displayModo(HORARIO);
			buffer[0] = '\0';
			Current_State = HORARIO;
		}
		else if(sM != NULL){
			displayModo(MANUAL);
			buffer[0] = '\0';
			Current_State = MANUAL;
		}
		else if(sA != NULL){
			displayModo(AUTOMATICO);
			buffer[0] = '\0';
			Current_State = AUTOMATICO;
		}
	}
}
void MANUAL_function(void){
	int apagar = 0;
	char *s     = strstr((char*)buffer,str_CAMBIAR_A_MODO);
	char *sE    = strstr((char*)buffer,str_ESPERA);
	char *sH    = strstr((char*)buffer,str_HORARIO);
	char *sA    = strstr((char*)buffer,str_AUTOMATICO);
	char *s_ON  = strstr((char*)buffer,str_ENCENDER_POR);
	char *s_OFF = strstr((char*)buffer,str_APAGAR);
	//Funcionamiento del modo MANUAL
	if(s_ON != NULL){
		char arr_tiempo[2] = {buffer[13],buffer[14]};
		buffer[0] = '\0';
		int tiempo = atoi(arr_tiempo);
		do{
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
			if(s_OFF != NULL){
				apagar = 1;
				buffer[0] = '\0';
			}
		} while(tiempoTranscurridoMinutos(tiempo) == 0 && apagar == 0);
	}
	//
	if(btn_pressed == 1){
		btn_pressed = 0;
		displayModo(HORARIO);
		buffer[0] = '\0';
		Current_State = HORARIO;
	}
	else if(s != NULL){
		if(sH != NULL){
			displayModo(HORARIO);
			buffer[0] = '\0';
			Current_State = HORARIO;
		}
		else if(sE != NULL){
			displayModo(ESPERA);
			buffer[0] = '\0';
			Current_State = ESPERA;
		}
		else if(sA != NULL){
			displayModo(AUTOMATICO);
			buffer[0] = '\0';
			Current_State = AUTOMATICO;
		}
	}
}
void HORARIO_function(void){
	char *s  = strstr((char*)buffer,str_CAMBIAR_A_MODO);
	char *sE = strstr((char*)buffer,str_ESPERA);
	char *sM = strstr((char*)buffer,str_MANUAL);
	char *sA = strstr((char*)buffer,str_AUTOMATICO);
	//Funcionamiento del modo horario

	//
	if(btn_pressed == 1){
		btn_pressed = 0;
		displayModo(AUTOMATICO);
		buffer[0] = '\0';
		Current_State = AUTOMATICO;
	}
	else if(s != NULL){
		if(sM != NULL){
			displayModo(MANUAL);
			buffer[0] = '\0';
			Current_State = MANUAL;
		}
		else if(sE != NULL){
			displayModo(ESPERA);
			buffer[0] = '\0';
			Current_State = ESPERA;
		}
		else if(sA != NULL){
			displayModo(AUTOMATICO);
			buffer[0] = '\0';
			Current_State = AUTOMATICO;
		}
	}
}

void AUTOMATICO_function(void){
	char *s  = strstr((char*)buffer,str_CAMBIAR_A_MODO);
	char *sE = strstr((char*)buffer,str_ESPERA);
	char *sM = strstr((char*)buffer,str_MANUAL);
	char *sH = strstr((char*)buffer,str_HORARIO);
	//Funcionamiento del modo automático
	//
	if(btn_pressed == 1){
		btn_pressed = 0;
		displayModo(ESPERA);
		buffer[0] = '\0';
		Current_State = ESPERA;
	}
	else if(s != NULL){
		if(sM != NULL){
			displayModo(MANUAL);
			buffer[0] = '\0';
			Current_State = MANUAL;
		}
		else if(sE != NULL){
			displayModo(ESPERA);
			buffer[0] = '\0';
			Current_State = ESPERA;
		}
		else if(sH != NULL){
			displayModo(HORARIO);
			buffer[0] = '\0';
			Current_State = HORARIO;
		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2){ //Horas
		horas++;
	}
	if(htim->Instance == TIM3){ //Minutos
		minutos++;
	}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)){};
    if (GPIO_Pin == GPIO_PIN_0) // check if the interrupt was triggered by PA0
    {
        uint32_t current_time = HAL_GetTick(); // get current time
        if (current_time < last_interrupt_time + DEBOUNCE_DELAY) // check if the interrupt was triggered within the debounce delay
        {
            return; // ignore interrupt
        }
        last_interrupt_time = current_time; // update last interrupt time
        btn_pressed = 1;
    }
}
/* USER CODE END 4 */

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
