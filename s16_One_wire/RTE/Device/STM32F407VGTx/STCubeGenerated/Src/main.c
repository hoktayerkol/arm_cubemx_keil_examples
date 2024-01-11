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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/***********************************************************************/
/* one-wire */

void make_output (void)
{
	//__HAL_RCC_GPIOD_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void make_input (void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void delayus (uint32_t us)
{

	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	//HAL_TIM_Base_Start(&htim2);
	while (__HAL_TIM_GET_COUNTER(&htim2) < us );  // wait for the counter to reach the us input in the parameter
	// HAL_TIM_Base_Stop(&htim2);
	
}

void LH_signal(uint32_t L_time, uint32_t H_time) {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
	delayus(L_time);//From pullup_HIGH to GND_LOW
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
	delayus(H_time);//From GND_LOW to pullup_HIGH
}

uint8_t read_bit(void) {
	uint8_t bit = 0;
	LH_signal(1, 10);
	make_input();
	bit = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) ? 1 : 0);
	delayus(50);
	make_output();
	return bit;
}

void write_bit(uint8_t bit) {
	if(bit == 0) LH_signal(60, 1);
	else LH_signal(1, 60);
}

uint8_t read_byte(void) {
	uint8_t data = 0;
	for (uint8_t i = 0; i < 8; i++)
	data += read_bit() << i;
	return data;
}

void write_byte(uint8_t data) {
	for (uint8_t i = 0; i < 8; i++)
		write_bit(data >> i & 1);
}

void get_presence(void) {
	uint8_t flag = 1;
	LH_signal(500, 0);
	make_input();
	delayus(60);//look for GND_LOW = DS18B20 exists, and set flag = 1
	flag = (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) ? 0 : 1); //not ?1:0
	make_output();
	delayus(400);
	if(flag) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1); //DS18B20 is ok, green led
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);	//DS18B20 not ok, red led
	} else  {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1); 
		 
	}
	
	HAL_Delay(1000);
}

void reset(void) {
	LH_signal(500, 500);
}

void get_ID(int * id_data) {
	// uint8_t id_data[] = {0,0,0,0,0,0,0,0};//8 Byte
	reset();
	write_byte(0x33);//Read ROM [33h] command
	for (uint8_t i = 0; i < 8; i++)
		id_data[i] = read_byte();//id_data[0] = 40 = 0x28
	/*
	for (uint8_t i = 0; i < 8; i++)
	{
		USB_Info_tutu("id", i, " = ", id_data[i]);
		HAL_Delay(200);
	}
	*/
}

void wait_for_1(uint32_t time) {
	make_input();
	delayus(time);
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0) == 0);
	make_output();
}

float get_temperature(void) {
	uint8_t pad_data[] = {0,0,0,0,0,0,0,0,0};//9 Byte
	reset();
	write_byte(0xCC);//Skip ROM [CCh]
	write_byte(0x44);//Convert Temperature [44h]
	wait_for_1(20);
	reset();
	write_byte(0xCC);//Skip ROM [CCh]
	write_byte(0xBE);//Read Scratchpad [BEh]
	for (uint8_t i = 0; i < 9; i++)
	pad_data[i] = read_byte();
	//factor out 1/16 and remember 1/16 != 1/16.0
	return ((pad_data[1] << 8) + pad_data[0])/16.0;
	//DS18B20.pdf: val of bit0: 2^(-4) = 1/16
}


/***********************************************************************/
	
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	
	
	// lcd_init
	HAL_GPIO_WritePin(RW_GPIO_Port, RW_Pin, 0);	
	Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
	Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
	Lcd_HandleTypeDef lcd;
	lcd = Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);

	// write something and wait 3sec
	Lcd_string(&lcd, "BANU Embedded Sys.");
	Lcd_cursor(&lcd, 1,6);
	Lcd_int(&lcd, 2023);
	HAL_Delay(1000);
	Lcd_clear(&lcd);
	

	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	char str[16];
	int a=60;
	
	uint32_t L_time=5;
	uint32_t H_time=60;
	make_output();
	
	int ID[8];
	int * pID=ID;
	double temp;
	
	HAL_TIM_Base_Start(&htim2);
	make_output();
	reset();
	
  while (1)
  {
	
		get_presence();	
		
		get_ID(pID);
		
		// write id to the lcd		
		for (int i=0; i<16; i=i+2)
		{
			Lcd_cursor(&lcd, 0, i);
			snprintf(str, 16, "%x ", ID[i]);
			Lcd_string(&lcd, str);		
		}
		
		temp=get_temperature();
		Lcd_cursor(&lcd, 1, 0);
		snprintf(str, 16, "Temp= %f ", temp);
		Lcd_string(&lcd, str);	
		
		
		
		
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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RS_Pin|RW_Pin|EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin
                           RS_Pin RW_Pin EN_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin
                          |RS_Pin|RW_Pin|EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
