// saniyede bir timer2 ile kesme üretilip ledler tersleniyor
#include "stm32f4xx.h"

void Init_Board(void);
void Delay_ms(int);	// this function will be rewriten later using timers
void TIM2_IRQHandler(void);

//	GPIO_PinState state; // Define a enum struct which contain boolean states 


int main()
{

	Init_Board();
	
	while (1)
	{		}

}


void Init_Board(void)
{
	// GPIO init
	__HAL_RCC_GPIOD_CLK_ENABLE();	// port d clock enable
	GPIO_InitTypeDef leds;		// define ports
	leds.Mode=GPIO_MODE_OUTPUT_PP;	// mode output push pull
	leds.Pin=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;		// leds on portd12
	HAL_GPIO_Init(GPIOD, &leds);	// init port
	


	// timer2 
	__HAL_RCC_TIM2_CLK_ENABLE();
	TIM_ClockConfigTypeDef sClockSourceConfig;
	sClockSourceConfig.ClockSource=TIM_CLOCKSOURCE_INTERNAL;
	sClockSourceConfig.ClockPrescaler=TIM_CLOCKPRESCALER_DIV1;	// no prescalar
	
	// TIM_HandleTypeDef htim2; // global olarak tanimlandi
	TIM_HandleTypeDef htim2;
	htim2.Instance=TIM2;
	htim2.Init.Prescaler=41999 ;	// 84 mhz to 2khz
	htim2.Init.Period=500;	// 2khz to 1s
	htim2.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	htim2.Init.CounterMode=TIM_COUNTERMODE_UP;
	htim2.Init.RepetitionCounter=0x0;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    // _Error_Handler(__FILE__, __LINE__);
  }
	
	HAL_TIM_ConfigClockSource (&htim2, &sClockSourceConfig);	// configure the clock source
	
	HAL_TIM_Base_Init(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	
  // Initialize interrupt triggering
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);// Enable interrupt from TIM3 (NVIC level)
	
}

void Delay_ms(int x)
{
	// create a delay
		for (int i=0; i<x*2000; i++)
		{}
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim2)
{
    GPIOD->ODR=255<<12U;
}
*/

void TIM2_IRQHandler(void) // interrupt routine
{
	
	//GPIOD->ODR=255<<12U;
  if(TIM2->SR & TIM_SR_UIF) // if UIF flag is set
  {
    TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
    //GPIOD->ODR=255<<12U;
		//HAL_NVIC_DisableIRQ(TIM2_IRQn);// Disable interrupt from TIM3 (NVIC level)
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
  }
	
}





