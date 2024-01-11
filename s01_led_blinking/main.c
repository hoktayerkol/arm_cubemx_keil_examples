#include "stm32f4xx.h"

int main()
{
	__HAL_RCC_GPIOD_CLK_ENABLE();	// port d clock enable
	GPIO_InitTypeDef leds;		// define ports
	leds.Mode=GPIO_MODE_OUTPUT_PP;	// mode output push pull
	leds.Pin=GPIO_PIN_12;		// led on portd.12
	HAL_GPIO_Init(GPIOD, &leds);	// init port
	
	while (1)
	{
	
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);		// toggle pin
		
		// create a delay
		for (int i=0; i<1000000; i++);
			
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);		// toggle pin
		
		// create a delay
		for (int i=0; i<1000000; i++);
		
	}
	
		
	
}
