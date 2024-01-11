/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 06/02/2023 21:30:50
 * Description : STM32Cube MX parameter definitions
 * Note        : This file is generated by STM32CubeMX (DO NOT EDIT!)
 ******************************************************************************/

#ifndef __MX_DEVICE_H
#define __MX_DEVICE_H

/*---------------------------- Clock Configuration ---------------------------*/

#define MX_LSI_VALUE                            32000
#define MX_LSE_VALUE                            32768
#define MX_HSI_VALUE                            16000000
#define MX_HSE_VALUE                            8000000
#define MX_EXTERNAL_CLOCK_VALUE                 12288000
#define MX_PLLCLKFreq_Value                     168000000
#define MX_SYSCLKFreq_VALUE                     168000000
#define MX_HCLKFreq_Value                       168000000
#define MX_FCLKCortexFreq_Value                 168000000
#define MX_CortexFreq_Value                     168000000
#define MX_AHBFreq_Value                        168000000
#define MX_APB1Freq_Value                       42000000
#define MX_APB2Freq_Value                       84000000
#define MX_APB1TimFreq_Value                    84000000
#define MX_APB2TimFreq_Value                    168000000
#define MX_48MHZClocksFreq_Value                84000000
#define MX_EthernetFreq_Value                   168000000
#define MX_I2SClocksFreq_Value                  192000000
#define MX_RTCFreq_Value                        32000
#define MX_WatchDogFreq_Value                   32000
#define MX_MCO1PinFreq_Value                    16000000
#define MX_MCO2PinFreq_Value                    168000000

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- TIM4       --------------------------------*/

#define MX_TIM4                                 1

/* GPIO Configuration */

/* Pin PD15 */
#define MX_S_TIM4_CH4_GPIO_ModeDefaultPP        GPIO_MODE_AF_PP
#define MX_S_TIM4_CH4_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_S_TIM4_CH4_Pin                       PD15
#define MX_S_TIM4_CH4_GPIOx                     GPIOD
#define MX_S_TIM4_CH4_GPIO_PuPd                 GPIO_NOPULL
#define MX_S_TIM4_CH4_GPIO_Pin                  GPIO_PIN_15
#define MX_S_TIM4_CH4_GPIO_AF                   GPIO_AF2_TIM4

/* Pin PD13 */
#define MX_S_TIM4_CH2_GPIO_ModeDefaultPP        GPIO_MODE_AF_PP
#define MX_S_TIM4_CH2_GPIO_Speed                GPIO_SPEED_FREQ_LOW
#define MX_S_TIM4_CH2_Pin                       PD13
#define MX_S_TIM4_CH2_GPIOx                     GPIOD
#define MX_S_TIM4_CH2_GPIO_PuPd                 GPIO_NOPULL
#define MX_S_TIM4_CH2_GPIO_Pin                  GPIO_PIN_13
#define MX_S_TIM4_CH2_GPIO_AF                   GPIO_AF2_TIM4

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PD12 */
#define MX_PD12_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD12_Pin                             PD12
#define MX_PD12_GPIOx                           GPIOD
#define MX_PD12_PinState                        GPIO_PIN_RESET
#define MX_PD12_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD12_GPIO_Pin                        GPIO_PIN_12
#define MX_PD12_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PD14 */
#define MX_PD14_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD14_Pin                             PD14
#define MX_PD14_GPIOx                           GPIOD
#define MX_PD14_PinState                        GPIO_PIN_RESET
#define MX_PD14_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD14_GPIO_Pin                        GPIO_PIN_14
#define MX_PD14_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

#endif  /* __MX_DEVICE_H */

