/******************************************************************************
 * File Name   : MX_Device.h
 * Date        : 09/02/2023 10:07:48
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

/*-------------------------------- DAC        --------------------------------*/

#define MX_DAC                                  1

/* GPIO Configuration */

/* Pin PA4 */
#define MX_COMP_DAC1_group_Pin                  PA4
#define MX_COMP_DAC1_group_GPIOx                GPIOA
#define MX_COMP_DAC1_group_GPIO_PuPd            GPIO_NOPULL
#define MX_COMP_DAC1_group_GPIO_Pin             GPIO_PIN_4
#define MX_COMP_DAC1_group_GPIO_Mode            GPIO_MODE_ANALOG

/*-------------------------------- SYS        --------------------------------*/

#define MX_SYS                                  1

/* GPIO Configuration */

/*-------------------------------- NVIC       --------------------------------*/

#define MX_NVIC                                 1

/*-------------------------------- GPIO       --------------------------------*/

#define MX_GPIO                                 1

/* GPIO Configuration */

/* Pin PE6 */
#define MX_PE6_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE6_Pin                              PE6
#define MX_PE6_GPIOx                            GPIOE
#define MX_PE6_PinState                         GPIO_PIN_RESET
#define MX_PE6_GPIO_PuPd                        GPIO_NOPULL
#define MX_D6                                   PE6
#define MX_PE6_GPIO_Pin                         GPIO_PIN_6
#define MX_PE6_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

/* Pin PD12 */
#define MX_PD12_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD12_Pin                             PD12
#define MX_PD12_GPIOx                           GPIOD
#define MX_PD12_PinState                        GPIO_PIN_RESET
#define MX_PD12_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD12_GPIO_Pin                        GPIO_PIN_12
#define MX_PD12_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE5 */
#define MX_PE5_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE5_Pin                              PE5
#define MX_PE5_GPIOx                            GPIOE
#define MX_PE5_PinState                         GPIO_PIN_RESET
#define MX_PE5_GPIO_PuPd                        GPIO_NOPULL
#define MX_D5                                   PE5
#define MX_PE5_GPIO_Pin                         GPIO_PIN_5
#define MX_PE5_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

/* Pin PE10 */
#define MX_PE10_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PE10_Pin                             PE10
#define MX_PE10_GPIOx                           GPIOE
#define MX_PE10_PinState                        GPIO_PIN_RESET
#define MX_PE10_GPIO_PuPd                       GPIO_NOPULL
#define MX_EN                                   PE10
#define MX_PE10_GPIO_Pin                        GPIO_PIN_10
#define MX_PE10_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE8 */
#define MX_PE8_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE8_Pin                              PE8
#define MX_PE8_GPIOx                            GPIOE
#define MX_PE8_PinState                         GPIO_PIN_RESET
#define MX_PE8_GPIO_PuPd                        GPIO_NOPULL
#define MX_RS                                   PE8
#define MX_PE8_GPIO_Pin                         GPIO_PIN_8
#define MX_PE8_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

/* Pin PD14 */
#define MX_PD14_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD14_Pin                             PD14
#define MX_PD14_GPIOx                           GPIOD
#define MX_PD14_PinState                        GPIO_PIN_RESET
#define MX_PD14_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD14_GPIO_Pin                        GPIO_PIN_14
#define MX_PD14_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE7 */
#define MX_PE7_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE7_Pin                              PE7
#define MX_PE7_GPIOx                            GPIOE
#define MX_PE7_PinState                         GPIO_PIN_RESET
#define MX_PE7_GPIO_PuPd                        GPIO_NOPULL
#define MX_D7                                   PE7
#define MX_PE7_GPIO_Pin                         GPIO_PIN_7
#define MX_PE7_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

/* Pin PD13 */
#define MX_PD13_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD13_Pin                             PD13
#define MX_PD13_GPIOx                           GPIOD
#define MX_PD13_PinState                        GPIO_PIN_RESET
#define MX_PD13_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD13_GPIO_Pin                        GPIO_PIN_13
#define MX_PD13_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE9 */
#define MX_PE9_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE9_Pin                              PE9
#define MX_PE9_GPIOx                            GPIOE
#define MX_PE9_PinState                         GPIO_PIN_RESET
#define MX_PE9_GPIO_PuPd                        GPIO_NOPULL
#define MX_RW                                   PE9
#define MX_PE9_GPIO_Pin                         GPIO_PIN_9
#define MX_PE9_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

/* Pin PD15 */
#define MX_PD15_GPIO_Speed                      GPIO_SPEED_FREQ_LOW
#define MX_PD15_Pin                             PD15
#define MX_PD15_GPIOx                           GPIOD
#define MX_PD15_PinState                        GPIO_PIN_RESET
#define MX_PD15_GPIO_PuPd                       GPIO_NOPULL
#define MX_PD15_GPIO_Pin                        GPIO_PIN_15
#define MX_PD15_GPIO_ModeDefaultOutputPP        GPIO_MODE_OUTPUT_PP

/* Pin PE4 */
#define MX_PE4_GPIO_Speed                       GPIO_SPEED_FREQ_LOW
#define MX_PE4_Pin                              PE4
#define MX_PE4_GPIOx                            GPIOE
#define MX_PE4_PinState                         GPIO_PIN_RESET
#define MX_PE4_GPIO_PuPd                        GPIO_NOPULL
#define MX_D4                                   PE4
#define MX_PE4_GPIO_Pin                         GPIO_PIN_4
#define MX_PE4_GPIO_ModeDefaultOutputPP         GPIO_MODE_OUTPUT_PP

#endif  /* __MX_DEVICE_H */

