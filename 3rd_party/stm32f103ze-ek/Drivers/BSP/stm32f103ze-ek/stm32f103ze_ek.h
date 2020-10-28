/**
  ******************************************************************************
  * @file    stm32f103ze_ek.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    30-December-2016
  * @brief   This file contains definitions for stm32f103ze-ek's LEDs,
  *          push-buttons and COM ports hardware resources.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F103ZE_EK_H
#define __STM32F103ZE_EK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F103ZE_EK
  * @{
  */

/** @addtogroup STM32F103ZE_EK_LOW_LEVEL
  * @{
  */

/** @defgroup STM32F103ZE_EK_LOW_LEVEL_Exported_Types STM32F103ZE_EK_LOW_LEVEL Exported Types
  * @{
  */

/**
 * @brief LED Types Definition
 */
typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3,

  LED_GREEN  = LED1,
  LED_ORANGE = LED2,
  LED_RED    = LED3,
  LED_BLUE   = LED4

}Led_TypeDef;

/**
 * @brief BUTTON Types Definition
 */
typedef enum 
{
  BUTTON_WAKEUP = 0,
  BUTTON_TAMPER = 1,
  BUTTON_KEY    = 2,
  BUTTON_SEL    = 3,
  BUTTON_LEFT   = 4,
  BUTTON_RIGHT  = 5,
  BUTTON_DOWN   = 6,
  BUTTON_UP     = 7,

} Button_TypeDef;

typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1,
  BUTTON_MODE_EVT  = 2

} ButtonMode_TypeDef;

/**
 * @brief JOYSTICK Types Definition
 */
typedef enum 
{ 
  JOY_SEL   = 0,
  JOY_LEFT  = 1,
  JOY_RIGHT = 2,
  JOY_DOWN  = 3,
  JOY_UP    = 4,
  JOY_NONE  = 5

}JOYState_TypeDef;

typedef enum 
{ 
  JOY_MODE_GPIO = 0,
  JOY_MODE_EXTI = 1

}JOYMode_TypeDef;

/**
 * @brief COM Types Definition
 */
typedef enum 
{
  COM1 = 0,
  COM2 = 1

} COM_TypeDef;  

/**
  * @}
  */

/** @defgroup STM32F103ZE_EK_LOW_LEVEL_Exported_Constants STM32F103ZE_EK_LOW_LEVEL Exported Constants
  * @{
  */

/**
  * @brief  Define for STM32746G_DISCOVERY board
  */
#if !defined (USE_STM32F103ZE_EK)
 #define USE_STM32F103ZE_EK
#endif

/** @addtogroup STM32746G_DISCOVERY_LOW_LEVEL_LED
  * @{
  */

#define LEDn                             ((uint8_t)4)

#define LED1_PIN                         GPIO_PIN_6             /* PF.06*/
#define LED1_GPIO_PORT                   GPIOF
#define LED1_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()
  
#define LED2_PIN                         GPIO_PIN_7             /* PF.07*/
#define LED2_GPIO_PORT                   GPIOF
#define LED2_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED2_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()


#define LED3_PIN                         GPIO_PIN_8            /* PF.08*/
#define LED3_GPIO_PORT                   GPIOF
#define LED3_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED3_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()


#define LED4_PIN                         GPIO_PIN_9            /* PF.09*/
#define LED4_GPIO_PORT                   GPIOF
#define LED4_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOF_CLK_ENABLE()
#define LED4_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOF_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__LED__) do { if ((__LED__) == LED1) LED1_GPIO_CLK_ENABLE(); else\
                                           if ((__LED__) == LED2) LED2_GPIO_CLK_ENABLE(); else \
                                           if ((__LED__) == LED3) LED3_GPIO_CLK_ENABLE(); else\
                                           if ((__LED__) == LED4) LED4_GPIO_CLK_ENABLE();} while(0)

#define LEDx_GPIO_CLK_DISABLE(__LED__)   (((__LED__) == LED1) ? LED1_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED2) ? LED2_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED3) ? LED3_GPIO_CLK_DISABLE() :\
                                          ((__LED__) == LED4) ? LED4_GPIO_CLK_DISABLE() : 0 )

/**
  * @}
  */

/** @addtogroup STM32F103ZE_EK_LOW_LEVEL_BUTTON
  * @{
  */
#define JOYn                             	((uint8_t)5)
#define BUTTONn                             ((uint8_t)3 + JOYn)

/**
 * @brief Tamper push-button
 */
#define TAMPER_BUTTON_PIN                   GPIO_PIN_13             /* PC.13*/
#define TAMPER_BUTTON_GPIO_PORT             GPIOC
#define TAMPER_BUTTON_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
#define TAMPER_BUTTON_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOC_CLK_DISABLE()
#define TAMPER_BUTTON_EXTI_IRQn             EXTI15_10_IRQn

/**
 * @brief Key push-button
 */
#define KEY_BUTTON_PIN                      GPIO_PIN_8             /* PG.08*/
#define KEY_BUTTON_GPIO_PORT                GPIOG
#define KEY_BUTTON_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOG_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOG_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn                EXTI9_5_IRQn

/**
 * @brief Wake-up push-button
 */
#define WAKEUP_BUTTON_PIN                   GPIO_PIN_0             /* PA.00*/
#define WAKEUP_BUTTON_GPIO_PORT             GPIOA
#define WAKEUP_BUTTON_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define WAKEUP_BUTTON_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define WAKEUP_BUTTON_EXTI_IRQn             EXTI0_IRQn

/**
 * @brief Joystick Right push-button
 */
#define RIGHT_JOY_PIN                       GPIO_PIN_13             /* PG.13*/
#define RIGHT_JOY_GPIO_PORT                 GPIOG
#define RIGHT_JOY_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOG_CLK_ENABLE()
#define RIGHT_JOY_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOG_CLK_DISABLE()
#define RIGHT_JOY_EXTI_IRQn                 EXTI15_10_IRQn

/**
 * @brief Joystick Left push-button
 */    
#define LEFT_JOY_PIN                        GPIO_PIN_14             /* PG.14*/
#define LEFT_JOY_GPIO_PORT                  GPIOG
#define LEFT_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOG_CLK_ENABLE()
#define LEFT_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOG_CLK_DISABLE()
#define LEFT_JOY_EXTI_IRQn                  EXTI15_10_IRQn

/**
 * @brief Joystick Up push-button
 */
#define UP_JOY_PIN                          GPIO_PIN_15             /* PG.15*/
#define UP_JOY_GPIO_PORT                    GPIOG
#define UP_JOY_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOG_CLK_ENABLE()
#define UP_JOY_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOG_CLK_DISABLE()
#define UP_JOY_EXTI_IRQn                    EXTI15_10_IRQn

/**
 * @brief Joystick Down push-button
 */   
#define DOWN_JOY_PIN                        GPIO_PIN_3             /* PD.03*/
#define DOWN_JOY_GPIO_PORT                  GPIOD
#define DOWN_JOY_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOD_CLK_ENABLE()
#define DOWN_JOY_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOD_CLK_DISABLE()
#define DOWN_JOY_EXTI_IRQn                  EXTI3_IRQn

/**
 * @brief Joystick Sel push-button
 */  
#define SEL_JOY_PIN                         GPIO_PIN_7             /* PG.07*/
#define SEL_JOY_GPIO_PORT                   GPIOG
#define SEL_JOY_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOG_CLK_ENABLE()
#define SEL_JOY_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOG_CLK_DISABLE()
#define SEL_JOY_EXTI_IRQn                   EXTI9_5_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__BUTTON__) do { if ((__BUTTON__) == BUTTON_TAMPER) TAMPER_BUTTON_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_KEY) KEY_BUTTON_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_WAKEUP) WAKEUP_BUTTON_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_SEL) SEL_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_LEFT) LEFT_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_RIGHT) RIGHT_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_DOWN) DOWN_JOY_GPIO_CLK_ENABLE(); else\
                                                 if ((__BUTTON__) == BUTTON_UP) UP_JOY_GPIO_CLK_ENABLE();} while(0)

#define BUTTONx_GPIO_CLK_DISABLE(__BUTTON__)    (((__BUTTON__) == BUTTON_TAMPER) ? TAMPER_BUTTON_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_KEY) ? KEY_BUTTON_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_WAKEUP) ? WAKEUP_BUTTON_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_SEL) ? SEL_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_LEFT) ? LEFT_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_RIGHT) ? RIGHT_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_DOWN) ? DOWN_JOY_GPIO_CLK_DISABLE()  :\
                                                 ((__BUTTON__) == BUTTON_UP) ? UP_JOY_GPIO_CLK_DISABLE() : 0 )

#define JOYx_GPIO_CLK_ENABLE(__JOY__)    do { if ((__JOY__) == JOY_SEL) SEL_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_LEFT) LEFT_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_RIGHT) RIGHT_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_DOWN) DOWN_JOY_GPIO_CLK_ENABLE(); else\
                                              if ((__JOY__) == JOY_UP) UP_JOY_GPIO_CLK_ENABLE();} while(0)

#define JOYx_GPIO_CLK_DISABLE(__JOY__)   (((__JOY__) == JOY_SEL) ? SEL_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_LEFT) ? LEFT_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_RIGHT) ? RIGHT_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_DOWN) ? DOWN_JOY_GPIO_CLK_DISABLE() :\
                                          ((__JOY__) == JOY_UP) ? UP_JOY_GPIO_CLK_DISABLE() : 0 )

/**
  * @}
  */

/** @addtogroup STM32F103ZE_EK_LOW_LEVEL_SIGNAL
  * @{
  */
#define SIGNALn                             ((uint8_t)1)

/**
  * @brief SD-detect signal
  */
#define SD_DETECT_PIN                        GPIO_PIN_13
#define SD_DETECT_GPIO_PORT                  GPIOC
#define SD_DETECT_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOC_CLK_ENABLE()
#define SD_DETECT_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOC_CLK_DISABLE()
#define SD_DETECT_EXTI_IRQn                  EXTI15_10_IRQn

/**
  * @brief Touch screen interrupt signal
  */
#define TS_INT_PIN                           GPIO_PIN_13
#define TS_INT_GPIO_PORT                     GPIOI
#define TS_INT_GPIO_CLK_ENABLE()             __HAL_RCC_GPIOI_CLK_ENABLE()
#define TS_INT_GPIO_CLK_DISABLE()            __HAL_RCC_GPIOI_CLK_DISABLE()
#define TS_INT_EXTI_IRQn                     EXTI15_10_IRQn

/**
  * @}
  */

/** @addtogroup STM32F103ZE_EK_LOW_LEVEL_COM
  * @{
  */
#define COMn                             ((uint8_t)2)

/**
 * @brief Definition for COM port1, connected to USART1
 */ 
#define EVAL_COM1                        USART1
#define EVAL_COM1_CLK_ENABLE()           __HAL_RCC_USART1_CLK_ENABLE()
#define EVAL_COM1_CLK_DISABLE()          __HAL_RCC_USART1_CLK_DISABLE()

#define EVAL_COM1_TX_PIN                 GPIO_PIN_9             /* PA.09*/
#define EVAL_COM1_TX_GPIO_PORT           GPIOA
#define EVAL_COM1_TX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM1_TX_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()

#define EVAL_COM1_RX_PIN                 GPIO_PIN_10             /* PA.10*/
#define EVAL_COM1_RX_GPIO_PORT           GPIOA
#define EVAL_COM1_RX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM1_RX_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()

#define EVAL_COM1_IRQn                   USART1_IRQn

/**
 * @brief Definition for COM port2, connected to USART2
 */ 
#define EVAL_COM2                        USART2
#define EVAL_COM2_CLK_ENABLE()           __HAL_RCC_USART2_CLK_ENABLE()
#define EVAL_COM2_CLK_DISABLE()          __HAL_RCC_USART2_CLK_DISABLE()

#define EVAL_COM2_TX_PIN                 GPIO_PIN_2             /* PA.02*/
#define EVAL_COM2_TX_GPIO_PORT           GPIOA
#define EVAL_COM2_TX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM2_TX_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()

#define EVAL_COM2_RX_PIN                 GPIO_PIN_3             /* PA.03*/
#define EVAL_COM2_RX_GPIO_PORT           GPIOA
#define EVAL_COM2_RX_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define EVAL_COM2_RX_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()

#define EVAL_COM2_IRQn                   USART2_IRQn

#define COMx_CLK_ENABLE(__INDEX__)              do { if((__INDEX__) == COM1) EVAL_COM1_CLK_ENABLE(); else\
                                                     if((__INDEX__) == COM2) EVAL_COM2_CLK_ENABLE();} while(0)

#define COMx_CLK_DISABLE(__INDEX__)             (((__INDEX__) == COM1) ? EVAL_COM1_CLK_DISABLE() :\
                                                 ((__INDEX__) == COM2) ? EVAL_COM2_CLK_DISABLE() : 0)

#define COMx_TX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) EVAL_COM1_TX_GPIO_CLK_ENABLE(); else\
                                                     if((__INDEX__) == COM2) EVAL_COM2_TX_GPIO_CLK_ENABLE();} while(0)

#define COMx_TX_GPIO_CLK_DISABLE(__INDEX__)     (((__INDEX__) == COM1) ? EVAL_COM1_TX_GPIO_CLK_DISABLE() :\
                                                 ((__INDEX__) == COM2) ? EVAL_COM2_TX_GPIO_CLK_DISABLE() : 0)

#define COMx_RX_GPIO_CLK_ENABLE(__INDEX__)      do { if((__INDEX__) == COM1) EVAL_COM1_RX_GPIO_CLK_ENABLE(); else\
                                                     if((__INDEX__) == COM2) EVAL_COM2_RX_GPIO_CLK_ENABLE();} while(0)

#define COMx_RX_GPIO_CLK_DISABLE(__INDEX__)     (((__INDEX__) == COM1) ? EVAL_COM1_RX_GPIO_CLK_DISABLE() :\
                                                 ((__INDEX__) == COM2) ? EVAL_COM2_RX_GPIO_CLK_DISABLE() : 0)

/* Exported constant IO ------------------------------------------------------*/

//#define LCD_I2C_ADDRESS                  ((uint16_t)0x70)
//#define CAMERA_I2C_ADDRESS               ((uint16_t)0x60)
//#define AUDIO_I2C_ADDRESS                ((uint16_t)0x34)
//#define EEPROM_I2C_ADDRESS_A01           ((uint16_t)0xA0)
//#define EEPROM_I2C_ADDRESS_A02           ((uint16_t)0xA6)
//#define TS_I2C_ADDRESS                   ((uint16_t)0x70)

/* I2C clock speed configuration (in Hz)
   WARNING:
   Make sure that this define is not already declared in other files (ie.
   STM32F103ZE_EK.h file). It can be used in parallel by other modules. */
//#ifndef I2C_SPEED
// #define I2C_SPEED                       ((uint32_t)100000)
//#endif /* I2C_SPEED */

/* User can use this section to tailor I2Cx/I2Cx instance used and associated
   resources */
/* Definition for AUDIO and LCD I2Cx resources */
//#define DISCOVERY_AUDIO_I2Cx                             I2C3
//#define DISCOVERY_AUDIO_I2Cx_CLK_ENABLE()                __HAL_RCC_I2C3_CLK_ENABLE()
//#define DISCOVERY_AUDIO_DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
//#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOH_CLK_ENABLE()

//#define DISCOVERY_AUDIO_I2Cx_FORCE_RESET()               __HAL_RCC_I2C3_FORCE_RESET()
//#define DISCOVERY_AUDIO_I2Cx_RELEASE_RESET()             __HAL_RCC_I2C3_RELEASE_RESET()

/* Definition for I2Cx Pins */
//#define DISCOVERY_AUDIO_I2Cx_SCL_PIN                     GPIO_PIN_7
//#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_GPIO_PORT           GPIOH
//#define DISCOVERY_AUDIO_I2Cx_SCL_SDA_AF                  GPIO_AF4_I2C3
//#define DISCOVERY_AUDIO_I2Cx_SDA_PIN                     GPIO_PIN_8

//#define EVAL_I2Cx_SCL_PIN                       GPIO_PIN_6        /* PB.06*/
//#define EVAL_I2Cx_SCL_GPIO_PORT                 GPIOB
//#define EVAL_I2Cx_SDA_PIN                       GPIO_PIN_7        /* PB.07*/
//#define EVAL_I2Cx_SDA_GPIO_PORT                 GPIOB

/* I2C interrupt requests */
//#define DISCOVERY_AUDIO_I2Cx_EV_IRQn                     I2C3_EV_IRQn
//#define DISCOVERY_AUDIO_I2Cx_ER_IRQn                     I2C3_ER_IRQn

/* Definition for external, camera and Arduino connector I2Cx resources */
//#define DISCOVERY_EXT_I2Cx                               I2C1
//#define DISCOVERY_EXT_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()
//#define DISCOVERY_EXT_DMAx_CLK_ENABLE()                  __HAL_RCC_DMA1_CLK_ENABLE()
//#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOB_CLK_ENABLE()

//#define DISCOVERY_EXT_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C1_FORCE_RESET()
//#define DISCOVERY_EXT_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C1_RELEASE_RESET()

//#define EVAL_I2Cx                               I2C1
//#define EVAL_I2Cx_CLK_ENABLE()                  __HAL_RCC_I2C1_CLK_ENABLE()
//#define EVAL_I2Cx_SDA_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
//#define EVAL_I2Cx_SCL_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE() 

//#define EVAL_I2Cx_FORCE_RESET()                 __HAL_RCC_I2C1_FORCE_RESET()
//#define EVAL_I2Cx_RELEASE_RESET()               __HAL_RCC_I2C1_RELEASE_RESET()

/* Definition for I2Cx's NVIC */
//#define EVAL_I2Cx_EV_IRQn                       I2C1_EV_IRQn
//#define EVAL_I2Cx_EV_IRQHandler                 I2C1_EV_IRQHandler
//#define EVAL_I2Cx_ER_IRQn                       I2C1_ER_IRQn
//#define EVAL_I2Cx_ER_IRQHandler                 I2C1_ER_IRQHandler
/* I2C clock speed configuration (in Hz) 
   WARNING: 
   Make sure that this define is not already declared in other files (ie. 
   stm3210e_eval.h file). It can be used in parallel by other modules. */
//#ifndef BSP_I2C_SPEED
// #define BSP_I2C_SPEED                            100000
//#endif /* I2C_SPEED */


/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
//#define EVAL_I2Cx_TIMEOUT_MAX                   3000

/* Definition for I2Cx Pins */
//#define DISCOVERY_EXT_I2Cx_SCL_PIN                       GPIO_PIN_8
//#define DISCOVERY_EXT_I2Cx_SCL_SDA_GPIO_PORT             GPIOB
//#define DISCOVERY_EXT_I2Cx_SCL_SDA_AF                    GPIO_AF4_I2C1
//#define DISCOVERY_EXT_I2Cx_SDA_PIN                       GPIO_PIN_9

/* I2C interrupt requests */
//#define DISCOVERY_EXT_I2Cx_EV_IRQn                       I2C1_EV_IRQn
//#define DISCOVERY_EXT_I2Cx_ER_IRQn                       I2C1_ER_IRQn

/* I2C TIMING Register define when I2C clock source is SYSCLK */
/* I2C TIMING is calculated from APB1 source clock = 50 MHz */
/* Due to the big MOFSET capacity for adapting the camera level the rising time is very large (>1us) */
/* 0x40912732 takes in account the big rising and aims a clock of 100khz */
//#ifndef DISCOVERY_I2Cx_TIMING
//#define DISCOVERY_I2Cx_TIMING                      ((uint32_t)0x40912732)
//#endif /* DISCOVERY_I2Cx_TIMING */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F103ZE_EK_LOW_LEVEL_Exported_Macros STM32F103ZE_EK_LOW_LEVEL Exported Macros
  * @{
  */
/**
  * @}
  */

/** @addtogroup STM32F103ZE_EK_LOW_LEVEL_Exported_Functions
  * @{
  */
uint32_t  BSP_GetVersion(void);
void      BSP_LED_Init(Led_TypeDef Led);
void      BSP_LED_DeInit(Led_TypeDef Led);
void      BSP_LED_On(Led_TypeDef Led);
void      BSP_LED_Off(Led_TypeDef Led);
void      BSP_LED_Toggle(Led_TypeDef Led);
void      BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void      BSP_PB_DeInit(Button_TypeDef Button);
uint32_t  BSP_PB_GetState(Button_TypeDef Button);
#ifdef HAL_UART_MODULE_ENABLED
void      BSP_COM_Init(COM_TypeDef COM, UART_HandleTypeDef *husart);
void      BSP_COM_DeInit(COM_TypeDef COM, UART_HandleTypeDef *huart);
#endif /* HAL_UART_MODULE_ENABLED */
uint8_t                 BSP_JOY_Init(JOYMode_TypeDef Joy_Mode);
JOYState_TypeDef        BSP_JOY_GetState(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F103ZE_EK_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
