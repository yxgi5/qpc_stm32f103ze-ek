/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f1xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "stm32f103ze_ek.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */
//#ifdef Q_SPY
//
///**
// * @brief UART MSP Initialization
// *        This function configures the hardware resources used in this example:
// *           - Peripheral's clock enable
// *           - Peripheral's GPIO Configuration
// * @param huart: UART handle pointer
// * @retval None
// */
//void HAL_UART_MspInit(UART_HandleTypeDef *huart)
//{
//    GPIO_InitTypeDef  GPIO_InitStruct;
//
//    /*##-1- Enable peripherals and GPIO Clocks #################################*/
//    /* Enable GPIO TX/RX clock */
//    USARTx_TX_GPIO_CLK_ENABLE();
//    USARTx_RX_GPIO_CLK_ENABLE();
//
//    /* Enable USARTx clock */
//    USARTx_CLK_ENABLE();
//
//    /*##-2- Configure peripheral GPIO ##########################################*/
//    /* UART TX GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = USARTx_TX_PIN;
//    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull      = GPIO_PULLUP;
//    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
//    GPIO_InitStruct.Alternate = USARTx_TX_AF;
//    HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
//
//    /* UART RX GPIO pin configuration  */
//    GPIO_InitStruct.Pin       = USARTx_RX_PIN;
//    GPIO_InitStruct.Alternate = USARTx_RX_AF;
//    HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
//
//    /*##-3- Configure the NVIC for UART ########################################*/
//    /* NVIC for USART */
//    HAL_NVIC_SetPriority(USARTx_IRQn, 0, 1);
//    HAL_NVIC_EnableIRQ(USARTx_IRQn);
//}
//
///**
// * @brief UART MSP De-Initialization
// *        This function frees the hardware resources used in this example:
// *          - Disable the Peripheral's clock
// *          - Revert GPIO configuration to their default state
// * @param huart: UART handle pointer
// * @retval None
// */
//void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
//{
//    /*##-1- Reset peripherals ##################################################*/
//    USARTx_FORCE_RESET();
//    USARTx_RELEASE_RESET();
//
//    /*##-2- Disable peripherals and GPIO Clocks #################################*/
//    /* Configure USART Tx as alternate function  */
//    HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
//    /* Configure USART Rx as alternate function  */
//    HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);
//
//    /*##-3- Disable the NVIC for UART ##########################################*/
//    HAL_NVIC_DisableIRQ(USARTx_IRQn);
//}
//
//#endif /* Q_SPY */
/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
