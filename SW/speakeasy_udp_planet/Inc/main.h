/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MII_TXD3_Pin GPIO_PIN_2
#define MII_TXD3_GPIO_Port GPIOE
#define AMP_Gain0_Pin GPIO_PIN_3
#define AMP_Gain0_GPIO_Port GPIOE
#define AMP_Gain1_Pin GPIO_PIN_4
#define AMP_Gain1_GPIO_Port GPIOE
#define AMP_Mute_Pin GPIO_PIN_5
#define AMP_Mute_GPIO_Port GPIOE
#define ADC_LOW_Pin GPIO_PIN_0
#define ADC_LOW_GPIO_Port GPIOC
#define MII_MDC_Pin GPIO_PIN_1
#define MII_MDC_GPIO_Port GPIOC
#define MII_TXD2_Pin GPIO_PIN_2
#define MII_TXD2_GPIO_Port GPIOC
#define MII_TX_CLK_Pin GPIO_PIN_3
#define MII_TX_CLK_GPIO_Port GPIOC
#define MII_CRS_Pin GPIO_PIN_0
#define MII_CRS_GPIO_Port GPIOA
#define MII_RX_CLK_RMII_REF_CLK_Pin GPIO_PIN_1
#define MII_RX_CLK_RMII_REF_CLK_GPIO_Port GPIOA
#define MII_MDIO_Pin GPIO_PIN_2
#define MII_MDIO_GPIO_Port GPIOA
#define MII_COL_Pin GPIO_PIN_3
#define MII_COL_GPIO_Port GPIOA
#define DAC_OUT_Pin GPIO_PIN_4
#define DAC_OUT_GPIO_Port GPIOA
#define ADC_HIGH_MID_Pin GPIO_PIN_5
#define ADC_HIGH_MID_GPIO_Port GPIOA
#define MII_RX_DV_RMII_CRSDV_Pin GPIO_PIN_7
#define MII_RX_DV_RMII_CRSDV_GPIO_Port GPIOA
#define MII_RXD0_Pin GPIO_PIN_4
#define MII_RXD0_GPIO_Port GPIOC
#define MII_RXD1_Pin GPIO_PIN_5
#define MII_RXD1_GPIO_Port GPIOC
#define MII_RXD2_Pin GPIO_PIN_0
#define MII_RXD2_GPIO_Port GPIOB
#define MII_RXD3_Pin GPIO_PIN_1
#define MII_RXD3_GPIO_Port GPIOB
#define PWM_LOW_Pin GPIO_PIN_9
#define PWM_LOW_GPIO_Port GPIOE
#define PWM_MID_Pin GPIO_PIN_11
#define PWM_MID_GPIO_Port GPIOE
#define PWM_HIGH_Pin GPIO_PIN_13
#define PWM_HIGH_GPIO_Port GPIOE
#define MII_TX_EN_Pin GPIO_PIN_11
#define MII_TX_EN_GPIO_Port GPIOB
#define MII_TXD0_Pin GPIO_PIN_12
#define MII_TXD0_GPIO_Port GPIOB
#define MII_TXD1_Pin GPIO_PIN_13
#define MII_TXD1_GPIO_Port GPIOB
#define MII_INT_Pin GPIO_PIN_14
#define MII_INT_GPIO_Port GPIOB
#define PB_PlayPause_Pin GPIO_PIN_8
#define PB_PlayPause_GPIO_Port GPIOD
#define ADC_Switch_Pin GPIO_PIN_11
#define ADC_Switch_GPIO_Port GPIOD
#define LED_Red_Pin GPIO_PIN_12
#define LED_Red_GPIO_Port GPIOD
#define LED_Amber_Pin GPIO_PIN_13
#define LED_Amber_GPIO_Port GPIOD
#define PB_VolD_Pin GPIO_PIN_8
#define PB_VolD_GPIO_Port GPIOA
#define PB_VolU_Pin GPIO_PIN_10
#define PB_VolU_GPIO_Port GPIOA
#define PB_Next_Pin GPIO_PIN_11
#define PB_Next_GPIO_Port GPIOA
#define PHY_Reset_Pin GPIO_PIN_12
#define PHY_Reset_GPIO_Port GPIOA
#define PHY_RX_ERR_Pin GPIO_PIN_7
#define PHY_RX_ERR_GPIO_Port GPIOD
#define PB_Prev_Pin GPIO_PIN_8
#define PB_Prev_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define BUFF_SIZE 20

#define DEST_IP_ADDR0   (uint8_t) 192
#define DEST_IP_ADDR1   (uint8_t) 168
#define DEST_IP_ADDR2   (uint8_t) 0
#define DEST_IP_ADDR3   (uint8_t) 2

#define UDP_SERVER_PORT    (uint16_t) 1234   /* define the UDP local connection port */
#define UDP_CLIENT_PORT    (uint16_t) 1234   /* define the UDP remote connection port */

enum controls {Vol_D, Prev, Play, Pause, Next, Vol_U, NOP};
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
