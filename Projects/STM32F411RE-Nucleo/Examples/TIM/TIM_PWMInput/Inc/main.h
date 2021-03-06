/**
  ******************************************************************************
  * @file    TIM/TIM_PWMInput/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-December-2014
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "cube_hal.h"
#include "x_nucleo_iks01a1.h"
#include "x_nucleo_iks01a1_imu_6axes.h"
#include "x_nucleo_iks01a1_magneto.h"
#include "x_nucleo_iks01a1_pressure.h"
#include "x_nucleo_iks01a1_hum_temp.h"

#include "cube_hal.h"

#include "osal.h"
#include "sample_service.h"
#include "role_type.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"



/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* User can use this section to tailor TIMx instance used and associated 
   resources */
/* Definition for TIMx clock resources */
#define TIMx                           TIM2
#define TIMx_CLK_ENABLE()              __HAL_RCC_TIM2_CLK_ENABLE()

/* Definition for TIMx Pins */
#define TIMx_CHANNEL_GPIO_PORT()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define GPIO_PORT                      GPIOA
#define GPIO_PIN_CHANNEL2              GPIO_PIN_5
#define GPIO_AF_TIMx                   GPIO_AF1_TIM2

/* Definition for TIMx's NVIC */
#define TIMx_IRQn                      TIM4_IRQn
#define TIMx_IRQHandler                TIM4_IRQHandler


/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
