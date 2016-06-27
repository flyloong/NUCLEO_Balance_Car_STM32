/**
  *******************************************************************************
  * @file    Projects/STM32F4xx-Nucleo/Applications/DataLogFusion/Inc/com.h
  * @author  MEMS Application Team
  * @version V1.1.0
  * @date    08 November 2014
  * @brief   header for com.c.
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COM__H
#define __COM__H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "serial_protocol.h"

/* Exported types ------------------------------------------------------------*/
/**
 * @brief  Serial message engine structure definition
 */
typedef struct {
	uint8_t *pDMA_Buffer;
	uint16_t StartOfMsg;
} TUart_Engine;

/* Exported defines ----------------------------------------------------------*/
#define UART_RxBufferSize (2*TMsg_MaxLen)

/* exported variables --------------------------------------------------------*/
extern volatile uint8_t UART_RxBuffer[UART_RxBufferSize];
extern TUart_Engine Uart_Engine;
extern volatile uint32_t Usart_BaudRate;

/* Exported macro ------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void USARTConfig(void);
int UART_ReceivedMSG(TMsg *Msg);
void UART_SendMsg(TMsg *Msg);
void USART_DMA_Configuration(void);

#endif /* __COM__H */
