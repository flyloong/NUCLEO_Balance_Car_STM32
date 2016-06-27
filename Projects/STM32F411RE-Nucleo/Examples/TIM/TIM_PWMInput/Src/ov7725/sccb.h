#ifndef __SCCB_H
#define __SCCB_H

#include "stm32f411xe.h"
#define SCL_H         HAL_GPIO_WritePin(GPIOC , GPIO_PIN_14,1) 
#define SCL_L         HAL_GPIO_WritePin(GPIOC , GPIO_PIN_14,0) 
   
#define SDA_H        HAL_GPIO_WritePin(GPIOC , GPIO_PIN_15,1) 
#define SDA_L        HAL_GPIO_WritePin(GPIOC , GPIO_PIN_15,1) 

#define SCL_read      HAL_GPIO_ReadPin(GPIOC , GPIO_PIN_14) 
#define SDA_read     HAL_GPIO_ReadPin(GPIOC , GPIO_PIN_15) 

#define ADDR_OV7725   0x42
typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
void SCCB_GPIO_Config(void);
int SCCB_WriteByte( u16 WriteAddress , u8 SendByte);
int SCCB_ReadByte(u8* pBuffer,   u16 length,   u8 ReadAddress);

#endif 
