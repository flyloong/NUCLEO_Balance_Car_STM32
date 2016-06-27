/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：NRF24L0.h
 * 描述         ：NRF24L0函数库
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/
#ifndef _NRF24L0_H_
#define _NRF24L0_H_     
#define RX				1			//普通接收
#define TX				2			//普通发送
#define ACK_RX			3			//接收模式2,带数据的ACK模式
#define ACK_TX			4			//发送模式2,带数据的ACK模式

//#include "control.h"

void NRF24L01_RT_Init(unsigned char model);
int NRF24L01_Check(void);
void  SPI_INIT(void);
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf);
void  NRF24L01_Tx(unsigned char *txbuf);
void NRF_TxPacket(unsigned char * tx_buf, unsigned char len);
void Nrf_Check_Event(void);

#endif


