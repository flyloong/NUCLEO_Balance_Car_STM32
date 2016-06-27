/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���       ��NRF24L0.h
 * ����         ��NRF24L0������
 *
 * ʵ��ƽ̨     ��Ұ��kinetis������
 * ��汾       ��
 * Ƕ��ϵͳ     ��
 *
 * ����         ��Ұ��Ƕ��ʽ����������
 * �Ա���       ��http://firestm32.taobao.com
 * ����֧����̳ ��http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/
#ifndef _NRF24L0_H_
#define _NRF24L0_H_     
#define RX				1			//��ͨ����
#define TX				2			//��ͨ����
#define ACK_RX			3			//����ģʽ2,�����ݵ�ACKģʽ
#define ACK_TX			4			//����ģʽ2,�����ݵ�ACKģʽ

//#include "control.h"

void NRF24L01_RT_Init(unsigned char model);
int NRF24L01_Check(void);
void  SPI_INIT(void);
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf);
void  NRF24L01_Tx(unsigned char *txbuf);
void NRF_TxPacket(unsigned char * tx_buf, unsigned char len);
void Nrf_Check_Event(void);

#endif


