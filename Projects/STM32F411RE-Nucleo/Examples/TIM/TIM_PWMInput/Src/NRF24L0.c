//#include "common.h"
#include    <stdio.h>                       //printf
#include    <string.h>                      //memcpy
#include    <stdlib.h>                      //malloc
#include "NRF24L0.h"
#include "stm32f4xx_hal.h"
#include "x_nucleo_iks01a1.h"
//#include "chip.h"
//#include "SPI.h"
//#include "iNEMO_AHRS.h"
/*******************************************************/
/* ��   �ƣ�NRF24L01+����USBͨ��ģ�����               */
/* ��   �ܣ��������ݷ��ͽ��ճ���                       */
/*          ��ʽ����λ�Ǹ����������Ҫ���͵�����       */
/*                ���磺����5���ֽ� 11 22 33 44 55     */
/*                ���Դ��ڷ��ͣ�1122334455            */
/* ���ڲ����ʣ�9600  �ӿ�P1.1��P1.2                     */
/* ��Ӧ�ӿ�  ��        MSP430      NRF24L01             */
/*           P1.0		  CSN                */
/*            P1.3        CE             	 */
/*            P1.4        IRQ                */
/*            P1.5        SCK                */
/*            P1.6        MISO               */
/*            P1.7        MOSI               */
/*******************************************************/
//#define LPC_SPI         			LPC_SPI0
//#define HW_SPI
#define uchar unsigned char
#define uint  unsigned int

/**********  NRF24L01�Ĵ�����������  ***********/
#define NRF_READ_REG        0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define NRF_WRITE_REG       0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���
/**********  NRF24L01�Ĵ�����ַ   *************/
#define CONFIG          0x00  //���üĴ�����ַ
#define EN_AA           0x01  //ʹ���Զ�Ӧ����
#define EN_RXADDR       0x02  //���յ�ַ����
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��)
#define SETUP_RETR      0x04  //�����Զ��ط�
#define RF_CH           0x05  //RFͨ��
#define RF_SETUP        0x06  //RF�Ĵ���
#define STATUS          0x07  //״̬�Ĵ���
#define OBSERVE_TX      0x08  // ���ͼ��Ĵ���
#define CD              0x09  // �ز����Ĵ���
#define RX_ADDR_P0      0x0A  // ����ͨ��0���յ�ַ
#define RX_ADDR_P1      0x0B  // ����ͨ��1���յ�ַ
#define RX_ADDR_P2      0x0C  // ����ͨ��2���յ�ַ
#define RX_ADDR_P3      0x0D  // ����ͨ��3���յ�ַ
#define RX_ADDR_P4      0x0E  // ����ͨ��4���յ�ַ
#define RX_ADDR_P5      0x0F  // ����ͨ��5���յ�ַ
#define TX_ADDR         0x10  // ���͵�ַ�Ĵ���
#define RX_PW_P0        0x11  // ��������ͨ��0��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P1        0x12  // ��������ͨ��1��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P2        0x13  // ��������ͨ��2��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P3        0x14  // ��������ͨ��3��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P4        0x15  // ��������ͨ��4��Ч���ݿ��(1~32�ֽ�)
#define RX_PW_P5        0x16  // ��������ͨ��5��Ч���ݿ��(1~32�ֽ�)
#define FIFO_STATUS     0x17  // FIFO״̬�Ĵ���
/*����������������������������������������������������������������������������������������������������������������������������������������*/

/******   STATUS�Ĵ���bitλ����      *******/
#define MAX_TX  	0x10  //�ﵽ����ʹ����ж�
#define TX_OK   	0x20  //TX��������ж�
#define RX_OK   	0x40  //���յ������ж�
/*����������������������������������������������������������������������������������������������������*/

/*********     24L01���ͽ������ݿ�ȶ���	  ***********/
#define TX_ADR_WIDTH    5   //5�ֽڵ�ַ���
#define RX_ADR_WIDTH    5   //5�ֽڵ�ַ���
#define TX_PLOAD_WIDTH  32  //32�ֽ���Ч���ݿ��
#define RX_PLOAD_WIDTH  32  //32�ֽ���Ч���ݿ��

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //���͵�ַ


uchar NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01���յ�������
uchar NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����
uchar NRF24L01_TXDATA_RC[RX_PLOAD_WIDTH];//nrf24l01��Ҫ���͵�����

#define CSN_H       GPIOC->BSRR =GPIO_PIN_8;
#define CSN_L       GPIOC->BSRR =GPIO_PIN_8<<16;
#define CE_H         GPIOC->BSRR =GPIO_PIN_9;
#define CE_L        GPIOC->BSRR =GPIO_PIN_9<<16;
#define SCK_H        GPIOC->BSRR =GPIO_PIN_10;
#define SCK_L        GPIOC->BSRR =GPIO_PIN_10<<16;
#define MOSI_H       GPIOC->BSRR =GPIO_PIN_11;
#define MOSI_L        GPIOC->BSRR =GPIO_PIN_11<<16;
#define MISO_IN  	GPIOC->IDR & GPIO_PIN_12         // MISO�����Ƿ�Ϊ�ߵ�ƽ����ó�����1bit������
//#define IRQ_IN   	Chip_GPIO_GetPinState(LPC_GPIO, 0, 11) 



void  SPI_INIT(void)
{
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_INACT));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	
//			 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 14);//��PIO0_31����Ϊ���
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 14, 1);
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 11);//��PIO0_31����Ϊ���
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 11, 0);
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 12);//��PIO0_31����Ϊ���
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 12, 0);
//		 Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 13);//��PIO0_31����Ϊ���
//		 
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 10);//��PIO0_31����Ϊ���
//	Chip_GPIO_SetPinState(LPC_GPIO, 0, 10, 0);
    
#if IS_USE_ISR
    exti_init(PORTE,6, falling_up);        //��ʼ��IRQ�ܽ�Ϊ :�½��ش������ڲ�����
#else
 //   gpio_init(PORTE,6, GPI,LOW);           //��ʼ��IRQ�ܽ�Ϊ����     
#endif
    
#ifdef HW_SPI
  // spi_init(SPI1,MASTER);
#else
  //   gpio_init(PORTE,2, GPO,LOW);           //��ʼ��SCK
  //		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 13);
	 //Chip_GPIO_SetPinState(LPC_GPIO, 0, 13, 0);
  //   gpio_init(PORTE,1, GPO,LOW);          //��ʼ��MOSI�ܽ�Ϊ���
   //      		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 12);
	// Chip_GPIO_SetPinState(LPC_GPIO, 0, 12, 0);
   //   gpio_init(PORTE,3, GPI,LOW);           //��ʼ��MISO�ܽ�Ϊ����  
	//	 Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 11);
	 //Chip_GPIO_SetPinState(LPC_GPIO, 0, 11, 0);
#endif
}

extern SPI_HandleTypeDef SpiHandle3;
static uchar SPI_RW(uchar Byte)
{
#ifdef HW_SPI
  uchar Read[1];
  uchar Write[1];
  Write[0]=Byte;
//	 spi_WR(SPI1,&Byte,1);
	HAL_SPI_TransmitReceive(&SpiHandle3,Write,Read,1,500);
        return Read[0];

   // return  Byte;
#else
	uchar bit_ctr;
	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // ���8λ
	{
		if(Byte&0x80)
		{
			MOSI_H;
		}
			else
				MOSI_L;
		Byte=(Byte<<1);	// shift next bit to MSB
		SCK_H;
			if(MISO_IN)
			{
				Byte|=1;
			}
			else
			Byte&=~1;
		SCK_L;
	}
	return Byte;
#endif
}

/*********************************************/
/* �������ܣ���24L01�ļĴ���дֵ��һ���ֽڣ� */
/* ��ڲ�����reg   Ҫд�ļĴ�����ַ          */
/*           value ���Ĵ���д��ֵ            */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
static uchar NRF24L01_Write_Reg(uchar reg,uchar value)
{
	uchar status;
	CSN_L;                  //CSN=0;
  	status = SPI_RW(reg);//???????,??????
	SPI_RW(value);
	CSN_H;                  //CSN=1;
	return status;
}
/*************************************************/
/* �������ܣ���24L01�ļĴ���ֵ ��һ���ֽڣ�      */
/* ��ڲ�����reg  Ҫ���ļĴ�����ַ               */
/* ���ڲ�����value �����Ĵ�����ֵ                */
/*************************************************/

static uchar NRF24L01_Read_Reg(uchar reg)
{
 uchar value;
	CSN_L;                 //CSN=0;
  	SPI_RW(reg);//??????(??),??????
	value = SPI_RW(0XFF);
	CSN_H;               //CSN=1;
	return value;
}
/*********************************************/
/* �������ܣ���24L01�ļĴ���ֵ������ֽڣ�   */
/* ��ڲ�����reg   �Ĵ�����ַ                */
/*           *pBuf �����Ĵ���ֵ�Ĵ������    */
/*           len   �����ֽڳ���              */
/* ���ڲ�����status ״ֵ̬                   */
/*********************************************/
static uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	CSN_L;                   //CSN=0
  	status=SPI_RW(reg);//���ͼĴ�����ַ,����ȡ״ֵ̬
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI_RW(0XFF);//��������
	CSN_H;                  //CSN=1
  	return status;        //���ض�����״ֵ̬
}
/**********************************************/
/* �������ܣ���24L01�ļĴ���дֵ������ֽڣ�  */
/* ��ڲ�����reg  Ҫд�ļĴ�����ַ            */
/*           *pBuf ֵ�Ĵ������               */
/*           len   �����ֽڳ���               */
/**********************************************/
static uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	CSN_L;
  	status = SPI_RW(reg);//���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	SPI_RW(*pBuf++); //д������
	CSN_H;
  	return status;          //���ض�����״ֵ̬
}

/*********************************************/
/* �������ܣ�24L01��������                   */
/* ��ڲ�����rxbuf ������������              */
/* ����ֵ�� 0   �ɹ��յ�����                 */
/*          1   û���յ�����                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)//���յ�����
	{
		CE_L;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//���RX FIFO�Ĵ���
		CE_H;
		//delay_150us();
		return 0;
	} 
	return 1;//û�յ��κ�����
}
/**********************************************/
/* �������ܣ�����24L01Ϊ����ģʽ              */
/* ��ڲ�����txbuf  ������������              */
/* ����ֵ�� 0x10    �ﵽ����ط�����������ʧ��*/
/*          0x20    �ɹ��������              */
/*          0xff    ����ʧ��                  */
/**********************************************/
static uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;

	CE_L;//CE���ͣ�ʹ��24L01����
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
 CE_H;//CE�øߣ�ʹ�ܷ���
 //state=NRF24L01_Read_Reg(STATUS);
	//while(IRQ_IN);//�ȴ��������
	state=NRF24L01_Read_Reg(STATUS);  //��ȡ״̬�Ĵ�����ֵ
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
//	if(state&MAX_TX)//�ﵽ����ط�����
//	{
//		NRF24L01_Write_Reg(FLUSH_TX,0xff);//���TX FIFO�Ĵ���
//		return MAX_TX;
//	}
//	if(state&TX_OK)//�������
//	{
//		return TX_OK; 
//	}
//	return 0xff;//����ʧ��
}

void  NRF24L01_Tx(uchar *txbuf){
  //uchar state;
                CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);
      //          state=NRF24L01_Read_Reg(STATUS); 
		//CE_H;
		//delay_us(15);
		NRF24L01_TxPacket(txbuf);
		//CE_L;
		NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG, 0x0f);
		CE_H;
}
/********************************************/
/* �������ܣ����24L01�Ƿ����              */
/* ����ֵ��  0  ����                        */
/*           1  ������                      */
/********************************************/
int NRF24L01_Check(void)
{
	uchar check_in_buf[5]={0x11,0x22,0x33,0x44,0x55};
	uchar check_out_buf[5]={0x00};

	SCK_L;
	CSN_H;
	CE_L;
//NRF24L01_Write_Reg(WRITE_REG+TX_ADDR, 0x11);
//check_out_buf[0]=	NRF24L01_Read_Reg(READ_REG+TX_ADDR);
	//if(check_out_buf[0]==0x11)return 0;
	
		
	NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR, check_in_buf, 5);

	NRF24L01_Read_Buf(NRF_READ_REG+TX_ADDR, check_out_buf, 5);

	if((check_out_buf[0] == 0x11)&&\
	   (check_out_buf[1] == 0x22)&&\
	   (check_out_buf[2] == 0x33)&&\
	   (check_out_buf[3] == 0x44)&&\
	   (check_out_buf[4] == 0x55))return 0;
	
	else return 1;
}
static void NRF_TxPacket(uint8_t * tx_buf, uint8_t len)
{	
	CE_L;		 //StandBy Iģʽ	
	
	//NRF24L01_Write_Buf(WRITE_REG + RX_ADDR_P0, (uchar*)TX_ADDRESS, TX_ADR_WIDTH); // װ�ؽ��ն˵�ַ
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // װ������	
	CE_H;		 //�ø�CE���������ݷ���
}

void NRF24L01_RT_Init(uchar model)
{
	CE_L;
        NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//дTX�ڵ��ַ
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��
 	if(model==1)
        {
        NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x2f);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
        }
        else if(model==2)		//TX
	{
         NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ��
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    // IRQ�շ�����жϿ���,16λCRC,������
        }
        else if(model==3)		//ACK_RX
       {
        NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x2f);  // ����ģʽ����Ҫ�������ж�
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
       }
       	else								//ACK_TX
	{
                NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,
	        NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	        NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
        }
	CE_H;		//CE�øߣ�ʹ�ܷ���

}


static void NRF_TxPacket_AP(uchar * tx_buf, uchar len)
{
	CE_L;		 //StandBy Iģʽ
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 // װ������
//	NRF24L01_Write_Buf(0xa9, tx_buf, len); 			 // װ������
//	NRF24L01_Write_Buf(0xaa, tx_buf, len); 			 // װ������
	CE_H;		 //�ø�CE
}

extern int32_t Speed_L;
extern int32_t Speed_R;
extern volatile AxesRaw_TypeDef ACC_Value_Raw;  
extern volatile AxesRaw_TypeDef GYR_Value_Raw;
extern float g_fSpeedControlIntegral;
extern float car_angle_set;
extern float g_fAngleControlP;
extern float g_fAngleControlI;
extern float g_fAngleControlD;
extern float g_fSpeedControlP;
extern float g_fSpeedControlI;
extern float g_fCarAngle;
extern float g_fAngleSpeed;
extern int32_t Speed_A;
extern float Speed_Need;
extern float Turn_Need;
extern float D_Err;
extern float D_Integral;
extern float Ult_Distance;
extern uint32_t Pre_Speed_R;
extern float g_fAngleControlOut;
extern float g_fSpeedControlOut;
extern float quartf[4]; /* ��Ԫ��*/
extern float gyrof[3];  /* �����Ƕ���*/
extern float accelf[3]; /* ���ٶ�ֵ*/
extern float yprf[3];   /* Euler �Ƕ�*/
extern volatile AxesRaw_TypeDef MAG_Value_Raw;
extern  float pitch,roll,yaw;
extern float twoKp;
extern float GYROSCOPE_ANGLE_RATIO;
extern uint32_t Capture_Value;
extern uint32_t Car_Cmd;
extern uint32_t  Inf_Distance;
extern int Block_Flag;
extern int Block_Cnt;
static  void lb_put_parameter(void) 
{
unsigned char check;	
char tx[27];
int temp[4];
float temp2[4];
char *p;
int i=0;
temp[0]=g_fAngleControlOut;//��PWM���ֵ�ŵ�int�������У�������ʾ��������1����ʾ
//temp[1]=g_fAngleSpeed*1000;//��С���ٶȷŵ�һ��int�������У�������ʾ��������2����ʾ
//temp[2]=GYR_Value_Raw.AXIS_Y;
//temp[3]=g_fCarAngle*1000;//��С��ƫ�Ƿŵ�һ��int�������У�������ʾ��������4����ʾ
temp[1]=Speed_A;
temp[2]=pitch*100;
temp[3]= Inf_Distance;
//temp[1]=Ult_Distance;
//temp[2]=D_Err;
//temp[3]=D_Integral;
	//temp[0]=accelf[0]/1024;
	//temp[1]=accelf[1]/1024;
	//temp[2]=accelf[2]/1024;
//temp[0]=roll;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
//temp[0]=Ult_Distance;
//temp[1]=pitch;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
//temp[2]=yaw;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
//temp[3]=g_fAngleControlOut;
 //     temp[1]=MAG_Value_Raw.AXIS_Y;
 //    temp[2]=MAG_Value_Raw.AXIS_Z;
//temp[3]=MAG_Value_Raw.AXIS_X;    
	//temp[0]=g_fAngleControlOut;
//temp[0]=pitch*100;
       // temp[1]=Speed_A;
   //     temp[3]=Speed_Need;
    //     temp[2]=Block_Flag;
      //     temp[3]=Block_Cnt;
      //  temp[0]=yaw;
	//temp[1]=g_fAngleSpeed*1000;
      //  temp[1]=g_fSpeedControlOutNew; 
	//temp[2]=-pitch*100;
       //   temp[1]=g_fSpeedControlIntegral;
   //    temp[1]= Ult_Distance;
     //  temp[1]= Inf_Distance;
      // temp[2]=g_fSpeedControlOut;
        
     //   temp[1]=Speed_R;
     //   temp[2]=Speed_L;
//	temp[3]=Speed_A;
temp2[0]=roll;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
temp2[1]=-pitch;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
temp2[2]=yaw;//��ŷ���Ƿŵ�һ��float�������У�������OpenGLͼ������ʾ
temp2[3]=0;
for(i=0;i<4;i++){//�����������ݱ���ַ���
	if(temp[i]<0)
	temp[i]=65536+temp[i];
    tx[2*i+2]=(temp[i]%256); 
	tx[2*i+3]=(temp[i]/256);
}
for(i=0;i<4;i++){//�����������ݱ���ַ���
p=(char *)&temp2[i];
tx[10+i*4]=*p;
tx[11+i*4]=*(p+1);
tx[12+i*4]=*(p+2); 
tx[13+i*4]=*(p+3);
}
tx[0]='s';//����ͷ
tx[1]='t';//����ͷ
check = 0;
for(i = 2; i < 26; i++) check += tx[i];//�������У��
tx[26] = check+NRF24L01_Read_Reg(0x09); //��NRF4l01���ź�ǿ�ȷŵ����У����
if(check==255)tx[26]=255;//��ֹ����
NRF_TxPacket_AP(tx,27);//��Ҫ���͵�27Ϊchar�����ݷŵ���Ҫack���͵ļĴ�����
}
static void lb_put_original(void){
 unsigned char check;	
char tx[27];
int temp[4];
float temp2[4];
char *p;
int i=0;
temp[0]=car_angle_set*10;
temp[1]=g_fAngleControlP;
temp[2]=g_fAngleControlD;
temp[3]=g_fAngleControlI;
temp2[0]=g_fSpeedControlP;
temp2[1]=g_fSpeedControlI;
temp2[2]=Speed_Need;
temp2[3]= Turn_Need;

for(i=0;i<4;i++){
	if(temp[i]<0)
	temp[i]=65536+temp[i];
    tx[2*i+2]=(temp[i]%256); 
	tx[2*i+3]=(temp[i]/256);
}
for(i=0;i<4;i++){
p=(char *)&temp2[i];
tx[10+i*4]=*p;
tx[11+i*4]=*(p+1);
tx[12+i*4]=*(p+2); 
tx[13+i*4]=*(p+3);
}
tx[0]='s';
tx[1]='u';
check = 0;
for(i = 2; i < 26; i++) check += tx[i];
tx[26] = check+NRF24L01_Read_Reg(0x09); 
if(check==255)tx[26]=255;
NRF_TxPacket_AP(tx,27);
  
}
//extern int cnt;

static char lb_get_parameter(void){
 	int temp[4];
	float temp2[4];
	int i=0;
	float *f;
	unsigned char check=0;
     int rx[4];
    float rx2[4];
  for(i = 2; i < 26; i++) check += NRF24L01_RXDATA[i];

//  if(NRF24L01_RXDATA[26]!=check)return 0;
    
  for(i=0;i<4;i++){
	temp[i]=(unsigned char )NRF24L01_RXDATA[2*i+2]+(unsigned char )NRF24L01_RXDATA[2*i+3]*256;
	if(temp[i]>32768){
		temp[i]=temp[i]-65536;
	}
	rx[i]=temp[i];
	}
  for(i=0;i<4;i++){
		f=(float *)&NRF24L01_RXDATA[10+4*i];
		rx2[i]=*f; 
		
		}
  	car_angle_set=0.1*rx[0];
	  g_fAngleControlP=rx[1];
		g_fAngleControlD=rx[2];
	g_fAngleControlI=rx[3];
	g_fSpeedControlP=rx2[0];
	g_fSpeedControlI=rx2[1];
	Speed_Need=rx2[2];
         Turn_Need=rx2[3];
	
        return 1;
}
void Nrf_Check_Event(void)
{
	uchar sta = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);
	if(sta & (RX_OK))//�����ж�
	{
		uchar rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);
		NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);
//NRF_TxPacket_AP(NRF24L01_RXDATA,rx_len);
		 if(NRF24L01_RXDATA[0]=='b')//������յ��ĵ�һ���ַ��ǡ�b��
                lb_put_parameter();//��Ҫ���͵���ز����ŵ���Ҫ���͵ļĴ�����
     else if(NRF24L01_RXDATA[0]=='l')//������յ��ĵ�һ���ַ��ǡ�l��
                lb_put_original();//��������ֵ�ŵ���Ҫ���͵ļĴ�����
		 else if(NRF24L01_RXDATA[0]=='s')//������յ��ĵ�һ���ַ��ǡ�s��
               lb_get_parameter();//�����յ���ֵ�޸�Ŀǰ�Ĳ���
	}
	if(sta & (TX_OK))
	{
	}
	if(sta & (MAX_TX))
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
	}
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta);//
	sta = NRF24L01_Read_Reg(NRF_READ_REG + STATUS);
}
/*
void Nrf_Check_Event(void)
{
	uchar sta = NRF24L01_Read_Reg(READ_REG + STATUS);
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (RX_OK))//�����ж�
	{
		uchar rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);
		NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);
                 uart_putchar(UART0, NRF24L01_RXDATA[0]);
         //   printf("%d", UART0,(u8)NRF24L01_RXDATA[0]);
      //      u8 temp;
       //         temp=NRF24L01_RXDATA[0];
       //         uart_putchar(UART0,temp);

	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (TX_OK))
	{
;
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	if(sta & (MAX_TX))
	{
//		static u16 max_cnt = 0;
//		max_cnt++;
//		PC_Debug_Show(7,max_cnt);

		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	NRF24L01_Write_Reg(WRITE_REG + STATUS, sta);//??nrf??????
	sta = NRF24L01_Read_Reg(READ_REG + STATUS);

}

*/
	
	


	