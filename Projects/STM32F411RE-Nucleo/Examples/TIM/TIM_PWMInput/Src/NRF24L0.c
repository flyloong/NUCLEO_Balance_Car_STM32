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
/* 名   称：NRF24L01+无线USB通信模块程序               */
/* 功   能：智能数据发送接收程序                       */
/*          格式：首位是个数，后面跟要发送的数据       */
/*                例如：发送5个字节 11 22 33 44 55     */
/*                电脑串口发送：1122334455            */
/* 串口波特率：9600  接口P1.1与P1.2                     */
/* 对应接口  ：        MSP430      NRF24L01             */
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

/**********  NRF24L01寄存器操作命令  ***********/
#define NRF_READ_REG        0x00  //读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG       0x20  //写配置寄存器,低5位为寄存器地址
#define R_RX_PL_WID   	0x60
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器
/**********  NRF24L01寄存器地址   *************/
#define CONFIG          0x00  //配置寄存器地址
#define EN_AA           0x01  //使能自动应答功能
#define EN_RXADDR       0x02  //接收地址允许
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道)
#define SETUP_RETR      0x04  //建立自动重发
#define RF_CH           0x05  //RF通道
#define RF_SETUP        0x06  //RF寄存器
#define STATUS          0x07  //状态寄存器
#define OBSERVE_TX      0x08  // 发送检测寄存器
#define CD              0x09  // 载波检测寄存器
#define RX_ADDR_P0      0x0A  // 数据通道0接收地址
#define RX_ADDR_P1      0x0B  // 数据通道1接收地址
#define RX_ADDR_P2      0x0C  // 数据通道2接收地址
#define RX_ADDR_P3      0x0D  // 数据通道3接收地址
#define RX_ADDR_P4      0x0E  // 数据通道4接收地址
#define RX_ADDR_P5      0x0F  // 数据通道5接收地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收数据通道0有效数据宽度(1~32字节)
#define RX_PW_P1        0x12  // 接收数据通道1有效数据宽度(1~32字节)
#define RX_PW_P2        0x13  // 接收数据通道2有效数据宽度(1~32字节)
#define RX_PW_P3        0x14  // 接收数据通道3有效数据宽度(1~32字节)
#define RX_PW_P4        0x15  // 接收数据通道4有效数据宽度(1~32字节)
#define RX_PW_P5        0x16  // 接收数据通道5有效数据宽度(1~32字节)
#define FIFO_STATUS     0x17  // FIFO状态寄存器
/*————————————————————————————————————————————————————————————————————*/

/******   STATUS寄存器bit位定义      *******/
#define MAX_TX  	0x10  //达到最大发送次数中断
#define TX_OK   	0x20  //TX发送完成中断
#define RX_OK   	0x40  //接收到数据中断
/*——————————————————————————————————————————————————*/

/*********     24L01发送接收数据宽度定义	  ***********/
#define TX_ADR_WIDTH    5   //5字节地址宽度
#define RX_ADR_WIDTH    5   //5字节地址宽度
#define TX_PLOAD_WIDTH  32  //32字节有效数据宽度
#define RX_PLOAD_WIDTH  32  //32字节有效数据宽度

const uchar TX_ADDRESS[TX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址
const uchar RX_ADDRESS[RX_ADR_WIDTH]={0x34,0x43,0x10,0x10,0x01}; //发送地址


uchar NRF24L01_RXDATA[RX_PLOAD_WIDTH];//nrf24l01接收到的数据
uchar NRF24L01_TXDATA[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据
uchar NRF24L01_TXDATA_RC[RX_PLOAD_WIDTH];//nrf24l01需要发送的数据

#define CSN_H       GPIOC->BSRR =GPIO_PIN_8;
#define CSN_L       GPIOC->BSRR =GPIO_PIN_8<<16;
#define CE_H         GPIOC->BSRR =GPIO_PIN_9;
#define CE_L        GPIOC->BSRR =GPIO_PIN_9<<16;
#define SCK_H        GPIOC->BSRR =GPIO_PIN_10;
#define SCK_L        GPIOC->BSRR =GPIO_PIN_10<<16;
#define MOSI_H       GPIOC->BSRR =GPIO_PIN_11;
#define MOSI_L        GPIOC->BSRR =GPIO_PIN_11<<16;
#define MISO_IN  	GPIOC->IDR & GPIO_PIN_12         // MISO输入是否为高电平这里得出来的1bit的数据
//#define IRQ_IN   	Chip_GPIO_GetPinState(LPC_GPIO, 0, 11) 



void  SPI_INIT(void)
{
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 13,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_INACT));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 14, (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 12, (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 11,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 10,  (IOCON_FUNC0 | IOCON_DIGITAL_EN | IOCON_MODE_PULLDOWN));
//	
//			 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 14);//把PIO0_31设置为输出
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 14, 1);
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 11);//把PIO0_31设置为输出
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 11, 0);
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 12);//把PIO0_31设置为输出
//Chip_GPIO_SetPinState(LPC_GPIO, 0, 12, 0);
//		 Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, 13);//把PIO0_31设置为输出
//		 
//		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 10);//把PIO0_31设置为输出
//	Chip_GPIO_SetPinState(LPC_GPIO, 0, 10, 0);
    
#if IS_USE_ISR
    exti_init(PORTE,6, falling_up);        //初始化IRQ管脚为 :下降沿触发，内部上拉
#else
 //   gpio_init(PORTE,6, GPI,LOW);           //初始化IRQ管脚为输入     
#endif
    
#ifdef HW_SPI
  // spi_init(SPI1,MASTER);
#else
  //   gpio_init(PORTE,2, GPO,LOW);           //初始化SCK
  //		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 13);
	 //Chip_GPIO_SetPinState(LPC_GPIO, 0, 13, 0);
  //   gpio_init(PORTE,1, GPO,LOW);          //初始化MOSI管脚为输出
   //      		 Chip_GPIO_SetPinDIROutput(LPC_GPIO, 0, 12);
	// Chip_GPIO_SetPinState(LPC_GPIO, 0, 12, 0);
   //   gpio_init(PORTE,3, GPI,LOW);           //初始化MISO管脚为输入  
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
	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // 输出8位
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
/* 函数功能：给24L01的寄存器写值（一个字节） */
/* 入口参数：reg   要写的寄存器地址          */
/*           value 给寄存器写的值            */
/* 出口参数：status 状态值                   */
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
/* 函数功能：读24L01的寄存器值 （一个字节）      */
/* 入口参数：reg  要读的寄存器地址               */
/* 出口参数：value 读出寄存器的值                */
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
/* 函数功能：读24L01的寄存器值（多个字节）   */
/* 入口参数：reg   寄存器地址                */
/*           *pBuf 读出寄存器值的存放数组    */
/*           len   数组字节长度              */
/* 出口参数：status 状态值                   */
/*********************************************/
static uchar NRF24L01_Read_Buf(uchar reg,uchar *pBuf,uchar len)
{
	uchar status,u8_ctr;
	CSN_L;                   //CSN=0
  	status=SPI_RW(reg);//发送寄存器地址,并读取状态值
 	for(u8_ctr=0;u8_ctr<len;u8_ctr++)
	pBuf[u8_ctr]=SPI_RW(0XFF);//读出数据
	CSN_H;                  //CSN=1
  	return status;        //返回读到的状态值
}
/**********************************************/
/* 函数功能：给24L01的寄存器写值（多个字节）  */
/* 入口参数：reg  要写的寄存器地址            */
/*           *pBuf 值的存放数组               */
/*           len   数组字节长度               */
/**********************************************/
static uchar NRF24L01_Write_Buf(uchar reg, uchar *pBuf, uchar len)
{
	uchar status,u8_ctr;
	CSN_L;
  	status = SPI_RW(reg);//发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<len; u8_ctr++)
	SPI_RW(*pBuf++); //写入数据
	CSN_H;
  	return status;          //返回读到的状态值
}

/*********************************************/
/* 函数功能：24L01接收数据                   */
/* 入口参数：rxbuf 接收数据数组              */
/* 返回值： 0   成功收到数据                 */
/*          1   没有收到数据                 */
/*********************************************/
uchar NRF24L01_RxPacket(uchar *rxbuf)
{
	uchar state;
	state=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&RX_OK)//接收到数据
	{
		CE_L;
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);//清除RX FIFO寄存器
		CE_H;
		//delay_150us();
		return 0;
	} 
	return 1;//没收到任何数据
}
/**********************************************/
/* 函数功能：设置24L01为发送模式              */
/* 入口参数：txbuf  发送数据数组              */
/* 返回值； 0x10    达到最大重发次数，发送失败*/
/*          0x20    成功发送完成              */
/*          0xff    发送失败                  */
/**********************************************/
static uchar NRF24L01_TxPacket(uchar *txbuf)
{
	uchar state;

	CE_L;//CE拉低，使能24L01配置
  	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
 CE_H;//CE置高，使能发送
 //state=NRF24L01_Read_Reg(STATUS);
	//while(IRQ_IN);//等待发送完成
	state=NRF24L01_Read_Reg(STATUS);  //读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
//	if(state&MAX_TX)//达到最大重发次数
//	{
//		NRF24L01_Write_Reg(FLUSH_TX,0xff);//清除TX FIFO寄存器
//		return MAX_TX;
//	}
//	if(state&TX_OK)//发送完成
//	{
//		return TX_OK; 
//	}
//	return 0xff;//发送失败
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
/* 函数功能：检测24L01是否存在              */
/* 返回值；  0  存在                        */
/*           1  不存在                      */
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
	CE_L;		 //StandBy I模式	
	
	//NRF24L01_Write_Buf(WRITE_REG + RX_ADDR_P0, (uchar*)TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			 // 装载数据	
	CE_H;		 //置高CE，激发数据发送
}

void NRF24L01_RT_Init(uchar model)
{
	CE_L;
        NRF24L01_Write_Buf(NRF_WRITE_REG+TX_ADDR,(uchar*)TX_ADDRESS,TX_ADR_WIDTH);//写TX节点地址
  	NRF24L01_Write_Buf(NRF_WRITE_REG+RX_ADDR_P0,(uchar*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答
  	NRF24L01_Write_Reg(NRF_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址
  	NRF24L01_Write_Reg(NRF_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_CH,40);       //设置RF通道为40
  	NRF24L01_Write_Reg(NRF_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启
 	if(model==1)
        {
        NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x2f);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,不开发送中断
        }
        else if(model==2)		//TX
	{
         NRF24L01_Write_Reg(NRF_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度
  	NRF24L01_Write_Reg(NRF_WRITE_REG+CONFIG,0x0e);    // IRQ收发完成中断开启,16位CRC,主发送
        }
        else if(model==3)		//ACK_RX
       {
        NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x2f);  // 发送模式，不要发送引中断
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
       }
       	else								//ACK_TX
	{
                NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,
	        NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(NRF_WRITE_REG+0x1c,0x01);
	        NRF24L01_Write_Reg(NRF_WRITE_REG+0x1d,0x06);
        }
	CE_H;		//CE置高，使能发送

}


static void NRF_TxPacket_AP(uchar * tx_buf, uchar len)
{
	CE_L;		 //StandBy I模式
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
//	NRF24L01_Write_Buf(0xa9, tx_buf, len); 			 // 装载数据
//	NRF24L01_Write_Buf(0xaa, tx_buf, len); 			 // 装载数据
	CE_H;		 //置高CE
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
extern float quartf[4]; /* 四元数*/
extern float gyrof[3];  /* 陀螺仪读数*/
extern float accelf[3]; /* 加速度值*/
extern float yprf[3];   /* Euler 角度*/
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
temp[0]=g_fAngleControlOut;//将PWM输出值放到int型数组中，将会在示波器曲线1中显示
//temp[1]=g_fAngleSpeed*1000;//将小车速度放到一个int型数组中，将会在示波器曲线2中显示
//temp[2]=GYR_Value_Raw.AXIS_Y;
//temp[3]=g_fCarAngle*1000;//将小车偏角放到一个int型数组中，将会在示波器曲线4中显示
temp[1]=Speed_A;
temp[2]=pitch*100;
temp[3]= Inf_Distance;
//temp[1]=Ult_Distance;
//temp[2]=D_Err;
//temp[3]=D_Integral;
	//temp[0]=accelf[0]/1024;
	//temp[1]=accelf[1]/1024;
	//temp[2]=accelf[2]/1024;
//temp[0]=roll;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
//temp[0]=Ult_Distance;
//temp[1]=pitch;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
//temp[2]=yaw;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
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
temp2[0]=roll;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
temp2[1]=-pitch;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
temp2[2]=yaw;//将欧拉角放到一个float型数组中，将会在OpenGL图像中显示
temp2[3]=0;
for(i=0;i<4;i++){//将整数型数据变成字符型
	if(temp[i]<0)
	temp[i]=65536+temp[i];
    tx[2*i+2]=(temp[i]%256); 
	tx[2*i+3]=(temp[i]/256);
}
for(i=0;i<4;i++){//将浮点型数据变成字符型
p=(char *)&temp2[i];
tx[10+i*4]=*p;
tx[11+i*4]=*(p+1);
tx[12+i*4]=*(p+2); 
tx[13+i*4]=*(p+3);
}
tx[0]='s';//数据头
tx[1]='t';//数据头
check = 0;
for(i = 2; i < 26; i++) check += tx[i];//采用求和校验
tx[26] = check+NRF24L01_Read_Reg(0x09); //将NRF4l01的信号强度放到求和校验中
if(check==255)tx[26]=255;//防止错误
NRF_TxPacket_AP(tx,27);//将要发送的27为char型数据放到将要ack发送的寄存器中
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
	if(sta & (RX_OK))//接收中断
	{
		uchar rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);
		NRF24L01_Read_Buf(RD_RX_PLOAD,NRF24L01_RXDATA,rx_len);
//NRF_TxPacket_AP(NRF24L01_RXDATA,rx_len);
		 if(NRF24L01_RXDATA[0]=='b')//如果接收到的第一个字符是‘b’
                lb_put_parameter();//将要发送的相关参数放到将要发送的寄存器中
     else if(NRF24L01_RXDATA[0]=='l')//如果接收到的第一个字符是‘l’
                lb_put_original();//将参数的值放到将要发送的寄存器中
		 else if(NRF24L01_RXDATA[0]=='s')//如果接收到的第一个字符是‘s’
               lb_get_parameter();//将接收到的值修改目前的参数
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
	if(sta & (RX_OK))//接收中断
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
	
	


	