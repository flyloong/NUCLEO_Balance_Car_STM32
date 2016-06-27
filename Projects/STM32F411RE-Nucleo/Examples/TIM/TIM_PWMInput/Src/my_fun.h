#ifndef __MY_FUN_H
#define __MY_FUN_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "LSM9DS0_AHRS.h"
#include "iNEMO_AHRS.h"
#include "math.h"
#include "control.h"
#include "ov7725.h"
#include "com.h"
#include "cube_hal.h"

#include "osal.h"
#include "sample_service.h"
#include "role_type.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"

#include "bluenrg_sdk_api.h"
uint8_t fangzhen=0;//仿真的时候这里要为1，实际运行的时候为0
/////////////////////////////////////////////////
#define BDADDR_SIZE 6

//BLE_RoleTypeDef BLE_Role = SERVER;
//BLE_RoleTypeDef BLE_Role = CLIENT;

extern volatile uint8_t set_connectable;
extern volatile int connected;
extern volatile uint8_t notification_enabled;

extern volatile uint8_t start_read_tx_char_handle;
extern volatile uint8_t start_read_rx_char_handle;
extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;



/////////////////////////////////////////////////////////
volatile float PRESSURE_Value;
volatile float HUMIDITY_Value;
volatile float TEMPERATURE_Value;
   unsigned char Speek_F[10]; 
int Turn_90_L_Flag=0;
int Lett_Right_Flag=0;
int Turn_Rest_Flag=0;
int Dance1_Flag=0;
int Dance2_Flag=0;
int Dance3_Flag=0;
int Steer_Cmd_Flag=0;
int Block_Flag=0;
int Dange_Flag=0;
int Shake_Heak_Flag=0;
int Block_Cnt=0;
int Ultrasonic_OK_Flag=0;
int Ultrasonic_May_Break=0;
int Ultrasonic_Ready_Flag=1;
int Ultrasonic_OK_Flag_Pre=0;
float GYROSCOPE_ANGLE_RATIO=0.000147f;
extern float pitch, yaw, roll;
 float yaw_Temp,yaw_Org=0;;
float pre_yaw=0.0f;
extern float g_fAngleControlOut;
extern float g_fCarAngle;
extern float  SteerConerolOut1;
extern float  SteerConerolOut2;
extern float Ult_Distance;
uint32_t Time_Start=0;
uint32_t Time_Long=0;
uint32_t Time_Long2=0;
uint32_t Time_Start_Pre=0;
int32_t Speed_L=0;
int32_t Speed_R=0;
int32_t Speed_A=0;
int Turn_Flag=0;
int Turn_Flag2=0;
int Turn_Flag1=0;
float Speed_Need=0;
float Turn_Need=0;
 float D_temp;
 float D_Err;
 float D_Integral;
int32_t Speed_A_Last=0;
uint32_t Pre_Speed_L=0;
uint32_t Pre_Speed_R=0;
uint32_t Car_Cmd=0;//自由行走避障模式
uint32_t Car_Cmd2=0;//总模式
uint32_t Car_Cmd3=0;//预警模式
uint32_t Car_Cmd4=0;//跟踪模式
char Steer_Cmd_Cnt=0;
uint32_t  Inf_Distance=0;
uint32_t  Inf_Distance_Last=0;
int Inf_Value_L[31];
int Inf_Value_R[31];
int Left_Right[2]={0,0};
static char Steer_Cmd_Array[6][30]={{79,80,81,82,83,84,85,84,83,82,81,80,79,78,77,76,75,74,73,74,75,76,77,78,79,0},//大数字向下，控制垂直方向，“是”的舵机控制码
                                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},//
                                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                      {79,80,81,82,83,84,85,84,83,82,81,80,79,78,77,76,75,74,73,74,75,76,77,78,79,0},//大数字向左，控制水平方向，“不是”的舵机控制码
                                      {79,77,75,73,71,69,67,69,71,73,75,73,71,69,67,65,67,69,71,73,71,69,67,65,63,0},
                                      {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};
uint8_t IIC_Mutex=0;
char name[20] = "Balance-Car";
UART_HandleTypeDef UartHandle6;
UART_HandleTypeDef UartHandle2;
 I2C_HandleTypeDef    I2C_EXPBD_Handle;
TIM_HandleTypeDef        TimHandleT1;//舵机
TIM_HandleTypeDef        TimHandleT2;//电机
TIM_HandleTypeDef        TimHandleT3;//编码器
TIM_HandleTypeDef        TimHandleT4;//编码器
TIM_HandleTypeDef        TimHandleT5;//定时
TIM_HandleTypeDef        TimHandleT9;//超声波
SPI_HandleTypeDef SpiHandle3;
ADC_HandleTypeDef    AdcHandle;
iNemoData s_xDataStruct;
/* Timer Input Capture Configuration Structure declaration */
TIM_IC_InitTypeDef       sConfig; 
TIM_OC_InitTypeDef       pwmConfig;//自己
TIM_Encoder_InitTypeDef  encoderConfig;
/* Slave configuration structure */
TIM_SlaveConfigTypeDef   sSlaveConfig;

volatile Axes_TypeDef ACC_Value;         /*!< Acceleration Value */
volatile Axes_TypeDef GYR_Value;         /*!< Gyroscope Value */
volatile Axes_TypeDef MAG_Value;         /*!< Magnetometer Value */

volatile AxesRaw_TypeDef ACC_Value_Raw;  
volatile AxesRaw_TypeDef GYR_Value_Raw;
volatile AxesRaw_TypeDef MAG_Value_Raw;



void Motor_Pwm_Init(void);
void Steer_Pwm_Init(void); 
void Time_Init(uint32_t ms);
void Encoder_Init(void);
void Ultrasonic_Time_Init(void);
void Camera_Init(void);
void Uart6_Init(uint32_t BaudRate);
void Uart2_Init(uint32_t BaudRate);
void Motor_Control_1(int32_t Pulse);
void Motor_Control_2(int32_t Pulse);
void Steer_Control_1(int32_t Pulse);
void Steer_Control_2(int32_t Pulse);
void Nrf24L01_GPIO_Init(void);
void Switch_GPIO_Init(void);
void AHRS_Init(void);
void AHRS_Update(void);
void Kalman_Filter(float angle_m,float gyro_m);
void Get_Speed(void);
void Adc_Init(void);
void Get_Inf_Distance(void);
void Inf_Analyze_L(void);
void Inf_Analyze_R(void);
void Steer_Control(void);
void Motor_Output(void);
void Steer_Output(void);
void Steer_Output(void);
 void Car_Turn_Deg(int *LR,int *Deg, float *Center);
 void Speech(unsigned char *buf);
 void SpeechT(unsigned char *buf1,unsigned char *buf2,unsigned char *buf3);
 void  Block_Detct(void);
 void  Dange_Detct(void);
 void Float2Char(float Value,char *array);
 void Shake_Heak(int *Shake_Heak_F);
 void Steer_Cmd_XY(int *Steer_Cmd);
 void Follow_Me(void);
 void User_Process(void);

 void BlueNRG_Init(void);
 void BlueNRG_Event(void);
 
 void BlueNRG_Event(void){
  
 //     if(set_connectable){
    /* Establish connection with remote device */
 //   Make_Connection();
 //   set_connectable = FALSE;
 //     }
 //     HCI_Process();
 }
 
  void BlueNRG_Init2(void){
    
    
  }
 static void adv_name_generate(uint8_t* uni_name) {
    char temp[3] = "_";
    /*adv name aplice*/
    sprintf(temp+1,"%01d%01d",*uni_name,*(uni_name+1));
    strcat(name, temp);
}
 
 void BlueNRG_Init(void){
   	uint8_t tx_power_level = 7;
    uint16_t adv_interval = 100;
    uint8_t bdAddr[6];
  
		 /* Initialize the BlueNRG SPI driver */
    
    BNRG_SPI_Init();
    /* Initialize the BlueNRG HCI */
    HCI_Init();
    /* Reset BlueNRG hardware */
   BlueNRG_RST();
  
    /*Gatt And Gap Init*/
    ble_init_bluenrg();
 
    HCI_get_bdAddr(bdAddr);
  
   adv_name_generate(bdAddr+4);

    ble_set_adv_param(name, bdAddr, tx_power_level, adv_interval);
    ble_device_start_advertising();
  
   
 }
 
 
 void Follow_Me(void){
     
      
   if(Ultrasonic_OK_Flag==0){
    if(Inf_Distance>1600)
      D_temp=44554.0/Inf_Distance+Inf_Distance/1400.0-9.0;
    else if  (Inf_Distance<=1600&&Inf_Distance>800)
          D_temp=133333.3/Inf_Distance+Inf_Distance/15.0-170;
    else if (Inf_Distance<=800)
         D_temp=50;   
   }
   else {
     if(Ult_Distance<500)
     D_temp=Ult_Distance/10;
     else D_temp=50;
   }
   
   D_Err=25-D_temp;
   D_Integral+=D_Err;
   if(D_Integral>5000)D_Integral=5000;
    if(D_Integral<-5000)D_Integral=-5000;
   //xianfu
    if(D_temp>5&&D_temp<50){
      Speed_Need=D_Err*60+D_Integral*0.3;
 if(Speed_Need<-2000)Speed_Need=-2000;
    }
    else{
      Speed_Need=0;
     D_Integral=0;
      
      
    }
 }
 void  Dange_Detct(void){
  
 
   if(Ultrasonic_OK_Flag==0){
   
     if(Inf_Distance>1000&&Dange_Flag==0&&Speed_Need<0){
         Dange_Flag=1;
          Speech("[m55]危险");  
     }
       else if(Inf_Distance<700&&Dange_Flag){
         Dange_Flag++;
        
         if(Dange_Flag>50){
           Dange_Flag=0;
           Speech("[m55]安全");  
         }      
     }
     else if(Inf_Distance>=700&&Dange_Flag)
       Dange_Flag=1;   
     if(Dange_Flag&&Speed_Need<0)Speed_Need=0;  
  }
   else{
      if(Ult_Distance<300&&Dange_Flag==0&&Speed_Need<0){
         Dange_Flag=1;
          Speech("[m55]危险");  
     }
       else if(Ult_Distance>500&&Dange_Flag){
         Dange_Flag++;
        
         if(Dange_Flag>50){
           Dange_Flag=0;
           Speech("[m55]安全");  
         }      
     }
     else if(Ult_Distance<=500&&Dange_Flag)
       Dange_Flag=1;   
     if(Dange_Flag&&Speed_Need<0)Speed_Need=0;  

   }
   
 }
     
   
 

 void Steer_Cmd_XY(int *SteerCmd){

   if(*SteerCmd){
     if(Steer_Cmd_Array[*SteerCmd][Steer_Cmd_Cnt])
     SteerConerolOut2=Steer_Cmd_Array[*SteerCmd][Steer_Cmd_Cnt];
     if (Steer_Cmd_Array[*SteerCmd-1][Steer_Cmd_Cnt])
        SteerConerolOut1=Steer_Cmd_Array[*SteerCmd-1][Steer_Cmd_Cnt];
     if(Steer_Cmd_Array[*SteerCmd][Steer_Cmd_Cnt]==0&&Steer_Cmd_Array[*SteerCmd-1][Steer_Cmd_Cnt]==0){
       *SteerCmd=0;
       Steer_Cmd_Cnt=0;
     }
     Steer_Cmd_Cnt++;
    
   }
 }
void Shake_Heak(int *Shake_Heak_F){
  
 if(*Shake_Heak_F==1){
   
   if(SteerConerolOut2>59&&Turn_Flag2==0){//从左至右
     SteerConerolOut2--;
   }
     else if(SteerConerolOut2<99&&Turn_Flag2==1){//从右至左
       SteerConerolOut2++;
     }
   else if(SteerConerolOut2==99){
    Turn_Flag2=0;
  }
  else if(SteerConerolOut2==59){    
      Turn_Flag2=1;
  }
  
  if(SteerConerolOut1<99&&Turn_Flag1==0){
    SteerConerolOut1++;
  }
  else if(SteerConerolOut1>59&&Turn_Flag1==1){
    SteerConerolOut1--;
  }
  else if(SteerConerolOut1==99){
    Turn_Flag1=1;
  }
  else if(SteerConerolOut1==59){    
      Turn_Flag1=0;
  }
 }
}
 
 
 void  Block_Detct(void){
  
   if(Speed_L<80&&Speed_L>-80&&Speed_R<80&&Speed_R>-80&&g_fAngleControlOut<100){
     Block_Cnt++;  
   }
   else if (Block_Cnt){
     Block_Cnt=0;
     
     
   }
   if(Block_Cnt>7){
     Block_Flag=1;
     Block_Cnt=0;
   }
   if(Block_Flag){
     Speed_Need=1000;
      Turn_Need=100;
     Block_Flag++;
     if(Block_Flag>20){
       Block_Flag=0;
       ////////左转
    //    Turn_90_L_Flag=90;
   //   Lett_Right_Flag=1;
    //      yaw_Temp=yaw;
     }
     
     
   }
   
   
 }
 
  void Speech(unsigned char *buf)
{ 
    unsigned char len = 0x00;
    unsigned char head[5] = {0xfd,0x00,0x00,0x01,0x00};
    while(buf[len++]!= 0);
head[2]=len+2;
HAL_UART_Transmit(&UartHandle6,head,5,100);
HAL_UART_Transmit(&UartHandle6,buf,len,100);
  
}

  void SpeechT(unsigned char *buf1,unsigned char *buf2,unsigned char *buf3)
{ 
    unsigned char len1 = 0x00;
     unsigned char len2 = 0x00;
      unsigned char len3 = 0x00;
    unsigned char head[5] = {0xfd,0x00,0x00,0x01,0x00};
    while(buf1[len1++]!= 0);
    while(buf2[len2++]!= '\0');
    while(buf3[len3++]!= 0);
head[2]+=len1;
head[2]+=len2;
head[2]+=len3;
head[2]+=0;
HAL_UART_Transmit(&UartHandle6,head,5,100);
HAL_UART_Transmit(&UartHandle6,buf1,len1-1,100);
HAL_UART_Transmit(&UartHandle6,buf2,len2-1,100);
HAL_UART_Transmit(&UartHandle6,buf3,len3,100);

  
}

/*
void Car_Turn_Deg(int *Deg){
  if(*Deg){
   if(yaw-pre_yaw<-300){
        yaw_Temp-=360;
      }
      else if(yaw-pre_yaw>300){
        yaw_Temp +=360;
      }  
   if(*Deg>0){
      Turn_Need=-400;
       if(yaw-yaw_Temp>*Deg) *Deg=0;
       
    }
   else {
      Turn_Need=400;
      if(yaw-yaw_Temp<*Deg) *Deg=0;
   }
       
       if(Dance1_Flag){
       SteerConerolOut2 =79-(yaw-yaw_Temp)/2;
      }
  }
   pre_yaw=yaw;
}
*/
/*
 void Car_Turn_Deg(int *LR,int *Deg, float *Center){
  if(*LR){
   if(yaw-pre_yaw<-300){
        *Center-=360;
      }
      else if(yaw-pre_yaw>300){
        *Center +=360;
      }  
   if(*LR==1){
      Turn_Need=-400;
      if(yaw-*Center>*Deg) {
         *Deg=0;
         *LR=0;
         
      }
    }
   else if(*LR==2){
      Turn_Need=400;
      if(yaw-*Center<*Deg){
        *Deg=0;
        *LR=0;
      }
   }
       
       if(Dance1_Flag){
       SteerConerolOut2 =79-(yaw-*Center)/2;
     //  SteerConerolOut1 =79-(yaw-*Center)/3;
      }
  }
   pre_yaw=yaw;
}  
  */
   void Car_Turn_Deg(int *LR,int *Deg, float *Center){
  if(*LR){
    
   if(yaw-pre_yaw<-300){
    //    Temp_Center=*Center-360;
     *Center-=360;
      }
      else if(yaw-pre_yaw>300){
      //  Temp_Center=*Center+360;
         *Center+=360;
      }  
 //  else Temp_Center=*Center;
   
   if(*LR==1){
      Turn_Need=-400;
      if(yaw-*Center>*Deg) {
         *Deg=0;
         *LR=0;
          
         
      }
    }
   else if(*LR==2){
      Turn_Need=400;
      if(yaw-*Center<*Deg){
        *Deg=0;
        *LR=0;
       
      }
   }
       
       if(Dance1_Flag){
       SteerConerolOut2 =73-(yaw-*Center)/2;
     //  SteerConerolOut1 =79-(yaw-*Center)/3;
      }
  }
   pre_yaw=yaw;
}

void Steer_Output(void){
  
  int Temp_SteerConerolOut1,Temp_SteerConerolOut2;
  Temp_SteerConerolOut1=(int)SteerConerolOut1;
  Temp_SteerConerolOut2=(int)SteerConerolOut2;
  if(Temp_SteerConerolOut2>130)Temp_SteerConerolOut2=130;
  if(Temp_SteerConerolOut2<34)Temp_SteerConerolOut2=34;
  if(Temp_SteerConerolOut1>130)Temp_SteerConerolOut1=130;
  if(Temp_SteerConerolOut1<34)Temp_SteerConerolOut1=34;
  
  
  Steer_Control_2(Temp_SteerConerolOut2);
Steer_Control_1(Temp_SteerConerolOut1);
}
  
void Steer_Control(void){
  
  if(SteerConerolOut2<100&&Turn_Flag==1){//从右至左
  Inf_Value_L[100-(int)SteerConerolOut2]=Inf_Distance;
 SteerConerolOut2++;
  }
  else if(SteerConerolOut2>70&&Turn_Flag==0){//从左至右
  Inf_Value_R[(int)SteerConerolOut2-70]=Inf_Distance;
  SteerConerolOut2--;
  }
  else if(SteerConerolOut2==100){
     Inf_Value_L[0]=Inf_Distance;
    Turn_Flag=0;
  Inf_Analyze_L();//从右到左，分析右边的数据
  }
  else if(SteerConerolOut2==70){
     Inf_Value_R[0]=Inf_Distance;
      Turn_Flag=1;
   Inf_Analyze_R();//从左到右
  }

  
}

void Inf_Analyze_L(void){
  int i;
  for(i=10;i<=30;i++){
    if(Inf_Value_L[i]>1000){
      Left_Right[0]=i;
    break;
  }
    if (i==30)
    Left_Right[0]=-1;
  }
  if(Car_Cmd&0x01)
 Turn_Control();
}
void Inf_Analyze_R(void){
  int i;
  for(i=10;i<=30;i++){
    if(Inf_Value_R[i]>1000){
      Left_Right[1]=i;
    break;
  }
    if (i==30)
    Left_Right[1]=-1;
  }
  if(Car_Cmd&0x01)
 Turn_Control();
}

void Get_Inf_Distance(void){
  
  //  HAL_ADC_PollForConversion(&AdcHandle,100);
 Inf_Distance_Last=HAL_ADC_GetValue(&AdcHandle);
 Inf_Distance*=0.5;
 Inf_Distance+=0.5*Inf_Distance_Last;
  
}
void Adc_Init(){
  ADC_ChannelConfTypeDef sConfig;
   GPIO_InitTypeDef          GPIO_InitStruct;
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_ADC1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  AdcHandle.Instance = ADC1;
  
  AdcHandle.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  AdcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle.Init.ScanConvMode = DISABLE;
  AdcHandle.Init.ContinuousConvMode = ENABLE;
  AdcHandle.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle.Init.NbrOfDiscConversion = 0;
  AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.NbrOfConversion = 1;
  AdcHandle.Init.DMAContinuousRequests = DISABLE;
  AdcHandle.Init.EOCSelection = DISABLE;
  
  HAL_ADC_Init(&AdcHandle);
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfig.Offset = 0;
  HAL_ADC_ConfigChannel(&AdcHandle, &sConfig);
  HAL_ADC_Start(&AdcHandle);
}
void Get_Speed(void){
  uint32_t TempL,TempR;
    TempL=HAL_TIM_ReadCapturedValue(&TimHandleT3, TIM_CHANNEL_1);//编码器读取
  //  if(TempL-Pre_Speed_L>30000){
  //    Speed_L=TempL-Pre_Speed_L-65535;}
  //  else if  (TempL-Pre_Speed_L<-30000){
  //    Speed_L=TempL-Pre_Speed_L+65535;  }
  //  else 
      Speed_L=TempL-Pre_Speed_L;
     Pre_Speed_L=TempL;  
     if(Speed_L<-20000){
        Speed_L+=65535;}
        else   if(Speed_L>20000){
        Speed_L-=65535;} 
    TempR=HAL_TIM_ReadCapturedValue(&TimHandleT4, TIM_CHANNEL_1);//编码器读取
  //  if(TempR-Pre_Speed_R>30000){
  //    Speed_R=TempR-Pre_Speed_R-65535;}
  //  else if  (TempR-Pre_Speed_R<-30000){
   //   Speed_R=TempR-Pre_Speed_R+65535;  }
  //  else
       Speed_R=TempR-Pre_Speed_R;
 Pre_Speed_R=TempR;  
       if(Speed_R<-20000){
        Speed_R+=65535;}
          else   if(Speed_R>20000){
        Speed_R-=65535;} 
   //   Speed_A=(Speed_L-Speed_R)/2;
        Speed_A_Last=(Speed_L-Speed_R)/2;
  Speed_A*=0.7;
    Speed_A+=Speed_A_Last*0.3;  
      
 
}
float ax_pre=0;

void AHRS_Update(void){
  float ax,ay,az,gx,gy,gz,mx,my,mz;
  
  while(IIC_Mutex);
  IIC_Mutex=1;
    BSP_IMU_6AXES_X_GetAxesRaw((AxesRaw_TypeDef *)&ACC_Value_Raw);
    BSP_IMU_6AXES_G_GetAxesRaw((AxesRaw_TypeDef *)&GYR_Value_Raw);
    //BSP_MAGNETO_M_GetAxesRaw((AxesRaw_TypeDef *)&MAG_Value_Raw);
   IIC_Mutex=0;
    ax=ACC_Value_Raw.AXIS_X;
   // ax=0.9*ax_pre+0.1*ax;
    ax_pre=ax;
    ay=ACC_Value_Raw.AXIS_Y;
    az=ACC_Value_Raw.AXIS_Z;
    gx=(GYR_Value_Raw.AXIS_X+162)*GYROSCOPE_ANGLE_RATIO;
    gy=(GYR_Value_Raw.AXIS_Y-50) *GYROSCOPE_ANGLE_RATIO;
    gz=(GYR_Value_Raw.AXIS_Z-455)*GYROSCOPE_ANGLE_RATIO;
  //   gx=(GYR_Value_Raw.AXIS_X+162)*0.00015f;
  //  gy=(GYR_Value_Raw.AXIS_Y-50) *0.00015f;
  //  gz=(GYR_Value_Raw.AXIS_Z-455)*0.00015f;
    mx=MAG_Value_Raw.AXIS_X+6000;
    my=MAG_Value_Raw.AXIS_Y+5800;
    mz=MAG_Value_Raw.AXIS_Z+1200;
    
    
    
  //  mahonyQuaternionUpdate( ax,  ay,  az,  gx,  gy,  gz,  mx,  my,  mz);
 //   madgwickQuaternionUpdate( ax,  ay,  az,  gx,  gy,  gz,  mx,  my,  mz);
  //   MahonyAHRSupdate( gx,  gy,  gz,ax,  ay,  az,    mx,  my,  mz);
MahonyAHRSupdateIMU( gx,  gy,  gz,ax,  ay,  az);
    LSM9DS0_AHRS_updateEulerAngles();
  //  Kalman_Filter(ax/182 ,-(GYR_Value_Raw.AXIS_Y-50)*0.01);
    SpeedControlOutput();
        AngleControl();
        Motor_Output();
       
    /*
   s_xDataStruct.xSensorData.m_fAcc[0]=ACC_Value_Raw.AXIS_X*9.81f/100.0f;
   s_xDataStruct.xSensorData.m_fAcc[1]=-ACC_Value_Raw.AXIS_Y*9.81f/100.0f;
   s_xDataStruct.xSensorData.m_fAcc[2]=-ACC_Value_Raw.AXIS_Z*9.81f/100.0f;
   s_xDataStruct.xSensorData.m_fGyro[0]=GYR_Value_Raw.AXIS_X/14.258*3.141592f/180.0f/10000;
   s_xDataStruct.xSensorData.m_fGyro[1]=-GYR_Value_Raw.AXIS_Y/14.258*3.141592f/180.0/10000;
   s_xDataStruct.xSensorData.m_fGyro[2]=-GYR_Value_Raw.AXIS_Z/14.258*3.141592f/180.0/10000;
   s_xDataStruct.xSensorData.m_fMag[0]=MAG_Value_Raw.AXIS_X/450;
   s_xDataStruct.xSensorData.m_fMag[1]=-MAG_Value_Raw.AXIS_Y/450;
   s_xDataStruct.xSensorData.m_fMag[2]=-MAG_Value_Raw.AXIS_Z/400;
    */
    /*
   s_xDataStruct.xSensorData.m_fAcc[0]=ACC_Value_Raw.AXIS_X*2.0/32768.0;
   s_xDataStruct.xSensorData.m_fAcc[1]=-ACC_Value_Raw.AXIS_Y*2.0/32768.0;
   s_xDataStruct.xSensorData.m_fAcc[2]=-ACC_Value_Raw.AXIS_Z*2.0/32768.0;
   s_xDataStruct.xSensorData.m_fGyro[0]=GYR_Value_Raw.AXIS_X*2000.0/32768.0* 3.1416 / 180.0;
   s_xDataStruct.xSensorData.m_fGyro[1]=-GYR_Value_Raw.AXIS_Y*2000.0/32768.0* 3.1416 / 180.0;
   s_xDataStruct.xSensorData.m_fGyro[2]=-GYR_Value_Raw.AXIS_Z*2000.0/32768.0* 3.1416 / 180.0;
   s_xDataStruct.xSensorData.m_fMag[0]=MAG_Value_Raw.AXIS_X*4.0/32768.0;
   s_xDataStruct.xSensorData.m_fMag[1]=-MAG_Value_Raw.AXIS_Y*4.0/32768.0;
   s_xDataStruct.xSensorData.m_fMag[2]=-MAG_Value_Raw.AXIS_Z*4.0/32768.0;
   */
    /*
  iNEMO_AHRS_Update(&(s_xDataStruct.xSensorData), &(s_xDataStruct.xEulerAngles), &(s_xDataStruct.xQuat));
  roll  = s_xDataStruct.xEulerAngles.m_fRoll* 180.0f / 3.141592f;
  pitch = s_xDataStruct.xEulerAngles.m_fPitch * 180.0f / 3.141592f;
  yaw  = s_xDataStruct.xEulerAngles.m_fYaw * 180.0f / 3.141592f;
   */
  
}
void Motor_Output(void){
  int temp1,temp2;
  temp1=(int)(g_fAngleControlOut+Turn_Need)*1.035;
  if(temp1>1000)temp1=1000;
  if(temp1<-1000)temp1=-1000;
    temp2=(int)(g_fAngleControlOut-Turn_Need);
  if(temp2>1000)temp2=1000;
  if(temp2<-1000)temp2=-1000;
   if(g_fCarAngle>-50&&g_fCarAngle<50){  
    Motor_Control_1(temp1 );
     Motor_Control_2(temp2);
        }
        else{
        Motor_Control_1(0);
     Motor_Control_2(0);    
        }
}
  void AHRS_Init(void){
    s_xDataStruct.xSensorData.m_fAccRef[0]=0;
      s_xDataStruct.xSensorData.m_fAccRef[1]=0;
      s_xDataStruct.xSensorData.m_fAccRef[2]=-9.81f;
      
      s_xDataStruct.xSensorData.m_fMagRef[0]=0.37f;
      s_xDataStruct.xSensorData.m_fMagRef[1]=0;
      s_xDataStruct.xSensorData.m_fMagRef[2]=-0.25f;
      
      
     
    //  if(s_uTimerFrequence<7)
    //    s_xDataStruct.xSensorData.m_fDeltaTime=1.0f/nFreqEnum2freVal[s_uTimerFrequence];
    //  else
        s_xDataStruct.xSensorData.m_fDeltaTime=0.002f;
      
      s_xDataStruct.xSensorData.m_fVarAcc=5.346e-6;
      s_xDataStruct.xSensorData.m_fVarMag=5.346e-6;//这里我将源程序的5.346e-6缩小了10倍

      iNEMO_AHRS_Init(&s_xDataStruct.xSensorData, &s_xDataStruct.xEulerAngles, &s_xDataStruct.xQuat);
    
  }
  void Switch_GPIO_Init(void){
  GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOD_CLK_ENABLE(); 
       GPIO_InitStruct.Mode    = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull      = GPIO_PULLUP;
   GPIO_InitStruct.Pin     =GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }

void Nrf24L01_GPIO_Init(void){
    GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOC_CLK_ENABLE(); 
     GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pin     =GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
 HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
       GPIO_InitStruct.Mode    = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
   GPIO_InitStruct.Pin     =GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
/*
__HAL_RCC_SPI3_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  SpiHandle3.Instance               = SPI3;
  SpiHandle3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  SpiHandle3.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle3.Init.CLKPhase          = SPI_PHASE_2EDGE;
  SpiHandle3.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  SpiHandle3.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle3.Init.CRCPolynomial     = 7;
  SpiHandle3.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle3.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle3.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle3.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle3.Init.Mode              = SPI_MODE_MASTER;

  HAL_SPI_Init(&SpiHandle3);
 */
}
  void Steer_Control_1(int32_t Pulse){
    
    
    
    HAL_TIM_PWM_Pulse(&TimHandleT1,TIM_CHANNEL_3,Pulse);
  }
  void Steer_Control_2(int32_t Pulse){
     HAL_TIM_PWM_Pulse(&TimHandleT1,TIM_CHANNEL_2,Pulse);
  }

void Motor_Control_1(int32_t Pulse){
  if(Pulse==0){
    GPIOB->BSRR =GPIO_PIN_2<<16;	
    GPIOB->BSRR =GPIO_PIN_12<<16;
  }
  else if(Pulse>0){
    GPIOB->BSRR =GPIO_PIN_2;	
    GPIOB->BSRR =GPIO_PIN_12<<16;
    HAL_TIM_PWM_Pulse(&TimHandleT2,TIM_CHANNEL_3,Pulse);
  }
  else{
        GPIOB->BSRR =GPIO_PIN_2<<16;	
    GPIOB->BSRR =GPIO_PIN_12;
    HAL_TIM_PWM_Pulse(&TimHandleT2,TIM_CHANNEL_3,-Pulse);
  }
}
void Motor_Control_2(int32_t Pulse){
    if(Pulse==0){
    GPIOA->BSRR =GPIO_PIN_13<<16;	
    GPIOA->BSRR =GPIO_PIN_14<<16;
    }
   else if(Pulse>0){
    GPIOA->BSRR =GPIO_PIN_13;	
    GPIOA->BSRR =GPIO_PIN_14<<16;
    HAL_TIM_PWM_Pulse(&TimHandleT2,TIM_CHANNEL_1,Pulse);
  }
  else{
    GPIOA->BSRR =GPIO_PIN_13<<16;	
    GPIOA->BSRR =GPIO_PIN_14;
    HAL_TIM_PWM_Pulse(&TimHandleT2,TIM_CHANNEL_1,-Pulse);
  }
  
  
}
void Uart6_Init(uint32_t BaudRate){
UartHandle6.Instance          = USART6;
  UartHandle6.Init.BaudRate     = BaudRate;
  UartHandle6.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle6.Init.StopBits     = UART_STOPBITS_1;
  UartHandle6.Init.Parity       = UART_PARITY_NONE;
  UartHandle6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle6.Init.Mode         = UART_MODE_TX;
  UartHandle6.Init.OverSampling = UART_OVERSAMPLING_16;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_USART6_CLK_ENABLE();
  GPIO_InitStruct.Pin       = GPIO_PIN_11;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
 // GPIO_InitStruct.Pull      = GPIO_PULLUP;
   GPIO_InitStruct.Pull      =GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //GPIO_InitStruct.Pin = GPIO_PIN_12;
  //GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  //HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   HAL_UART_Init(&UartHandle6); 

}
void Uart2_Init(uint32_t BaudRate){
UartHandle2.Instance          = USART2;
  UartHandle2.Init.BaudRate     = BaudRate;
  UartHandle2.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle2.Init.StopBits     = UART_STOPBITS_1;
  UartHandle2.Init.Parity       = UART_PARITY_NONE;
  UartHandle2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle2.Init.Mode         = UART_MODE_TX_RX;
  UartHandle2.Init.OverSampling = UART_OVERSAMPLING_16;
  
  GPIO_InitTypeDef  GPIO_InitStruct;
  __HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_USART2_CLK_ENABLE(); 
  GPIO_InitStruct.Pin       = GPIO_PIN_2;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin       = GPIO_PIN_3;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
   HAL_UART_Init(&UartHandle2); 

}
 // printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");

void Encoder_Init(void){
TimHandleT3.Instance = TIM3;
  TimHandleT3.Init.Period =  0xFFFF;
  TimHandleT3.Init.Prescaler = 0;
  TimHandleT3.Init.ClockDivision = 0;
  TimHandleT3.Init.CounterMode = TIM_COUNTERMODE_UP;  
  
  encoderConfig.EncoderMode =TIM_ENCODERMODE_TI1;
  encoderConfig.IC1Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC1Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC1Prescaler=0;
  encoderConfig.IC1Filter   =6;
  encoderConfig.IC2Polarity =TIM_ICPOLARITY_RISING;
  encoderConfig.IC2Selection=TIM_ICSELECTION_DIRECTTI;
  encoderConfig.IC2Prescaler=0;
  encoderConfig.IC2Filter   =6;
HAL_TIM_Encoder_Init(&TimHandleT3,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT3,TIM_CHANNEL_1);

  TimHandleT4.Instance = TIM4;
  TimHandleT4.Init.Period =  0xFFFF;
  TimHandleT4.Init.Prescaler = 0;
  TimHandleT4.Init.ClockDivision = 0;
  TimHandleT4.Init.CounterMode = TIM_COUNTERMODE_UP;  
HAL_TIM_Encoder_Init(&TimHandleT4,  &encoderConfig);
HAL_TIM_Encoder_Start(&TimHandleT4,TIM_CHANNEL_1);

}

void Motor_Pwm_Init(void){
   
  
  TimHandleT2.Instance = TIM2;
  TimHandleT2.Init.Period =  1000 - 1;;
  TimHandleT2.Init.Prescaler = 20-1;
  TimHandleT2.Init.ClockDivision = 0;
  TimHandleT2.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT2);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=700;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT2, &pwmConfig, TIM_CHANNEL_3);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT2, &pwmConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&TimHandleT2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT2, TIM_CHANNEL_1);
  
  
  //电机控制引脚
     __HAL_RCC_GPIOA_CLK_ENABLE();
   GPIO_InitTypeDef GPIO_InitStruct;
     GPIO_InitStruct.Mode    = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;

  GPIO_InitStruct.Pin     =GPIO_PIN_13|GPIO_PIN_14;
if(fangzhen==0)
 HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   GPIO_InitStruct.Pin     =GPIO_PIN_2|GPIO_PIN_12;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
  }

void Steer_Pwm_Init(void){
  
    GPIO_InitTypeDef GPIO_InitStruct;
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
     __HAL_RCC_TIM1_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;

     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
  TimHandleT1.Instance = TIM1;
  TimHandleT1.Init.Period =  1000 - 1;;
  TimHandleT1.Init.Prescaler = 2000-1;
  TimHandleT1.Init.ClockDivision = 0;
  TimHandleT1.Init.CounterMode = TIM_COUNTERMODE_UP;  
  HAL_TIM_PWM_Init(&TimHandleT1);

  pwmConfig.OCMode=TIM_OCMODE_PWM1;
  pwmConfig.Pulse=79;
  HAL_TIM_PWM_ConfigChannel(&TimHandleT1, &pwmConfig, TIM_CHANNEL_2);
   HAL_TIM_PWM_ConfigChannel(&TimHandleT1, &pwmConfig, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&TimHandleT1, TIM_CHANNEL_2);
  
  

     __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_TIM1_CLK_ENABLE();
  
  
  }
void Time_Init(uint32_t ms){
  TimHandleT5.Instance = TIM5;
  TimHandleT5.Init.Period = ms - 1;
  TimHandleT5.Init.Prescaler = 10000-1;
  TimHandleT5.Init.ClockDivision = 0;
  TimHandleT5.Init.CounterMode = TIM_COUNTERMODE_UP;
 
    __HAL_RCC_TIM5_CLK_ENABLE();
  HAL_NVIC_SetPriority(TIM5_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  HAL_TIM_Base_Init(&TimHandleT5);
  HAL_TIM_Base_Start_IT(&TimHandleT5);
}
void Ultrasonic_Time_Init(void){
  TimHandleT9.Instance = TIM9;
  TimHandleT9.Init.Period = 0xFFFF;
  TimHandleT9.Init.Prescaler = 640;
  TimHandleT9.Init.ClockDivision = 0;
  TimHandleT9.Init.CounterMode = TIM_COUNTERMODE_UP;

  __HAL_RCC_TIM9_CLK_ENABLE();
  
  
  HAL_TIM_Base_Init(&TimHandleT9);
  HAL_TIM_Base_Start(&TimHandleT9);
  /*
   GPIO_InitTypeDef   GPIO_InitStructure;
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
   GPIO_InitStructure.Speed =GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  */
  
   GPIO_InitTypeDef   GPIO_InitStructure;
  __HAL_RCC_GPIOA_CLK_ENABLE(); 
  
  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
  //GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  //GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Pin = GPIO_PIN_12;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  
}
void Camera_Init(void){
   GPIO_InitTypeDef   GPIO_InitStructure;

  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
   GPIO_InitStructure.Speed =GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//**************************************************************************
//   Kalman滤波
//**************************************************************************
extern float g_fCarAngle;
extern float g_fGyroscopeAngleSpeed;
float angle, angle_dot;         //外部需要引用的变量
static float q_bias;
float R_angle;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)	          //gyro_m:gyro_measure
{

  	 //-------------------------------------------------------
	//float Q_angle=0.001, Q_gyro=0.003,  dt=0.005; //R_angle越大跟踪速度越快
				                      //注意：dt的取值为kalman滤波器采样时间;
        float dt=0.005;  //0.005
       
        float Q_angle=0.001, Q_gyro=0.003; 
        float R_angle=160;
        
	float P[2][2] = {	{ 1, 0 },
				{ 0, 1 }
				};
		
	float Pdot[4] ={0,0,0,0};
	
	const char C_0 = 1;
	
	float  angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	//-------------------------------------------------------


	g_fCarAngle+=(gyro_m-q_bias) * dt;//先验估计
	
	Pdot[0]=Q_angle - P[0][1] - P[1][0];// Pk-' 先验估计误差协方差的微分
	Pdot[1]=- P[1][1];
	Pdot[2]=- P[1][1];
	Pdot[3]=Q_gyro;
	
	P[0][0] += Pdot[0] * dt;// Pk- 先验估计误差协方差微分的积分 = 先验估计误差协方差
	P[0][1] += Pdot[1] * dt;
	P[1][0] += Pdot[2] * dt;
	P[1][1] += Pdot[3] * dt;
	
	
	angle_err = angle_m - g_fCarAngle;//zk-先验估计
	
	
	PCt_0 = C_0 * P[0][0];
	PCt_1 = C_0 * P[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;//Kk
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * P[0][1];

	P[0][0] -= K_0 * t_0;//后验估计误差协方差
	P[0][1] -= K_0 * t_1;
	P[1][0] -= K_1 * t_0;
	P[1][1] -= K_1 * t_1;
	
	
	g_fCarAngle+= K_0 * angle_err;//后验估计
	q_bias	+= K_1 * angle_err;//后验估计
	g_fGyroscopeAngleSpeed = gyro_m-q_bias;//输出值（后验估计）的微分 = 角速度
  

}
void Float2Char(float Value, char *array) 
{
int IntegerPart;
float DecimalPart;
int i = 0;
int j = 0;
char temp;
//分离整数部分与小数部分：
//整数部分保存在IntegerPart中
//小数部分保存在DecimalPart中
if (Value>=1)
{
IntegerPart = (int)Value;
DecimalPart = Value-IntegerPart;
}
else
{
IntegerPart=0;
DecimalPart=Value-IntegerPart;
}
//转换整数部分
if(IntegerPart==0)
{
array[0]=0+48;
array[1]='.';
i=1;
}
else
{
while(IntegerPart>0)
{
array[i]=IntegerPart%10+48;
IntegerPart=IntegerPart/10;
i++;
}
i--;
//修正转换结果的顺序
for(j=0;j+1<=(i+1)/2;j++)
{
temp=array[j];
array[j]=array[i-j];
array[i-j]=temp;
}
i++;
array[i]='.';
}
//转换小数部分，此处设置最多转换到第四位小数
i++;
array[i++] = (int)(DecimalPart*10)%10+48;
//array[i++] = (int)(DecimalPart*100)%10+48;
//array[i++] = (int)(DecimalPart*1000)%10+48;
//array[i++] = (int)(DecimalPart*10000)%10+48;
array[i] = '\0'; //结束符
} 



#endif 