/**
  ******************************************************************************
  * @file    TIM/TIM_PWMInput/Src/main.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    26-December-2014
  * @brief   This example shows how to use the TIM peripheral to measure the
  *          frequency and duty cycle of an external signal.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
    #include "my_fun.h"
    #include "outputdata.h"
        #include "NRF24L0.h"
#include "stm32f4xx_hal_i2c.h"

//unsigned char temp=0;
extern float g_fAngleSpeed;
extern float g_fAngleControlOut;
extern float Ult_Distance;
TMsg Msg;
int New_Data_Flag=0;
int temp=0;
int temp2=75;
int Time_Cnt=0;
int Time_Cnt2=0;
int Time_Cnt3=0;
int Time_Cnt4=0;
int16_t Axe[3];
uint8_t id[3];
int Inf_Value[31];
unsigned char databuf[7] = {0};
int NRF24L01_OK_Flag=0;
/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Input
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Timer handler declaration */


/* Captured Value */
__IO uint32_t            uwIC2Value = 0;
/* Duty Cycle Value */
__IO uint32_t            uwDutyCycle = 0;
/* Frequency Value */
__IO uint32_t            uwFrequency = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
uint16_t MY_counter=0;
uint8_t MY_Msg_Data[20];
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */


void ble_device_on_message(uint8_t type, uint16_t length, uint8_t* value)
{
//  if(type!=1)return;
//  if(*value!='s'||*(value+1)!='t')return;
      MY_counter++;
      MY_Msg_Data[0]=*(value);
         MY_Msg_Data[1]= *(value+1);
       MY_Msg_Data[2]=*(value+2);
         MY_Msg_Data[3]= *(value+3);
          MY_Msg_Data[4]= *(value+4);
           MY_Msg_Data[5]= *(value+5);
            MY_Msg_Data[6]= *(value+6);
             MY_Msg_Data[7]= *(value+7);

}

void ble_device_on_disconnect(uint8_t reason)
{
    /* Make the device connectable again. */
    Ble_conn_state = BLE_CONNECTABLE;
    ble_device_start_advertising();
}

 int main(void)
{ 
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
   
  /* Configure the system clock to 100 MHz */

   SystemClock_Config();
    Uart6_Init(9600);
 // HAL_Delay(500);
  //  HAL_Delay(10000);
//Speech("[V1][m55][t5]现在");
   
  /* Configure LED2 */
  BSP_LED_Init(LED2);
 Uart2_Init(115200);
 
    BSP_IMU_6AXES_Init();
 //HAL_UART_Transmit(&UartHandle2,"2",1,100);
    BSP_MAGNETO_Init();
    BSP_HUM_TEMP_Init();
    BSP_PRESSURE_Init();
     
   // AHRS_Init();
  LSM9DS0_AHRS();
 // Ov7725_GPIO_Config();
 // VSYNC_Init();
  // int temp;
 // while(1){
 //   for(temp=0;temp<10000;temp++){
 // BSP_IMU_6AXES_X_GetAxesRaw((AxesRaw_TypeDef *)&ACC_Value_Raw);
  //BSP_IMU_6AXES_X_GetAxes((Axes_TypeDef *)&ACC_Value);
 //   }
 // HAL_Delay(50);
 // }
 //   
 BSP_LED_On(LED2);
  /*##-1- Configure the TIM peripheral #######################################*/ 
  /* Set TIMx instance */
  //

  extern UART_HandleTypeDef UartHandle;
  USARTConfig();
  /*
  while(1){
     if (UART_ReceivedMSG((TMsg*) &Msg))
    {
   // UART_SendMsg(&Msg);
   HAL_UART_Transmit(&UartHandle,"1",1,100);
    }
  }
  */
  
 /*
  while(1){
    HAL_Delay(10);
     if(HAL_UART_Receive(&UartHandle2,databuf,7,100)==HAL_OK)
    
    {
   //  HAL_UART_Transmit(&UartHandle2,databuf,11,100);
     if(databuf[0]=='s'&&databuf[6]==databuf[0]+databuf[1]+databuf[2]+databuf[3]+databuf[4]+databuf[5]){//保护，帧头与校验
     Car_Cmd=databuf[1];
     if(Car_Cmd==0){
       Speed_Need=(databuf[2]-50)*40;
       Turn_Need=(databuf[3]-50)*4;
       SteerConerolOut1=databuf[4]+30;
       SteerConerolOut2=databuf[5]+30;
       
      }
     }
    }
  }
  */
  
  
  Motor_Pwm_Init();
  Steer_Pwm_Init();
   Encoder_Init();
  Nrf24L01_GPIO_Init();
  //Switch_GPIO_Init();
  
 // while(NRF24L01_Check()); // 等待检测到NRF24L01，程序才会向下执行
  
 // if(NRF24L01_Check()==0){
//NRF24L01_RT_Init(ACK_RX);
//NRF24L01_OK_Flag=1;
 // }
   Ultrasonic_Time_Init();
 //Camera_Init();
Adc_Init();
 Steer_Control_1((int)SteerConerolOut1);

Msg.Data[3]=50;
Msg.Data[4]=50;
 //HAL_Delay(5000);
//HAL_UART_Transmit(&UartHandle2,"3",1,100);
BlueNRG_Init();
 //HAL_UART_Transmit(&UartHandle2,"4",1,100);
          Time_Init(25);//10000=1S
          
 // extern uint8_t MY_Msg_Data[8];
 // extern uint8_t MY_Msg_Long;
 // extern  uint16_t MY_counter;  
  uint16_t Pre_MY_counter; 
  
  while (1)
  {//HAL_Delay(10);
    
   HCI_Process();
        if(Ble_conn_state) {
            Ble_conn_state = BLE_NOCONNECTABLE;
        }

    if(MY_counter!=Pre_MY_counter)
    {
     Msg.Data[0]= MY_Msg_Data[0];
       Msg.Data[1]= MY_Msg_Data[1];
        Msg.Data[2]= MY_Msg_Data[2];
         Msg.Data[3]= MY_Msg_Data[3];
          Msg.Data[4]= MY_Msg_Data[4];
           Msg.Data[5]= MY_Msg_Data[5];
            Msg.Data[6]= MY_Msg_Data[6];
             Msg.Data[7]= MY_Msg_Data[7];
      New_Data_Flag=1;
    }
    Pre_MY_counter=MY_counter;
 
   //   if (UART_ReceivedMSG((TMsg*) &Msg))
  if(New_Data_Flag)
    {
      New_Data_Flag=0;
   // UART_SendMsg(&Msg);
  
      if(Msg.Data[0]=='s'&&Msg.Data[1]=='t'){//保护，帧头与校验&&databuf[6]==(databuf[0]+databuf[1]+databuf[2]+databuf[3]+databuf[4]+databuf[5])
    Car_Cmd2=Msg.Data[7]; 
    if(Car_Cmd!=(Msg.Data[2]&0x01)){
    Car_Cmd=Msg.Data[2]&0x01;  
    if(Car_Cmd)  
      Speech("[m55]自动模式");
      else 
        Speech("[m55]手动模式");
    }
     
     if(Car_Cmd3!=(Msg.Data[2]&0x02)){
   Car_Cmd3=Msg.Data[2]&0x02;
   if(Car_Cmd3)  
      Speech("[m55]开启预警");
      else 
Speech("[m55]关闭预警");
    }
    
    
    
     if(Car_Cmd4!=(Msg.Data[2]&0x04)){
   Car_Cmd4=Msg.Data[2]&0x04;
    if(Car_Cmd4)  
      Speech("[m55]跟踪模式");
      else 
        Speech("[m55]停止跟踪");
    }
  // Dange_Cmd=Msg.Data[2]&0x02;
    // HAL_UART_Transmit(&UartHandle,"1",1,100);
    switch (Car_Cmd2){
    case 'l'://左转90°
      Turn_90_L_Flag=90;
      Lett_Right_Flag=1;
 //      Speech("sound101");
          yaw_Temp=yaw;
          break;
    case 'r'://右转90°
       Turn_90_L_Flag=-90;
       Lett_Right_Flag=2;
 //        Speech("sound102");
          yaw_Temp=yaw;
          break;
      case 'w'://跳舞
        Dance1_Flag=4;
 //        Speech("msgh");
      //  yaw_Temp=yaw;
        break; 
              case 'v'://跳舞2
        Dance2_Flag=3;
 //        Speech("msgh");
      //  yaw_Temp=yaw;
        break;
    case 'a'://跳舞3
      if(Dance3_Flag==0){
       Dance3_Flag=1;
         yaw_Temp=yaw;
          Turn_Need=-400;
      }
      else {
        Dance3_Flag=0;
      }
      break;
      case 'x':         //确定朝向
  //      Speech("sound401");
        yaw_Temp=yaw;
        break;  
            case 'i':         //自我介绍
        Speech("[m55]大家好，我叫七零儿[=er5]");
        yaw_Temp=yaw;
          temp=3;
        break;  
    case 'c':
         Speech("sound402");
         
         break;
    case 'P':
      IIC_Mutex=1;
      while(IIC_Mutex);
      
        BSP_PRESSURE_GetPressure((float *)&PRESSURE_Value);
       // IIC_Mutex=0;
          Float2Char(PRESSURE_Value,Speek_F) ;
          SpeechT("[m55]现在的压强是",Speek_F,"帕斯卡");
       break;
    case 'H':
      IIC_Mutex=1;
      while(IIC_Mutex);
      temp=1;
         BSP_HUM_TEMP_GetHumidity((float *)&HUMIDITY_Value);
      //   IIC_Mutex=0;
          Float2Char(HUMIDITY_Value,Speek_F) ;
          SpeechT("[m55]现在的相对湿度是百分之",Speek_F," ");
       break;
    case 'T':
      IIC_Mutex=1;
      while(IIC_Mutex);
      temp=2;
     // IIC_Mutex=1;
         BSP_HUM_TEMP_GetTemperature((float *)&TEMPERATURE_Value);
     //    
      Float2Char(TEMPERATURE_Value,Speek_F) ;
        SpeechT("[m55]现在的温度是",Speek_F,"度");
        break;
     case 'U':
       if(Ultrasonic_OK_Flag){
      Float2Char(Ult_Distance/10,Speek_F) ;
        SpeechT("[m55]超声波测得距前方障碍物",Speek_F,"厘米");
       }
       else Speech("[m55]请插入超声波");
        break;    
        
    case 'D':
      
      Get_Inf_Distance();
      if(Inf_Distance>1600){
      D_temp=44554.0/Inf_Distance-Inf_Distance/1400.0-9.0;
  Float2Char(D_temp,Speek_F) ;
       SpeechT("[m55]距离前方障碍物",Speek_F,"厘米");
      }
      else if  (Inf_Distance<=1600&&Inf_Distance>800){
          D_temp=133333.3/Inf_Distance-Inf_Distance/15-170;
       Float2Char(D_temp,Speek_F) ;
       SpeechT("[m55]距离前方障碍物",Speek_F,"厘米");     
      }
      else if (Inf_Distance<=800){
           Speech("[m55]距离前方障碍物大于50厘米");
      }
    
        break;
    case 'Y':
        Speech("[m55]是的");
    
           Steer_Cmd_Flag=1;
        break;
    case 'N':
        Speech("[m55]没有");   
           Steer_Cmd_Flag=3;
        break;    
    case 'L':
        Speech("[m55]哈[=ha4]哈[=ha3]哈[=ha3]");
     Steer_Cmd_Flag=5;
        break;
    case 'S'://摇头
       if(Shake_Heak_Flag==0){
        Shake_Heak_Flag=1;
        SteerConerolOut2=99;
        SteerConerolOut1=79;
      }
        else Shake_Heak_Flag=0;
        break;

    case 'R':
        Speech("[m55]然后呢[=ne5]");
         
        break;
    case 'O':
        Speech("[m55]欢迎光临！");
       
        break;
    case 'C'://读诗
        Speech("[s0][m55]锄禾日当午，汗滴禾下土。谁知盘中餐，粒粒皆辛苦");
        break;
    }
    
      
     
     
    
        if(Car_Cmd==0&&Steer_Cmd_Flag==0&&Shake_Heak_Flag==0){
          
          if(!Dance2_Flag)
        Speed_Need=(Msg.Data[3])*100;
        
        if(Turn_90_L_Flag==0&&Dance3_Flag==0)
        Turn_Need=(Msg.Data[4])*10;
        
        
        SteerConerolOut2=Msg.Data[5]+24+50;
        SteerConerolOut1=Msg.Data[6]+30+50;
        }
        else if(Car_Cmd==1){
          SteerConerolOut1=79;
        }
        
      }
    }
Pre_MY_counter= MY_counter; 
 
   ////////////////////////////////////////////////
    if(Dance3_Flag){
      
      
         if(yaw-pre_yaw<-300){
    //    Temp_Center=*Center-360;
     yaw_Temp-=360;
      }
      else if(yaw-pre_yaw>300){
      //  Temp_Center=*Center+360;
      yaw_Temp+=360;
      }  
      
      if((yaw-yaw_Temp>=0&&yaw-yaw_Temp<=90)||(yaw-yaw_Temp<360&&yaw-yaw_Temp>=270)){
          SteerConerolOut2 =74-(yaw-yaw_Temp)/2;
      }
      else if(yaw-yaw_Temp>90&&yaw-yaw_Temp<270){
        SteerConerolOut2=124;
      }
      else if(yaw-yaw_Temp>360){
        Dance3_Flag=0;
      }
      
       pre_yaw=yaw;
      
    }
    
    
    
    
    if(Dance2_Flag){
      if(Dance2_Flag==3&&Dance1_Flag==0){
        Dance1_Flag=4;
        Dance2_Flag--;
        Speed_Need=800;
      }
      else if(Dance2_Flag==2&&Dance1_Flag==0){
        Dance1_Flag=4;
        Dance2_Flag--;
        Speed_Need=-800;
      }
      else if(Dance2_Flag==1&&Dance1_Flag==0){//完成
        Dance2_Flag--;
        
      }
    }
    
    if(Dance1_Flag){
      if(Dance1_Flag==4&&Lett_Right_Flag==0){
        Turn_90_L_Flag=-90;//右转90°
        Lett_Right_Flag=2;
        Dance1_Flag--;
      }
      else if(Dance1_Flag==3&&Lett_Right_Flag==0){
        Turn_90_L_Flag=90;//左转180°
         Lett_Right_Flag=1;
       Dance1_Flag--;
      }
      else if(Dance1_Flag==2&&Lett_Right_Flag==0){
       Turn_90_L_Flag=0;//右转90°
        Lett_Right_Flag=2;
        Dance1_Flag--;
   //     yaw_Temp+=90;
   //     if(yaw_Temp>180) yaw_Temp-=360;
       
      }
       else if(Dance1_Flag==1&&Lett_Right_Flag==0){
         Dance1_Flag--;//结束
          if( yaw_Temp>180){
          yaw_Temp-=360;
        }
        else if(yaw_Temp<-180){
           yaw_Temp+=360;
        }
       }
    }
     
    Car_Turn_Deg(&Lett_Right_Flag,&Turn_90_L_Flag,&yaw_Temp);
    
  
    /*
    
    if(Turn_90_L_Flag>0){
      Turn_Need=-200;
      
      if(yaw-pre_yaw<-300){
        yaw_Temp-=360;
      }
      else if(yaw-pre_yaw>300){
        yaw_Temp +=360;
      }
      if(yaw-yaw_Temp>90){
        Turn_90_L_Flag--;
      }
      if(Dance1_Flag){
       SteerConerolOut2 =79-(yaw-yaw_Temp)/2;
        
      }
      
    }
   else if (Turn_90_L_Flag<0){//右转
      Turn_Need=200;
      
      if(yaw-pre_yaw<-300){
        yaw_Temp-=360;
      }
      else if(yaw-pre_yaw>300){
        yaw_Temp +=360;
      }
      if(yaw-yaw_Temp<-90){
        Turn_90_L_Flag++;
      }
       if(Dance1_Flag){
       SteerConerolOut2 =79-(yaw-yaw_Temp)/2;
        
      }
    }
    */
    /*
   else if(Turn_Rest_Flag1){
     float temp;
     temp=yaw-yaw_Org;
     if(temp<180&&temp>-180){
       if(temp>0){
        //左转|temp°|
       }
       else{
         //右转|temp°|
       }
     }
     else{
       if(temp>0){
         //右转|temp-360|
       }
       else{
         //左转|temp+360|
       }
       
     }

     
   }
   */
 
    
    
   //Speed_L=HAL_TIM_ReadCapturedValue(&TimHandleT3, TIM_CHANNEL_1);//编码器读取
  //  Speed_R=HAL_TIM_ReadCapturedValue(&TimHandleT4, TIM_CHANNEL_1);//编码器读取
   
//Motor_Control_1(temp);

  //temp2=TIM9->CNT;
  //temp = GPIOC->IDR ;
  // HAL_TIM_PWM_Pulse(&TimHandleT3, TIM_CHANNEL_1,temp2);
//  printf("\n\r UART Printf Example: retarget the C library printf function to the UART\n\r");
// HAL_UART_Transmit(&UartHandle2,"aaa",3,100);
  
  
 //  OutData[0]=ACC_Value_Raw.AXIS_X;
 //  OutData[1]=GYR_Value_Raw.AXIS_X;
 //  OutData[2]=GYR_Value_Raw.AXIS_Y;
 //  OutData[3] =GYR_Value_Raw.AXIS_Z;      
   
//OutData[0]=  Pre_Speed_L;
 // OutData[1]=  Speed_L;
 //   OutData[0]=pitch;
 //   OutData[1]=g_fAngleControlOut;
  //  OutData[2]=g_fAngleSpeed;
 //  OutData[0]=g_fCarAngle*100;
 // OutData[0]=-pitch;
 // OutData[1]=ACC_Value_Raw.AXIS_X/1.82;
//OutData[2]=-10*g_fGyroscopeAngleSpeed;
//OutData[3]=GYR_Value_Raw.AXIS_Y;
 //  OutData[3]=-g_fCarAngle;
 //  OutData[3] += OutData[1]- OutData[2];
 //        OutPut_Data(); 
   //      if(ACC_Value_Raw.AXIS_X>0){
   //        Motor_Control_1(300);
   //        Motor_Control_2(300);
   //      }
   //      else{
   //         Motor_Control_1(-300);
   //        Motor_Control_2(-300);
   //      }

  }
}

/**
  * @brief  Input Capture callback in non blocking mode 
  * @param  htim: TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    /* Get the Input Capture value */
    uwIC2Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    
    if (uwIC2Value != 0)
    {
      /* Duty cycle computation */
      uwDutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1)) * 100) / uwIC2Value;
      
      /* uwFrequency computation
      TIM4 counter clock = (RCC_Clocks.HCLK_Frequency) */      
      uwFrequency = (HAL_RCC_GetHCLKFreq()) / uwIC2Value;
    }
    else
    {
      uwDutyCycle = 0;
      uwFrequency = 0;
    }
  }
}
int32_t Capture_Start=0;
int32_t Capture_End=0;
int32_t Capture_Value=0;
unsigned char Capture_Flag=0;

/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //if(GPIO_Pin == GPIO_PIN_12&&Ultrasonic_Ready_Flag)
  if(GPIO_Pin == GPIO_PIN_12)
  {
    
    if( (GPIOA->IDR & GPIO_PIN_12)==GPIO_PIN_12){
//    BSP_LED_Toggle(LED2);
   Capture_Start=TIM9->CNT;
   Capture_Flag=1;
   
     if(Ultrasonic_OK_Flag==0){
        Speech("[m55]接入超声波");
        Ultrasonic_OK_Flag=1;
      
    }
    
    if(Ultrasonic_OK_Flag>100)Ultrasonic_OK_Flag=1;
Ultrasonic_OK_Flag++;
    }
   else {
      if(Capture_Flag){
        Capture_End=TIM9->CNT;
        Capture_Value=Capture_End-Capture_Start;
        if(Capture_Value<-20000){
          Capture_Value+=65535;
        }
         Capture_Flag=0;
        if(Capture_Value>600)Capture_Value=600;
          Ult_Distance*=0.5;
    Ult_Distance+=Capture_Value*0.5;
  Ultrasonic_Ready_Flag=0;
  
      }
      else {
        Capture_Flag=0;
        
      }
    }
  }
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   // BSP_IMU_6AXES_X_GetAxesRaw((AxesRaw_TypeDef *)&ACC_Value_Raw);
    //BSP_IMU_6AXES_G_GetAxesRaw((AxesRaw_TypeDef *)&GYR_Value_Raw);
   // 
 //  pwmConfig.Pulse=temp++;
   //if(temp>=1000)temp=0;
  // temp=250;
  // HAL_TIM_PWM_Pulse(&TimHandleT3, TIM_CHANNEL_1,temp2);
  // HAL_TIM_PWM_ConfigChannel(&TimHandle, &pwmConfig, TIM_CHANNEL_1);
  //BSP_LED_Toggle(LED2);
  
  Time_Start=TIM9->CNT;
  Time_Long=Time_Start-Time_Start_Pre;
  Time_Start_Pre=Time_Start;
 AHRS_Update();
  Get_Inf_Distance();
 Time_Cnt++;
 Time_Cnt2++;
 Time_Cnt3++;
  
 if(Car_Cmd3==2) Dange_Detct();
 
 
 
   if(Time_Cnt>20){
     Time_Cnt=0;
  Get_Speed();
  if(Car_Cmd)Block_Detct();
  
  
  SpeedControl();
  BSP_LED_Toggle(LED2);
  if(Ultrasonic_OK_Flag){
   //    Ultrasonic_Ready_Flag=1;
    
         Ultrasonic_May_Break++;
          if(Ultrasonic_May_Break>5){
            Ultrasonic_May_Break=0;
            if(Ultrasonic_OK_Flag== Ultrasonic_OK_Flag_Pre){
            Speech("[m55]断开超声波");
          Ultrasonic_OK_Flag=0;
          Ultrasonic_OK_Flag_Pre=0;
          
          }
          Ultrasonic_OK_Flag_Pre=Ultrasonic_OK_Flag;
          }
       /*
   if( (GPIOA->IDR & GPIO_PIN_5)==GPIO_PIN_5){ 
    
     if(Ultrasonic_OK_Flag== Ultrasonic_OK_Flag_Pre){
       Ultrasonic_May_Break++;
       if(Ultrasonic_May_Break>5){
       
      Speech("[m55]断开超声波");
     Ultrasonic_OK_Flag=0;
     Ultrasonic_OK_Flag_Pre=0;
     Ultrasonic_May_Break=0;
       }
     }
      Ultrasonic_OK_Flag_Pre=Ultrasonic_OK_Flag;
         
   }
   */ 
  }
  //if(Car_Cmd==1)
  //Turn_Control();
  
     
  
   }
   /*
   if(SteerConerolOut2<100&&Turn_Flag==1)
  SteerConerolOut2+=0.5;
  else if(SteerConerolOut2>60&&Turn_Flag==0)
     SteerConerolOut2-=0.5;
  else if(SteerConerolOut2==100)
    Turn_Flag=0;
  else if(SteerConerolOut2==60)
     Turn_Flag=1;
Steer_Control_2((int)SteerConerolOut2);
*/
   if(Time_Cnt2>3){
   
  
     if(Car_Cmd==1){
      Steer_Control();
     }
Steer_Output();
   Time_Cnt2=0;
   }
    if(Time_Cnt3>10){ 
      Steer_Cmd_XY(&Steer_Cmd_Flag);
       Shake_Heak( &Shake_Heak_Flag);  
        Time_Cnt3=0;
    }
 Time_Long2=(TIM9->CNT)-Time_Start;
Get_Inf_Distance();
    
if(Car_Cmd4&&!Car_Cmd3&&!Car_Cmd){
  Follow_Me();
  
}
//if(NRF24L01_OK_Flag==1){
 //Nrf_Check_Event();//nrf24l01相关操作
//}
 
 // OutData[0]=Inf_Distance;
    // OutData[0]=pitch*1000;
 //OutData[0]=g_fCarAngle*1000;
  //    OutData[1]=GYR_Value_Raw.AXIS_Y;
 //OutData[0]=ACC_Value_Raw.AXIS_Y/100;
 //  OutData[1]=ACC_Value_Raw.AXIS_X/100;
 //    OutData[2]=GYR_Value_Raw.AXIS_Z/100;
  //    OutData[3]=GYR_Value_Raw.AXIS_X/100;
  
     OutData[0]= Turn_Need/10;
     OutData[1]= Speed_Need/100;
     OutData[2]= SteerConerolOut2;
     OutData[3]= SteerConerolOut1;
  //    OutData[1]=MAG_Value_Raw.AXIS_Y;
  //    OutData[2]=MAG_Value_Raw.AXIS_Z;
   ///   OutData[3]=MAG_Value_Raw.AXIS_X;      
      
     //    OutPut_Data(); 
         
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED2 (GREEN) stays on */
  BSP_LED_On(LED2);
  while(1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            PLL_M                          = 16
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale2 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
