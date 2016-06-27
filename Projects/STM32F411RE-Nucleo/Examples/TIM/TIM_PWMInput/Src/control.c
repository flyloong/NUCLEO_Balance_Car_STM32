

#include "control.h"
#include "stm32f4xx_hal.h"
#include "x_nucleo_iks01a1.h"
extern float gyrof[3];  /* 陀螺仪读数*/
extern float accelf[3]; /* 加速度值*/
extern float yprf[3];   /* Euler 角度*/

float car_angle_set=-1;
float g_fCarAngle=0;
float g_fGyroscopeAngleSpeed=0;;
float g_fAngleSpeed=0;
float g_fAngleSpeed_Last=0;
float e_CarAngle_Last=0;
float	g_fAngleControlOut=0;
float  SteerConerolOut1=79;
float  SteerConerolOut2=74;
float g_fSpeedControlOut=0;
float pre_g_fCarAngle=0;

float g_fAngleControlP=110;
float g_fAngleControlI=0;
float g_fAngleI=0;
float g_fAngleControlD=20;
float g_fSpeedControlP=0.4;
float g_fSpeedControlI=1;
float g_fSpeedControlIntegral=0;
#define SPEED_CONTROL_OUT_MAX 150
#define SPEED_CONTROL_OUT_MIN -150
float Ult_Distance=0;
extern  uint32_t Inf_Distance;
float Gyro_Y_Offset=-25;
extern AxesRaw_TypeDef GYR_Value_Raw;
extern  float pitch;
extern  int32_t Speed_A;
extern float Speed_Need;
extern float Turn_Need;
extern int32_t Capture_Value;
extern int Left_Right[2];
int      g_fSpeed_ControlPeriod=0;
float g_fSpeed_ControlOutOld=0;
float g_fSpeedControlOutNew=0;
float g_fSpeedControlOutValue=0;
//------------------------------------------------------------------------------
void AngleControl(void) 
{
	float fValue,e_CarAngle; 
//		if(yprf[0]<0)
//g_fCarAngle=yprf[0]+180;
//	else g_fCarAngle=yprf[0]-180;
         g_fCarAngle=pitch;
		//			g_fAngleSpeed=(pre_g_fCarAngle-g_fCarAngle);
	g_fAngleSpeed=-(GYR_Value_Raw.AXIS_Y-Gyro_Y_Offset)/1000;
        // g_fAngleSpeed*=0.5;
   // g_fAngleSpeed+=g_fAngleSpeed_Last*0.5;
		//g_fAngleSpeed=-gyrof[0]/16384;
		pre_g_fCarAngle=g_fCarAngle;
		e_CarAngle=(car_angle_set - g_fCarAngle);
            g_fAngleI+=e_CarAngle;
            if(g_fAngleI>1000)g_fAngleI=1000;
            else if(g_fAngleI<-1000)g_fAngleI=-1000;
                
    
                
                
                
   // e_CarAngle_Last*=0.5;
   // e_CarAngle_Last+=e_CarAngle*0.5;
   // e_CarAngle=e_CarAngle_Last;
		//	  if(e_CarAngle<-30||e_CarAngle>30) fValue = (e_CarAngle) * g_fAngleControlP *1.5+ g_fAngleSpeed * g_fAngleControlD*1.0; 
	//else if (e_CarAngle<-20||e_CarAngle>20) fValue = (e_CarAngle) * g_fAngleControlP*1.4 + g_fAngleSpeed * g_fAngleControlD*1.0; 
//else if (e_CarAngle<-10||e_CarAngle>10) fValue = (e_CarAngle) * g_fAngleControlP*1.3 + g_fAngleSpeed * g_fAngleControlD*1.00; 
	//else if (e_CarAngle<-8||e_CarAngle>8) fValue = (e_CarAngle) * g_fAngleControlP*1.2 + g_fAngleSpeed * g_fAngleControlD*1.00; 
	//else if(e_CarAngle<-5||e_CarAngle>5) fValue = (e_CarAngle) * g_fAngleControlP*1.1  + g_fAngleSpeed * g_fAngleControlD*1.00; 
//else 
            
	fValue = e_CarAngle * g_fAngleControlP + 
          g_fAngleI*0.001*g_fAngleControlI+
          g_fAngleSpeed * g_fAngleControlD+
          g_fSpeedControlOut; 
       
        
	if(fValue > 1000)		fValue = 1000;
	else if(fValue < -1000) 	fValue = -1000;
	g_fAngleControlOut = fValue;
}


void SpeedControl(void){
  float fP;
  fP=Speed_A+Speed_Need;
 g_fSpeedControlIntegral += fP/1000.0f;
	        
	if(g_fSpeedControlIntegral > SPEED_CONTROL_OUT_MAX)	
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MAX;
	if(g_fSpeedControlIntegral < SPEED_CONTROL_OUT_MIN)  	
		g_fSpeedControlIntegral = SPEED_CONTROL_OUT_MIN;   

g_fSpeed_ControlOutOld = g_fSpeedControlOutNew;
g_fSpeedControlOutNew =fP*g_fSpeedControlP+g_fSpeedControlIntegral*g_fSpeedControlI;
//        g_fSpeedControlOutNew =Speed_A*g_fSpeedControlP+g_fSpeedControlIntegral*g_fSpeedControlI;
g_fSpeedControlOutValue=g_fSpeedControlOutNew-g_fSpeed_ControlOutOld;	


 g_fSpeed_ControlPeriod =0;
   }

void SpeedControlOutput(void) 
{      
        g_fSpeed_ControlPeriod++;
	g_fSpeedControlOut = g_fSpeedControlOutValue * ((float)g_fSpeed_ControlPeriod)  / 20.0+ g_fSpeed_ControlOutOld;
   //     g_fSpeedControlOut = g_fSpeedControlOutValue + g_fSpeed_ControlOutOld;
        
}
void Turn_Control(void){
 if(Left_Right[0]==-1&&Left_Right[1]==-1){
     Speed_Need=-1000;
    Turn_Need=0;}
  if(Left_Right[0]==-1&&Left_Right[1]!=-1){
     Speed_Need=-1000;
    Turn_Need=100;}
   if(Left_Right[0]!=-1&&Left_Right[1]==-1){
     Speed_Need=-1000;
    Turn_Need=-100;}
   if(Left_Right[0]!=-1&&Left_Right[1]!=-1){
     Speed_Need=0;
   
     if(Left_Right[0]<Left_Right[1])
    Turn_Need=-100;
   else
   Turn_Need=100;
   }


 //   if(Inf_Distance<500){
 //   Speed_Need=-1000;
 //   Turn_Need=0;}
 // else if(Inf_Distance>1500){
 //     Speed_Need=0;
  //  Turn_Need=100;  
 // }
 /* 
  if(Ult_Distance>900){
    Speed_Need=-1000;
    Turn_Need=0;}
  else if(Ult_Distance<500){
      Speed_Need=0;
    Turn_Need=100;  
  }
  */
}


