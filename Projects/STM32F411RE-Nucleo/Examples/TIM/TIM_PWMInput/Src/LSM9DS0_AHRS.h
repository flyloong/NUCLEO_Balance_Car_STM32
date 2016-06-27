//
//  LSM9SDS0_AHRS.h
//  LSM9SDS0 AHRS
//
//  Created by Nicholas Robinson on 04/19/14.
//  Copyright (c) 2014 Nicholas Robinson. All rights reserved.
//

#ifndef __LSM9DS0_AHRS_H__
#define __LSM9DS0_AHRS_H__
#include "stm32f4xx_hal.h"
#include "math.h"


//#if defined(ARDUINO) && ARDUINO >= 100
//  #include "Arduino.h"
//#else
//  #include "WProgram.h"
//  #include "pins_arduino.h"
//#endif

#define LSM9DS0_XM  0x1D
#define LSM9DS0_G   0x6B

// Madgwick Constants
#define GyroMeasError PI * (10.0f / 180.0f)
#define beta sqrt(3.0f / 4.0f) * GyroMeasError

// Mahony Constants
//#define Kp 2.0f * 5.0f
//#define Ki 0.0f
#define Kp 2.0f 
#define Ki 0.000f
#define PI 3.1416f



	    
		void LSM9DS0_AHRS( void);
		
		
		void LSM9DS0_AHRS_updateEulerAngles(void);
		static float q[4];
		static float dt;
		void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
		void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
                void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
                void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

                
		void LSM9DS0_AHRS_update();
		//LSM9DS0* marg;
		
		static float mahonyErrors[3];
		static uint16_t lastUpdate;

#endif // __LSM9DS0_AHRS_H__