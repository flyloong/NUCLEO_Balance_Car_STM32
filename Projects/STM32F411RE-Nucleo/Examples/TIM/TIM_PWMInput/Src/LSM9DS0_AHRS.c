//
//  LSM9SDS0_AHRS.cpp
//  LSM9SDS0 AHRS
//
//  Created by Nicholas Robinson on 04/19/14.
//  Copyright (c) 2014 Nicholas Robinson. All rights reserved.
//

#include "LSM9DS0_AHRS.h"
 float pitch, yaw, roll;
void LSM9DS0_AHRS( void)
{
	//marg = initializeMarg();
	
	
	
	dt = 5 / 1000.0f;

	
	q[0] = 1.0f;
	q[1] = 0.0f;
	q[2] = 0.0f;
	q[3] = 0.0f;
	
	mahonyErrors[0] = 0.0f;
	mahonyErrors[1] = 0.0f;
	mahonyErrors[2] = 0.0f;
}
#define sampleFreq 400
float invSqrt(float x);
volatile float twoKp = 1;// 2 * proportional gain (Kp)
volatile float twoKi = 0.002;// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;// integral error terms scaled by Ki
//void LSM9DS0_AHRS_update(LSM9DS0_t* lsm_t)
//{
//    LSM9DS0_AHRS_readMarg(lsm_t);
//	LSM9DS0_AHRS_updateQuaternions();
//	LSM9DS0_AHRS_updateEulerAngles(lsm_t);
	//tick();
//}

//LSM9DS0* LSM9DS0_AHRS_initializeMarg()
//{
  /*
	marg = new LSM9DS0(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
	
	marg->begin();
	
	marg->setAccelScale(marg->A_SCALE_2G);
	marg->setAccelODR(marg->A_ODR_100);
	marg->setAccelABW(marg->A_ABW_50);
	
	marg->setGyroScale(marg->G_SCALE_245DPS);
	marg->setGyroODR(marg->G_ODR_190_BW_125);
	
	marg->setMagScale(marg->M_SCALE_2GS);
	marg->setMagODR(marg->M_ODR_125);
	
	return marg;
  */
//}
/*
void LSM9DS0_AHRS_readMarg(LSM9DS0_t* lsm_t)
{
   LSM9DS0_readMag(lsm_t);
  LSM9DS0_readAccel(lsm_t);
LSM9DS0_readGyro(lsm_t);
 //  mx=lsm_t->mx;
   mx=calcMag(lsm_t,lsm_t->mx);
   my=calcMag(lsm_t,lsm_t->my);
   mz=calcMag(lsm_t,lsm_t->mz);
   gx=calcMag(lsm_t,lsm_t->gx);
   gy=calcMag(lsm_t,lsm_t->gy);
   gz=calcMag(lsm_t,lsm_t->gz);
   gx=gx * PI / 180.0f;
   gy=gy * PI / 180.0f;
   gz=gz * PI / 180.0f;
   ax=calcAccel(lsm_t,lsm_t->ax);
   ay=calcAccel(lsm_t,lsm_t->ay);
   az=calcAccel(lsm_t,lsm_t->az);
  
}
*/
 

void LSM9DS0_AHRS_updateEulerAngles(void)
{
	yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	yaw *= 180.0f / PI ;
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	pitch *= 180.0f / PI;
	roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	roll *= 180.0f / PI;
}

//void LSM9DS0_AHRS_tick()
//{
	//uint16_t now = micros();
	//dt = (now - lastUpdate) / 1000000.0f;
	//lastUpdate = now;
//}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void madgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;
	q4 += qDot4 * dt;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}
  
// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones. 
void mahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;

  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;   

  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  

  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
    mahonyErrors[0] += ex;      // accumulate integral error
    mahonyErrors[1] += ey;
    mahonyErrors[2] += ez;
  }
  else
  {
    mahonyErrors[0] = 0.0f;     // prevent integral wind up
    mahonyErrors[1] = 0.0f;
    mahonyErrors[2] = 0.0f;
  }

  // Apply feedback terms
  gx = gx + Kp * ex + Ki * mahonyErrors[0];
  gy = gy + Kp * ey + Ki * mahonyErrors[1];
  gz = gz + Kp * ez + Ki * mahonyErrors[2];

  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * dt);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * dt);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * dt);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * dt);

  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;
}

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; 
float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; 
float hx, hy, bx, bz;
float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
float halfex, halfey, halfez;
float qa, qb, qc;
 
// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
 
MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
return;
 
}
 
// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 

// Normalise accelerometer measurement
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;    
 
// Normalise magnetometer measurement
recipNorm = invSqrt(mx * mx + my * my + mz * mz);
mx *= recipNorm;
my *= recipNorm;
mz *= recipNorm;  
 
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;  
 
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
 
// Estimated direction of gravity and magnetic field
halfvx = q1q3 - q0q2;
halfvy = q0q1 + q2q3;
halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2); 
 
// Error is sum of cross product between estimated direction and measured direction of field vectors
halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
 
// Compute and apply integral feedback if enabled
if(twoKi > 0.0f) {
 
integralFBx += twoKi * halfex * (1.0f / sampleFreq);// integral error scaled by Ki
integralFBy += twoKi * halfey * (1.0f / sampleFreq);
integralFBz += twoKi * halfez * (1.0f / sampleFreq);
gx += integralFBx;// apply integral feedback
gy += integralFBy;
gz += integralFBz;
 
}
else {
 
integralFBx = 0.0f;// prevent integral windup
integralFBy = 0.0f;
integralFBz = 0.0f;
 
}
 
// Apply proportional feedback
gx += twoKp * halfex;
gy += twoKp * halfey;
gz += twoKp * halfez;
 
}
 
// Integrate rate of change of quaternion
gx *= (0.5f * (1.0f / sampleFreq));// pre-multiply common factors
gy *= (0.5f * (1.0f / sampleFreq));
gz *= (0.5f * (1.0f / sampleFreq));
qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz);
q1 += (qa * gx + qc * gz - q3 * gy);
q2 += (qa * gy - qb * gz + q3 * gx);
q3 += (qa * gz + qb * gy - qc * gx);
 
// Normalise quaternion
recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
 q[0] =q0 * recipNorm;
 q[1] =q1 * recipNorm;
 q[2] =q2 * recipNorm;
 q[3] =q3 * recipNorm;
 
}
 
//---------------------------------------------------------------------------------------------------
// IMU algorithm update
 
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3]; 
float recipNorm;
float halfvx, halfvy, halfvz;
float halfex, halfey, halfez;
float qa, qb, qc;
 
// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
 
 
// Normalise accelerometer measurement
recipNorm = invSqrt(ax * ax + ay * ay + az * az);
ax *= recipNorm;
ay *= recipNorm;
az *= recipNorm;       
 
// Estimated direction of gravity and vector perpendicular to magnetic flux
halfvx = q1 * q3 - q0 * q2;
halfvy = q0 * q1 + q2 * q3;
halfvz = q0 * q0 - 0.5f + q3 * q3;
 
// Error is sum of cross product between estimated and measured direction of gravity
halfex = (ay * halfvz - az * halfvy);
halfey = (az * halfvx - ax * halfvz);
halfez = (ax * halfvy - ay * halfvx);
 
// Compute and apply integral feedback if enabled
if(twoKi > 0.0f) {
 
integralFBx += twoKi * halfex * (1.0f / sampleFreq);// integral error scaled by Ki
integralFBy += twoKi * halfey * (1.0f / sampleFreq);
integralFBz += twoKi * halfez * (1.0f / sampleFreq);
gx += integralFBx;// apply integral feedback
gy += integralFBy;
gz += integralFBz;
 
}
else {
 
integralFBx = 0.0f;// prevent integral windup
integralFBy = 0.0f;
integralFBz = 0.0f;
 
}
 
// Apply proportional feedback
gx += twoKp * halfex;
gy += twoKp * halfey;
gz += twoKp * halfez;
 
}
 
// Integrate rate of change of quaternion
gx *= (0.5f * (1.0f / sampleFreq));// pre-multiply common factors
gy *= (0.5f * (1.0f / sampleFreq));
gz *= (0.5f * (1.0f / sampleFreq));
qa = q0;
qb = q1;
qc = q2;
q0 += (-qb * gx - qc * gy - q3 * gz);
q1 += (qa * gx + qc * gz - q3 * gy);
q2 += (qa * gy - qb * gz + q3 * gx);
q3 += (qa * gz + qb * gy - qc * gx);
 
// Normalise quaternion
recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
 q[0] =q0 * recipNorm;
 q[1] =q1 * recipNorm;
 q[2] =q2 * recipNorm;
 q[3] =q3 * recipNorm;
 
}
float invSqrt(float x) {
 
float halfx = 0.5f * x;
float y = x;
long i = *(long*)&y;
i = 0x5f3759df - (i>>1);
y = *(float*)&i;
y = y * (1.5f - (halfx * y * y));
return y;
 
}
