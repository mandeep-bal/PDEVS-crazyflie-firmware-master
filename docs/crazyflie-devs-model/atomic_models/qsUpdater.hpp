/**
* Cristina Ruiz Martin
* ARSLab - Carleton University
*
* qsUpdater Model:
* This model calculates the values for the quaternion using the one from mahony or the one from madwick and resends the gyro x, y, z values
* CrazyFlie controller.
*/

#ifndef BOOST_SIMULATION_PDEVS_QS_UPDATER_H
#define BOOST_SIMULATION_PDEVS_QS_UPDATER_H
#include <assert.h>
#include <math.h>
#include <memory>
#include <vector>
#include <iostream>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"
#include "../vendor/britime.hpp"

#define MINIMUM_TIME_FOR_QS_UPDATER BRITime(1,1000000);

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

enum class QuaternionType {MAHONY, MADWICK};
/**
 * @class qsUpdater calculates the values from the quaternion (q0,q1,q2,q3) using Mahony or Madwick based on the model instantation
 * and the values received (accx,accy,accz,gyrox,gyroy,gyroz).
 * When this calculation is made, it sends as output the quaternion (q0,q1,q2,q3) and fordware the values gyrox gyroy, gyroz
 */
template<class TIME, class MSG>

class qsUpdater : public pdevs::atomic<TIME, MSG>{
private:

	TIME   next_internal;
  QuaternionType type;
	
	float q0;
	float q1;
	float q2;
	float q3;
	float dt;
	float gx; 
	float gy; 
	float gz;

  //MAHONY

  #define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
  #define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
  
  float twoKp;    // 2 * proportional gain (Kp)
  float twoKi;    // 2 * integral gain (Ki)
  float integralFBx;
  float integralFBy;
  float integralFBz;  // integral error terms scaled by Ki

  //MADWICK

  #define BETA_DEF     0.01f    // 2 * proportional gain
  
  float beta;     // 2 * proportional gain (Kp)
	
public:

  /**
   * @constructor
   * Initiales the model passivated, with all variables equal creo and the type of quaternio to be used.
   * @param deltat - time step to update the quaternio. I has to match with the sensor pulling time
   * @param  t - type of quaternion used - MAHONY or MADWICK
   */

  explicit qsUpdater(float deltat, QuaternionType t) noexcept {

  twoKp          = TWO_KP_DEF;    // 2 * proportional gain (Kp)
  twoKi          = TWO_KI_DEF;    // 2 * integral gain (Ki)
  integralFBx    = 0.0f;
  integralFBy    = 0.0f;
  integralFBz    = 0.0f;  // integral error terms scaled by Ki

  beta           = BETA_DEF;     // 2 * proportional gain (Kp)

  type           = t; 
	q0             = 1.0f;
	q1             = 0.0f;
	q2             = 0.0f;
	q3             = 0.0f;
	dt             = deltat;
  next_internal  = pdevs::atomic<TIME, MSG>::infinity;

  }

  /**
   * Passivates the model
   */

  void internal() noexcept {

  	next_internal = pdevs::atomic<TIME, MSG>::infinity;

  }

  /**
   * Return the next time advanced calculated in the internal function or in the external funtion
   */

  TIME advance() const noexcept {

  	return next_internal;
  }

  /**
   * Calculates the output of the model.
   * @return a messages with the following structure:
   * {q0, q1,q2, q3, gyro x, gyro y, gyro z}
   */

  vector<MSG> out() const noexcept {

  	
  	vector<MSG> output;
    MSG qs_output(MsgType::QS, q0, q1, q2, q3, gx, gy, gz);

    output.push_back(qs_output);
    return output;

  }

  /**
   * Calculates the values of the quaternion
   * @param  - a message with the following structure
   * {MsgType::SENSOR_DATA_QS,accx,accy,accz,gyrox, gyroy, gyroz}
   */

  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {
	  assert((next_internal == pdevs::atomic<TIME, MSG>::infinity)); // Wrong time to receive message.
	  assert((mb.size() == 1)); // Wrong message bag size. Should be one.
	  assert((mb.front().type == MsgType::SENSOR_DATA_QS) || (mb.front().type == MsgType::SENSOR_DATA_CONTROLLER) ); // Wrong message type.

    if(mb.front().type == MsgType::SENSOR_DATA_QS){

      SensorData msg = mb.front().sensorData;
      gx = msg.gyro_x;
      gy = msg.gyro_y;
      gz = msg.gyro_z;

      vector<float> values;
      values.clear();
  
      if(type == QuaternionType::MAHONY){
        values = mahonyCalculation(msg.acc_x, msg.acc_y, msg.acc_z, gx, gy, gz, dt, integralFBx, integralFBy, integralFBz, twoKi, twoKp, q0, q1, q2, q3);
        integralFBx = values[0];
        integralFBy = values[1];
        integralFBz = values[2];
        q0 = values[3];
        q1 = values[4];
        q2 = values[5];
        q3 = values[6];
      }else if(type == QuaternionType::MADWICK){
        values = madwickCalculation(msg.acc_x, msg.acc_y, msg.acc_z, gx, gy, gz, dt, beta, q0, q1, q2, q3);
        q0 = values[0];
        q1 = values[1];
        q2 = values[2];
        q3 = values[3];
            
      }else{
        assert(false && "Wrong Quaternion type");
      }    
      
    	next_internal = MINIMUM_TIME_FOR_QS_UPDATER; //Advance time of the model;
    }
    
  }

   /**
   * There is not possible confluence in the model design. 
  */

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {

  	assert(false && "Non posible confluence function in this model");
  }

  /***************************************
  ********* helper functions *************
  ***************************************/

  /**
   * Calculates the inverse of the sqrt
   * @param x - a float number 
   * @return - the inverse of the sqrt of the parameter
  */

  float invSqrt(float x) {
  	float halfx = 0.5f * x;
  	float y = x;
  	long i = *(long*)&y;
  	i = 0x5f3759df - (i>>1);
  	y = *(float*)&i;
  	y = y * (1.5f - (halfx * y * y));
  	return y;
	}

  /**
   * Calculates the values of the quaternion and updates the integral limits following the mahony one
   * @param ax1 - accelerometer x
   * @param ay1 - accelerometer y
   * @param az1 - accelerometer z
   * @param gx1 - accelerometer x
   * @param gy1 - accelerometer y
   * @param gz1 - accelerometer z
   * @param dt1 - time interval for the calculation
   * @param integralFBx1 - integral limit for axis x
   * @param integralFBy1 - integral limit for axis y
   * @param integralFBz1 - integral limit for axis z
   * @param twoKi - two times the ki value
   * @param twoKp - two times the kp value
   * @param q0i - previous value of the 0 component of the quaternio
   * @param q1i - previous value of the 1 component of the quaternio
   * @param q2i - previous value of the 2 component of the quaternio
   * @param q3i - previous value of the 3 component of the quaternio
   * @return - a vector with the new intregal limits and the values of the quaternion
  */

  vector<float> mahonyCalculation(float ax1, float ay1, float az1, float gx1, float gy1, float gz1,float dt1, float integralFBx1, float integralFBy1, float integralFBz1, float twoKi, float twoKp, float q0i, float q1i, float q2i, float q3i){

    using namespace std;

    float ax = ax1;
    float ay = ay1;
    float az = az1;
    float gx = gx1;
    float gy = gy1;
    float gz = gz1;
    float integralFBx = integralFBx1;
    float integralFBy = integralFBy1;
    float integralFBz = integralFBz1;
    float q0 = q0i;
    float q1 = q1i;
    float q2 = q2i;
    float q3 = q3i;
    float dt = dt1;

    std::vector<float> v;
    v.clear();
    
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
  
    gx = gx * M_PI / 180;
    gy = gy * M_PI / 180;
    gz = gz * M_PI / 180;
  
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))){
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
        integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
        integralFBy += twoKi * halfey * dt;
        integralFBz += twoKi * halfez * dt;
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
      }else {
        integralFBx = 0.0f; // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
      }
  
      // Apply proportional feedback
      gx += twoKp * halfex;
      gy += twoKp * halfey;
      gz += twoKp * halfez;
    }
  
    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);   // pre-multiply common factors
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
  
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  
  
    v.push_back(integralFBx);
    v.push_back(integralFBy);
    v.push_back(integralFBz);
    v.push_back(q0);
    v.push_back(q1);
    v.push_back(q2);
    v.push_back(q3);

    return v;
  }
/**
   * Calculates the values of the quaternion following the madwick one
   * @param ax1 - accelerometer x
   * @param ay1 - accelerometer y
   * @param az1 - accelerometer z
   * @param gx1 - accelerometer x
   * @param gy1 - accelerometer y
   * @param gz1 - accelerometer z
   * @param dt1 - time interval for the calculation
   * @param beta - two times the proportinal gain x
   * @param q0i - previous value of the 0 component of the quaternio
   * @param q1i - previous value of the 1 component of the quaternio
   * @param q2i - previous value of the 2 component of the quaternio
   * @param q3i - previous value of the 3 component of the quaternio
   * @return - a vector with the new values of the quaternion
  */

vector<float> madwickCalculation(float ax1, float ay1, float az1, float gx1, float gy1, float gz1,float dt1, float beta, float q0i, float q1i, float q2i, float q3i){

  float ax = ax1;
  float ay = ay1;
  float az = az1;
  float gx = gx1;
  float gy = gy1;
  float gz = gz1;
  float q0 = q0i;
  float q1 = q1i;
  float q2 = q2i;
  float q3 = q3i;
  float dt = dt1;

  std::vector<float> v;
  v.clear();

  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  
  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalise accelerometer measurement
      recipNorm = invSqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;
      // Auxiliary variables to avoid repeated arithmetic
      _2q0 = 2.0f * q0;
      _2q1 = 2.0f * q1;
      _2q2 = 2.0f * q2;
      _2q3 = 2.0f * q3;
      _4q0 = 4.0f * q0;
      _4q1 = 4.0f * q1;
      _4q2 = 4.0f * q2;
      _8q1 = 8.0f * q1;
      _8q2 = 8.0f * q2;
      q0q0 = q0 * q0;
      q1q1 = q1 * q1;
      q2q2 = q2 * q2;
      q3q3 = q3 * q3; 
      // Gradient decent algorithm corrective step
      s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
      recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
      s0 *= recipNorm;
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;  
      // Apply feedback step
      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt; 
    // Normalise quaternion
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    
    v.push_back(q0);
    v.push_back(q1);
    v.push_back(q2);
    v.push_back(q3);

    return v;
}


};

#endif // BOOST_SIMULATION_PDEVS_QS_UPDATER_H
