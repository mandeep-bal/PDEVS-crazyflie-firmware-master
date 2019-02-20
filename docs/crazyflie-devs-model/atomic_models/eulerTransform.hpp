/**
* Cristina Ruiz Martin
* ARSLab - Carleton University
*
* Euler Transform Model:
* This model calculates the actual euler roll, pitch and yaw and resends the gyro x, y, z values
* CrazyFlie controller.
*/


#ifndef BOOST_SIMULATION_PDEVS_EULER_TRANSFORM_H
#define BOOST_SIMULATION_PDEVS_EULER_TRANSFORM_H
#include <math.h> 
#include <assert.h>
#include <memory>
#include <iomanip>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

#define MINIMUM_TIME_FOR_EULER_TRANSFORM BRITime(1,10000000);
/**
 * @class eulerTransform calculates the euler roll, pitch and yaw based on the values received (q0,q1,q2,q3,gyrox gyroy, gyroz).
 * When this calculation is made, it sends as output the euler roll, pitch and yaw and fordware the values gyrox gyroy, gyroz
 */
template<class TIME, class MSG>
class eulerTransform : public pdevs::atomic<TIME, MSG>{
private:

  TIME   next_internal;
  float  euler_roll;
  float  euler_pitch;
  float  euler_yaw;
  float  gyro_x;
  float  gyro_y;
  float  gyro_z;

public:

  /**
   * @constructor
   * Initiales the model passivated and with all variables equal creo. 
   */

  explicit eulerTransform() noexcept {

    euler_roll      = 0.0f;
    euler_pitch     = 0.0f;
    euler_yaw       = 0.0f;
    gyro_x          = 0.0f;
    gyro_y          = 0.0f;
    gyro_z          = 0.0f;
    next_internal   = pdevs::atomic<TIME, MSG>::infinity;
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
   * {euler roll, euler pitch, euler yaw, gyro x, gyro y, gyro z}
   */

  vector<MSG> out() const noexcept {

    vector<MSG> output;
    MSG euler_data_output(MsgType::EULER_DATA, euler_roll, euler_pitch, euler_yaw, gyro_x, gyro_y, gyro_z);

    output.push_back(euler_data_output);
    return output;
  }

  /**
   * Calculates the euler roll, pitch and yaw.
   * @param  - a message with the following structure
   * {MsgType::QS,q0,q1,q2,q3,gyrox gyroy, gyroz}
   */


  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {

    
    assert((next_internal == pdevs::atomic<TIME, MSG>::infinity)); // Wrong time to receive message.
    assert((mb.size() == 1)); // Wrong message bag size. Should be one.
    assert((mb.front().type == MsgType::QS)); // Wrong message type.

    Qs msg = mb.front().qs;

    float q0 = msg.q0;
    float q1 = msg.q1;
    float q2 = msg.q2;
    float q3 = msg.q3;
    gyro_x   = msg.gyro_x;
    gyro_y   = msg.gyro_y;
    gyro_z   = msg.gyro_z;

    
    float gx, gy, gz; // estimated gravity direction

    gx = 2 * (q1*q3 - q0*q2);
    gy = 2 * (q0*q1 + q2*q3);
    gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    if (gx>1) gx=1;
    if (gx<-1) gx=-1;

    //Calculate the roll, pitch and yaw based on the inputs and the previus calculations

    euler_yaw = atan2(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * 180 / M_PI;
    euler_pitch = asin(gx) * 180 / M_PI; 
    euler_roll = atan2(gy, gz) * 180 / M_PI;

    next_internal = MINIMUM_TIME_FOR_EULER_TRANSFORM; //Advance time of the model;
  }
  /**
   * There is not possible confluence in the model design. 
  */

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
    assert(false && "Non posible confluence function in this model");
  }
};

#endif // BOOST_SIMULATION_PDEVS_EULER_TRANSFORM_H