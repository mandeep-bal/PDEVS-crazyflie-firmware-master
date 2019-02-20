/**
* Cristina Ruiz Martin
* ARSLab - Carleton University
*
* sensorDistributor Model:
* Redirects the sensor information. On time for calculating the euler roll pitch and yaw and other time directly to the movement stabilizer.
* The model do this changind the message type.
* CrazyFlie controller.
*/

#ifndef BOOST_SIMULATION_PDEVS_SWICH_H
#define BOOST_SIMULATION_PDEVS_SWICH_H
#include <assert.h>
#include <memory>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"

using namespace boost::simulation::pdevs;
using namespace boost::simulation; 
using namespace std;

enum class State {SENSOR_DATA_QS, SENSOR_DATA_CONTROLLER};

#define MINIMUM_TIME_FOR_SWICH BRITime(1,100000);

/**
 * @class sensorDistributor foreward the message received to the qsupdater or to the movement stabilizer based on its state.
 */

template<class TIME, class MSG>
class sensorDistributor : public pdevs::atomic<TIME, MSG>{
private:

  TIME   next_internal;
  State  internal_state;
  MSG    output_message;


public:

/**
   * @constructor
   * Initiales the model passivated, with the internal state equal to send to qsUpdater
   */
  explicit sensorDistributor() noexcept {

    next_internal   = pdevs::atomic<TIME, MSG>::infinity;
    internal_state  = State::SENSOR_DATA_QS;
  }
/**
 * actualize the state and passivates the model
 */

  void internal() noexcept {

    next_internal = pdevs::atomic<TIME, MSG>::infinity;
    switch(internal_state){
      case State::SENSOR_DATA_QS:
        internal_state = State::SENSOR_DATA_CONTROLLER;
        break;
      case State::SENSOR_DATA_CONTROLLER:
        internal_state = State::SENSOR_DATA_QS;
        break;
      default:
        assert(false && "wrong state for internal.");
    }
  }

  /**
   * Return the next time advanced calculated in the internal function or in the external funtion
   */


  TIME advance() const noexcept {

    return next_internal;
  }

  /**
   * Calculates the output of the model.
   * @return the message with the message type updated in the external
   */

  vector<MSG> out() const noexcept {

    vector<MSG> output;
    
    output.push_back(output_message);
    return output;
  }

  /**
   * Decides to which model the message has to be foreward changing the message type.
   * @param  - a message with the following structure
   * {MsgType::SENSOR_DATA,accx,accy,accz,gyrox, gyroy, gyroz}
   */

  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {

    
    assert((next_internal == pdevs::atomic<TIME, MSG>::infinity)); // Wrong time to receive message.
    assert((mb.size() == 1)); // Wrong message bag size. Should be one.
    assert((mb.front().type == MsgType::SENSOR_DATA)); // Wrong message type.

    output_message = mb[0];
    switch(internal_state){
      case State::SENSOR_DATA_QS:
        output_message.type = MsgType::SENSOR_DATA_QS;
        break;
      case State::SENSOR_DATA_CONTROLLER:
        output_message.type = MsgType::SENSOR_DATA_CONTROLLER;
        break;
      default:
        assert(false && "wrong state for internal.");
    }


    next_internal = MINIMUM_TIME_FOR_SWICH; //Advance time of the model;
  }

  /**
  * There is not possible confluence in the model design. 
  */

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
    assert(false && "Non posible confluence function in this model");
  }
};

#endif // BOOST_SIMULATION_PDEVS_SWICH_H