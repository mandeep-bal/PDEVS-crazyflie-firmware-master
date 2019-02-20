#ifndef BOOST_SIMULATION_PDEVS_PID_H
#define BOOST_SIMULATION_PDEVS_PID_H

/**
* Laouen Mayal Louan Belloli
* ARSLab - Carleton University
*
* PID Model:
* This model calculates the desired angle rate for the close loop
* CrazyFlight controller.
*/

#include <memory>
#include <string>
#include <utility>
#include <assert.h>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"
#include "../vendor/britime.hpp"

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

#define MINIMUM_TIME_FOR_PID_OUTPUT BRITime(1,10000000);
#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0

template<class TIME, class MSG>
class pid : public pdevs::atomic<TIME, MSG>{
private:

  // PID setting attributes
  PIDId id;
  float kp;
  float ki;
  float kd;
  bool angle_error;

  // integral attributes
  float integral;
  float integral_max;
  float integral_min;

  // derivate attributes
  float previous_error;

  // output attributes
  float value;
  TIME next_internal;

public:
  /**
  * PID model initialization.
  *
  * @param[PIDId] other_id
  * @param[float] other_kp: The proportional gain.
  * @param[float] other_ki: The integral gain.
  * @param[float] other_kd: The derivative gain.
  * @param[Bool] other_angle_error: it says if an angle error must be calculated.
  */
  explicit pid(
  const PIDId other_id,
  const float other_kp,
  const float other_ki, 
  const float other_kd,
  const bool other_angle_error,
  const float other_integration_max,
  const float other_integration_min ) noexcept {
    
    // setting PID attributes
    this->id          = other_id;
    this->kp          = other_kp;
    this->ki          = other_ki;
    this->kd          = other_kd;
    this->angle_error = other_angle_error;
   
    // setting Integrative attributes
    this->integral        = 0;
    this->integral_max    = other_integration_max;
    this->integral_min    = other_integration_min;
 
    // setting derivative attributes
    this->previous_error  = 0;
    
    // output attributes
    this->next_internal = pdevs::atomic<TIME, MSG>::infinity;
  }

  void internal() noexcept {
    this->next_internal = pdevs::atomic<TIME, MSG>::infinity;
  }

  TIME advance() const noexcept {

    return this->next_internal;
  }

  vector<MSG> out() const noexcept {
    
    MSG new_output(MsgType::PID_OUT, this->id, this->value);
    vector<MSG> output(1, new_output);
    return output;
  }

  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {

    assert((this->next_internal == pdevs::atomic<TIME, MSG>::infinity));
    // selecting correct message
    int counter = 0;
    PIDInput msg;
    for (typename vector<MSG>::const_iterator i = mb.cbegin(); i != mb.cend(); ++i) {
      if (i->type == MsgType::PID_INPUT && i->pidInput.PID_id == this->id) {
        msg = i->pidInput;
        counter++;
      } 
    }
    assert(counter <= 1);

    if (counter == 1) {
      switch(msg.request) {
      case PIDCommand::RESET:
        this->previous_error  = 0;
        this->integral        = 0;
        break;
      case PIDCommand::CALCULATE:
        //cout << this->id << " model desired: " << msg.desired << " ctrl actual: " << msg.actual << endl;
        // Error calculation
        float error = calculateError(msg.desired, msg.actual);
        //cout << this->id << endl;
        // PID calculation
        this->value = calculatePID(error, t);
        //cout << this->id << " " << this->value << endl;
        
        // updating PID internal state
        this->previous_error  = error;
        this->next_internal   = MINIMUM_TIME_FOR_PID_OUTPUT;
        break;
      }
    }
  }

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
    internal();
    external(mb, t);
  }

  /***************************************
  ********* helper functions *************
  ***************************************/

  float calculateError(float desired, float actual) {

    //cout << this->id << " model desired: " << actual << " ctrl actual: " << desired << endl;
    float error = desired - actual;
    // If angle error is true, then overflow error must be take in consideration
    if (this->angle_error) {
      if (error > 180.0)
        error -= 360.0;
      else if (error < -180.0)
        error += 360.0;
    }

    return error;
  }

  float calculatePID(float error, TIME t) {
    
    float dt = convertBRITimeToFloat(t);
    //cout << "error " << error << endl;
    //cout << "dt " << dt << endl;
    
    // Integrative part calclation
    //cout << "sumando de integral: "<< (error * dt) << endl;
    //cout << "integral prev: " << this->integral << endl;
    this->integral += error * dt;
    //cout << "integral intermedio: " << this->integral << endl;
    if (this->integral > this->integral_max) {       
      //cout << "max integral: " << this->integral_max << " current integral: " << this->integral << endl;
      this->integral = this->integral_max;
    } else if (this->integral < this->integral_min) {       
      //cout << "min integral: " << this->integral_min << " current integral: " << this->integral << endl;
      this->integral = this->integral_min;
    }
    //cout << "integral final: " << this->integral << endl;

    // Derivative part calculation
    float derivative = (error - this->previous_error) / dt;
    //cout << "derivate: " << derivative << endl;

    // result calculation
    float out_P = this->kp * error;
    float out_I = this->ki * this->integral;
    float out_D = this->kd * derivative;

    return out_P + out_I + out_D;
  }

  float convertBRITimeToFloat(const BRITime& bt) {

    return boost::rational_cast<float>(bt._value);
  }

 // float convertTimeToFloat(const Time& t) {
//
  //  return t.asMicsecs() * 1e-6;
  //}

  void show() {

    cout << "[PID model] internal state: " << endl;
    cout << "Setting attributes: " << endl;
    cout << "  id: " << this->id << endl;
    cout << "  Kp: " << this->kp << endl;
    cout << "  Ki: " << this->ki << endl;
    cout << "  Kd: " << this->kd << endl;
    cout << "  angle_error: " << (angle_error ? "true" : "false") << endl;

    cout << "Integrative attributes: " << endl;
    cout << "  integral: " <<  this->integral << endl;
    cout << "  integral_max: " <<  this->integral_max << endl;
    cout << "  integral_min: " <<  this->integral_min << endl;

    cout << "Derivative attributes: " << endl;
    cout << "  previous_error: " <<  this->previous_error << endl;

    cout << "Output attributes: " << endl;
    cout << "  output size: " <<  this->output.size() << endl;
    cout << "  next_internal: " <<   this->next_internal << endl;
    cout << "[PID model]" << endl;
  }
};

#endif // BOOST_SIMULATION_PDEVS_PID_H
