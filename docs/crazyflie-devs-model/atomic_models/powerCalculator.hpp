#ifndef BOOST_SIMULATION_PDEVS_POWER_CALCULATOR_H
#define BOOST_SIMULATION_PDEVS_POWER_CALCULATOR_H
#include <assert.h>
#include <memory>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"

#define MINIMUM_TIME_FOR_POWER_CALCULATOR_OUTPUT BRITime(1,1000000);
enum class Formation { QUAD_FORMATION_X, QUAD_FORMATION_NORMAL };

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

template<class TIME, class MSG>
class powerCalculator : public pdevs::atomic<TIME, MSG>{
private:

Formation formation;
uint32_t M1;
uint32_t M2;
uint32_t M3;
uint32_t M4;
TIME next_internal;

public:

  explicit powerCalculator(const Formation o_qfx) noexcept
  : M1(0), M2(0), M3(0), M4(0), formation(o_qfx), next_internal(pdevs::atomic<TIME, MSG>::infinity) {}

  void internal() noexcept {
  	this->next_internal = pdevs::atomic<TIME, MSG>::infinity;
  }

  TIME advance() const noexcept {
  	return this->next_internal;
  }

  vector<MSG> out() const noexcept {
  	vector<MSG> output;
    MSG data_output(MsgType::MOTOR_INPUT, this->M1, this->M2, this->M3, this->M4);

    output.push_back(data_output);
    return output;
  }

  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {
    PowerData msg;

    if (getMessage(mb, msg)) {
    	if (this->formation == Formation::QUAD_FORMATION_X) {
    	
    		int16_t r = msg.roll >> 1;
    		int16_t p = msg.pitch >> 1;
    		M1 = limitThrust(msg.thrust - r + p + msg.yaw);
    		M2 = limitThrust(msg.thrust - r - p - msg.yaw);
    		M3 = limitThrust(msg.thrust + r - p + msg.yaw);
    		M4 = limitThrust(msg.thrust + r + p - msg.yaw);
    	} else if (this->formation == Formation::QUAD_FORMATION_NORMAL){

    		M1 = limitThrust(msg.thrust + msg.pitch + msg.yaw);
      	M2 = limitThrust(msg.thrust - msg.roll - msg.yaw);
      	M3 = limitThrust(msg.thrust - msg.pitch + msg.yaw);
      	M4 = limitThrust(msg.thrust + msg.roll - msg.yaw);
    	}

      next_internal = MINIMUM_TIME_FOR_POWER_CALCULATOR_OUTPUT;
    }
  }

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
  	assert(false && "Non posible confluence function in this model");
  }

  /***************************************
  ********* helper functions *************
  ***************************************/

  uint16_t limitThrust(int32_t value) {
    if (value > UINT16_MAX) value = UINT16_MAX;
    else if(value < 0)      value = 0;

    return (uint16_t)value;
  }

  bool getMessage(const std::vector<MSG>& mb, PowerData& msg) {
    int amout = 0;

    for (typename std::vector<MSG>::const_iterator m = mb.cbegin(); m != mb.cend(); ++m) {
      if (m->type == MsgType::POWER_DATA) {
        msg = m->powerData;
        ++amout;
      } 
    }

    assert(amout <= 1);
    return (amout == 1);
  }
};

#endif // BOOST_SIMULATION_PDEVS_POWER_CALCULATOR_H
