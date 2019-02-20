#include <iostream>
#include <chrono>
#include <utility>
#include <memory>
#include <sys/types.h>
#include <cstdint> // This is to use int16_t, uint32_t, etc

// data structures
#include "../../vendor/britime.hpp"
#include "../../data_structures/message.hpp"

// Boost simalator include
#include <boost/simulation.hpp>


#define BOOST_TEST_MODULE powerCalculator
#include <boost/test/included/unit_test.hpp>

// Atomic model
#include "../../atomic_models/powerCalculator.hpp"
#include "stabilizer/stabilizer.h"

using namespace boost;
using namespace boost::unit_test;
using namespace std;

BOOST_AUTO_TEST_CASE( power_calculator_QUAD_FORMATION_X ) {
  
  int count = 40;
  uint32_t motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4;
  Message msg;
  msg.type = MsgType::POWER_DATA;
  BRITime elapsed_time(1,1);

  powerCalculator<BRITime, Message> powerCalculatorModel(Formation::QUAD_FORMATION_X); 

  for (uint16_t thrust = 0; thrust < count; ++thrust) {
		for (int16_t roll = 0; roll < count; ++roll) {
      for (int16_t pitch = 0; pitch < count; ++pitch) {
        for (int16_t yaw = 0; yaw < count; ++yaw) {

          distributePower_formationX( thrust,       roll,
                                      pitch,        yaw,
                                      &motorPowerM1, &motorPowerM2, 
                                      &motorPowerM3, &motorPowerM4);

          msg.powerData.roll    = roll;
          msg.powerData.pitch   = pitch;
          msg.powerData.yaw     = yaw;
          msg.powerData.thrust  = thrust;

          powerCalculatorModel.external({msg}, elapsed_time);
          vector<Message> result = powerCalculatorModel.out();
          powerCalculatorModel.internal();

          BOOST_CHECK_EQUAL(result.size(), 1);
          BOOST_CHECK_EQUAL(result.front().type, MsgType::MOTOR_INPUT);
          BOOST_CHECK_EQUAL(result.front().motorInput.M1, motorPowerM1);
          BOOST_CHECK_EQUAL(result.front().motorInput.M2, motorPowerM2);
          BOOST_CHECK_EQUAL(result.front().motorInput.M3, motorPowerM3);
          BOOST_CHECK_EQUAL(result.front().motorInput.M4, motorPowerM4);
        }
      }
    }
	}
}

BOOST_AUTO_TEST_CASE( power_calculator_QUAD_FORMATION_NORMAL ) {
  
  int count = 40;
  uint32_t motorPowerM1, motorPowerM2, motorPowerM3, motorPowerM4;
  Message msg;
  msg.type = MsgType::POWER_DATA;
  BRITime elapsed_time(1,1);

  powerCalculator<BRITime, Message> powerCalculatorModel(Formation::QUAD_FORMATION_NORMAL); 

  for (uint16_t thrust = 0; thrust < count; ++thrust) {
    for (int16_t roll = 0; roll < count; ++roll) {
      for (int16_t pitch = 0; pitch < count; ++pitch) {
        for (int16_t yaw = 0; yaw < count; ++yaw) {

          distributePower_normal( thrust,       roll,
                                  pitch,        yaw,
                                  &motorPowerM1, &motorPowerM2, 
                                  &motorPowerM3, &motorPowerM4);

          msg.powerData.roll    = roll;
          msg.powerData.pitch   = pitch;
          msg.powerData.yaw     = yaw;
          msg.powerData.thrust  = thrust;

          powerCalculatorModel.external({msg}, elapsed_time);
          vector<Message> result = powerCalculatorModel.out();
          powerCalculatorModel.internal();

          BOOST_CHECK_EQUAL(result.size(), 1);
          BOOST_CHECK_EQUAL(result.front().type, MsgType::MOTOR_INPUT);
          BOOST_CHECK_EQUAL(result.front().motorInput.M1, motorPowerM1);
          BOOST_CHECK_EQUAL(result.front().motorInput.M2, motorPowerM2);
          BOOST_CHECK_EQUAL(result.front().motorInput.M3, motorPowerM3);
          BOOST_CHECK_EQUAL(result.front().motorInput.M4, motorPowerM4);
        }
      }
    }
  }
}