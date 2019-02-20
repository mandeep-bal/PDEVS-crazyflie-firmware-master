#include <iostream>
#include <chrono>
#include <utility>
#include <memory>
#include <sys/types.h>
#include <cstdint> // This is to use int16_t

// data structures
#include "../../vendor/britime.hpp"
#include "../../data_structures/message.hpp"

// Boost simalator include
#include <boost/simulation.hpp>


#define BOOST_TEST_MODULE pid
#include <boost/test/included/unit_test.hpp>

// Atomic model
#include "../../atomic_models/pid.hpp"
#include "controller/pid.h"

using namespace boost;
using namespace boost::unit_test;
using namespace std;

BOOST_AUTO_TEST_CASE( pid_angle_test_type ) {

  float rollRateDesired, pitchRateDesired, yawRateDesired;  
  BRITime elapsed_time = BRITime(1,500);
  PidObject pidRoll, pidPitch, pidYaw;
  

  // controller init
  pidInit(&pidRoll, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitch, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYaw, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRoll, PID_ROLL_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYaw, PID_YAW_INTEGRATION_LIMIT);

  // model init
  pid<BRITime, Message> pid_roll(PIDId::PID_ANGLE_ROLL, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, false, PID_ROLL_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);
  pid<BRITime, Message> pid_pitch(PIDId::PID_ANGLE_PITCH, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, false, PID_PITCH_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);
  pid<BRITime, Message> pid_yaw(PIDId::PID_ANGLE_YAW, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, true, PID_YAW_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);

  Message roll_msg, pitch_msg, yaw_msg;
  roll_msg.type = pitch_msg.type = yaw_msg.type = MsgType::PID_INPUT;
  

  for (float eulerRollActual = 0.0; eulerRollActual < 10.0; eulerRollActual += 1.0) {
    for (float eulerPitchActual = 0.0; eulerPitchActual < 10.0; eulerPitchActual += 1.0) {
      for (float eulerYawActual = 0.0; eulerYawActual < 10.0; eulerYawActual += 1.0) {
        for (float eulerRollDesired = 0.0; eulerRollDesired < 10.0; eulerRollDesired += 1.0) {
          for (float eulerPitchDesired = 0.0; eulerPitchDesired < 10.0; eulerPitchDesired += 1.0) {
            for (float eulerYawDesired = 0.0; eulerYawDesired < 10.0; eulerYawDesired += 1.0) {
              
              // controller call
              controllerCorrectAttitudePID( &pidRoll,         &pidPitch,          &pidYaw,
                                            eulerRollActual,  eulerPitchActual,   eulerYawActual,
                                            eulerRollDesired, eulerPitchDesired,  eulerYawDesired,
                                            &rollRateDesired, &pitchRateDesired, &yawRateDesired );
              // message initialization
              roll_msg.pidInput.PID_id  = PIDId::PID_ANGLE_ROLL;
              roll_msg.pidInput.request = PIDCommand::CALCULATE;
              roll_msg.pidInput.desired = eulerRollDesired;
              roll_msg.pidInput.actual  = eulerRollActual;

              pitch_msg.pidInput.PID_id  = PIDId::PID_ANGLE_PITCH;
              pitch_msg.pidInput.request = PIDCommand::CALCULATE;
              pitch_msg.pidInput.desired = eulerPitchDesired;
              pitch_msg.pidInput.actual  = eulerPitchActual;

              yaw_msg.pidInput.PID_id  = PIDId::PID_ANGLE_YAW;
              yaw_msg.pidInput.request = PIDCommand::CALCULATE;
              yaw_msg.pidInput.desired = eulerYawDesired;
              yaw_msg.pidInput.actual  = eulerYawActual;
              
              pid_roll.external({roll_msg}, elapsed_time);
              pid_pitch.external({pitch_msg}, elapsed_time);
              pid_yaw.external({yaw_msg}, elapsed_time);
              
              vector<Message> roll_result   = pid_roll.out();
              vector<Message> pitch_result  = pid_pitch.out();
              vector<Message> yaw_result    = pid_yaw.out();

              pid_roll.internal();
              pid_pitch.internal();
              pid_yaw.internal();
              
              BOOST_CHECK_EQUAL(roll_result.front().type, MsgType::PID_OUT);
              BOOST_CHECK_EQUAL(pitch_result.front().type, MsgType::PID_OUT);
              BOOST_CHECK_EQUAL(yaw_result.front().type, MsgType::PID_OUT);
              
              BOOST_CHECK_EQUAL(roll_result.front().pidOut.value, rollRateDesired);
              BOOST_CHECK_EQUAL(pitch_result.front().pidOut.value, pitchRateDesired);
              BOOST_CHECK_EQUAL(yaw_result.front().pidOut.value, yawRateDesired);
            }
          }
        }     
      }
    }
  }
}

BOOST_AUTO_TEST_CASE( pid_rate_test_type ) {

  int16_t rollOutput, pitchOutput, yawOutput;  
  BRITime elapsed_time = BRITime(1,500);
  PidObject pidRollRate, pidPitchRate, pidYawRate;

  // controller init
  pidInit(&pidRollRate, 0, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, IMU_UPDATE_DT);
  pidInit(&pidPitchRate, 0, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT);
  pidInit(&pidYawRate, 0, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, IMU_UPDATE_DT);
  pidSetIntegralLimit(&pidRollRate, PID_ROLL_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidPitchRate, PID_PITCH_RATE_INTEGRATION_LIMIT);
  pidSetIntegralLimit(&pidYawRate, PID_YAW_RATE_INTEGRATION_LIMIT);

  // model init
  pid<BRITime, Message> pid_roll_rate(PIDId::PID_RATE_ROLL, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, false, PID_ROLL_RATE_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);
  pid<BRITime, Message> pid_pitch_rate(PIDId::PID_RATE_PITCH, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, false, PID_PITCH_RATE_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);
  pid<BRITime, Message> pid_yaw_rate(PIDId::PID_RATE_YAW, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, false, PID_YAW_RATE_INTEGRATION_LIMIT, -DEFAULT_PID_INTEGRATION_LIMIT);

  Message roll_msg, pitch_msg, yaw_msg;
  roll_msg.type = pitch_msg.type = yaw_msg.type = MsgType::PID_INPUT;  

  for (float eulerRollActual = 0.0; eulerRollActual < 10.0; eulerRollActual += 1.0) {
    for (float eulerPitchActual = 0.0; eulerPitchActual < 10.0; eulerPitchActual += 1.0) {
      for (float eulerYawActual = 0.0; eulerYawActual < 10.0; eulerYawActual += 1.0) {
        for (float eulerRollDesired = 0.0; eulerRollDesired < 10.0; eulerRollDesired += 1.0) {
          for (float eulerPitchDesired = 0.0; eulerPitchDesired < 10.0; eulerPitchDesired += 1.0) {
            for (float eulerYawDesired = 0.0; eulerYawDesired < 10.0; eulerYawDesired += 1.0) {       
              // controller call
              controllerCorrectRatePID( &pidRollRate,     &pidPitchRate,      &pidYawRate,
                                        eulerRollActual,  eulerPitchActual,   eulerYawActual,
                                        eulerRollDesired, eulerPitchDesired,  eulerYawDesired,
                                        &rollOutput,      &pitchOutput,       &yawOutput );

              roll_msg.pidInput.PID_id  = PIDId::PID_RATE_ROLL;
              roll_msg.pidInput.request = PIDCommand::CALCULATE;
              roll_msg.pidInput.desired = eulerRollDesired;
              roll_msg.pidInput.actual  = eulerRollActual;

              pitch_msg.pidInput.PID_id  = PIDId::PID_RATE_PITCH;
              pitch_msg.pidInput.request = PIDCommand::CALCULATE;
              pitch_msg.pidInput.desired = eulerPitchDesired;
              pitch_msg.pidInput.actual  = eulerPitchActual;

              yaw_msg.pidInput.PID_id  = PIDId::PID_RATE_YAW;
              yaw_msg.pidInput.request = PIDCommand::CALCULATE;
              yaw_msg.pidInput.desired = eulerYawDesired;
              yaw_msg.pidInput.actual  = eulerYawActual;

              pid_roll_rate.external({roll_msg}, elapsed_time);
              pid_pitch_rate.external({pitch_msg}, elapsed_time);
              pid_yaw_rate.external({yaw_msg}, elapsed_time);
              
              vector<Message> roll_result   = pid_roll_rate.out();
              vector<Message> pitch_result  = pid_pitch_rate.out();
              vector<Message> yaw_result    = pid_yaw_rate.out();

              pid_roll_rate.internal();
              pid_pitch_rate.internal();
              pid_yaw_rate.internal();
              
              BOOST_CHECK_EQUAL(roll_result.front().type, MsgType::PID_OUT);
              BOOST_CHECK_EQUAL(pitch_result.front().type, MsgType::PID_OUT);
              BOOST_CHECK_EQUAL(yaw_result.front().type, MsgType::PID_OUT);
              
              BOOST_CHECK_EQUAL(saturateSignedInt16(roll_result.front().pidOut.value), rollOutput);
              BOOST_CHECK_EQUAL(saturateSignedInt16(pitch_result.front().pidOut.value), pitchOutput);
              BOOST_CHECK_EQUAL(saturateSignedInt16(yaw_result.front().pidOut.value), yawOutput);
            }
          }
        }     
      }
    }
  }
}