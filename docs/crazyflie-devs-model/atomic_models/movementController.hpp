#ifndef BOOST_SIMULATION_PDEVS_MOVEMENT_CONTROLER_H
#define BOOST_SIMULATION_PDEVS_MOVEMENT_CONTROLER_H
#include <assert.h>
#include <memory>
#include <assert.h>
#include <cstdint>
#include <iostream>
#include <boost/simulation/pdevs/atomic.hpp>

#include "../data_structures/message.hpp"
#include "../vendor/britime.hpp"

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

enum class CtrlState {WAITING_SENSOR_DATA, IN_PROGRESS, READY_POWER_DATA};
enum class AxisDataState {IDLE, SENDING_TO_PID_ANGLE, WAITING_FOR_PID_ANGLE, SENDING_TO_PID_RATE, WAITING_FOR_PID_RATE, READY};

ostream& operator<<(ostream& os, const AxisDataState& axs) {

  os << "AxisDataState: ";
  switch(axs) {
  case AxisDataState::IDLE: cout << "IDLE" << endl; break; 
  case AxisDataState::SENDING_TO_PID_ANGLE: cout << "SENDING_TO_PID_ANGLE" << endl; break; 
  case AxisDataState::WAITING_FOR_PID_ANGLE: cout << "WAITING_FOR_PID_ANGLE" << endl; break; 
  case AxisDataState::SENDING_TO_PID_RATE: cout << "SENDING_TO_PID_RATE" << endl; break; 
  case AxisDataState::WAITING_FOR_PID_RATE: cout << "WAITING_FOR_PID_RATE" << endl; break; 
  case AxisDataState::READY: cout << "READY" << endl; break;
  } 
  return os;
}

ostream& operator<<(ostream& os, const CtrlState& cs) {

  os << "CtrlState: ";
  switch(cs) {
  case CtrlState::WAITING_SENSOR_DATA: cout << "WAITING_SENSOR_DATA" << endl; break; 
  case CtrlState::IN_PROGRESS: cout << "IN_PROGRESS" << endl; break; 
  case CtrlState::READY_POWER_DATA: cout << "READY_POWER_DATA" << endl; break;
  } 
  return os;
}

struct AxisData {

  AxisData(AxisDataState s) : state(s), value(0), finalValue(0) {}
  float         desired;
  float         value;
  int16_t       finalValue;
  AxisDataState state;
  CommInpType   desiredType;
  PIDId         to;
};

#define MINIMUM_TIME_FOR_CONTROLER_OUTPUT BRITime(1,1000000);

template<class TIME, class MSG>
class movementController : public pdevs::atomic<TIME, MSG>{
private:

	// Commander Input
	uint16_t desired_thrust;

	// Sensors Input
	float sensor_roll;
	float sensor_pitch;
	float sensor_yaw;
	float gyro_x;
	float gyro_y;
	float gyro_z;

  //Axis data
  AxisData roll_data;
  AxisData pitch_data;
  AxisData yaw_data;
  // controller state
  CtrlState state;
	TIME next_internal;

public:

  explicit movementController() noexcept 
  : desired_thrust(0), 
    sensor_roll(0), 
    sensor_pitch(0), 
    sensor_yaw(0), 
    gyro_x(0), 
    gyro_y(0), 
    gyro_z(0),
    roll_data(AxisDataState::IDLE),
    pitch_data(AxisDataState::IDLE),
    yaw_data(AxisDataState::IDLE),
    state(CtrlState::WAITING_SENSOR_DATA),
    next_internal(pdevs::atomic<TIME, MSG>::infinity) {}

  void internal() noexcept {
    switch(this->state){
    case CtrlState::IN_PROGRESS:
      if (isSendingData(this->roll_data))   waitForData(this->roll_data);
      if (isSendingData(this->pitch_data))  waitForData(this->pitch_data);
      if (isSendingData(this->yaw_data))    waitForData(this->yaw_data);
      break;
    case CtrlState::READY_POWER_DATA:
      this->state = CtrlState::WAITING_SENSOR_DATA;
      this->roll_data.state = AxisDataState::IDLE;
      this->pitch_data.state = AxisDataState::IDLE;
      this->yaw_data.state = AxisDataState::IDLE;
      break;
    default:
      assert(false && "wrong state for internal.");
    }

    this->next_internal = pdevs::atomic<TIME, MSG>::infinity;
  }

  TIME advance() const noexcept {
    return this->next_internal;
  }

  vector<MSG> out() const noexcept {
    vector<MSG> output;

    switch(this->state) {
    case CtrlState::IN_PROGRESS:
      if (isSendingData(this->roll_data))   output.push_back(getOutputFrom(this->roll_data));
      if (isSendingData(this->pitch_data))  output.push_back(getOutputFrom(this->pitch_data));
      if (isSendingData(this->yaw_data))    output.push_back(getOutputFrom(this->yaw_data));
      break;
    case CtrlState::READY_POWER_DATA:
      if (this->positiveThrust()) { 
        output.push_back(getPowerOutput());
      } else {
        output.push_back(getStopOutput());
        this->setResetPIDsMessages(output);
      }
      break;
    default:
      assert(false && "wrong state in out function.");
    }

    return output;
  }

  void external(const std::vector<MSG>& mb, const TIME& t) noexcept {
    for (typename vector<MSG>::const_iterator msg = mb.cbegin(); msg != mb.cend(); ++msg) {
      switch(msg->type) {
      case MsgType::COMMANDER_INPUT:
        this->setCommanderInput(msg->commanderInput);
        break;
      case MsgType::EULER_DATA:
        this->setEulerData(msg->eulerTransform);
        break;
      case MsgType::SENSOR_DATA_CONTROLLER:
        this->setSensorData(msg->sensorData);
        break;
      case MsgType::PID_OUT:
        assert(this->state != CtrlState::WAITING_SENSOR_DATA);
        switch(msg->pidOut.PID_id) {
        case PIDId::PID_ANGLE_ROLL:
        case PIDId::PID_ANGLE_PITCH:
        case PIDId::PID_ANGLE_YAW:
          this->processPIDAngle(msg->pidOut);
          break;
        case PIDId::PID_RATE_ROLL:
        case PIDId::PID_RATE_PITCH:
        case PIDId::PID_RATE_YAW:
          this->processPIDRate(msg->pidOut);
          break;
        }
      case MsgType::SENSOR_DATA_QS:
        break; // ignore the sensor data sent to qsUpdater model.
      default:
        assert(false && "wrong message type.");
      }
    }

    if (this->isReadyPowerData()) this->state = CtrlState::READY_POWER_DATA;
  
    this->setNextInternal();
  }

  virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
    internal();
    external(mb, t);
  }

/* HELPER FUNCTIONS */

/******************************************/
/*********** internal function ************/
/******************************************/

  void waitForData(AxisData& ad) {
    if (ad.state == AxisDataState::SENDING_TO_PID_ANGLE) ad.state = AxisDataState::WAITING_FOR_PID_ANGLE;
    else ad.state = AxisDataState::WAITING_FOR_PID_RATE;
  } 

/*************************************/
/*********** out function ************/
/*************************************/

  bool isSendingData(const AxisData& ad) const {
    return  ad.state == AxisDataState::SENDING_TO_PID_ANGLE || ad.state == AxisDataState::SENDING_TO_PID_RATE;
  }

  MSG getPowerOutput() const {
    return MSG(MsgType::POWER_DATA, this->roll_data.finalValue, this->pitch_data.finalValue, this->yaw_data.finalValue, this->desired_thrust);
  }

  MSG getOutputFrom(const AxisData& ad) const {
    float actual;
    switch(ad.state) {
    case AxisDataState::SENDING_TO_PID_ANGLE: 
      switch(ad.to) {
      case PIDId::PID_ANGLE_ROLL:   actual = this->sensor_roll; break;
      case PIDId::PID_ANGLE_PITCH:  actual = this->sensor_pitch; break;
      case PIDId::PID_ANGLE_YAW:    actual = this->sensor_yaw; break;
      }
      return MSG(MsgType::PID_INPUT, ad.to, PIDCommand::CALCULATE, ad.value, actual); 
      break;
    case AxisDataState::SENDING_TO_PID_RATE:
      switch(ad.to) {
      case PIDId::PID_RATE_ROLL:   actual = this->gyro_x;  break;
      case PIDId::PID_RATE_PITCH:  actual = -this->gyro_y; break;
      case PIDId::PID_RATE_YAW:    actual = this->gyro_z;   break;
      }
      return MSG(MsgType::PID_INPUT, ad.to, PIDCommand::CALCULATE, ad.value, actual); 
      break;
    default:
      assert(false && "wrogn axis state to get output message.");
      break;
    }
  }

  MSG getStopOutput() const {
    return MSG(MsgType::POWER_DATA, (int16_t)0, (int16_t)0, (int16_t)0, (uint16_t)0);
  }

  void setResetPIDsMessages(vector<MSG> output) const {
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_ANGLE_ROLL, PIDCommand::RESET));
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_ANGLE_PITCH, PIDCommand::RESET));
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_ANGLE_YAW, PIDCommand::RESET));
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_RATE_ROLL, PIDCommand::RESET));
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_RATE_PITCH, PIDCommand::RESET));
    output.push_back(MSG(MsgType::PID_INPUT, PIDId::PID_RATE_YAW, PIDCommand::RESET));
  }

/******************************************/
/*********** external function ************/
/******************************************/

  bool isReadyPowerData() const {
    return  (this->state       == CtrlState::IN_PROGRESS) && 
            (roll_data.state   == AxisDataState::READY) &&
            (pitch_data.state  == AxisDataState::READY) &&
            (yaw_data.state    == AxisDataState::READY);
  }

  void setNextInternal() {
    switch(this->state) {
    case CtrlState::WAITING_SENSOR_DATA:
      next_internal = pdevs::atomic<TIME, MSG>::infinity;
      break;
    case CtrlState::IN_PROGRESS:
    case CtrlState::READY_POWER_DATA:
      next_internal = MINIMUM_TIME_FOR_CONTROLER_OUTPUT;
    }
  }

  void processPIDRate(const PIDOut& pidOut) {
    switch(pidOut.PID_id) {
    case PIDId::PID_RATE_ROLL:
      assert(this->roll_data.state == AxisDataState::WAITING_FOR_PID_RATE);
      this->roll_data.finalValue  = this->saturateSignedInt16(pidOut.value);
      this->roll_data.state       = AxisDataState::READY;
      break;
    case PIDId::PID_RATE_PITCH:
      assert(this->pitch_data.state == AxisDataState::WAITING_FOR_PID_RATE);
      this->pitch_data.finalValue   = this->saturateSignedInt16(pidOut.value);    
      this->pitch_data.state        = AxisDataState::READY;
      break;
    case PIDId::PID_RATE_YAW:  
      assert(this->yaw_data.state == AxisDataState::WAITING_FOR_PID_RATE);
      this->yaw_data.finalValue   = this->saturateSignedInt16(pidOut.value);    
      this->yaw_data.state        = AxisDataState::READY;
      break;
    default:
      assert(false && "wrong PID out message id");
    }
  }

  void processPIDAngle(const PIDOut& pidOut) {
    switch(pidOut.PID_id) {
    case PIDId::PID_ANGLE_ROLL: this->setPidAngleValues(this->roll_data, pidOut, PIDId::PID_RATE_ROLL); break;
    case PIDId::PID_ANGLE_PITCH: this->setPidAngleValues(this->pitch_data, pidOut, PIDId::PID_RATE_PITCH); break;
    case PIDId::PID_ANGLE_YAW: this->setPidAngleValues(this->yaw_data, pidOut, PIDId::PID_RATE_YAW); break;
    default:
      assert(false && "Wrong PID out message id.");
    }
  }

  void setPidAngleValues(AxisData& ad, const PIDOut& pidOut, PIDId pID) {
    assert(ad.state == AxisDataState::WAITING_FOR_PID_ANGLE);
    ad.value   = pidOut.value;
    ad.state   = AxisDataState::SENDING_TO_PID_RATE;
    ad.to      = pID;
  }

  void setEulerData(const EulerTransform& et) {
    assert(this->state == CtrlState::WAITING_SENSOR_DATA);

    this->sensor_roll   = et.euler_roll;
    this->sensor_pitch  = et.euler_pitch;
    this->sensor_yaw    = et.euler_yaw;
    this->gyro_x        = et.gyro_x;
    this->gyro_y        = et.gyro_y;
    this->gyro_z        = et.gyro_z;

    // Setting the Axis data for the next close loop iteration
    setAxisDataToStart(this->roll_data, PIDId::PID_ANGLE_ROLL, PIDId::PID_RATE_ROLL, 1);
    setAxisDataToStart(this->pitch_data, PIDId::PID_ANGLE_PITCH, PIDId::PID_RATE_PITCH, 1);
    setAxisDataToStart(this->yaw_data, PIDId::PID_ANGLE_YAW, PIDId::PID_RATE_YAW, -1);

    this->state = CtrlState::IN_PROGRESS;
  }

  void setSensorData(const SensorData& sd) {
    this->gyro_x = sd.gyro_x;
    this->gyro_y = sd.gyro_y;
    this->gyro_z = sd.gyro_z;

    // Setting the Axis data for the next close loop iteration
    setAxisDataToStartSkipingAngle(this->roll_data, PIDId::PID_RATE_ROLL, 1);
    setAxisDataToStartSkipingAngle(this->pitch_data, PIDId::PID_RATE_PITCH, 1);
    setAxisDataToStartSkipingAngle(this->yaw_data, PIDId::PID_RATE_YAW, -1);

    this->state = CtrlState::IN_PROGRESS;
  }

  void setAxisDataToStart(AxisData& ad, PIDId toAngle, PIDId toRate, int factor) {
    assert(ad.state == AxisDataState::IDLE);

    ad.value = factor * ad.desired;
    if (ad.desiredType == CommInpType::ANGLE) {
      ad.to    = toAngle;
      ad.state = AxisDataState::SENDING_TO_PID_ANGLE;
    } else {
      ad.to    = toRate;
      ad.state = AxisDataState::SENDING_TO_PID_RATE;
    }
  }

  void setAxisDataToStartSkipingAngle(AxisData& ad, PIDId toRate, int factor) {
    assert(ad.state == AxisDataState::IDLE);
    ad.to    = toRate;
    ad.state = AxisDataState::SENDING_TO_PID_RATE;
    if (ad.desiredType == CommInpType::RATE) ad.value = factor * ad.desired;

  }

  void setCommanderInput(const CommanderInput& command) {
    this->roll_data.desired     = command.desired_roll;
    this->roll_data.desiredType = command.roll_type;
    
    this->pitch_data.desired      = command.desired_pitch;
    this->pitch_data.desiredType  = command.pitch_type;
    
    this->yaw_data.desired     = command.desired_yaw;
    this->yaw_data.desiredType = command.yaw_type;
    
    this->desired_thrust        = command.desired_thrust;
  }

  int16_t saturateSignedInt16(float in) {
    // don't use INT16_MIN, because later we may negate it, which won't work for that value.
    if      (in > INT16_MAX)  return INT16_MAX;
    else if (in < -INT16_MAX) return -INT16_MAX;
    else                      return (int16_t)in;
  }

  /*************************************************/
  /*************** common function *****************/
  /*************************************************/

  bool positiveThrust() const {
    return this->desired_thrust > 0;
  }
};

#endif // BOOST_SIMULATION_PDEVS_MOVEMENT_CONTROLER_H
