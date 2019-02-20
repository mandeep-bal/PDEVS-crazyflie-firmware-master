#ifndef BOOST_SIMULATION_MESSAGE_H
#define BOOST_SIMULATION_MESSAGE_H

#include <iostream>
#include <assert.h>

using namespace std;

enum class PIDCommand { RESET, CALCULATE };
enum class SensorRequest { DATA_REQUEST };
enum class Axis { ROLL, PITCH, YAW, THRUST };
enum class MsgType { SENSOR_REQUEST, EULER_DATA, QS, PID_INPUT, PID_OUT, SENSOR_DATA, SENSOR_DATA_QS, SENSOR_DATA_CONTROLLER, COMMANDER_INPUT, MOTOR_INPUT, POWER_DATA, ACTUATOR_DATA };
enum class PIDId {PID_ANGLE_ROLL, PID_ANGLE_PITCH, PID_ANGLE_YAW, PID_RATE_ROLL, PID_RATE_PITCH, PID_RATE_YAW};
enum class CommInpType {RATE, ANGLE};

struct Qs {

  Qs() {}
  Qs(float o_q0, float o_q1, float o_q2, float o_q3, float o_gx, float o_gy, float o_gz)
  : q0(o_q0), q1(o_q1), q2(o_q2), q3(o_q3), gyro_x(o_gx), gyro_y(o_gy), gyro_z(o_gz) {}
  
  float q0;
  float q1;
  float q2;
  float q3;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

struct PIDInput {

  PIDInput() {}
  PIDInput(PIDId o_PID_id, PIDCommand o_request, float o_desired, float o_actual)
  : PID_id(o_PID_id), request(o_request), desired(o_desired), actual(o_actual) {}
  PIDInput(PIDId o_PID_id, PIDCommand o_request)
  : PID_id(o_PID_id), request(o_request) {}

  PIDCommand  request;
  PIDId       PID_id;
  float       desired;
  float       actual;
};

struct PIDOut {

  PIDOut() {}
  PIDOut(PIDId o_PID_id, float o_value)
  : PID_id(o_PID_id), value(o_value) {}

  PIDId PID_id;
  float value;
};

struct SensorData {

  SensorData() {}
  SensorData(float o_gx, float o_gy, float o_gz, float o_ax, float o_ay, float o_az)
  : gyro_x(o_gx), gyro_y(o_gy), gyro_z(o_gz), acc_x(o_ax), acc_y(o_ay), acc_z(o_az) {}

  float gyro_x;
  float gyro_y;
  float gyro_z;
  float acc_x;
  float acc_y;
  float acc_z;
};

struct EulerTransform {


  EulerTransform() {}
  EulerTransform(float o_er, float o_ep, float o_ey, float o_gx, float o_gy, float o_gz)
  : euler_roll(o_er), euler_pitch(o_ep), euler_yaw(o_ey), gyro_x(o_gx), gyro_y(o_gy), gyro_z(o_gz) {}
  
  float euler_roll;
  float euler_pitch;
  float euler_yaw;
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

struct CommanderInput {

  CommanderInput() {}
  CommanderInput(float o_dr, float o_dp, float o_dy, uint16_t o_dt, CommInpType rt, CommInpType pt,  CommInpType yt)
  : desired_roll(o_dr), desired_pitch(o_dp), desired_yaw(o_dy), desired_thrust(o_dt), roll_type(rt), pitch_type(pt), yaw_type(yt) {}
  
  float       desired_roll;
  float       desired_pitch;
  float       desired_yaw;
  uint16_t    desired_thrust;
  CommInpType roll_type;
  CommInpType pitch_type;
  CommInpType yaw_type;
};

struct MotorInput {

  MotorInput() {}
  MotorInput(uint32_t o_M1, uint32_t o_M2, uint32_t o_M3, uint32_t o_M4)
  : M1(o_M1), M2(o_M2), M3(o_M3), M4(o_M4) {}
  
  uint32_t M1;
  uint32_t M2;
  uint32_t M3;
  uint32_t M4;
};

struct PowerData {

  PowerData() {}
  PowerData(int16_t o_r, int16_t o_p, int16_t o_y, uint16_t o_t)
  : roll(o_r), pitch(o_p), yaw(o_y), thrust(o_t) {}
  
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  uint16_t thrust;
};

struct ActuatorData {

  ActuatorData() {}
  ActuatorData(Axis o_a, int16_t o_v)
  : axis(o_a), axis_value(o_v) { assert(o_a != Axis::THRUST); }
  ActuatorData(Axis o_a, uint16_t o_v)
  : axis(o_a), thrust(o_v) { assert(o_a == Axis::THRUST); }

  Axis axis;
  int16_t axis_value;
  uint16_t thrust;
};

struct Message {

  Message() {}

  Message(MsgType t, float o_q0, float o_q1, float o_q2, float o_q3, float o_gx, float o_gy, float o_gz)
  : qs(o_q0, o_q1, o_q2, o_q3, o_gx, o_gy, o_gz) {
    assert(t == MsgType::QS);
    type = t;
  }

  Message(MsgType t) {
    assert(t == MsgType::SENSOR_REQUEST);
    sensorRequest = SensorRequest::DATA_REQUEST;
    type = t;
  }

  Message(MsgType t, uint32_t o_M1, uint32_t o_M2, uint32_t o_M3, uint32_t o_M4)
  : motorInput(o_M1, o_M2, o_M3, o_M4) {
    assert(t == MsgType::MOTOR_INPUT);
    type = t;
  }

  Message(MsgType t, float o_dr, float o_dp, float o_dy, uint16_t o_dt, CommInpType rt, CommInpType pt,  CommInpType yt) 
  : commanderInput(o_dr, o_dp, o_dy, o_dt, rt, pt, yt) {
    assert(t == MsgType::COMMANDER_INPUT);
    type = t;
  }

  Message(MsgType t, int16_t o_r, int16_t o_p, int16_t o_y, uint16_t o_t)
  : powerData(o_r, o_p, o_y, o_t) {
    assert(t == MsgType::POWER_DATA);
    type = t;
  }

  Message(MsgType t, PIDId id, PIDCommand o_request, float desired, float actual)
  : pidInput(id, o_request, desired, actual){
    assert(t == MsgType::PID_INPUT);
    type = t;
  }

  Message(MsgType t, PIDId id, PIDCommand o_request)
  : pidInput(id, o_request){
    assert(t == MsgType::PID_INPUT);
    type = t;
  }  
  
  Message(MsgType t, PIDId id, float value)
  : pidOut(id, value){
    assert(t == MsgType::PID_OUT);
    type = t;
  }

  Message(MsgType t, float arg1, float arg2, float arg3, float arg4, float arg5, float arg6) {
    switch(t) {
    case MsgType::SENSOR_DATA:
    case MsgType::SENSOR_DATA_QS:
    case MsgType::SENSOR_DATA_CONTROLLER:
      sensorData.gyro_x = arg1;
      sensorData.gyro_y = arg2;
      sensorData.gyro_z = arg3;
      sensorData.acc_x  = arg4;
      sensorData.acc_y  = arg5;
      sensorData.acc_z  = arg6;
      break;
    case MsgType::EULER_DATA:
      eulerTransform.euler_roll  = arg1;
      eulerTransform.euler_pitch = arg2;
      eulerTransform.euler_yaw   = arg3;
      eulerTransform.gyro_x      = arg4;
      eulerTransform.gyro_y      = arg5;
      eulerTransform.gyro_z      = arg6;
      break;
    default:
      assert(false && "Message type and arguments does't match.");
    }
    type = t;
  }

  // This is for Axis data
  Message(MsgType t, Axis o_a, int16_t o_v)
  : actuatorData(o_a, o_v) {
    assert(t == MsgType::ACTUATOR_DATA);
    type = t;
  }

  // this is for Thrust data
  Message(MsgType t, Axis o_a, uint16_t o_v)
  : actuatorData(o_a, o_v) {
    assert(t == MsgType::ACTUATOR_DATA);
    type = t;
  }

  // Message attributes
  MsgType         type;
  //EulerData       eulerData;
  Qs              qs;
  EulerTransform  eulerTransform;
  ActuatorData    actuatorData;
  SensorRequest   sensorRequest;
  SensorData      sensorData;
  CommanderInput  commanderInput;
  MotorInput      motorInput;
  PowerData       powerData;
  PIDInput        pidInput;
  PIDOut          pidOut;
};

istream& operator>> (istream& is, PIDCommand& c);
istream& operator>> (istream& is, PIDId& id);
istream& operator>> (istream& is, Axis& a);
istream& operator>> (istream& is, MsgType& t);
istream& operator>> (istream& is, SensorRequest& sr);
istream& operator>> (istream& is, PIDInput& pa);
istream& operator>> (istream& is, PIDOut& pr);
istream& operator>> (istream& is, SensorData& sd);
istream& operator>> (istream& is, CommanderInput& ci);
istream& operator>> (istream& is, MotorInput& mi);
istream& operator>> (istream& is, PowerData& pd);
istream& operator>> (istream& is, Qs& qs);
istream& operator>> (istream& is, EulerTransform& mi);
istream& operator>> (istream& is, ActuatorData& ad);
istream& operator>> (istream& is, CommInpType& com);
istream& operator>> (istream& is, Message& msg);

ostream& operator<<(ostream& os, const PIDCommand& c);
ostream& operator<<(ostream& os, const PIDId& id);
ostream& operator<<(ostream& os, const SensorRequest& sr);
ostream& operator<<(ostream& os, const PIDInput& pa);
ostream& operator<<(ostream& os, const PIDOut& pr);
ostream& operator<<(ostream& os, const SensorData& sd);
ostream& operator<<(ostream& os, const CommanderInput& ci);
ostream& operator<<(ostream& os, const MotorInput& mi);
ostream& operator<<(ostream& os, const PowerData& pd);
ostream& operator<<(ostream& os, const ActuatorData& ad);
ostream& operator<<(ostream& os, const Qs& qs);
ostream& operator<<(ostream& os, const EulerTransform& mi);
ostream& operator<<(ostream& os, const Message& msg);
ostream& operator<<(ostream& os, const Axis& a);
ostream& operator<<(ostream& os, const CommInpType& t);
ostream& operator<<(ostream& os, const MsgType& t);

#endif // BOOST_SIMULATION_MESSAGE_H