#include "message.hpp"

/***************************************************/
/************* Output stream ************************/
/***************************************************/

ostream& operator<<(ostream& os, const PIDCommand& c) {

  os << "PID Command: ";
  switch(c) {
  case PIDCommand::CALCULATE: os << "CALCULATE"; break;
  case PIDCommand::RESET: os << "RESET"; break;
  } 
  return os;
}

ostream& operator<<(ostream& os, const SensorRequest& sr) {

  os << "Sensor request: ";
  switch(sr) {
  case SensorRequest::DATA_REQUEST: os << "DATA_REQUEST"; break;
  } 
  return os;
}

ostream& operator<<(ostream& os, const PIDInput& pi) {

  os << "PID ID: " << pi.PID_id << endl;
  os << "Desired: " << pi.desired << endl;
  os << "Actual: " << pi.actual << endl;
  return os;
}

ostream& operator<<(ostream& os, const PIDOut& po) {

  os << "PID ID: " << po.PID_id << endl;
  os << "value: " << po.value << endl;
  return os;
}

ostream& operator<<(ostream& os, const SensorData& sd) {
  
  os << "Gyro_x: " << sd.gyro_x << endl;
  os << "Gyro_y: " << sd.gyro_y << endl;
  os << "Gyro_z: " << sd.gyro_z << endl;
  os << "Acc_x: " << sd.acc_x << endl;
  os << "Acc_y: " << sd.acc_y << endl;
  os << "Acc_z: " << sd.acc_z;
  return os;
}

ostream& operator<<(ostream& os, const CommanderInput& ci) {

  os << "Desired_roll: " << ci.desired_roll << " Type: " << ci.roll_type << endl;
  os << "Desired_pitch: " << ci.desired_pitch << " Type: " << ci.pitch_type << endl;
  os << "Desired_yaw: " << ci.desired_yaw << " Type: " << ci.yaw_type << endl;
  os << "Desired_thrust: " << ci.desired_thrust;
  return os;
}

ostream& operator<<(ostream& os, const MotorInput& mi) {

  os << "M1: " << mi.M1 << endl;
  os << "M2: " << mi.M2 << endl;
  os << "M3: " << mi.M3 << endl;
  os << "M4: " << mi.M4;
  return os;
}

ostream& operator<<(ostream& os, const PowerData& pd) {

  os << "Roll: " << pd.roll << endl;
  os << "Pitch: " << pd.pitch << endl;
  os << "Yaw: " << pd.yaw << endl;
  os << "Thrust: " << pd.thrust;
  return os;
}

ostream& operator<<(ostream& os, const ActuatorData& ad) {
    
  os << "Axis: " << ad.axis << endl;
  if (ad.axis == Axis::THRUST) os << "thrust_value: " << ad.thrust;
  else os << "axis_value: " << ad.axis_value;

  return os;
}

ostream& operator<<(ostream& os, const Qs& qs) {
    
  os << "q0: " << qs.q0 << endl;
  os << "q1: " << qs.q1 << endl;
  os << "q2: " << qs.q2 << endl;
  os << "q3: " << qs.q3 << endl;
  os << "gyro_x: " << qs.gyro_x <<endl;
  os << "gyro_y: " << qs.gyro_x <<endl;
  os << "gyro_z: " << qs.gyro_x;

  return os;
}

ostream& operator<<(ostream& os, const EulerTransform& mi) {
  os << "gyro_x: " << mi.gyro_x << endl;
  os << "gyro_y: " << mi.gyro_x << endl;
  os << "gyro_z: " << mi.gyro_x << endl;
  os << "euler_roll: " << mi.euler_roll << endl;
  os << "euler_pitch: " << mi.euler_pitch << endl;
  os << "euler_yaw: " << mi.euler_yaw;

  return os;
}

ostream& operator<<(ostream& os, const Message& msg) {
    
  os << "type: " << msg.type << endl;
  
  switch(msg.type) {
  case MsgType::SENSOR_REQUEST: os << msg.sensorRequest; break;
  case MsgType::PID_INPUT: os << msg.pidInput; break;
  case MsgType::PID_OUT: os << msg.pidOut; break;
  case MsgType::SENSOR_DATA: os << msg.sensorData; break;
  case MsgType::COMMANDER_INPUT: os << msg.commanderInput; break;
  case MsgType::MOTOR_INPUT: os << msg.motorInput; break;
  case MsgType::POWER_DATA: os << msg.powerData; break;
  case MsgType::ACTUATOR_DATA: os << msg.actuatorData; break;
  case MsgType::QS: os << msg.qs; break;
  case MsgType::EULER_DATA: os << msg.eulerTransform; break;
  default: os << msg.type << " it is not implemented on Message << operator."; break;
  }

  return os;
}

ostream& operator<<(ostream& os, const Axis& a) {
  
  switch(a) {
  case Axis::ROLL: os << "ROLL"; break;
  case Axis::PITCH: os << "PITCH"; break;
  case Axis::YAW: os << "YAW"; break;
  case Axis::THRUST: os << "THRUST"; break;
  }

  return os;
}

ostream& operator<<(ostream& os, const CommInpType& a) {
  
  switch(a) {
  case CommInpType::RATE: os << "RATE"; break;
  case CommInpType::ANGLE: os << "ANGLE"; break;
  }

  return os;
}

ostream& operator<<(ostream& os, const MsgType& t) {
  
  switch(t) {
  case MsgType::SENSOR_REQUEST: os << "SENSOR_REQUEST"; break;
  case MsgType::PID_INPUT: os << "PID_INPUT"; break;
  case MsgType::SENSOR_DATA: os << "SENSOR_DATA"; break;
  case MsgType::COMMANDER_INPUT: os << "COMMANDER_INPUT"; break;
  case MsgType::MOTOR_INPUT: os << "MOTOR_INPUT"; break;
  case MsgType::POWER_DATA: os << "POWER_DATA"; break;
  case MsgType::ACTUATOR_DATA: os << "ACTUATOR_DATA"; break;
  case MsgType::QS: os << "QS"; break;
  case MsgType::EULER_DATA: os << "EULER_DATA"; break;
  default: os << "MsgType number "<< t << " doesn't have the << operator implemented."; break;
  }

  return os;
}

ostream& operator<<(ostream& os, const PIDId& id) {
  
  switch(id) { 
  case PIDId::PID_ANGLE_ROLL: os << "PID_ANGLE_ROLL"; break;
  case PIDId::PID_ANGLE_PITCH: os << "PID_ANGLE_PITCH"; break;
  case PIDId::PID_ANGLE_YAW: os << "PID_ANGLE_YAW"; break;
  case PIDId::PID_RATE_ROLL: os << "PID_RATE_ROLL"; break;
  case PIDId::PID_RATE_PITCH: os << "PID_RATE_PITCH"; break;
  case PIDId::PID_RATE_YAW: os << "PID_RATE_YAW"; break;
  default: os << "PIDId number "<< id << " doesn't have the << operator implemented."; break;
  }

  return os;
}


/***************************************************/
/************* Input stream ************************/
/***************************************************/

istream& operator>> (istream& is, PIDCommand& c) {
  
  string isC;
  is >> isC;
  if (isC == "CALCULATE")   c = PIDCommand::CALCULATE; 
  else if (isC == "RESET")  c = PIDCommand::RESET;
  else assert(false && "wrong pid command.");

  return is;
}

istream& operator>> (istream& is, Axis& a) {
  
  string isA;
  is >> isA;
  if (isA == "ROLL")        a = Axis::ROLL; 
  else if (isA == "PITCH")  a = Axis::PITCH; 
  else if (isA == "YAW")    a = Axis::YAW; 
  else if (isA == "THRUST") a = Axis::THRUST;
  else assert(false && "wrong Axis.");

  return is;
}

istream& operator>> (istream& is, MsgType& t) {

  string isT;
  is >> isT;
  if (isT == "SENSOR_REQUEST")        t = MsgType::SENSOR_REQUEST;   
  else if (isT == "EULER_DATA")       t = MsgType::EULER_DATA;   
  else if (isT == "QS")               t = MsgType::QS;   
  else if (isT == "PID_INPUT")        t = MsgType::PID_INPUT;   
  else if (isT == "PID_OUT")          t = MsgType::PID_OUT;   
  else if (isT == "SENSOR_DATA")      t = MsgType::SENSOR_DATA;   
  else if (isT == "EULER_DATA")       t = MsgType::EULER_DATA;   
  else if (isT == "COMMANDER_INPUT")  t = MsgType::COMMANDER_INPUT;   
  else if (isT == "MOTOR_INPUT")      t = MsgType::MOTOR_INPUT;   
  else if (isT == "POWER_DATA")       t = MsgType::POWER_DATA;   
  else if (isT == "ACTUATOR_DATA")    t = MsgType::ACTUATOR_DATA;  
  else assert(false && "wrgon MsgType.");

  return is;
}

istream& operator>> (istream& is, SensorRequest& sr) {

  string isSR;
  is >> isSR;
  if (isSR == "DATA_REQUEST") sr = SensorRequest::DATA_REQUEST;   
  else assert(false && "wrgon SensorRequest.");

  return is;
}

istream& operator>> (istream& is, PIDInput& pi) {

  is >> pi.PID_id;
  is >> pi.request;
  if (pi.request == PIDCommand::CALCULATE) {
    is >> pi.desired;
    is >> pi.actual;
  }
  return is;
}

istream& operator>> (istream& is, PIDOut& po) {

  is >> po.PID_id;
  is >> po.value;
  return is;
}

istream& operator>> (istream& is, SensorData& sd) {

  is >> sd.gyro_x;
  is >> sd.gyro_y;
  is >> sd.gyro_z;
  is >> sd.acc_x;
  is >> sd.acc_y;
  is >> sd.acc_z;
  return is;
}

istream& operator>> (istream& is, CommanderInput& ci) {

  is >> ci.desired_roll;
  is >> ci.desired_pitch;
  is >> ci.desired_yaw;
  is >> ci.desired_thrust;
  is >> ci.roll_type;
  is >> ci.pitch_type;
  is >> ci.yaw_type;
  return is;
}

istream& operator>> (istream& is, MotorInput& mi) {

  is >> mi.M1;
  is >> mi.M2;
  is >> mi.M3;
  is >> mi.M4;
  return is;
}

istream& operator>> (istream& is, PowerData& pd) {

  is >> pd.roll;
  is >> pd.pitch;
  is >> pd.yaw;
  is >> pd.thrust;
  return is;
}

istream& operator>> (istream& is, Qs& qs) {

  is >> qs.q0;
  is >> qs.q1;
  is >> qs.q2;
  is >> qs.q3;
  is >> qs.gyro_x;
  is >> qs.gyro_y;
  is >> qs.gyro_z;
  return is;
}

istream& operator>> (istream& is, EulerTransform& mi) {

  is >> mi.euler_roll;
  is >> mi.euler_pitch;
  is >> mi.euler_yaw;
  is >> mi.gyro_x;
  is >> mi.gyro_y;
  is >> mi.gyro_z;
  return is;
}

istream& operator>> (istream& is, ActuatorData& ad) {
  Axis axis;

  is >> ad.axis;
  switch(ad.axis) {
  case Axis::ROLL: is >> ad.axis_value; break;
  case Axis::PITCH: is >> ad.axis_value; break;
  case Axis::YAW: is >> ad.axis_value; break;
  case Axis::THRUST: is >> ad.thrust; break;
  }

  return is;
}

istream& operator>> (istream& is, CommInpType& com) {

  string isT;
  is >> isT;
  if (isT == "ANGLE")     com = CommInpType::ANGLE;   
  else if (isT == "RATE") com = CommInpType::RATE; 
  else assert(false && "wrong Commander input type.");

  return is;
}

istream& operator>> (istream& is, Message& msg) {

  is >> msg.type;
  switch(msg.type) {
  case MsgType::SENSOR_REQUEST: is >> msg.sensorRequest ; break;
  case MsgType::QS: is >> msg.qs ; break; 
  case MsgType::PID_INPUT: is >> msg.pidInput ; break; 
  case MsgType::PID_OUT: is >> msg.pidOut ; break; 
  case MsgType::SENSOR_DATA: is >> msg.sensorData ; break; 
  case MsgType::EULER_DATA: is >> msg.eulerTransform ; break; 
  case MsgType::COMMANDER_INPUT: is >> msg.commanderInput ; break; 
  case MsgType::MOTOR_INPUT: is >> msg.motorInput ; break; 
  case MsgType::POWER_DATA: is >> msg.powerData ; break; 
  case MsgType::ACTUATOR_DATA: is >> msg.actuatorData ; break;
  default: assert(false && "wrong Message type."); break;
  }

  return is;
}

istream& operator>> (istream& is, PIDId& id) {

  switch(id) {
  case PIDId::PID_ANGLE_ROLL: is >> id ; break;
  case PIDId::PID_ANGLE_PITCH: is >> id ; break; 
  case PIDId::PID_ANGLE_YAW: is >> id ; break; 
  case PIDId::PID_RATE_ROLL: is >> id ; break; 
  case PIDId::PID_RATE_PITCH: is >> id ; break; 
  case PIDId::PID_RATE_YAW: is >> id ; break; 
  default: assert(false && "wrong Message type."); break;
  }

  return is;
}