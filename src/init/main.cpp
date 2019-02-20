#include <iostream>
// PDEVS
#include <ecdboost/simulation.hpp>
#include <ecdboost/utilities/embedded_time.hpp>
#include <ecdboost/utilities/embedded_message.hpp>

#include "atomic_models/motor.hpp"
#include "atomic_models/open_loop_controller.hpp"
#include "ports/command_input_port.hpp"

using namespace std;
using namespace ecdboost;

#ifdef ENABLE_SIMULATION

#include <ecdboost/builtins/linux_timer.hpp>
#include <ecdboost/builtins/output_logger_port.hpp>

using Timer = LinuxTimer;

#else

#include "CF2_timer.hpp"
#include "ports/motor_port.hpp"

using Timer = CF2Timer;

extern "C" {
  void set_led_GR();
  void set_led_GL();
  void set_led_RR();
  void time_loop();

  void ledInit();
  void powerDistributionInit();
  void pmInit();

  void motorsSetRatio(uint32_t id, uint16_t ithrust);
}

#endif

using Time = EmbeddedTime<Timer>;
using Message = EmbeddedMessage<Time, int>;

void open_loop_model(Time until) {
  auto open_loop = make_atomic_ptr<OpenLoopController<Time, Message>>();

  shared_ptr<flattened_coupled<Time, Message>> ControlUnit( new flattened_coupled<Time, Message>{{open_loop}, {open_loop}, {}, {open_loop}});

  #ifdef ENABLE_SIMULATION
  // Only one can be chosen
  //std::ostream &log_stream = std::cout;
  std::ofstream log_stream; log_stream.open("input_for_motor_ports.txt");

  auto motor_1 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_1", log_stream);
  auto motor_2 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_2", log_stream);
  auto motor_3 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_3", log_stream);
  auto motor_4 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_4", log_stream);
  #else
  auto motor_1 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("motor_1", 0);
  auto motor_2 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("motor_2", 1);
  auto motor_3 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("motor_3", 2);
  auto motor_4 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("motor_4", 3);
  #endif

  erunner<Time, Message> root{
    ControlUnit,
    {  },
    { {motor_1, open_loop}, {motor_2, open_loop}, {motor_3, open_loop}, {motor_4, open_loop} }
  };

  root.runUntil(until);
}

void basic_model(Time until) {
  // Atomic models definition
  auto motorDEVS = make_atomic_ptr<MotorDEVS<Time, Message>>();

  //Coupled model definition
  shared_ptr<flattened_coupled<Time, Message>> ControlUnit( new flattened_coupled<Time, Message>{{motorDEVS}, {motorDEVS}, {}, {motorDEVS}});

  //Top I/O port definition
  // Input ports
  auto cmd_in = make_port_ptr< CommandInputPort< Time, Message >, const string &, const Time & >("port_cmd_input", Time(0,0,1,0));

  // Output ports
  #ifdef ENABLE_SIMULATION
  // Only one can be chosen
  std::ostream &log_stream = std::cout;
  //std::ofstream log_stream; log_stream.open("input_for_motor_ports.txt");

  auto motor_1 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_1", log_stream);
  auto motor_2 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_2", log_stream);
  auto motor_3 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_3", log_stream);
  auto motor_4 = make_port_ptr< OutputLoggerPort< Time, Message >, const string &, std::ostream& >("motor_4", log_stream);
  #else
  auto motor_1 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("port_motor1", 0);
  auto motor_2 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("port_motor2", 1);
  auto motor_3 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("port_motor3", 2);
  auto motor_4 = make_port_ptr< MotorPort< Time, Message >, const string &, const int& >("port_motor4", 3);
  #endif

  // Execution parameter definition
  erunner<Time, Message> root{
    ControlUnit,
    { {cmd_in,motorDEVS} },
    { {motor_1,motorDEVS}, {motor_2,motorDEVS}, {motor_3,motorDEVS}, {motor_4,motorDEVS} }
  };

  root.runUntil(until);
}

void setMotors(uint16_t thrust) {
  #ifndef ENABLE_SIMULATION
  for (int i = 0; i < 4; i++) { motorsSetRatio(i, thrust); };
  Time ts = Time::currentTime();
  while ((Time::currentTime() - ts) < Time(0,0,2,0)) {}
  #endif
}

void use_motors() {
  #ifndef ENABLE_SIMULATION
  set_led_RR();
  setMotors(0);

  set_led_GR();
  setMotors(5000);

  set_led_RR();
  setMotors(20000);

  set_led_GR();
  setMotors(40000);

  set_led_RR();
  setMotors(20000);

  set_led_GR();
  setMotors(5000);

  set_led_RR();
  setMotors(0);
  #endif
}

int main() {
  #ifndef ENABLE_SIMULATION
  ledInit();
  pmInit();
  powerDistributionInit();
  #endif
  
  Time until(0,0,15,0);
  open_loop_model(until);
  //basic_model(until);
  //use_motors();

  #ifndef ENABLE_SIMULATION
  do {
    set_led_GR();
    time_loop();
    set_led_GL();
    time_loop();
  } while (true);
  #endif

  return 0;
}

