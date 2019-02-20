//--------------- DEVS MAIN CODE -------------------

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <assert.h>
#include <algorithm>
#include <limits>

#include <boost/algorithm/string.hpp>
#include <boost/simulation.hpp>

//#include "datagram/datagram.hpp"
#include "rt_includes/eTime.h"
//#include "rt_includes/hwTime.h"
//#include "rt_includes/britime.hpp"
//#include "rt_includes/pdevs_tools.hpp"


using namespace std;
using namespace boost::simulation;
using namespace boost::simulation::pdevs;
using namespace boost::simulation::pdevs::basic_models;



#define SIMULATION_DURATION_SEC    20

#define M_PI		3.14159265358979323846	/* pi */


extern "C" {
void setLed_1();
void setLed_2();
void setLed_3();
void setLed_4();
void setLed_5();
void setLed_6();
void setLed_7();
void setLed_8();
void setLed_9();
void setLed_10();
void setLed_11();

void setLed_Police();
void setLed_Amb();
}

#include "rt_includes/eMessage_s.hpp"
#include "devs_am/motorDEVS.hpp"
#include "devs_ports/ports_impl.hpp"


int main(){

	setLed_1();

	// Atomic models definition
//	printf("Creating atomic models ... \n");
	auto motorDEVS = make_atomic_ptr<MotorDEVS<Time, Message>>();

	//Coupled model definition
//	printf("Creating Coupled models ... \n");
	shared_ptr<flattened_coupled<Time, Message>> ControlUnit( new flattened_coupled<Time, Message>{{motorDEVS}, {motorDEVS}, {}, {motorDEVS}});

	//Top I/O port definition
//	printf("Defining top I/O ports ... \n");
	// Input ports
//	auto sensor_in = make_port_ptr< MotionSensorPort<Time, Message>, const string &, const Time & >("port_motion_sensor", Time(0,0,1,0));
	auto cmd_in = make_port_ptr< CmdInputPort<Time, Message>, const string &, const Time & >("port_cmd_input", Time(0,0,0,200));

	// Output ports
	auto motor_1 = make_port_ptr<MotorPort<Time, Message>, const int&, const string &>(0, "port_motor1");
	auto motor_2 = make_port_ptr<MotorPort<Time, Message>, const int&, const string &>(1, "port_motor2");
	auto motor_3 = make_port_ptr<MotorPort<Time, Message>, const int&, const string &>(2, "port_motor3");
	auto motor_4 = make_port_ptr<MotorPort<Time, Message>, const int&, const string &>(3, "port_motor4");

    setLed_4();

	// Execution parameter definition
//	printf("Preparing runner \n");
	Time initial_time{Time::currentTime()};
	erunner<Time, Message> root{ControlUnit, {{cmd_in,motorDEVS}} , {{motor_1,motorDEVS}, {motor_2,motorDEVS}, {motor_3,motorDEVS}, {motor_4,motorDEVS}} };
	Time end_time{Time(0,0,2,500)};

//	printf(("Starting simulation until time: seconds " + end_time.asString() + "\n").c_str());
	end_time = root.runUntil(end_time);

	setLed_OFF();

    return 0;
}
