#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <assert.h>
#include <algorithm>
#include <limits>

#include <boost/algorithm/string.hpp>
#include <boost/simulation.hpp>

#include "vendor/input_event_stream.hpp"

#include "data_structures/message.hpp"
#include "vendor/britime.hpp"
#include "vendor/pdevs_tools.hpp"

using namespace std;
using namespace boost::simulation;
using namespace boost::simulation::pdevs;
using namespace boost::simulation::pdevs::basic_models;


using hclock = chrono::high_resolution_clock;
using Time = BRITime;

#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define IMU_UPDATE_FREQ   500
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz
#define IMU_UPDATE_DT     (float)(1.0/IMU_UPDATE_FREQ)

#define PID_ROLL_RATE_KP  70.0
#define PID_ROLL_RATE_KI  0.0
#define PID_ROLL_RATE_KD  0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  70.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  70.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  3.5
#define PID_ROLL_KI  2.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  3.5
#define PID_PITCH_KI  2.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  0.0
#define PID_YAW_KI  0.0
#define PID_YAW_KD  0.0
#define PID_YAW_INTEGRATION_LIMIT     360.0

#define DEFAULT_PID_INTEGRATION_LIMIT  5000.0

#include "model_generator/modelGenerator.hpp"

int main(int argc, char** argv){

    if (argc != 3) {
      cout << "usage: " << argv[0] << " sensor.input commander.input" << endl;
      exit(1); 
    }
    
    // the model generation class is the one in chage to generate the controller model
    ModelGenerator<Time,Message> mg(
      PID_ROLL_INTEGRATION_LIMIT,
      PID_PITCH_INTEGRATION_LIMIT,
      PID_YAW_INTEGRATION_LIMIT,
      PID_ROLL_RATE_INTEGRATION_LIMIT,
      PID_PITCH_RATE_INTEGRATION_LIMIT,
      PID_YAW_RATE_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT,
      -DEFAULT_PID_INTEGRATION_LIMIT
    );

    auto controller = mg.createController(argv[1], argv[2]);

    // CDBoost tool to generate an UML diagram of the model
    //pdevs_tools::pdevs_coupling_diagram<Time, Message> pd{*controller};
    //string puml_result = pd.get_plant_uml();
    //cout << puml_result;
    //return 0;

    cout << "Preparing runner" << endl;
    Time initial_time = Time(0);
    runner<Time, Message> r(controller, initial_time, cout, [](ostream& os, Message m){ os << m;});


    cout << "Starting simulation until passivate" << endl;
    auto start = hclock::now(); //to measure simulation execution time
    r.runUntilPassivate();
    auto elapsed = chrono::duration_cast<std::chrono::duration<double, std::ratio<1>>> (hclock::now() - start).count();

    cout << "Simulation took: " << elapsed << "sec" << endl;
    return 0;
}
