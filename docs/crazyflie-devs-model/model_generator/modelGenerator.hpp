#ifndef MODEL_GENERATOR_H
#define MODEL_GENERATOR_H
#include <assert.h>

#include <boost/simulation.hpp>

#include "../atomic_models/pid.hpp"
#include "../atomic_models/eulerTransform.hpp"
#include "../atomic_models/qsUpdater.hpp"
#include "../atomic_models/powerCalculator.hpp"
#include "../atomic_models/movementController.hpp"
#include "../atomic_models/sensorDistributor.hpp"

using namespace boost::simulation::pdevs::basic_models;
using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

template<class TIME>
using vectorOfmodels_t = vector<shared_ptr<model<TIME>>>;
template<class TIME>
using vectorOfmodelpairs_t = vector<pair<shared_ptr<model<TIME>>, shared_ptr<model<TIME>>>>;

/**
 * @class ModelGenerator create the controller model. 
 * 
 * Instantiates the atomic and create the coupled model of the Crazyflie quadcopter controller.
 * Each time a model creation method is called, a new model instance is generated and thus all its
 * sub-components. Example: if you call the method createSensorController two times, you will get two
 * different instance of the sensor controller model and their sub-component. These models won't be 
 * conected in any way.
 * 
 * Note: If you call the createController method multiple times you will have multiple different instances
 * of the controller model that are not conected in any way. 
 */
template<class TIME, class MSG>
class ModelGenerator {
private:

	// maximum Integration limits
	float _pid_roll_integration_limit_max;
	float _pid_pitch_integration_limit_max;
	float _pid_yaw_integration_limit_max;
	float _pid_rate_roll_integration_limit_max;
	float _pid_rate_pitch_integration_limit_max;
	float _pid_rate_yaw_integration_limit_max;
	
	// minimum Integration limits
	float _pid_roll_integration_limit_min;
	float _pid_pitch_integration_limit_min;
	float _pid_yaw_integration_limit_min;
	float _pid_rate_roll_integration_limit_min;
	float _pid_rate_pitch_integration_limit_min;
	float _pid_rate_yaw_integration_limit_min;

public:

	/**
	 * @constructor
	 *
	 * Default constructor that doesn't initialize the integration limit. If this method is called,
	 * the setIntegrationLimits method should be used to set the integration limit values before
	 * create the controller model.
	 */
	ModelGenerator() {}
	
	/**
	 * @constructor 
	 *
	 * Takes as parameter the integration limits, these values can be 
	 * changed later using the method setIntegrationLimits as well. 
	 */
	ModelGenerator(	
		const float other_pid_roll_integration_limit_max,
		const float other_pid_pitch_integration_limit_max,
		const float other_pid_yaw_integration_limit_max,
		const float other_pid_rate_roll_integration_limit_max,
		const float other_pid_rate_pitch_integration_limit_max,
		const float other_pid_rate_yaw_integration_limit_max,
		const float other_pid_roll_integration_limit_min,
		const float other_pid_pitch_integration_limit_min,
		const float other_pid_yaw_integration_limit_min,
		const float other_pid_rate_roll_integration_limit_min,
		const float other_pid_rate_pitch_integration_limit_min,
		const float other_pid_rate_yaw_integration_limit_min ) 
	{
		_pid_roll_integration_limit_max = other_pid_roll_integration_limit_max;
		_pid_pitch_integration_limit_max = other_pid_pitch_integration_limit_max;
		_pid_yaw_integration_limit_max = other_pid_yaw_integration_limit_max;
		_pid_rate_roll_integration_limit_max = other_pid_rate_roll_integration_limit_max;
		_pid_rate_pitch_integration_limit_max = other_pid_rate_pitch_integration_limit_max;
		_pid_rate_yaw_integration_limit_max = other_pid_rate_yaw_integration_limit_max;
		_pid_roll_integration_limit_min = other_pid_roll_integration_limit_min;
		_pid_pitch_integration_limit_min = other_pid_pitch_integration_limit_min;
		_pid_yaw_integration_limit_min = other_pid_yaw_integration_limit_min;
		_pid_rate_roll_integration_limit_min = other_pid_rate_roll_integration_limit_min;
		_pid_rate_pitch_integration_limit_min = other_pid_rate_pitch_integration_limit_min;
		_pid_rate_yaw_integration_limit_min = other_pid_rate_yaw_integration_limit_min;
	};

	/**
	 * Sets the integration limit for the integrative part of the PID models. 
	 * This values are used as upper and lower saturation limits, if the integrative 
	 * part is over or lower the integratio limit, then the value will be the 
	 * integration limit it self.
	 * 
	 * @param other_pid_roll_integration_limit_max       - the maximum integration value acepted for the roll pid
	 * @param other_pid_pitch_integration_limit_max      - the maximum integration value acepted for the pitch pid
	 * @param other_pid_yaw_integration_limit_max        - the maximum integration value acepted for the yaw pid
	 * @param other_pid_rate_roll_integration_limit_max  - the maximum integration value acepted for the rate roll pid
	 * @param other_pid_rate_pitch_integration_limit_max - the maximum integration value acepted for the rate pitch pid
	 * @param other_pid_rate_yaw_integration_limit_max   - the maximum integration value acepted for the rate yaw pid
	 * @param other_pid_roll_integration_limit_min       - the minimum integration value acepted for the roll pid
	 * @param other_pid_pitch_integration_limit_min      - the minimum integration value acepted for the pitch pid
	 * @param other_pid_yaw_integration_limit_min        - the minimum integration value acepted for the yaw pid
	 * @param other_pid_rate_roll_integration_limit_min  - the minimum integration value acepted for the rate roll pid
	 * @param other_pid_rate_pitch_integration_limit_min - the minimum integration value acepted for the rate pitch pid
	 * @param other_pid_rate_yaw_integration_limit_min   - the minimum integration value acepted for the rate yaw pid
	 */
	void setIntegrationLimits(	
		const float other_pid_roll_integration_limit_max,
		const float other_pid_pitch_integration_limit_max,
		const float other_pid_yaw_integration_limit_max,
		const float other_pid_rate_roll_integration_limit_max,
		const float other_pid_rate_pitch_integration_limit_max,
		const float other_pid_rate_yaw_integration_limit_max,
		const float other_pid_roll_integration_limit_min,
		const float other_pid_pitch_integration_limit_min,
		const float other_pid_yaw_integration_limit_min,
		const float other_pid_rate_roll_integration_limit_min,
		const float other_pid_rate_pitch_integration_limit_min,
		const float other_pid_rate_yaw_integration_limit_min ) 
	{
		_pid_roll_integration_limit_max = other_pid_roll_integration_limit_max;
		_pid_pitch_integration_limit_max = other_pid_pitch_integration_limit_max;
		_pid_yaw_integration_limit_max = other_pid_yaw_integration_limit_max;
		_pid_rate_roll_integration_limit_max = other_pid_rate_roll_integration_limit_max;
		_pid_rate_pitch_integration_limit_max = other_pid_rate_pitch_integration_limit_max;
		_pid_rate_yaw_integration_limit_max = other_pid_rate_yaw_integration_limit_max;
		_pid_roll_integration_limit_min = other_pid_roll_integration_limit_min;
		_pid_pitch_integration_limit_min = other_pid_pitch_integration_limit_min;
		_pid_yaw_integration_limit_min = other_pid_yaw_integration_limit_min;
		_pid_rate_roll_integration_limit_min = other_pid_rate_roll_integration_limit_min;
		_pid_rate_pitch_integration_limit_min = other_pid_rate_pitch_integration_limit_min;
		_pid_rate_yaw_integration_limit_min = other_pid_rate_yaw_integration_limit_min;
	};

	/**
	 * Creates a new instance of the sensor controller model.
	 * Each time this method is called, new "sensor distributor", "qs updater" and "euler transform" models
	 * are created and coupled in order to get a new and fresh instance of a "sensor controller" model
	 *
	 * @return A shared pointer to the new sensor controller model. This is a flattened_coupled model. 
	 */
	shared_ptr<flattened_coupled<TIME, MSG>> createSensorController() {
		auto sensor_distributor = make_atomic_ptr<sensorDistributor<TIME, MSG>>();
		auto qs_updater = make_atomic_ptr<qsUpdater<TIME, MSG>, float, QuaternionType>(FUSION_UPDATE_DT, QuaternionType::MAHONY);
		auto euler_transform  = make_atomic_ptr<eulerTransform<TIME, MSG>>();
		
		vectorOfmodels_t<TIME> models = {sensor_distributor, euler_transform, qs_updater};
		vectorOfmodels_t<TIME> eic = {sensor_distributor};
		vectorOfmodels_t<TIME> eoc = {euler_transform, sensor_distributor};
		vectorOfmodelpairs_t<TIME> ic = {{sensor_distributor, qs_updater},{qs_updater, euler_transform}};

		return make_shared<flattened_coupled<TIME, MSG>>(models, eic, ic, eoc);
	}

	/**
	 * Creates a new instance of the movement stabilizer model.
	 * Each time this method is called, new "powe calculator", "movement controller" and "pid coupled" models
	 * are created and coupled in order to get a new and fresh instance of a "movement stabilizer" model.
	 *
	 * @return A shared pointer to the new movement stabilizer model. This is a flattened_coupled model. 
	 */
	shared_ptr<flattened_coupled<TIME, MSG>> createMovementStabilizer() {
		auto power_calculator = make_atomic_ptr<powerCalculator<TIME, MSG>, Formation>(Formation::QUAD_FORMATION_X);
		auto movement_controller = make_atomic_ptr<movementController<TIME, MSG>>();
		auto pid_coupled = this->createPidCoupled();

		shared_ptr<flattened_coupled<TIME, MSG>> power_calculator_coupled = this->atomicToCoupled(power_calculator);
		shared_ptr<flattened_coupled<TIME, MSG>> movement_controller_coupled = this->atomicToCoupled(movement_controller);


		vectorOfmodels_t<TIME> models = {movement_controller_coupled, pid_coupled, power_calculator_coupled};
		vectorOfmodels_t<TIME> eic = {movement_controller_coupled};
		vectorOfmodels_t<TIME> eoc = {power_calculator_coupled};
		vectorOfmodelpairs_t<TIME> ic = {
			{movement_controller_coupled, pid_coupled},
			{movement_controller_coupled, power_calculator_coupled},
			{pid_coupled, movement_controller_coupled}
		};

		return make_shared<flattened_coupled<TIME, MSG>>(models, eic, ic, eoc);
	}

	/**
	 * Creates a new instance of the pid coupled model.
	 * Each time this method is called, new atomic "pid" models are created and coupled in order to get a new 
	 * and fresh instance of a "pid coupled" model.
	 *
	 * @return A shared pointer to the new pid coupled model. This is a flattened_coupled model. 
	 */
	shared_ptr<flattened_coupled<TIME, MSG>> createPidCoupled() {
		auto pid_roll         = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_ANGLE_ROLL, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, false, _pid_roll_integration_limit_max, _pid_roll_integration_limit_min);
		auto pid_pitch        = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_ANGLE_PITCH, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, false, _pid_pitch_integration_limit_max, _pid_pitch_integration_limit_min);
		auto pid_yaw          = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_ANGLE_YAW, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, true, _pid_yaw_integration_limit_max, _pid_yaw_integration_limit_min);
		auto pid_rate_roll    = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_RATE_ROLL, PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD, false, _pid_rate_roll_integration_limit_max, _pid_rate_roll_integration_limit_min);
		auto pid_rate_pitch   = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_RATE_PITCH, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, false, _pid_rate_pitch_integration_limit_max, _pid_rate_pitch_integration_limit_min);
		auto pid_rate_yaw     = make_atomic_ptr<pid<TIME, MSG>, PIDId, float, float, float, bool, float, float>(PIDId::PID_RATE_YAW, PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, false, _pid_rate_yaw_integration_limit_max, _pid_rate_yaw_integration_limit_min);
		
		vectorOfmodels_t<TIME> models = {pid_roll, pid_pitch, pid_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw};
		vectorOfmodels_t<TIME> eic = {pid_roll, pid_pitch, pid_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw};
		vectorOfmodels_t<TIME> eoc = {pid_roll, pid_pitch, pid_yaw, pid_rate_roll, pid_rate_pitch, pid_rate_yaw};
		vectorOfmodelpairs_t<TIME> ic = {};

		return make_shared<flattened_coupled<TIME, MSG>>(models, eic, ic, eoc);
	}

	/**
	 * Creates a new instance of the main controller model.
	 * Each time this method is called, new "sensor controller", "movement stabilizer" and input models are 
	 * created and coupled in order to get a new and fresh instance of a "controller" model.
	 *
	 * Note: For the simulation instance, and only for debugging purpose, there are two input generator models that read the input
	 * events from a file and generate the apropiated input from the sensors and the commander. These model will later be replaced 
	 * for the real input.
	 *
	 * @param sensor_input_file 	- The path to the file where the simulation sensor input is declared.
	 * @param commander_input_file 	- The path to the file where the simulation commander input is declared.
	 *
	 * @return A shared pointer to the new controller model. This is a flattened_coupled model. 
	 */
	shared_ptr<flattened_coupled<TIME, MSG>> createController(char* sensor_input_file, char* commander_input_file) {

		// input generator models, they reads external files to generate the correct input
		string commander_input = this->fileToString(commander_input_file);
		string sensor_input = this->fileToString(sensor_input_file);

		shared_ptr<flattened_coupled<TIME, MSG>> commanfer_input_generator = this->atomicToCoupled(this->createInputGenerator(commander_input));
		shared_ptr<flattened_coupled<TIME, MSG>> sensor_input_generator = this->atomicToCoupled(this->createInputGenerator(sensor_input));

		auto sensor_controller = this->createSensorController();
		auto movement_stabilizer = this->createMovementStabilizer();

		vectorOfmodels_t<TIME> models = {commanfer_input_generator, sensor_input_generator, sensor_controller, movement_stabilizer};
		vectorOfmodels_t<TIME> eic = {};
		vectorOfmodels_t<TIME> eoc = {movement_stabilizer};
		vectorOfmodelpairs_t<TIME> ic = {
			{commanfer_input_generator, movement_stabilizer},
			{sensor_input_generator, sensor_controller},
			{sensor_controller, movement_stabilizer}
		};

		return make_shared<flattened_coupled<TIME, MSG>>(models, eic, ic, eoc);
	}

	/**
	 * Creates a new input generator model
	 *
	 * The created model uses the string passed as parameter to generate the input event for the model.
	 * The string is splitted in lines (i.e. using the "\n" sequence of characters) and parsed using the TIME 
	 * istream >> operator to get the time first and the MSG istream >> operator to get the message.
	 *
	 * @param input - The string with the input to parse in order to generate the model input messages.
	 * @return the created input generator model.
	 */
	shared_ptr<atomic<TIME, MSG>> createInputGenerator(string input) {
		shared_ptr<istringstream> piss{ new istringstream{} };
    	piss->str(input);
		return make_atomic_ptr<input_event_stream<TIME, MSG, TIME, MSG>, shared_ptr<istringstream>, TIME>(piss, TIME(0), 
			[](const string& s, Time& t_next, Message& m_next)->void{ //parsing function        
            
		      istringstream ss;
		      ss.str(s);      
		      ss >> t_next;
		      ss >> m_next;
		                        
		      string thrash;
		      ss >> thrash;
		      if ( 0 != thrash.size()) throw exception();
		    });
	}

	/**
	 * Create a coupled model that has the exact same behavior of the atomic model passed as parameter.
	 * The new coupled model is a wrapper of the atomic model, This is useful because the CDBoost simulator bug.
	 * Note: the CDBoost simulator bug appear whenever a coupled model send messages to atomic models. This bug
	 * exist in the c++11 version of the simulatior even when using the flattened version.
	 *
	 * @parameter atomic_model - A shared pointer to the atomic model to wrap in a coupled model.
	 * @return a shared pointer to the wrapper coupled model.
	 */
	shared_ptr<flattened_coupled<TIME, MSG>> atomicToCoupled(shared_ptr<atomic<TIME, MSG>> atomic_model) {
		vectorOfmodels_t<TIME> models = {atomic_model};
		vectorOfmodels_t<TIME> eic = {atomic_model};
		vectorOfmodels_t<TIME> eoc = {atomic_model};
		vectorOfmodelpairs_t<TIME> ic = {};
		return make_shared<flattened_coupled<TIME, MSG>>(models, eic, ic, eoc);
	}

	/**
	 * Stores the content of a file in a string format to be used in the program.
	 * 
	 * @param  filepath - the file path where to read.
	 * @return An string with the file content in string format.
	 */
	string fileToString(char * filepath) {
		ifstream file(filepath);
	    string str;
	    string file_contents;
	    while (getline(file, str)){

	      file_contents += str;
	      file_contents.push_back('\n');
	    }
	    return file_contents;
	}
};

#endif // MODEL_GENERATOR_H