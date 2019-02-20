/*
 * robocart_driver.hpp
 *
 *  Created on: July 23, 2015
 *      Author: Daniella Niyonkuru
 */

#ifndef robocart_driver_HPP_
#define robocart_driver_HPP_


extern "C" {
void setLed_Switch();

void setLed_ON();
void setLed_OFF();

void setLed_Amb();
void time_loop();

void motorsSetRatio(int id, uint16_t ratio);
}


#include <boost/simulation/pdevs/port.hpp>

#include "devs_ports/input_ports.hpp"
#include "devs_ports/output_ports.hpp"
//#include "datagram/datagram.hpp"

#include "led.h"
#include "motors.h"

using namespace std;
using namespace boost::simulation;
using namespace boost::simulation::pdevs;



//using Value = Datagram;
using Value = int;


//static int led_count = 0;
extern bool ledOn;
extern int ali_count;
extern int values[];


/* INPUT PORTS DRIVERS */
template<class TIME, class MSG>
bool CmdInputPort<TIME, MSG>::pDriver(Value &v) const noexcept {

    /*
    if (imu6IsCalibrated()) {

    	float eulerRollDesired, eulerPitchDesired, eulerYawDesired;
    	uint16_t actuatorThrust;
		RPYType rollType, pitchType, yawType;

        commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
        commanderGetRPYType(&rollType, &pitchType, &yawType);
        commanderGetThrust(&actuatorThrust);

        //parse cmd into roll, pitch, yaw and thrust
        CommanderInput output(eulerRollDesired, eulerPitchDesired, eulerYawDesired, actuatorThrust,
        		                  rollType, pitchType, yawType);

        v = output;  //cast v appropriately

        return true;
    }

    return false;
    */
//    CommanderInput output(11.0, 11.0, 11.0, 500, CommInpType::ANGLE, CommInpType::ANGLE, CommInpType::RATE);
//
//    v = output;  //cast v appropriately

	if(ali_count < 7) {
	    v = values[ali_count++];
		time_loop();
		time_loop();
		time_loop();
//		time_loop();
//		time_loop();
	} else
		v = 0;

//    setLed_ON();
    return true;
}

template<class TIME, class MSG>
bool MotionSensorPort<TIME, MSG>::pDriver(Value &v) const noexcept {

    /*
    Axis3f gyro; // Gyro axis data in deg/sOn
    Axis3f acc;  // Accelerometer axis data in mG

	imu6Read(&gyro, &acc);

	SensorData output(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z);
	*/

//    SensorData output(1, 2, 3, 4, 5, 6);
//	v = output;  //cast v appropriately


		setLed_OFF();


	return true;
}

/* OUTPUT PORTS DRIVERS */
template<class TIME, class MSG>
bool MotorPort<TIME, MSG>::pDriver(Value& v) const noexcept {

//    motorsSetRatio(motor_num, (dynamic_cast<MotorThrust*>(&v))->thrust);  //#include "cf-firmware/drivers/interface/motors.h"

//    if(ledOn) {
//        motorsSetRatio(motor_num, 6000);
//    } else {
//        motorsSetRatio(motor_num, 0);
//    }
//	setLed_ON();

	motorsSetRatio(motor_num, v);

//	if(motor_num == 0)
//		setLed_1();
//	else if(motor_num == 1)
//		setLed_2();
//	else if(motor_num == 2)
//		setLed_3();
//	else if(motor_num == 3)
//		setLed_4();

    return true;
}

#endif /* robocart_driver_HPP_ */
