/**
 * @author Ali Salaheddin
 * ARSLab - Carleton University
 *
 * motorMaster Model:
 * Receives motorInput message from the Movement Stabilizer model. It then extracts the speed rates for each
 * motor and outputs it to them respectively.
 * This model has 1 state. We don't need to keep track of it in the code.
 */

#ifndef MOTOR_MASTER_H
#define MOTOR_MASTER_H
#include <assert.h>
#include <memory>
#include <boost/simulation/pdevs/atomic.hpp>

#include "rt_includes/eMessage_s.hpp"

using namespace boost::simulation::pdevs;
using namespace boost::simulation;
using namespace std;

#define MINIMUM_TIME_FOR_SWITCH TIME(00,00,0,00,100) //BRITime();

/**
 * @class motorMaster
 */

template<class TIME, class MSG>
class MotorDEVS: public pdevs::atomic<TIME, MSG> {
private:

    int thrust_m1, thrust_m2, thrust_m3, thrust_m4;
    TIME next_internal;

public:

    /**
     * @constructor
     * Initiales the model passivated, with the internal state equal to send to qsUpdater
     */
    explicit MotorDEVS() noexcept
            : atomic<TIME, MSG>("MotorDEVS"),
			  thrust_m1(0),
			  thrust_m2(0),
			  thrust_m3(0),
			  thrust_m4(0)
	{

        next_internal = pdevs::atomic<TIME, MSG>::infinity;
    }
    /**
     * actualize the state and passivates the model
     */

    void internal() noexcept {

        //do nothing
        next_internal = Time(00,00,00,250);
    }

    /**
     * Return the next time advanced calculated in the internal function or in the external funtion
     */

    TIME advance() const noexcept {

        return next_internal;
    }

    /**
     * Calculates the output of the model.
     * @return the message with the message type updated in the external
     */

    vector<MSG> out() const noexcept {

        vector<MSG> output;
        MSG output_m1("port_motor1", thrust_m1),
            output_m2("port_motor2", thrust_m2),
            output_m3("port_motor3", thrust_m3),
            output_m4("port_motor4", thrust_m4);

        output.push_back(output_m1);
        output.push_back(output_m2);
        output.push_back(output_m3);
        output.push_back(output_m4);

        return output;
    }

    /**
     * Decides to which model the message has to be foreward changing the message type.
     * @param  - a message with the following structure
     * {DataType::MotorInput,accx,accy,accz,gyrox, gyroy, gyroz}
     */

    void external(const std::vector<MSG>& mb, const TIME& t) noexcept {

        assert((mb.size() == 1)); // Wrong message bag size. Should be one.
//        assert((mb.front().payload.getType() == DataType::MOTOR_INPUT)); // Wrong message type.

//        const int* payload = static_cast<const int*>(&mb.front().val);
        uint32_t value = mb.front().val;

//        if(value == 10)
//            setLed_Amb();
//        else
//        	setLed_Police();

        thrust_m1 = value;
        thrust_m2 = value;
        thrust_m3 = value;
        thrust_m4 = value;

        next_internal = Time::Zero; //MINIMUM_TIME_FOR_SWITCH; //Advance time of the model;
    }

    /**
     * There is not possible confluence in the model design.
     */

    virtual void confluence(const std::vector<MSG>& mb, const TIME& t)
            noexcept {
    	internal();
//        assert(false && "Non posible confluence function in this model");
    }

    void print() noexcept {}

};

#endif // MOTOR_MASTER_H
