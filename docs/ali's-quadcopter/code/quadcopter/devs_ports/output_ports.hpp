/*
 * motors_port.hpp
 *
 *  Created on: July 23, 2015
 *      Author: Daniella Niyonkuru
 */

#ifndef motors_port_HPP_
#define motors_port_HPP_

#include <boost/simulation/pdevs/port.hpp>

using namespace std;
using namespace boost::simulation;
using namespace boost::simulation::pdevs;


extern "C" {
//void motorsInit();
void powerDistributionInit();
}

enum MotorEnum {
    MOTOR_1, MOTOR_2, MOTOR_3, MOTOR_4
};

template<class TIME, class MSG>
class MotorPort: public port<TIME, MSG> {

public:
    using Value = int;

    int motor_num;

    /**
     * @brief motor1_port constructor.
     *
     * @param n name assigned to the port
     */
    explicit MotorPort(const int& _motor_num, const std::string &n) noexcept
            : port<TIME, MSG>(n), motor_num(_motor_num)
    {motorsInit();}

    void print() noexcept {} //printf("MOTOR 1 \n");
    bool pDriver(Value &v) const noexcept;
};

#endif /* motors_port_HPP_ */
