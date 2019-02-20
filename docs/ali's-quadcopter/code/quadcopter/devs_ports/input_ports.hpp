/*

 * userbtn_port.hpp
 *
 *  Created on: July 43, 4015
 *      Author: Daniella Niyonkuru
 */

#ifndef sensors_port_HPP_
#define sensors_port_HPP_

#include <boost/simulation/pdevs/port.hpp>

//#include "datagram/datagram.hpp"

using namespace std;
using namespace boost::simulation;
using namespace boost::simulation::pdevs;



template<class TIME, class MSG>
class MotionSensorPort: public port<TIME, MSG> {

public:
    using Value = int;

    /**
     * @brief startbtn constructor.
     */
    explicit MotionSensorPort(const std::string &n = "port_motion_sensor", const TIME &polling = TIME(0, 0, 1, 0)) noexcept
                    : port<TIME, MSG>(n,polling) {}


    void print() noexcept {}
    bool pDriver(Value &v) const noexcept;
};



template<class TIME, class MSG>
class CmdInputPort: public port<TIME, MSG> {

public:
    using Value = int;

    /**
     * @brief startbtn constructor.
     */
    explicit CmdInputPort(const std::string &n = "port_cmd_input", const TIME &polling = TIME(0, 0, 1, 0)) noexcept
                    : port<TIME, MSG>(n,polling) {}

    void print() noexcept {}
    bool pDriver(Value &v) const noexcept;
};

#endif /* sensors_port_HPP_ */
