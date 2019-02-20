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

#include <ecdboost/simulation.hpp>

using namespace std;
using namespace ecdboost;

template<class TIME, class MSG>
class MotorDEVS: public ecdboost::atomic<TIME, MSG> {
  private:
  int thrust_m1, thrust_m2, thrust_m3, thrust_m4;
  TIME next_internal;

  public:
  explicit MotorDEVS() noexcept :
    atomic<TIME, MSG>("MotorDEVS"),
    thrust_m1(0),
    thrust_m2(0),
    thrust_m3(0),
    thrust_m4(0)
  {

    next_internal = TIME::Infinity;
  }

  void internal() noexcept { next_internal = TIME::Infinity; }

  TIME advance() const noexcept { return next_internal; }

  vector<MSG> out() const noexcept {
    vector<MSG> output;
    MSG output_m1("port_motor1", 5000),
        output_m2("port_motor2", 35000),
        output_m3("port_motor3", 5000),
        output_m4("port_motor4", 35000);

    output.push_back(output_m1);
    output.push_back(output_m2);
    output.push_back(output_m3);
    output.push_back(output_m4);

    return output;
  }

  void external(const vector<MSG>& mb, const TIME& t) noexcept {
    uint32_t value = mb.front().val;
    thrust_m1 = value;
    thrust_m2 = value;
    thrust_m3 = value;
    thrust_m4 = value;

    next_internal = TIME::Zero;
  }

  virtual void confluence(const vector<MSG>& mb, const TIME& t) noexcept {
    internal();
  }

  void print() noexcept {}

};

#endif // MOTOR_MASTER_H
