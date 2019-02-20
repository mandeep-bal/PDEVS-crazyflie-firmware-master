#ifndef OPEN_LOOP_CONTROLLER__HPP
#define OPEN_LOOP_CONTROLLER__HPP

#include <ecdboost/simulation.hpp>

using namespace std;
using namespace ecdboost;

enum stage { waiting, starting, taking_off, flying, landing, stopped };

#ifndef ENABLE_SIMULATION

extern "C" {
  void set_led_GR();
  void set_led_RR();
  void set_led_GL();
  void time_loop();

  void led_blocking_assert(bool condition);
}

#else

#include <cstdlib>

#endif

template<class TIME, class MSG>
class OpenLoopController : public ecdboost::atomic<TIME, MSG> {
  private:
    stage current_stage;
    int next_thrust;  // Represents the thrust of the next stage
 
  public:
    explicit OpenLoopController() noexcept : atomic<TIME, MSG>("OpenLoopController") {
      #ifdef ENABLE_SIMULATION
      std::cout << "init @ " << TIME::currentTime() << std::endl;
      #endif
      current_stage = stage::waiting;
      next_thrust = 40000;
    }

    void internal() noexcept {
      // the `next_thrust` assigned value is the thrust the motors will receive in the next stage
      #ifdef ENABLE_SIMULATION
      std::cout << "internal @ " << TIME::currentTime() << std::endl;
      #endif
      switch (current_stage) {
        case stage::waiting:
          current_stage = stage::starting;
          next_thrust = 50000;
          break;
        case stage::starting: 
          current_stage = stage::taking_off;
          next_thrust = 35000;
          break;
        case stage::taking_off:
          current_stage = stage::flying;
          next_thrust = 20000;
          break;
        case stage::flying:
          current_stage = stage::landing;
          next_thrust = 0;
          break;
        case stage::landing:
          current_stage = stage::stopped;
          next_thrust = -1;
          break;
        default:
          #ifdef ENABLE_SIMULATION
          std::cerr << "[Error: internal] Should never reach this state" << std::endl;
          exit(1);
          #else
          led_blocking_assert(false);
          #endif
      }
    }

    TIME advance() const noexcept {
      #ifdef ENABLE_SIMULATION
      std::cout << "advance @ " << TIME::currentTime() << std::endl;
      #endif
      // The return value is how much time the model will keep itself in current_stage
      switch (current_stage) {
        case stage::waiting:
          return TIME(0,0,1,0);
        case stage::starting: 
          return TIME(0,0,2,0);
        case stage::taking_off:
          return TIME(0,0,0,400);
        case stage::flying:
          return TIME(0,0,5,0);
        case stage::landing:
          return TIME(0,0,2,0);
        case stage::stopped:
          return TIME::Infinity;
        default:
          #ifdef ENABLE_SIMULATION
          std::cerr << "[Error: advance] Should never reach this state" << std::endl;
          exit(1);
          #else
          led_blocking_assert(false);
          #endif
      }
    }

    vector<MSG> out() const noexcept {
      #ifdef ENABLE_SIMULATION
      std::cout << "out @ " << TIME::currentTime() 
        << " with previous stage " << current_stage 
        << " and next thrust " << next_thrust
        << std::endl;
      #endif

      vector<MSG> output;
      MSG output_m1("motor_1", next_thrust),
          output_m2("motor_2", next_thrust),
          output_m3("motor_3", next_thrust),
          output_m4("motor_4", next_thrust);

      output.push_back(output_m1);
      output.push_back(output_m2);
      output.push_back(output_m3);
      output.push_back(output_m4);

      return output;
    }

    void external(const std::vector<MSG>& mb, const TIME& t) noexcept {
      #ifdef ENABLE_SIMULATION
      std::cout << "external @ " << TIME::currentTime() << std::endl;
      #endif

      // Should never happen
      #ifdef ENABLE_SIMULATION
      std::cerr << "[Error: external] Should never reach this state" << std::endl;
      exit(1);
      #else
      led_blocking_assert(false);
      #endif
    }

    virtual void confluence(const std::vector<MSG>& mb, const TIME& t) noexcept {
      internal();
    }

    void print() noexcept {}

};

#endif // OPEN_LOOP_CONTROLLER__HPP

