#ifndef OPEN_LOOP_CONTROLLER__HPP
#define OPEN_LOOP_CONTROLLER__HPP

#include <ecdboost/simulation.hpp>

using namespace std;
using namespace ecdboost;

enum stage { waiting, starting, starting_one, starting_two, starting_three, starting_four, starting_five, starting_six, starting_seven, starting_eight, starting_nine, starting_ten, starting_eleven, taking_off, flying, flying_one, flying_two, flying_three,flying_four, flying_five, flying_six, flying_seven, flying_eight, flying_nine, flying_ten,flying_eleven, flying_twelve, flying_thirteen, hover, hover_one, hover_two, hover_three,hover_four, hover_five, hover_six, hover_seven, hover_eight, hover_nine, hover_ten,hover_eleven, hover_twelve, hover_thirteen, landing, landing_one, stopped };

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
      next_thrust = 20000;
    }

    void internal() noexcept {
      // the `next_thrust` assigned value is the thrust the motors will receive in the next stage
      #ifdef ENABLE_SIMULATION
      std::cout << "internal @ " << TIME::currentTime() << std::endl;
      #endif
      switch (current_stage) {
        case stage::waiting:
          current_stage = stage::starting;
          next_thrust = 30000;
          break;
        case stage::starting: 
          current_stage = stage::starting_one;
          next_thrust = 35000;
          break;
case stage::starting_one: 
          current_stage = stage::starting_two;
          next_thrust = 37000;
          break;
case stage::starting_two: 
          current_stage = stage::starting_three;
          next_thrust = 37250;
          break;
case stage::starting_three: 
          current_stage = stage::starting_four;
          next_thrust = 37500;
          break;
case stage::starting_four: 
          current_stage = stage::starting_five;
          next_thrust = 37750;
          break;
case stage::starting_five: 
          current_stage = stage::starting_six;
          next_thrust = 38000;
          break;
case stage::starting_six: 
          current_stage = stage::starting_seven;
          next_thrust = 38250;
          break;
case stage::starting_seven: 
          current_stage = stage::starting_eight;
          next_thrust = 38500;
          break;
case stage::starting_eight: 
          current_stage = stage::starting_nine;
          next_thrust = 38750;
          break;
case stage::starting_nine: 
          current_stage = stage::starting_ten;
          next_thrust = 39000;
          break;
case stage::starting_ten: 
          current_stage = stage::starting_eleven;
          next_thrust = 39250;
          break;
case stage::starting_eleven: 
          current_stage = stage::taking_off;
          next_thrust = 39500;
          break;
        case stage::taking_off:
          current_stage = stage::flying;
          next_thrust = 39750;
          break;
case stage::flying:
          current_stage = stage::flying_one;
          next_thrust =40000;
          break;
case stage::flying_one:
          current_stage = stage::flying_two;
          next_thrust =40100;
          break;
 case stage::flying_two:
          current_stage = stage::flying_three;
          next_thrust =40200;
          break;
case stage::flying_three:
          current_stage = stage::flying_four;
          next_thrust =40200;
          break;
case stage::flying_four:
          current_stage = stage::flying_five;
          next_thrust =40350;
          break;
case stage::flying_five:
          current_stage = stage::flying_six;
          next_thrust =40360;
          break;
case stage::flying_six:
          current_stage = stage::flying_seven;
          next_thrust =40370;
          break;
 case stage::flying_seven:
          current_stage = stage::flying_eight;
          next_thrust =40380;
          break;
case stage::flying_eight:
          current_stage = stage::flying_nine;
          next_thrust =40390;
          break;
case stage::flying_nine:
          current_stage = stage::flying_ten;
          next_thrust =40400;
          break;
case stage::flying_ten:
          current_stage = stage::flying_eleven;
          next_thrust =40410;
          break;
case stage::flying_eleven:
          current_stage = stage::flying_twelve;
          next_thrust =40420;
          break;
case stage::flying_twelve:
          current_stage = stage::flying_thirteen;
          next_thrust =40430;
          break;
case stage::flying_thirteen:
          current_stage = stage::hover;
          next_thrust =40440;
          break;
case stage::hover:
          current_stage = stage::hover_one;
          next_thrust =40450;
          break;
case stage::hover_one:
          current_stage = stage::hover_two;
          next_thrust =40450;
          break;
 case stage::hover_two:
          current_stage = stage::hover_three;
          next_thrust =40439;
          break;
case stage::hover_three:
          current_stage = stage::hover_four;
          next_thrust =40429;
          break;
case stage::hover_four:
          current_stage = stage::hover_five;
          next_thrust =40419;
          break;
case stage::hover_five:
          current_stage = stage::hover_six;
          next_thrust =40399;
          break;
case stage::hover_six:
          current_stage = stage::hover_seven;
          next_thrust =40389;
          break;
 case stage::hover_seven:
          current_stage = stage::hover_eight;
          next_thrust =40379;
          break;
case stage::hover_eight:
          current_stage = stage::hover_nine;
          next_thrust =40369;
          break;
case stage::hover_nine:
          current_stage = stage::hover_ten;
          next_thrust =40359;
          break;
case stage::hover_ten:
          current_stage = stage::hover_eleven;
          next_thrust =40339;
          break;
case stage::hover_eleven:
          current_stage = stage::hover_twelve;
          next_thrust =40319;
          break;
case stage::hover_twelve:
          current_stage = stage::hover_thirteen;
          next_thrust =40299;
          break;
        case stage::hover_thirteen:
          current_stage = stage::landing;
          next_thrust = 40000;
          break;
        case stage::landing:
          current_stage = stage::landing_one;
          next_thrust = 0;
          break;
case stage::landing_one:
          current_stage = stage::stopped;
          next_thrust = 0;
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
          return TIME(0,0,2,0);
        case stage::starting: 
          return TIME(0,0,3,0);
case stage::starting_one: 
          return TIME(0,0,0,200);
case stage::starting_two: 
          return TIME(0,0,0,215);
case stage::starting_three: 
          return TIME(0,0,0,230);
case stage::starting_four:
          return TIME(0,0,0,245);
case stage::starting_five:
          return TIME(0,0,0,250);
case stage::starting_six:
          return TIME(0,0,0,265);
case stage::starting_seven:
          return TIME(0,0,0,280);
case stage::starting_eight:
          return TIME(0,0,0,295);
case stage::starting_nine:
          return TIME(0,0,0,310);
case stage::starting_ten:
          return TIME(0,0,0,325);
case stage::starting_eleven:
          return TIME(0,0,0,350);
        case stage::taking_off:
          return TIME(0,0,0,370);
        case stage::flying:
          return TIME(0,0,0,390);
        case stage::flying_one:
          return TIME(0,0,0,400);
        case stage::flying_two:
          return TIME(0,0,0,450);
        case stage::flying_three:
          return TIME(0,0,0,500);
case stage::flying_four:
          return TIME(0,0,0,550);
case stage::flying_five:
          return TIME(0,0,0,575);
case stage::flying_six:
          return TIME(0,0,0,600);
case stage::flying_seven:
          return TIME(0,0,0,650);
case stage::flying_eight:
          return TIME(0,0,0,700);
case stage::flying_nine:
          return TIME(0,0,0,750);
case stage::flying_ten:
          return TIME(0,0,0,800);
case stage::flying_eleven:
          return TIME(0,0,0,810);
case stage::flying_twelve:
          return TIME(0,0,0,820);
case stage::flying_thirteen:
          return TIME(0,0,0,830);
        case stage::hover:
          return TIME(0,0,0,819);
        case stage::hover_one:
          return TIME(0,0,0,809);
        case stage::hover_two:
          return TIME(0,0,0,799);
case stage::hover_three:
          return TIME(0,0,0,789);
case stage::hover_four:
          return TIME(0,0,0,769);
case stage::hover_five:
          return TIME(0,0,0,759);
case stage::hover_six:
          return TIME(0,0,0,740);
case stage::hover_seven:
          return TIME(0,0,0,699);
case stage::hover_eight:
          return TIME(0,0,0,640);
case stage::hover_nine:
          return TIME(0,0,0,599);
case stage::hover_ten:
          return TIME(0,0,0,540);
case stage::hover_eleven:
          return TIME(0,0,0,499);
case stage::hover_twelve:
          return TIME(0,0,0,440);
case stage::hover_thirteen:
          return TIME(0,0,0,440);
        case stage::landing:
          return TIME(0,0,0,399);
        case stage::landing_one:
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


