#include <ecdboost/simulation.hpp>

using namespace std;
using namespace ecdboost;

#ifndef ENABLE_SIMULATION
extern "C" {
  void set_led_GR();
  void set_led_RR();
}
#endif

using Value = int;

template<class TIME, class MSG>
class CommandInputPort: public port<TIME, MSG> {
  public:
    explicit CommandInputPort(const std::string &name, const TIME &polling) noexcept
      : port<TIME, MSG>(name, polling) { }

    void print() noexcept {}
    bool pDriver(typename port<TIME, MSG>::MSG_VALUE &v) const noexcept {
      v = interaction_counter;
      interaction_counter = (interaction_counter + 1) % 4;

      #ifndef ENABLE_SIMULATION
      if (interaction_counter == 1) set_led_GR();
      else set_led_RR();
      #endif

      return true;
    }

  protected:
    mutable int interaction_counter;
};

