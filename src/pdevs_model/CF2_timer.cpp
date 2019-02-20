#include <cstdint>

#include "CF2_timer.hpp"


extern "C" {
    void setLed_1();
    void setLed_2();
    void setLed_8();
    void time_loop();
    void initUsecTimer(void);
    uint64_t usecTimestamp(void);
}


bool CF2Timer::initialized = false;

int CF2Timer::start = -1;

void CF2Timer::initialize() {
    initUsecTimer();
    initialized = true;
    unsigned long long ts = usecTimestamp();
    //if (ts > 32767) setLed_1();
    //if (ts > 2147483647) setLed_2();
    start = (int) ts;
}

int CF2Timer::micro_seconds_since_started() {
    if (!initialized) { initialize(); }

    unsigned long long ts = usecTimestamp();
    //if (ts > 32767) setLed_1();
    //if (ts > 2147483647) setLed_2();
    int current = (int) ts;

    return current - start;

}

