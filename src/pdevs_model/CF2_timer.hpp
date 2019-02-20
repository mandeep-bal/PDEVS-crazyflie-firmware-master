#ifndef CF2_TIMER__HPP
#define CF2_TIMER__HPP

class CF2Timer {
    public:
        static void initialize();
        static int micro_seconds_since_started();

    private:
        static bool initialized;
        static int start;
};


#endif  // CF2_TIMER__HPP 

