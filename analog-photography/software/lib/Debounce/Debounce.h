#ifndef Debounce_h
#define Debounce_h
#include "Arduino.h"
#define DEFAULT_DEBOUNCE_THRESHOLD_MS 50

enum SW_STATES {
    OPEN,
    CLOSED
};

class Debounce {
    private:
        int swPin;
        unsigned int debounceThresh_ms;
        enum SW_STATES state;
        enum SW_STATES oldState;
        int oldReading;
        unsigned long lastChange;
        int fellFlag;
        int roseFlag;

    public:
        Debounce(int pin);
        Debounce(int pin, int threshold_ms);
        SW_STATES getState();
        void poll();
        int rose();
        int fell();
};

#endif
