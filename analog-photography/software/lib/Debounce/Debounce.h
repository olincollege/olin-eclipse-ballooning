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
        int pin;
        unsigned int debounceThresh_ms;
        enum SW_STATES state = OPEN;
        enum SW_STATES oldState = OPEN;
        int oldReading = OPEN;
        unsigned long lastChange = 0;

    public:
        Debounce(int pin, int debounceThresh_ms = DEFAULT_DEBOUNCE_THRESHOLD_MS);
        SW_STATES getState() const;
        void poll();
};

#endif
