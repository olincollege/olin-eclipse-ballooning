#include "Debounce.h"

Debounce::Debounce(int pin) {
    swPin = pin;
    debounceThresh_ms = DEFAULT_DEBOUNCE_THRESHOLD_MS;
    oldReading = OPEN;
    fellFlag = 0;
    roseFlag = 0;
    state = OPEN;
    oldState = CLOSED;
}

Debounce::Debounce(int pin, int threshold_ms) {
    swPin = pin;
    oldReading = OPEN;
    fellFlag = 0;
    roseFlag = 0;
    state = OPEN;
    oldState = CLOSED;
    debounceThresh_ms = threshold_ms;
}

SW_STATES Debounce::getState() {
    return state;
}

void Debounce::poll() {
    // reverse the readings to correct for pullup resistor
    int reading = !digitalRead(swPin);
    if (reading != oldReading) {
        lastChange = millis();
    } else  {
        if ((millis() - lastChange) > debounceThresh_ms && reading != state) {
            if (oldState == OPEN && reading == CLOSED) {
                roseFlag = 1;
                fellFlag = 0;
            } else if (oldState == CLOSED && reading == OPEN) {
                fellFlag = 1;
                roseFlag = 0;
            }
            oldState = state;
            state = reading;
        }
    }
    oldReading = reading;
}

int Debounce::rose() {
    if (roseFlag) {
        roseFlag = !roseFlag;   // clear flag
        return 1;
    }
    return 0;
}

int Debounce::fell() {
    if (fellFlag) {
        fellFlag = !fellFlag;
        return 1;
    }
    return 0;
}
