#include "Debounce.h"

Debounce::Debounce(int pin, int debounceThresh_ms /* = DEFAULT_DEBOUNCE_THRESHOLD_MS*/) : 
    pin(pin), debounceThresh_ms(debounceThresh_ms) {}

SW_STATES Debounce::getState() const {
    return state;
}

void Debounce::poll() {
    // reverse the readings to correct for pullup resistor
    int reading = !digitalRead(pin);
    if (reading != oldReading) {
        lastChange = millis();
    } else  {
        if ((millis() - lastChange) > debounceThresh_ms && reading != state) {
            oldState = state;
            state = static_cast<SW_STATES>(reading);
        }
    }
    oldReading = reading;
}
