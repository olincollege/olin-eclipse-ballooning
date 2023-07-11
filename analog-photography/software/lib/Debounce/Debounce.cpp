#include "Debounce.h"

Debounce::Debounce() {}

SW_STATES Debounce::getState() const {
    return state;
}

void Debounce::attach(int pin) {
    pinMode(pin, INPUT_PULLUP);
    this->pin = pin;
}

int Debounce::getPin() {
    return pin;
}

void Debounce::poll() {
    // reverse the readings to correct for pullup resistor
    int reading = !digitalRead(pin);
    if (reading != oldReading) {
        lastChange = millis();
    } else  {
        if ((millis() - lastChange) > DEFAULT_DEBOUNCE_THRESHOLD_MS && reading != state) {
            oldState = state;
            state = static_cast<SW_STATES>(reading);
        }
    }
    oldReading = reading;
}
