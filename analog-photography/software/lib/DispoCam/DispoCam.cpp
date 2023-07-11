#include "DispoCam.h"

DispoCam::DispoCam() {}

DispoCam::DispoCam(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch) :
    windServo(windServo), shutterServo(shutterServo), limitSwitch(limitSwitch),
    state(!digitalRead(limitSwitch->getPin()) ? WOUND : UNWOUND)
{
    windServo->write(windServoStop);
    shutterServo->write(shutterServoHome);
}

CAM_STATES DispoCam::getState() {
    return state;
}

void DispoCam::attach(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch) {
    this->windServo = windServo;
    this->shutterServo = shutterServo;
    this->limitSwitch = limitSwitch;
    windServo->write(windServoStop);
    shutterServo->write(shutterServoHome);
    this->state = !digitalRead(limitSwitch->getPin()) ? WOUND : UNWOUND;
}

void DispoCam::readyNext() {
    nextStateReady = !servoActive();
}

bool DispoCam::servoActive() {
    return (state == WINDING || state == PRESSING);
}

void DispoCam::update() {
    limitSwitch->poll();

    // FSM for winding film and taking pictures
    switch (state) {
    case UNWOUND:   // reset shutter servo and wait some amount of time
        // ACTION: reset shutter servo (move to home/idle position)
        shutterPos = shutterServoHome;
        shutterServo->write(shutterPos);
        // STATE CHANGE: if enough time passed → WINDING
        if ((millis() - lastShutterTime) > windDelay_ms && nextStateReady) {
            state = WINDING;
            nextStateReady = false;
            // DEBUG_SERIAL.println("UNWOUND -> WINDING");
        }
        break;
    case WINDING:   // wind film until it's wound (switch closes)
        // ACTION: turn wind servo, poll limit switch
        windServo->write(windServoStop + windServoSpeed);

        // STATE CHANGE: if wind switch is closed → WOUND
        if (limitSwitch->getState()) {
            state = WOUND;
            nextStateReady = false;
            // DEBUG_SERIAL.println("WINDING -> WOUND");
        }
        break;
    case WOUND:     // done winding, stop the servo and wait to take picture
        // ACTION: stop wind servo
        windServo->write(windServoStop);

        // STATE CHANGE: if trigger button pressed → PRESSING
        if (nextStateReady) {
            state = PRESSING;
            nextStateReady = false;
            // DEBUG_SERIAL.println("WOUND -> PRESSING");
        }
        break;
    case PRESSING:  // take a picture
        // ACTION: incrementally move shutter servo
        shutterPos += shutterServoIncrement;
        if (shutterPos >= 170) {    // constrain servo angle to 10-170°
            shutterPos = 170;
        } else if (shutterPos < 10) {
            shutterPos = 10;
        }
        shutterServo->write(shutterPos);
        
        // STATE CHANGE: wind switch open → UNWOUND
        if (!limitSwitch->getState()) {
            state = UNWOUND;
            lastShutterTime = millis(); // reset timer
            nextStateReady = false;
            // DEBUG_SERIAL.println("PRESSING -> UNWOUND");
        }
        break;
    default:
        break;
    }
}
