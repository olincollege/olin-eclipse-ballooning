#include "DispoCam.h"

#define DEBUG false
#define DEBUG_SERIAL if (DEBUG) Serial

uint8_t cameraCount = 0;

DispoCam::DispoCam() {
    cameraCount++;
    cameraIndex = cameraCount - 1;
}

DispoCam::DispoCam(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch, int maxPics /* = DEFAULT_MAX_PICS*/) :
    windServo(windServo), shutterServo(shutterServo), limitSwitch(limitSwitch),
    state(!digitalRead(limitSwitch->getPin()) ? WOUND : UNWOUND), maxPics(maxPics), picsRemaining(maxPics)
{
    windServo->write(windServoStop);
    shutterServo->write(shutterServoHome);
    cameraCount++;
    cameraIndex = cameraCount - 1;
}

CAM_STATES DispoCam::getState() {
    return state;
}

int DispoCam::getPicsRemaining() {
    return picsRemaining;
}

void DispoCam::attach(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch, int maxPics /* = DEFAULT_MAX_PICS*/) {
    this->windServo = windServo;
    this->shutterServo = shutterServo;
    this->limitSwitch = limitSwitch;
    this->maxPics = maxPics;
    this->picsRemaining = maxPics;
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
        if ((millis() - lastShutterTime) > windDelay_ms && nextStateReady && picsRemaining > 0) {
            state = WINDING;
            nextStateReady = false;
            DEBUG_SERIAL.print("Camera ");
            DEBUG_SERIAL.print(cameraIndex);
            DEBUG_SERIAL.println(": UNWOUND -> WINDING");
        }
        break;
    case WINDING:   // wind film until it's wound (switch closes)
        // ACTION: turn wind servo, poll limit switch
        windServo->write(windServoStop + windServoSpeed);

        // STATE CHANGE: if wind switch is closed → WOUND
        if (limitSwitch->getState()) {
            state = WOUND;
            windServo->write(windServoStop);
            nextStateReady = false;
            DEBUG_SERIAL.print("Camera ");
            DEBUG_SERIAL.print(cameraIndex);
            DEBUG_SERIAL.println(": WINDING -> WOUND");
        }
        break;
    case WOUND:     // done winding, stop the servo and wait to take picture
        // ACTION: stop wind servo
        windServo->write(windServoStop);

        // STATE CHANGE: if trigger button pressed → PRESSING
        if (nextStateReady) {
            state = PRESSING;
            nextStateReady = false;
            DEBUG_SERIAL.print("Camera ");
            DEBUG_SERIAL.print(cameraIndex);
            DEBUG_SERIAL.println(": WOUND -> PRESSING");
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
            picsRemaining--;
            state = UNWOUND;
            lastShutterTime = millis(); // reset timer
            nextStateReady = false;
            DEBUG_SERIAL.print("Camera ");
            DEBUG_SERIAL.print(cameraIndex);
            DEBUG_SERIAL.println(": PRESSING -> UNWOUND");
        }
        break;
    default:
        break;
    }
}
