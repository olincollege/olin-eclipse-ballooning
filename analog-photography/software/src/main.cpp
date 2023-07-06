#include "Debounce.h"
#include <Arduino.h>
#include <Servo.h>

#define DEBUG true
#define DEBUG_SERIAL if (DEBUG) Serial

// HW pin definitions
#define WINDING_LIM_PIN 2
#define TRIGGER_BTN_PIN 7
#define FILM_PIN 5
#define SHUTTER_PIN 8

// declare servos
Servo windServo;    // continuous rotation (90 is stopped)
Servo shutterServo;

const int windServoStop = 90;   // "zero" position for servo to stop moving
const int windServoSpeed = 20;  // added to stop position for moving servo
const int shutterServoHome = 20;
const int shutterServoIncrement = 10;

int shutterPos;     // current shutter servo position

Debounce triggerButton = Debounce(TRIGGER_BTN_PIN);
Debounce windLimSwitch = Debounce(WINDING_LIM_PIN);

unsigned long windDelay = 1000;
unsigned long lastShutterTime = 0;

enum cameraStates {
    UNWOUND,    // idle; limit open
    WINDING,    // wind servo active
    WOUND,      // servos stopped
    PRESSING,   // shutter servo active
    UNPRESSING  // shutter servo homing
};

enum cameraStates cameraState;

void setup() {
    DEBUG_SERIAL.begin(9600);
    pinMode(WINDING_LIM_PIN, INPUT_PULLUP);
    pinMode(TRIGGER_BTN_PIN, INPUT_PULLUP);
    windServo.attach(FILM_PIN);
    shutterServo.attach(SHUTTER_PIN);

    // initialize winding limit switch
    (!digitalRead(WINDING_LIM_PIN)) ? cameraState = WOUND : cameraState = UNWOUND;
    windServo.write(windServoStop);
    shutterServo.write(shutterServoHome);
    shutterPos = shutterServoHome;
}

void loop() {
    triggerButton.poll();
    windLimSwitch.poll();

    // FSM for winding film and taking pictures
    switch (cameraState) {
    case UNWOUND:   // reset shutter servo and wait some amount of time
        // ACTION: reset shutter servo (move to home/idle position)
        shutterPos = shutterServoHome;
        shutterServo.write(shutterPos);

        // STATE CHANGE: if enough time passed → WINDING
        if ((millis() - lastShutterTime) > windDelay) {
            cameraState = WINDING;
            DEBUG_SERIAL.println("UNWOUND -> WINDING");
        }
        break;
    case WINDING:   // wind film until it's wound (switch closes)
        // ACTION: turn wind servo, poll limit switch
        windServo.write(windServoStop + windServoSpeed);

        // STATE CHANGE: if wind switch is closed → WOUND
        if (windLimSwitch.getState()) {
            cameraState = WOUND;
            DEBUG_SERIAL.println("WINDING -> WOUND");
        }
        break;
    case WOUND:     // done winding, stop the servo and wait to take picture
        // ACTION: stop wind servo
        windServo.write(windServoStop);

        // STATE CHANGE: if trigger button pressed → PRESSING
        if (triggerButton.getState()) {
            cameraState = PRESSING;
            DEBUG_SERIAL.println("WOUND -> PRESSING");
        }
        break;
    case PRESSING:  // take a picture
        // STATE CHANGE: wind switch open → UNWOUND
        if (!windLimSwitch.getState()) {
            cameraState = UNWOUND;
            lastShutterTime = millis(); // reset timer
            DEBUG_SERIAL.println("PRESSING -> UNWOUND");
        } else { // only move servo anymore if not changing state
        // ACTION: incrementally move shutter servo
            shutterPos += shutterServoIncrement;
            // constrain servo angle to 0-180°
            if (shutterPos >= 180) {
                shutterPos = 180;
            } else if (shutterPos < 0) {
                shutterPos = 0;
            }
            shutterServo.write(shutterPos);
        }
        break;
    default:
        break;
    }
}
