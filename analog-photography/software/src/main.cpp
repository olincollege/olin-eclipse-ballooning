#include "Debounce.h"
#include "DispoCam.h"
#include <Arduino.h>
#include <Servo.h>

#define DEBUG true
#define DEBUG_SERIAL if (DEBUG) Serial

#define CAMERA_AMOUNT 4

// HW pin definitions
#define TRIGGER_BTN_PIN 7
int windServoPins[] = {2, 4, 6, 8};
int shutterServoPins[] = {3, 5, 7, 9};
int windLimitPins[] = {10, 11, 12, 13};

// declare servos
Servo windServos[CAMERA_AMOUNT];    // continuous rotation (90 is stopped)
Servo shutterServos[CAMERA_AMOUNT];

Debounce windLimits[CAMERA_AMOUNT];

DispoCam cameras[CAMERA_AMOUNT];

Debounce triggerButton;

int currentCamera = 0;
uint8_t camsInTargetState = 0;
CAM_STATES targetState;

void setup() {
    DEBUG_SERIAL.begin(9600);
    for (int i = 0; i < CAMERA_AMOUNT; i++) {
        windServos[i].attach(windServoPins[i]);
        shutterServos[i].attach(shutterServoPins[i]);
        windLimits[i].attach(windLimitPins[i]);
        cameras[i].attach(&windServos[i], &shutterServos[i], &windLimits[i]);
    }
    
    triggerButton.attach(TRIGGER_BTN_PIN);
}

void loop() {
    triggerButton.poll();
    camsInTargetState = 0;
    // update each camera FSM and store whether or not it's in the target state
    for (int i = 0; i < CAMERA_AMOUNT; i++) {
        cameras[i].update();
        camsInTargetState |= (cameras[i].getState() == targetState) << i;
    }

    // instruct cameras to either wind film or take a picture (depending on targetState)
    if (camsInTargetState & (1 << currentCamera)) {   // if current cam in target state,
        currentCamera++;    // go to next camera
    } else {
        cameras[currentCamera].readyNext(); // otherwise, tell current cam to move to target state
    }

    if (!~camsInTargetState) {    // if all cams have reached the target state (i.e. they're done doing what they were doing)
        if (targetState == UNWOUND) {   // after taking pictures, wind film immediately
            targetState = WOUND;
        } else if (targetState == WOUND && triggerButton.getState()) {  // after winding film, wait until triggered to take pictures
            targetState = UNWOUND;
        }
    }

    // reset the current camera index when needed
    if (currentCamera >= CAMERA_AMOUNT) {
        currentCamera = 0;
    }   
}
