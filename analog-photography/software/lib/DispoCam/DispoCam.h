#ifndef DispoCam_h
#define DispoCam_h
#include "Arduino.h"
#include "Debounce.h"
#include "Servo.h"

const int DEFAULT_MAX_PICS = 36;

const int windDelay_ms = 1000;  // time in ms to wait before winding film

const int windServoStop = 90;   // "zero" position for servo to stop moving
const int windServoSpeed = 10;  // added to stop position for moving servo
const int shutterServoHome = 90;
const int shutterServoIncrement = 5;

enum CAM_STATES {
    UNWOUND,    // idle; limit open
    WINDING,    // wind servo active
    WOUND,      // servos stopped
    PRESSING,   // shutter servo active
};

class DispoCam {
    private:
        Servo *windServo;
        Servo *shutterServo;
        Debounce *limitSwitch;
        enum CAM_STATES state;
        int maxPics;
        int picsRemaining;
        int limitPin;
        int shutterPos = shutterServoHome;
        unsigned long lastShutterTime = 0;
        bool nextStateReady = false;
        uint8_t cameraIndex; 
        

    public:
        DispoCam();
        DispoCam(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch, int maxPics = DEFAULT_MAX_PICS);
        void attach(Servo *windServo, Servo *shutterServo, Debounce *limitSwitch, int maxPics = DEFAULT_MAX_PICS);
        void update();  // poll switches, update FSM
        bool servoActive(); // true if in a state where a servo is moving
        CAM_STATES getState();
        int getPicsRemaining();
        void readyNext();

};

#endif
