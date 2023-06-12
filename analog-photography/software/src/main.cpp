#include <Arduino.h>
#include <Servo.h>

// HW pin definitions
#define WINDING_LIM_PIN 2
#define SHUTTER_BTN 3
#define FILM_PIN 5
#define SHUTTER_PIN 6

// declare servos
Servo windServo;    // continuous rotation servo
Servo shutterServo;

int shutterPos = 0;     // shutter servo position

int shutterButtonState = LOW;
int lastShutterButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned long windDelay = 1000;
unsigned long lastShutterTime = 0;

enum windStates {
    UNWOUND, // switch open
    WOUND,    // switch closed
    WINDING         // switch open, winding servo moving
};

enum shutterStates {
    UNPRESSED,
    PRESSING,   // transition from unpressed -> pressed
    PRESSED,
    UNPRESSING  // transition from pressed -> unpressed
};

enum windStates winderState;
enum shutterStates shutterState = UNPRESSED;

void setup() {
    pinMode(WINDING_LIM_PIN, INPUT_PULLUP);
    pinMode(SHUTTER_BTN, INPUT_PULLUP);
    windServo.attach(FILM_PIN);
    shutterServo.attach(SHUTTER_PIN);

    // initialize winding limit switch
    (!digitalRead(WINDING_LIM_PIN)) ? winderState = WOUND : winderState = UNWOUND;
    windServo.write(90);
    shutterServo.write(0);
}

void loop() {
    // reverse the readings to correct for pullup resistor
    int windingLimitClosed = !digitalRead(WINDING_LIM_PIN);
    int shutterButtonReading = !digitalRead(SHUTTER_BTN);

    // debounce routine
    if (shutterButtonReading != lastShutterButtonState) {
        lastDebounceTime = millis();
    }

    // enough time has passed since state change to not be a bounce input
    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (shutterButtonReading != shutterButtonState) {
            shutterButtonState = shutterButtonReading;
        }
    }
    lastShutterButtonState = shutterButtonReading;

    // FSMs for film winding and triggering shutter
    switch (winderState) {
    case UNWOUND:   // shutter switch open, servo stopped
        // change to wound if shutter switch is closed
        if (windingLimitClosed) {
            windServo.write(90);
            winderState = WOUND;
        // otherwise check if enough time has passed and start winding if so
        } else if ((millis() - lastShutterTime) > windDelay) {
            winderState = WINDING;
            windServo.write(70);
        }
        break;
    case WINDING:   // shutter switch open, servo turning
        // change to wound if shutter switch is closed
        if (windingLimitClosed) {
            windServo.write(90);    // stop winding
            winderState = WOUND;
        }
        break;
    case WOUND:     // shutter switch closed, servo stopped
        windServo.write(90); // stop winding
        if (!windingLimitClosed) {  // change states to unwound if the switch opens (shutter was triggered)
            lastShutterTime = millis();
            winderState = UNWOUND;
        }
        break;
    default:
        windServo.write(90);
        break;
    }

    switch (shutterState) {
    case UNPRESSED:
        // if trigger button is pressed and the film is wound, press the shutter
        if (shutterButtonState && winderState == WOUND) {
            shutterServo.write(45);
            shutterState = PRESSING;
        }
        break;
    case PRESSING:
        // wait until the shutter is pressed (film is no longer wound)
        // and then move to unpressing state
        if (winderState == UNWOUND) {
            shutterState = UNPRESSING;
        }
        break;
    case UNPRESSING:
        // move servo to zero position and go to unpressed state
        shutterServo.write(0);
        shutterState = UNPRESSED;
        break;
    default:
        shutterServo.write(0);
        break;
    }
}
