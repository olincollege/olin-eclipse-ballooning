#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create motor shield object
Adafruit_StepperMotor *panMotor = AFMS.getStepper(200, 2);  // Connect stepper motor to port #1
Adafruit_StepperMotor *tiltMotor = AFMS.getStepper(200, 1);  // Connect stepper motor to port #2

#define panSwitchPin 12
#define tiltSwitchPin 5

bool panSwitchEnabled = false;
bool tiltSwitchEnabled = false;
int panSwitchState;
int tiltSwitchState;
signed long panMotorPosition;
signed long tiltMotorPosition;

void setup() {
  Serial.begin(9600);
  pinMode(panSwitchPin, INPUT_PULLUP);
  pinMode(tiltSwitchPin, INPUT_PULLUP);
  while (!Serial);
  AFMS.begin();  // Initialize motor shield

  panMotor->setSpeed(10);  // Set motor speed (10 rpm)
  tiltMotor->setSpeed(10);

  int tiltSwitchState = digitalRead(tiltSwitchPin);
  while (!panSwitchEnabled){
    int panSwitchState = digitalRead(panSwitchPin);
    if (panSwitchState == HIGH) {  // Check if the pan switch is enabled
      Serial.println(panSwitchEnabled);
      if (!panSwitchEnabled) {
        // panMotor->release();  // Release motor if switch is enabled
        panSwitchEnabled = true;
      }
    } else {
      panSwitchEnabled = false;
      panMotor->step(1, FORWARD, SINGLE);  // Step motor forward
    }
  }
  Serial.println("Pan stepper motor is now calibrated.");
}

void loop() {
  panMotor->step(1, FORWARD, SINGLE);  // Step motor forward
}
