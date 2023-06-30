#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();         // Create motor shield object
Adafruit_StepperMotor *panMotor = AFMS.getStepper(200, 1);  // Connect stepper motor to port #1
Adafruit_StepperMotor *tiltMotor = AFMS.getStepper(200, 2); // Connect stepper motor to port #2

// #define panSwitchPin 2
#define tiltSwitchPin 12 // Connect tilt limit switch to pin 12

bool panSwitchEnabled = false;
bool tiltSwitchEnabled = false;
int panSwitchState;
int tiltSwitchState;
signed long panMotorPosition;
signed long tiltMotorPosition;

void setup()
{
  Serial.begin(115200);
  //  pinMode(panSwitchPin, INPUT_PULLUP);
  pinMode(tiltSwitchPin, INPUT_PULLUP);
  while (!Serial)
    ;
  AFMS.begin(); // Initialize motor shield

  panMotor->setSpeed(10); // Set motor speed (10 rpm)
  tiltMotor->setSpeed(10);

  attachInterrupt(0, magnet_detect, RISING); // Initialize the magnet intterrupt pin (Arduino digital pin 2)

  // Calibrate pan iwth magnet
  while (!panSwitchEnabled)
  {
    panMotor->step(1, FORWARD, SINGLE); // Step motor forward (unless interrupted by magnet detection)
  }

  Serial.println("Pan stepper motor is now calibrated.");

  // Calibrate tilt with limit switch
  while (!tiltSwitchEnabled)
  {
    int tiltSwitchState = digitalRead(tiltSwitchPin);
    if (tiltSwitchState == HIGH)
    { // Check if the pan switch is enabled
      Serial.println(tiltSwitchEnabled);
      if (!tiltSwitchEnabled)
      {
        tiltMotor->release(); // Release motor if switch is enabled
        tiltSwitchEnabled = true;
      }
    }
    else
    {
      tiltSwitchEnabled = false;
      tiltMotor->step(1, FORWARD, SINGLE); // Step motor forward
    }
  }
  Serial.println("Tilt stepper motor is now calibrated.");

  panMotor->release(); // Release both motors
  tiltMotor->release();
}

void magnet_detect() // Called whenever a magnet/interrupt is detected by the arduino
{
  panSwitchEnabled = true;
  Serial.println("detect");
}

void loop()
{
  //  panMotor->step(1, FORWARD, SINGLE);  // Step motor forward
}