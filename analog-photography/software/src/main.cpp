#include <Arduino.h>
#include <Servo.h>

#define SHUTTER_PIN 3
Servo shutterServo;

int shutterPos = 0;

void setup() {
    shutterServo.attach(SHUTTER_PIN);
}

void loop() {
    for (shutterPos = 0; shutterPos <= 180; shutterPos += 30) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    shutterServo.write(shutterPos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15 ms for the servo to reach the position
  }
  for (shutterPos = 180; shutterPos >= 0; shutterPos -= 30) { // goes from 180 degrees to 0 degrees
    shutterServo.write(shutterPos);              // tell servo to go to position in variable 'pos'
    delay(500);                       // waits 15 ms for the servo to reach the position
  }
}
