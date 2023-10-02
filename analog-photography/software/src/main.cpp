#include "Debounce.h"
#include "DispoCam.h"
#include <Arduino.h>
#include <Servo.h>
#include <RTClib.h>
#include <SerialCommands.h>

#define DEBUG true
#define DEBUG_SERIAL \
    if (DEBUG)       \
    Serial

#define CAMERA_AMOUNT 4

// HW pin definitions
const int startButtonPin = 20;
const int windServoPins[] = {12, 10, 6, 16};
const int shutterServoPins[] = {11, 9, 5, 14};
const int windLimitPins[] = {21, 22, 23, 15};

RTC_DS3231 rtc;
DateTime lastPictureTime;
const int32_t pictureInterval = 10 * 60; // time between pictures in seconds

// ALL TIMES IN UTC
// eclipse times for junction, tx (30.4893°, -99.7714°, Height:	522m)
//    date    | type | p start  |a/t start | a/t max  | a/t end  |  p end
// 14.10.2023 |	 A   | 16:21:37 | 17:48:38 | 17:50:54 | 17:53:09 | 19:28:36
// 08.04.2024 |  T   | 18:14:38 | 19:32:23 | 19:33:55 | 19:35:28 | 20:54:57

// annular
// const DateTime partialEclipseStart = DateTime(2023, 10, 14, 16, 21, 37);    // 10/14/23, 16:21:37
// const DateTime eclipseStart        = DateTime(2023, 10, 14, 17, 48, 38);    // 10/14/23, 17:48:38
// const DateTime eclipseMax          = DateTime(2023, 10, 14, 17, 50, 54);    // 10/14/23, 17:50:54
// const DateTime eclipseEnd          = DateTime(2023, 10, 14, 17, 53, 9);     // 10/14/23, 17:53:09
// const DateTime partialEclipseEnd   = DateTime(2023, 10, 14, 19, 28, 36);    // 10/14/23, 19:28:36

// total
// const DateTime partialEclipseStart    = DateTime(2024, 4, 8, 18, 14, 38);   // 4/8/24, 18:14:38
// const DateTime eclipseStart           = DateTime(2024, 4, 8, 19, 32, 23);   // 4/8/24, 19:32:23
// const DateTime eclipseMax             = DateTime(2024, 4, 8, 19, 33, 55);   // 4/8/24, 19:33:55
// const DateTime eclipseEnd             = DateTime(2024, 4, 8, 19, 35, 28);   // 4/8/24, 19:35:28
// const DateTime partialEclipseEnd      = DateTime(2024, 4, 8, 20, 54, 57);   // 4/8/24, 20:54:57

#if DEBUG
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char buffer[40];
DateTime lastTimePrint = DateTime();
#endif

// declare servos
Servo windServos[CAMERA_AMOUNT]; // continuous rotation (90 is stopped)
Servo shutterServos[CAMERA_AMOUNT];

Debounce windLimits[CAMERA_AMOUNT];

DispoCam cameras[CAMERA_AMOUNT];

Debounce startButton;
bool triggerFlag = false;
// once set to true by button press, a picture is taken after the set pictureInterval elapses until
// out of film
bool startFlag = false;

int currentCamera = 0;
uint8_t camsInTargetState = 0b11111111 << CAMERA_AMOUNT;
uint8_t camsInTargetState_old = camsInTargetState;
CAM_STATES targetState = WOUND;

char serial_command_buffer_[32];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\n", " ");

unsigned long lastLedChange = 0;
int ledState = LOW;

void cmd_pic(SerialCommands *sender)
{
    triggerFlag = true;
}

SerialCommand cmd_pic_("PIC", cmd_pic);

void setup()
{
    DEBUG_SERIAL.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);

    if (!rtc.begin())
    {
        DEBUG_SERIAL.println("Couldn't find RTC");
        DEBUG_SERIAL.flush();
        digitalWrite(LED_BUILTIN, HIGH); // turn LED on if there's a problem
        while (1)
            delay(10);
    }

    if (rtc.lostPower())
    {
        DEBUG_SERIAL.println("RTC lost power, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)) + TimeSpan(0, 4, 0, 0)); // adding 4 hours to convert from EDT to UTC
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }

    for (int i = 0; i < CAMERA_AMOUNT; i++)
    {
        windServos[i].attach(windServoPins[i]);
        shutterServos[i].attach(shutterServoPins[i]);
        windLimits[i].attach(windLimitPins[i]);
        cameras[i].attach(&windServos[i], &shutterServos[i], &windLimits[i]);
    }
    startButton.attach(startButtonPin);
    serial_commands_.AddCommand(&cmd_pic_);
}

// only sets the trigger flag to true - it is unset in the loop when the shutter activates
// this prevents an early trigger (e.g. when the film is being wound) from being ignored
void updateTriggerCondition()
{
    startButton.poll();
    DateTime now = rtc.now();
    unsigned long nowMillis = millis();
    if (startFlag)
    {
        if (nowMillis - lastLedChange > 500) // blink LED if start flag on
        {
            ledState = !ledState;
            digitalWrite(LED_BUILTIN, ledState);
            lastLedChange = nowMillis;
        }
        if ((!triggerFlag) && ((now - lastPictureTime).totalseconds() >= pictureInterval))
        {
            DEBUG_SERIAL.println("Trigger flag set!");
            triggerFlag = true;
            lastPictureTime = now;
        }
    }
    else if (startButton.getState())
    {
        DEBUG_SERIAL.println("Start flag set!");
        startFlag = true;
        lastPictureTime = now;
    }
    // triggerFlag = startFlag && (startButton.getState() || triggerFlag);    // for manual operation
}

void loop()
{
#if DEBUG
    DateTime now = rtc.now();
    if ((now - lastTimePrint).totalseconds() >= 10)
    {
        sprintf(buffer, "%02d/%02d/%02d (%s) %02d:%02d:%02d",
                now.year(), now.month(), now.day(), daysOfTheWeek[now.dayOfTheWeek()],
                now.hour(), now.minute(), now.second());
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.println(buffer);
        lastTimePrint = now;
    }
#endif
    updateTriggerCondition();
    camsInTargetState = 0b11111111 << CAMERA_AMOUNT; // reset target state check
    // update each camera FSM and store whether or not it's in the target state
    for (int i = 0; i < CAMERA_AMOUNT; i++)
    {
        cameras[i].update();
        camsInTargetState |= (cameras[i].getState() == targetState) << i;
    }
    if (camsInTargetState != camsInTargetState_old)
    {
        DEBUG_SERIAL.print("camsInTargetState = ");
        DEBUG_SERIAL.println(camsInTargetState, 2);
    }
    // instruct cameras to either wind film or take a picture (depending on targetState)
    if (camsInTargetState & (1 << currentCamera)) // if current cam in target state,
    {
        if (camsInTargetState != camsInTargetState_old)
        {
            DEBUG_SERIAL.print("Current camera ");
            DEBUG_SERIAL.print(currentCamera);
            DEBUG_SERIAL.println(" reached target state.");
        }
        currentCamera++; // go to next camera
    }
    else
    {
        cameras[currentCamera].readyNext(); // otherwise, tell current cam to move to target state
    }

    uint8_t notCamsInTargetState = ~camsInTargetState; // need to do in separate step to prevent weird compiler warning
    if (!notCamsInTargetState)
    { // if all cams have reached the target state (i.e. they're done doing what they were doing)
        currentCamera = 0;
        if (camsInTargetState != camsInTargetState_old)
            DEBUG_SERIAL.println("All cameras have reached target state!");

        if (targetState == UNWOUND)
        { // after taking pictures, wind film immediately
            DEBUG_SERIAL.println("Changing target state to WOUND. (winding film)");
            targetState = WOUND;
        }
        else if (targetState == WOUND && triggerFlag)
        { // after winding film, wait until triggered to take pictures
            DEBUG_SERIAL.println("Changing target state to UNWOUND. (taking pictures)");
            targetState = UNWOUND;
            triggerFlag = false;
        }
        // DEBUG_SERIAL.println();
    }

    // reset the current camera index when needed
    if (currentCamera >= CAMERA_AMOUNT)
    {
        currentCamera = 0;
        DEBUG_SERIAL.println("Reset current camera to 0.");
    }
    camsInTargetState_old = camsInTargetState;
    serial_commands_.ReadSerial();
}
