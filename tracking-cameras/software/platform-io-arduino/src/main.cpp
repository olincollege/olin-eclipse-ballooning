#include <Arduino.h>
#include <SolTrack.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <AccelStepper.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <SPI.h>

SFE_UBLOX_GNSS myGNSS;

// Define pin connections
#define hallEffectPin 2
#define topLimitPin 4
#define bottomLimitPin 5
const int panMotorDIR = 32;
const int panMotorSTEP = 31;
const int tiltMotorDIR = 7;
const int tiltMotorSTEP = 6;

// Define motor interface type
#define motorInterfaceType 1

AccelStepper panMotor(motorInterfaceType, panMotorSTEP, panMotorDIR);
AccelStepper tiltMotor(motorInterfaceType, tiltMotorSTEP, tiltMotorDIR);

bool panSwitchEnabled = false;
bool tiltSwitchEnabled = false;
int panSwitchState;
int tiltSwitchState;

unsigned long lastGPSPrint = 0;

// BNO Initialization
unsigned long lastIMURead = 0;

#define BNO08X_RESET -1

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t
{
  float yaw;
  float pitch;
  float roll;
} ypr;

void setReports(void);
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees);
void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees);

// Define Functions
void initializeGPS();
void initializeIMU();
void printGPSData();
void homeStepperMotors();
float compass();
float movingAverageFilter(float angle_deg);
void magnetInterrupt();
void topLimitInterrupt();
void bottomLimitInterrupt();

void computeSunPos();
void PVTUpdate(UBX_NAV_PVT_data_t *ubxDataStruct);
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
float convertTo360(float value);
void adjustCameraTiltAngle();
void adjustCameraPanAngle();

// Rolling Average Code
const int numReadings = 50;
float x_readings[numReadings] = {0}; // Array to store the x-coordinates of the readings
float y_readings[numReadings] = {0}; // Array to store the y-coordinates of the readings
int readIndex = 0;
float avgDeg; // Output of the moving average

// Variables for IMU Code
int magneticDeclination;
float adjustedValue;  // Cardinal direction adjusted for magnetic declination
float convertedValue; // Converts and maintains the adjusted value within the 360-deg scale

// Additional Variables for Sun Tracking logic
double azimuth, altitude;                  // double containing the azimuth and altitude of the sun.
float stepperPanAngle;                     // Current stepper motor position
float currentTiltMotorPosition;            // Current stepper motor position
const float PAN_TOLERANCE = 2.0;           // Tolerance for camera pan angle (in degrees)
const float TILT_TOLERANCE = 2.0;          // Tolerance for camera tilt angle (in degrees)
const float RECALIBRATION_TOLERANCE = 3.0; // Tolerance for re-running the code (in degrees)
const int INIT_PAN_POS = 0;
const int INIT_TILT_POS = 90;
const float PAN_GEAR_RATIO = 1.0 / 3.0;
const float TILT_GEAR_RATIO = 1.0 / 4.0;
const float STEPPER_STEP_SIZE = 1.8;
const float PAN_STEP_SIZE = PAN_GEAR_RATIO * STEPPER_STEP_SIZE / 2;
const float TILT_STEP_SIZE = TILT_GEAR_RATIO * STEPPER_STEP_SIZE / 4;

double currentPanPos = INIT_PAN_POS;
double currentTiltPos = INIT_TILT_POS;
// Global variables to track limit switch states
volatile bool topLimitReached = false;
volatile bool bottomLimitReached = false;

float timeDelta; // change in time
unsigned long millisOld;

// Global variables and structs:
const int useDegrees = 1;               // Input (geographic position) and output are in degrees
const int useNorthEqualsZero = 1;       // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
const int computeRefrEquatorial = 0;    // Compute refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
const int computeDistance = 0;          // Compute the distance to the Sun in AU: 0-no, 1-yes
const float FIXED_NOM_PRESSURE = 101.0; // Atmospheric pressure in kPa
const float FIXED_NOM_TEMP = 290.3;     // Atmospheric temperature in K

struct STTime sttime;  // Struct for date and time variables
struct STLocation loc; // Struct for geographic location variables
float hMSL;            // height above mean sea level (m)

void setup()
{
  while (!Serial)
  {
    Serial.begin(115200);
  }
  Wire.begin();
  Serial.println("Initializing GPS");
  initializeGPS();
  Serial.println("GPS Initialized, Initializing IMU");
  initializeIMU();
  Serial.println("IMU Initialized, Calibrating Stepper Motors");
  homeStepperMotors();
  Serial.println("Stepper Motors Calibrated, Starting Loop");
  computeSunPos();
  delay(2000);

  // using constant pressure and temperature since they barely affect the calculation
  loc.pressure = FIXED_NOM_PRESSURE;
  loc.temperature = FIXED_NOM_TEMP;
}

void loop()
{
  unsigned long now = millis();
  // Configure this so that GPS
  myGNSS.checkUblox();     // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.
  // if (loc.latitude != 0)
  if (now < 5000)
  {
    compass();
    Serial.println(now);
  }
  else
  {
    if (now - lastIMURead > 100)
    {
      compass();
      adjustCameraPanAngle();
      adjustCameraTiltAngle();
      lastIMURead = now;
    }
    if (now - lastGPSPrint > 2000)
    {
      printGPSData();
      lastGPSPrint = now;
    }
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t *rotational_vector, euler_t *ypr, bool degrees)
{
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t *ypr, bool degrees)
{
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees)
  {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void setReports(void)
{
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
  {
    Serial.println("Could not enable rotation vector");
  }
}

/**
 * Computes the compass direction based on sensor readings from the IMU.
 * @return The compass direction in degrees.
 */
float compass()
{
  if (bno08x.wasReset())
  {
    Serial.print("BNO sensor was reset, ");
    setReports();
  }
  if (bno08x.getSensorEvent(&sensorValue))
  {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR)
    {
      quaternionToEulerRV(&sensorValue.un.rotationVector, &ypr, true);
    }
  }
  // Apply the magnetic declination adjustment and subtract 90 to make north 0 degrees
  adjustedValue = ypr.yaw - 90 + magneticDeclination;

  // Ensure the adjusted value stays within the 360-degree range and convert the adjusted value to the 360-degree scale
  convertedValue = convertTo360(adjustedValue);

  return convertedValue;
}

void homeStepperMotors()
{

  // Set up the interrupt pins for the limit switches
  pinMode(topLimitPin, INPUT_PULLUP);
  pinMode(bottomLimitPin, INPUT_PULLUP);

  // Attach the interrupt handlers to the corresponding pins
  attachInterrupt(digitalPinToInterrupt(topLimitPin), topLimitInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(bottomLimitPin), bottomLimitInterrupt, CHANGE);
  attachInterrupt(hallEffectPin, magnetInterrupt, RISING);

  panMotor.setMaxSpeed(1000);
  panMotor.setAcceleration(100);
  panMotor.setSpeed(200);
  tiltMotor.setMaxSpeed(1000);
  tiltMotor.setAcceleration(100);
  tiltMotor.setSpeed(200);

  // Calibrate pan with magnet
  while (!panSwitchEnabled)
  {
    panMotor.runSpeed(); // Step motor forward (unless interrupted by magnet detection)
  }
  panMotor.stop();

  while (!digitalRead(topLimitPin))
  {
    tiltMotor.runSpeed(); // Step pan motor forward
  }
  tiltMotor.stop();
  Serial.println("Stepper Motors are now calibrated");
  delay(250);
  tiltMotor.setCurrentPosition(90 / TILT_STEP_SIZE);
  delay(250);
}

// Interrupt handler for the top limit switch
void topLimitInterrupt()
{
  topLimitReached = true;
}

// Interrupt handler for the bottom limit switch
void bottomLimitInterrupt()
{
  bottomLimitReached = true;
}
//! Potentially change the function to use a type so that it selects between the
//! two different pan/tilt movingAverageFilter(float angle_deg, int type)
float movingAverageFilter(float angle_deg)
{
  // Moving Average Filter
  float angle_rad = angle_deg * PI / 180.0; // Convert to radians

  // Convert the angle to Cartesian coordinates
  float x = cos(angle_rad);
  float y = sin(angle_rad);

  // Remove the oldest reading from the running totals
  x_readings[readIndex] = x;
  y_readings[readIndex] = y;

  // Add the new reading to the running totals
  float x_total = 0;
  float y_total = 0;
  for (int i = 0; i < numReadings; i++)
  {
    x_total += x_readings[i];
    y_total += y_readings[i];
  }
  // TODO: Find out the current number of values, and then divide the current
  // TODO: values by the number of values, if the number of values is less than numReadings.
  // Compute the average in Cartesian coordinates
  float x_avg = x_total / numReadings;
  float y_avg = y_total / numReadings;

  // Convert back to an angle
  float avg_rad = atan2(y_avg, x_avg);
  avgDeg = avg_rad * 180.0 / PI;
  if (avgDeg < 0)
    avgDeg += 360; // Ensure the angle is in the range [0, 360)

  // Update the index
  readIndex = (readIndex + 1) % numReadings;

  delay(1);

  return avgDeg;
}

void magnetInterrupt()
{ // Called whenever a magnet/interrupt is detected by the arduino
  panSwitchEnabled = true;
}

/*
 * Callback for when the GNSS module receives new data. Updates the time, date, latitude,
 * longitude, and height, and recalculates the sun's position.
 */
void PVTUpdate(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  // update solTrack time struct if we have a valid date and time to use
  if (ubxDataStruct->valid.bits.validDate && ubxDataStruct->valid.bits.validTime)
  {
    sttime = (STTime){
        (int)(ubxDataStruct->year), ubxDataStruct->month, ubxDataStruct->day,
        ubxDataStruct->hour, ubxDataStruct->min, (double)(ubxDataStruct->sec)};
  }

  // update latitude, longitude, and height if we have valid value to use
  // TODO: add accuracy tolerance threshold using ubxDataStruct->hAcc and ubxDataStruct->vAcc
  if (!ubxDataStruct->flags3.bits.invalidLlh)
  {
    loc.latitude = ubxDataStruct->lat / 10000000.0;  // deg * 1e-7 -> deg
    loc.longitude = ubxDataStruct->lon / 10000000.0; // deg * 1e-7 -> deg
    hMSL = ubxDataStruct->hMSL / 1000.0;             // mm -> m
  }

  computeSunPos();
}

void computeSunPos()
{
  struct STPosition pos;
  SolTrack(sttime, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);
  altitude = pos.altitudeRefract;
  azimuth = pos.azimuthRefract;
}

void printGPSData()
{
  char gpsOutputStr[96];

  // convert floats to str separately from sprintf b/c arduino doesn't support float in sprintf
  char latstr[12], longstr[12], hMSLstr[11];
  dtostrf(loc.latitude, 10, 6, latstr);
  dtostrf(loc.longitude, 11, 6, longstr);
  dtostrf(loc.longitude, 10, 6, hMSLstr);

  sprintf(gpsOutputStr, "Lat, Lon: %s, %s degrees\r\nhMSL: %s m\n%i-%02i-%02i %02i:%02i:%02i UTC\n",
          latstr, longstr, hMSLstr,
          sttime.year, sttime.month, sttime.day,
          sttime.hour, sttime.minute, sttime.second);
  Serial.println(gpsOutputStr);
}

/**
 * Adjusts the camera pan angle.
 */
void adjustCameraPanAngle()
{
  Serial.println("");
  Serial.println("-- Pan Detection Loop --");

  // Convert compass yaw to degrees East (positive clockwise)
  double boxAngle = convertTo360(ypr.yaw);

  // Calculate the difference between the target azimuth and the current motor position
  // azimuth = 30;
  double azimuthDiff = azimuth - boxAngle - currentPanPos;

  // Normalize the azimuth difference to the shortest distance within the range of -180 to 180 degrees
  if (azimuthDiff < -180.0)
  {
    azimuthDiff += 360.0;
  }
  else if (azimuthDiff > 180.0)
  {
    azimuthDiff -= 360.0;
  }

  double cameraDirection = currentPanPos + boxAngle;

  // Normalize camera direciton to be within the range of 0 to 360 degrees
  if (cameraDirection < 0)
  {
    cameraDirection += 360.0;
  }
  else if (cameraDirection > 360.0)
  {
    cameraDirection -= 360.0;
  }

  // Print current values
  Serial.print("Compass Average Output: ");
  Serial.println(avgDeg);
  Serial.print("Azimuth of the Sun: ");
  Serial.println(azimuth);
  Serial.print("Difference from the Azimuth of the Sun: ");
  Serial.println(azimuthDiff);
  Serial.print("Compass yaw: ");
  Serial.println(boxAngle);
  Serial.print("Current Pan Position: ");
  Serial.println(currentPanPos);
  Serial.print("Current Pan Motor Position: ");
  Serial.println(panMotor.currentPosition());
  Serial.print("Camera Direction: ");
  Serial.println(cameraDirection);

  // Determine the most efficient direction of movement (clockwise or counterclockwise)
  double absAzimuthDiff = abs(azimuthDiff);
  if (absAzimuthDiff > PAN_TOLERANCE)
  {
    // Convert the azimuth difference to the number of steps for the stepper motor
    int steps = static_cast<int>(absAzimuthDiff / PAN_STEP_SIZE);

    if (azimuthDiff < 0.0)
    {
      steps = -steps;
    }
    Serial.print("Steps to take: ");
    Serial.println(steps);

    // Step the motor in the appropriate direction
    panMotor.runToNewPosition(panMotor.currentPosition() + steps);

    // Update the current position
    currentPanPos += azimuthDiff;
    currentPanPos = fmod(currentPanPos, 360.0);
    if (currentPanPos < 0.0)
    {
      currentPanPos += 360.0;
    }
  }
  else
  {
    // Camera position is within the tolerance region
    Serial.print("Pan motor is in tolerance. Current position: ");
  }
}

void adjustCameraTiltAngle()
{
  // Calculate the difference between the target altitude and the current motor position
  altitude = 45;
  if (altitude < 0)
  {
    altitude = -altitude;
  }

  // set roll and pitch angles to a number between -180 to 180
  float rollDegrees = 360 - convertTo360(ypr.roll); // also flips the positive axis
  if (rollDegrees > 180.0)
  {
    rollDegrees -= 360.0;
  }

  float pitchDegrees = convertTo360(ypr.pitch);
  if (pitchDegrees > 180.0)
  {
    pitchDegrees -= 360.0;
  }

  float tiltAdj = rollDegrees * cos(currentPanPos * (PI / 180)) + pitchDegrees * sin(currentPanPos * (PI / 180));

  Serial.println("");
  Serial.println("-- Tilt Detection Loop --");

  float compassMountOffset = 0;

  float tiltDiff = altitude - currentTiltPos - tiltAdj - compassMountOffset;

  Serial.print("Gyro Pitch: ");
  Serial.println(ypr.pitch - compassMountOffset);
  Serial.print("Altitude of the Sun: ");
  Serial.println(altitude);
  Serial.print("Difference from the Altitude of the Sun: ");
  Serial.println(tiltDiff);
  Serial.print("Compass pitch: ");
  Serial.println(convertTo360(ypr.pitch));
  Serial.print("Compass roll: ");
  Serial.println(360 - convertTo360(ypr.roll));

  Serial.print("Current Tilt Position: ");
  Serial.println(currentTiltPos);
  Serial.print("Current Tilt Motor Position: ");
  Serial.println(tiltMotor.currentPosition());

  // Determine the most efficient direction of movement (up or down)
  if (abs(tiltDiff) > TILT_TOLERANCE)
  {
    // Convert the tilt difference to the number of steps for the stepper motor
    int steps = static_cast<int>(abs(tiltDiff) / TILT_STEP_SIZE);
    if (tiltDiff < 0.0)
    {
      steps = -steps;
    }
    Serial.print("Steps: ");
    Serial.println(steps);
    // TODO: Find the max number of steps that the tilting can happen before it reaches the limits, do not pass this point.
    int maxSteps = steps;
    Serial.print("Bottom limit: ");
    Serial.println(bottomLimitReached);
    Serial.println(-currentTiltPos / TILT_STEP_SIZE);
    Serial.print("Top limit: ");
    Serial.println(topLimitReached);
    Serial.println((90 - currentTiltPos) / TILT_STEP_SIZE);
    if (steps < 0)
    {
      // Limit the downward tilt to prevent going below the bottom limit switch
      if (!bottomLimitReached)
      {
        maxSteps = max(steps, -currentTiltPos / TILT_STEP_SIZE);
        // maxSteps = max(steps, 5);
      }
    }
    else
    {
      // Limit the upward tilt to prevent going above the top limit switch
      if (!topLimitReached)
      {
        Serial.println("here");
        maxSteps = min(steps, (90 - currentTiltPos) / TILT_STEP_SIZE);
        // maxSteps = min(steps, 90 / TILT_STEP_SIZE);
      }
    }
    maxSteps = max(steps, -currentTiltPos / TILT_STEP_SIZE);
    maxSteps = min(steps, (90 - currentTiltPos) / TILT_STEP_SIZE);
    Serial.print("MaxSteps: ");
    Serial.println(maxSteps);
    // Step the motor in the appropriate direction
    tiltMotor.runToNewPosition(maxSteps + tiltMotor.currentPosition());

    // Update the current position
    currentTiltPos += maxSteps * TILT_STEP_SIZE;
  }
  else
  {
    // Camera position is within the tolerance region
    Serial.print("Tilt motor is in tolerance. Current position: ");
    Serial.println(currentTiltPos);
  }
}

/**
 * Converts a value to the 360-degree scale by adjusting it within the range [0, 360).
 * @param value The value to be converted.
 * @return The converted value within the 360-degree scale.
 */
float convertTo360(float value)
{
  if (value < 0)
  {
    value += 360.0;
  }
  else if (value >= 360.0)
  {
    value -= 360.0;
  }
  return 360 - value;
}

void initializeGPS()
{
  if (myGNSS.begin() == false)
  { // Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      delay(10);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  // myGNSS.saveConfiguration();        // Optional: Save the current settings to flash and BBR

  myGNSS.setNavigationFrequency(2);         // Produce two solutions per second
  myGNSS.setAutoPVTcallbackPtr(&PVTUpdate); // Enable automatic NAV PVT messages with callback to PVTUpdate
}

void initializeIMU()
{
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!bno08x.begin_I2C())
  {
    Serial.println("Failed to find BNO08x chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++)
  {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }
  setReports();
  //* If you want to manually insert magnetic declination, uncomment this code
  // magneticDeclination = Serial.parseInt();
  // Serial.println("Input the magnetic declination in your current location:");
  // while (Serial.available() == 0) {
  // }
  magneticDeclination = -12;
  Serial.print("The magnetic declination is set to ");
  Serial.print(magneticDeclination);
  Serial.println(" degrees.");
}
