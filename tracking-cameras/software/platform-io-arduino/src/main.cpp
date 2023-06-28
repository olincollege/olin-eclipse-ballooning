#include <Arduino.h>
#include <SolTrack.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <EEPROM.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
// #include <IMU_func.h>
#include <SPI.h>

SFE_UBLOX_GNSS myGNSS;

// Stepper Motor Initialization
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create motor shield object
Adafruit_StepperMotor *panMotor = AFMS.getStepper(200, 1);  // Connect stepper motor to port #1
Adafruit_StepperMotor *tiltMotor = AFMS.getStepper(200, 2);  // Connect stepper motor to port #2

#define panSwitchPin 13
#define tiltSwitchPin 12

bool panSwitchEnabled = false;
bool tiltSwitchEnabled = false;
int panSwitchState;
int tiltSwitchState;
// signed long panMotorPosition;
// signed long tiltMotorPosition;

unsigned long lastGPSPrint = 0;

// BNO Initialization
unsigned long lastIMURead = 0;
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Define Functions
void initializeGPS();
void initializeIMU();
void printGPSData();
void homeStepperMotors();
float compass();
float movingAverageFilter(float angle_deg);

void computeSunPos();
void PVTUpdate(UBX_NAV_PVT_data_t *ubxDataStruct);
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
float convertTo360(float value);
void adjustCameraTiltAngle();
void adjustCameraPanAngle();

// Rolling Average Code
const int numReadings = 10;
float x_readings[numReadings] = {0}; // Array to store the x-coordinates of the readings
float y_readings[numReadings] = {0}; // Array to store the y-coordinates of the readings
int readIndex = 0;
float avgDeg; // Output of the moving average

// Variables for IMU Code

float thetaM; // Measured Pitch
float phiM; // Measured Roll
float thetaFold=0; // Old Filtered Value of Accelerometer Pitch
float thetaFnew; // Updated Filtered Value of Accelerometer Pitch
float phiFold=0; // Old Filtered Value of Accelerometer Pitch
float phiFnew; // Updated Filtered Value of Accelerometer Pitch

float thetaG=0; // Gyro Pitch
float phiG=0; // Gyro Roll

float theta; // Accelerometer Pitch
float phi; // Accelerometer Roll

float thetaRad; // Pitch in Radians
float phiRad; // Pitch in Radians

float Xm; // x-component of magnetometer
float Ym; // y-component of magnetometer
float psi; // (Yaw/Heading angle) Angle of Magnetometer tilt, using the x and y components of magetometer
int magneticDeclination;
float adjustedValue; // Cardinal direction adjusted for magnetic declination
float convertedValue; // Converts and maintains the adjusted value within the 360-deg scale

// Additional Variables for Sun Tracking logic
double azimuth, altitude; // double containing the azimuth and altitude of the sun.
float stepperPanAngle;  // Current stepper motor position
float currentTiltMotorPosition;  // Current stepper motor position
const float PAN_TOLERANCE = 5.0;  // Tolerance for camera pan angle (in degrees)
const float TILT_TOLERANCE = 5.0;  // Tolerance for camera tilt angle (in degrees)
const float RECALIBRATION_TOLERANCE = 4.0;  // Tolerance for re-running the code (in degrees)
const int INIT_PAN_POS = 0;
const int INIT_TILT_POS = 90;
const float GEAR_RATIO = 1.0 / 3.0;
const float STEPPER_STEP_SIZE = 1.8;
const float STEP_SIZE = GEAR_RATIO * STEPPER_STEP_SIZE;

double currentPanPos = INIT_PAN_POS;
float currentTiltPos = INIT_TILT_POS;

float timeDelta; // change in time
unsigned long millisOld;

// Global variables and structs:
const int useDegrees = 1;             // Input (geographic position) and output are in degrees
const int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
const int computeRefrEquatorial = 0;  // Compute refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
const int computeDistance = 0;        // Compute the distance to the Sun in AU: 0-no, 1-yes
// const float LONGITUDE = -71.2639859;  // Olin College of Engineering
// const float LATITUDE = 42.2929003;
const float FIXED_NOM_PRESSURE = 101.0;      // Atmospheric pressure in kPa
const float FIXED_NOM_TEMP = 290.3;      // Atmospheric temperature in K

struct STTime sttime;               // Struct for date and time variables
struct STLocation loc;            // Struct for geographic location variables
float hMSL;   // height above mean sea level (m)


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Initializing GPS");
  initializeGPS();
  Serial.println("GPS Initialized, Initializing IMU");
  initializeIMU();
  Serial.println("IMU Initialized, Calibrating Stepper Motors");
  homeStepperMotors();
  Serial.println("Stepper Motors Calibrated, Starting Loop");
  
  // using constant pressure and temperature since they barely affect the calculation
  loc.pressure    = FIXED_NOM_PRESSURE;
  loc.temperature = FIXED_NOM_TEMP;
}

void loop() {
  unsigned long now = millis();
  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  if (now - lastIMURead > 100) {
    movingAverageFilter(compass());
    Serial.print("Compass Output: ");
    Serial.println(avgDeg);
    Serial.println("Adjusting Pan Angle");
    adjustCameraPanAngle();
    Serial.println("Adjusting Tilt Angle");
    adjustCameraTiltAngle();
    lastIMURead = now;
  }
  if (now - lastGPSPrint > 2000) {
    printGPSData();
  }
}

/**
 * Computes the compass direction based on sensor readings from the IMU.
 *
 * @param bno The Adafruit_BNO055 object representing the IMU sensor.
 * @return The compass direction in degrees.
 */
float compass() {
  uint8_t system, gyro, accel, mg = 0;
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Extracting values from the sensors on the IMU
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  // Calculate thetaM and phiM based on accelerometer readings
  thetaM = -atan2(acc.x() / 9.8, acc.z() / 9.8) / (2 * 3.141592654) * 360;
  phiM = -atan2(acc.y() / 9.8, acc.z() / 9.8) / (2 * 3.141592654) * 360;
  
  // Update the filtered values for phi and theta
  phiFnew = 0.95 * phiFold + 0.05 * phiM;
  thetaFnew = 0.95 * thetaFold + 0.05 * thetaM;
  
  // Calculate the time difference between the current and previous readings
  timeDelta = (millis() - millisOld) / 1000.0;
  millisOld = millis();
  
  // Update the gyro-based values for phi and theta
  theta = (theta + gyr.y() * timeDelta) * 0.95 + thetaM * 0.05; // Degrees
  phi = (phi - gyr.x() * timeDelta) * 0.95 + phiM * 0.05; // Degrees
  thetaG = thetaG + gyr.y() * timeDelta;
  phiG = phiG - gyr.x() * timeDelta;
  
  // Convert phi and theta to radians
  phiRad = phi / (360 * 2 * 3.141592654); // Radians
  thetaRad = theta / 360 * (2 * 3.141592654); // Radians

  // Calculate the compensated x-component of the magnetometer
  Xm = mag.x() * cos(thetaRad) - mag.y() * sin(phiRad) * sin(thetaRad) + mag.z() * cos(phiRad) * sin(thetaRad);
  
  // Calculate the compensated y-component of the magnetometer
  Ym = mag.y() * cos(phiRad) + mag.z() * sin(phiRad);
  
  // Calculate the angle of the magnetometer tilt (psi) in degrees
  psi = (atan2(Ym, Xm) * 180) / 3.141592654;
  
  // Update the values for phi and theta
  phiFold=phiFnew;
  thetaFold=thetaFnew;

  // Apply the magnetic declination adjustment
  adjustedValue = psi + magneticDeclination;

  // Ensure the adjusted value stays within the 360-degree range and convert the adjusted value to the 360-degree scale
  convertedValue = convertTo360(adjustedValue);

  return convertedValue;
}

void homeStepperMotors(){
  pinMode(panSwitchPin, INPUT_PULLUP);
  pinMode(tiltSwitchPin, INPUT_PULLUP);
  while (!Serial);
  AFMS.begin();  // Initialize motor shield

  while (!panSwitchEnabled || !tiltSwitchEnabled) {
    panSwitchState = digitalRead(panSwitchPin);
    tiltSwitchState = digitalRead(tiltSwitchPin);

    if (panSwitchState == HIGH) {  // Check if the pan switch is enabled
      if (!panSwitchEnabled) {
        Serial.println("Pan Switch Engaged!");
        panMotor->release();  // Release pan motor if switch is enabled
        panSwitchEnabled = true;
        stepperPanAngle = 0.0;
      }
    } else {
      panSwitchEnabled = false;
      panMotor->step(1, FORWARD, SINGLE);  // Step pan motor forward
    }

    if (tiltSwitchState == HIGH) {  // Check if the tilt switch is enabled
      if (!tiltSwitchEnabled) {
        Serial.println("Tilt Switch Engaged!");
        tiltMotor->release();  // Release tilt motor if switch is enabled
        tiltSwitchEnabled = true;
        currentTiltMotorPosition = 0.0;
      }
    } else {
      tiltSwitchEnabled = false;
      tiltMotor->step(1, FORWARD, SINGLE);  // Step tilt motor forward
    }
  }
  delay(500);
}

float movingAverageFilter(float angle_deg){
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
  for (int i = 0; i < numReadings; i++) {
    x_total += x_readings[i];
    y_total += y_readings[i];
  }

  // Compute the average in Cartesian coordinates
  float x_avg = x_total / numReadings;
  float y_avg = y_total / numReadings;

  // Convert back to an angle
  float avg_rad = atan2(y_avg, x_avg);
  avgDeg = avg_rad * 180.0 / PI;
  if (avgDeg < 0) avgDeg += 360; // Ensure the angle is in the range [0, 360)

  // Update the index
  readIndex = (readIndex + 1) % numReadings;

  delay(1);

  return avgDeg;
}

/*
* Callback for when the GNSS module receives new data. Updates the time, date, latitude,
* longitude, and height, and recalculates the sun's position.
*/
void PVTUpdate(UBX_NAV_PVT_data_t *ubxDataStruct) {
  // update solTrack time struct if we have a valid date and time to use
  if (ubxDataStruct->valid.bits.validDate && ubxDataStruct->valid.bits.validTime) {
    sttime = (STTime){
      ubxDataStruct->year, ubxDataStruct->month, ubxDataStruct->day,
      ubxDataStruct->hour, ubxDataStruct->min, (double)(ubxDataStruct->sec)
    };
  }

  // update latitude, longitude, and height if we have valid value to use
  // TODO: add accuracy tolerance threshold using ubxDataStruct->hAcc and ubxDataStruct->vAcc
  if (!ubxDataStruct->flags3.bits.invalidLlh) {
    loc.latitude = ubxDataStruct->lat / 10000000.0;   // deg * 1e-7 -> deg
    loc.longitude = ubxDataStruct->lon / 10000000.0;  // deg * 1e-7 -> deg
    hMSL = ubxDataStruct->hMSL / 1000.0;  // mm -> m
  }

  computeSunPos();
}

void computeSunPos() {
  struct STPosition pos;
  SolTrack(sttime, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);
  altitude = pos.altitudeRefract;
  azimuth = pos.azimuthRefract;
}

void printGPSData() {
  char gpsOutputStr[96];
  sprintf(gpsOutputStr, "Lat, Lon: %.6f, %.6f degrees\r\nhMSL: %.3f m\n%i-%02i-%02i %02i:%02i:%02.f UTC\n",
    loc.latitude, loc.longitude, hMSL, 
    sttime.year, sttime.month, sttime.day,
    sttime.hour, sttime.minute, sttime.second);
  Serial.println(gpsOutputStr);
}

/**
 * Adjusts the camera pan angle.
 */
void adjustCameraPanAngle() {
  // Calculate the difference between the target azimuth and the current motor position
  Serial.println("");
  Serial.println("Math to find the difference");
  double azimuthDiff = azimuth - avgDeg - currentPanPos;
  Serial.print("Step Size: ");
  Serial.println(STEP_SIZE);
  Serial.print("Compass Average Output: ");
  Serial.println(avgDeg);
  Serial.print("Azimuth of the Sun: ");
  Serial.println(azimuth);
  Serial.print("Azimuth Difference: ");
  Serial.println(azimuthDiff);

  // Normalize the azimuth difference to ensure it wraps around within the range of 0 to 360 degrees
  azimuthDiff = fmod(azimuthDiff, 360.0);
  if (azimuthDiff < 0.0) {
    azimuthDiff += 360.0;
  }

  // Determine the most efficient direction of movement (clockwise or counterclockwise)
  double absAzimuthDiff = abs(azimuthDiff);
  if (absAzimuthDiff > PAN_TOLERANCE) {
    int direction = (azimuthDiff > 180.0) ? BACKWARD : FORWARD;

    // Convert the azimuth difference to the number of steps for the stepper motor
    int steps = static_cast<int>(absAzimuthDiff / STEP_SIZE);

    // Check if the difference is negative and adjust the number of steps
    if (azimuthDiff < 0.0 && direction == FORWARD) {
      steps = -steps;
    }

    // Step the motor in the appropriate direction
    Serial.println("Now moving stepper motor");
    Serial.print("Number of steps to take: ");
    Serial.println(steps);
    panMotor->step(steps, direction, SINGLE);

    // Update the current position
    currentPanPos = fmod(currentPanPos + steps * STEP_SIZE, 360.0);
    if (currentPanPos < 0.0) {
      currentPanPos += 360.0;
    }
  } else {
    // Camera position is within the tolerance region
    Serial.print("Current Position: ");
    Serial.println(currentPanPos);
  }
}

/**
 * Adjusts the camera tilt angle.
 */
void adjustCameraTiltAngle() {
  // Calculate the difference between the target altitude and the current motor position
  float altitudeDiff = altitude - currentTiltMotorPosition;
  if (abs(altitudeDiff) > TILT_TOLERANCE) {
    // Adjust the motor position by stepping in the appropriate direction
    if (altitudeDiff > 0) {
      tiltMotor->step(1, FORWARD, SINGLE);
      currentTiltMotorPosition++;
    } else if (altitudeDiff < 0) {
      tiltMotor->step(1, BACKWARD, SINGLE);
      currentTiltMotorPosition--;
    }
  }
}

/**
 * Converts a value to the 360-degree scale by adjusting it within the range [0, 360).
 * @param value The value to be converted.
 * @return The converted value within the 360-degree scale.
 */
float convertTo360(float value) {
  if (value < 0) {
    value += 360.0;
  } else if (value >= 360.0) {
    value -= 360.0;
  }
  return value;
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void) {
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void initializeGPS() {
  if (myGNSS.begin() == false) { //Connect to the u-blox module using Wire port
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) delay(10);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
  // myGNSS.saveConfiguration();        // Optional: Save the current settings to flash and BBR
    
  myGNSS.setNavigationFrequency(2); // Produce two solutions per second
  myGNSS.setAutoPVTcallbackPtr(&PVTUpdate); // Enable automatic NAV PVT messages with callback to PVTUpdate
}

void initializeIMU() {
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
  }

  //* If you want to manually insert magnetic declination, uncomment this code
  // magneticDeclination = Serial.parseInt();
  // Serial.println("Input the magnetic declination in your current location:");
  // while (Serial.available() == 0) {
  // }
  magneticDeclination = -12;
  Serial.print("The magnetic declination is set to ");
  Serial.print(magneticDeclination);
  Serial.println(" degrees.");
  // IMU code
  int8_t imu_temp=bno.getTemp(); // Temperature of IMU?
  millisOld=millis(); // Current Time

  // Offset initialization
  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  // Look for the sensor's unique ID at the beginning oF EEPROM.
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  // Crystal must be configured AFTER loading calibration data into BNO055.
  bno.setExtCrystalUse(true);
  sensors_event_t event;
  bno.getEvent(&event);
  // recalibrating magnetometer
  if (foundCalib){
      Serial.println("Move sensor slightly to calibrate magnetometers");
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }
  else
  {
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
        bno.getEvent(&event);

        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.print(event.orientation.z, 4);

        /* Optional: Display calibration status */
        displayCalStatus();

        /* New line for the next sample */
        Serial.println("");

        /* Wait the specified delay before requesting new data */
        delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");
  delay(1000);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
      Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
  Serial.print("Accelerometer: ");
  Serial.print(calibData.accel_offset_x); Serial.print(" ");
  Serial.print(calibData.accel_offset_y); Serial.print(" ");
  Serial.print(calibData.accel_offset_z); Serial.print(" ");

  Serial.print("\nGyro: ");
  Serial.print(calibData.gyro_offset_x); Serial.print(" ");
  Serial.print(calibData.gyro_offset_y); Serial.print(" ");
  Serial.print(calibData.gyro_offset_z); Serial.print(" ");

  Serial.print("\nMag: ");
  Serial.print(calibData.mag_offset_x); Serial.print(" ");
  Serial.print(calibData.mag_offset_y); Serial.print(" ");
  Serial.print(calibData.mag_offset_z); Serial.print(" ");

  Serial.print("\nAccel Radius: ");
  Serial.print(calibData.accel_radius);

  Serial.print("\nMag Radius: ");
  Serial.print(calibData.mag_radius);
}