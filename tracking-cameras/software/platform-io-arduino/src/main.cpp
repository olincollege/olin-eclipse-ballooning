#include <Arduino.h>
#include <SolTrack.h>
#include <RTClib.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <EEPROM.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
// #include <IMU_func.h>
#include <SPI.h>

// Stepper Motor Initialization

Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create motor shield object
Adafruit_StepperMotor *panMotor = AFMS.getStepper(200, 1);  // Connect stepper motor to port #1
Adafruit_StepperMotor *tiltMotor = AFMS.getStepper(200, 2);  // Connect stepper motor to port #2

#define panSwitchPin 13
#define tiltSwitchPin 5

bool panSwitchEnabled = false;
bool tiltSwitchEnabled = false;
int panSwitchState;
int tiltSwitchState;
// signed long panMotorPosition;
// signed long tiltMotorPosition;

// BNO Initialization
unsigned long lastIMURead = 0;
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData);
float convertTo360(float value);

void solTrack_computeSunPos(DateTime dt, double& azimuth, double& altitude);

// Rolling Average Code
const int numReadings = 10;
float x_readings[numReadings] = {0}; // Array to store the x-coordinates of the readings
float y_readings[numReadings] = {0}; // Array to store the y-coordinates of the readings
int readIndex = 0;
float avg_deg; // Output of the moving average

// Variables for IMU Code

float thetaM; // Measured Pitch
float phiM; // Measured Roll
float thetaFold=0; // Old Filtered Value of Acceleromter Pitch
float thetaFnew; // Updated Filtered Value of Acceleromter Pitch
float phiFold=0; // Old Filtered Value of Acceleromter Pitch
float phiFnew; // Updated Filtered Value of Acceleromter Pitch

float thetaG=0; // Gyro Pitch
float phiG=0; // Gyro Roll

float theta; // Acceleromter Pitch
float phi; // Acceleromter Roll

float thetaRad; // Pitch in Radians
float phiRad; // Pitch in Radians

float Xm; // x-component of magnetometer
float Ym; // y-component of magnetometer
float psi; // (Yaw/Heading angle) Angle of Magnetometer tilt, using the x and y components of magetometer
int magneticDeclination;
float adjustedValue; // Cardinal direction adjusted for magnetic declination
float convertedValue; // Converteds and maintians the adjusted value within the 360-deg scale
double azimuth, altitude; // double containing the azimuth and altitude of the sun.
float currentPanOffset;  // Current stepper motor position
float currentTiltMotorPosition;  // Current stepper motor position

float dt; // change in time
unsigned long millisOld;

//* RTC & SUN CODE

RTC_DS3231 rtc;


// Global variables and structs:
int useDegrees = 1;             // Input (geographic position) and output are in degrees
int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
int computeRefrEquatorial = 0;  // Compute refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
int computeDistance = 0;        // Compute the distance to the Sun in AU: 0-no, 1-yes

struct STTime time;               // Struct for date and time variables
struct STLocation loc;            // Struct for geographic location variables

void solTrack_computeSunPos(DateTime dt, double& azimuth, double& altitude);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino");
  //* RTC
  if (!rtc.begin()) {
      Serial.println("Couldn't find RTC");
      Serial.flush();
      while (1) delay(10);
  }
  Serial.println("Post Begin");
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    // plus 4 hours to adjust from eastern time to universal time
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))+TimeSpan(0, 4, 0, 0));
  }
  Serial.println("Post Lost Power");
  //* SUN
  loc.longitude   = -71.2639859;  // Olin College of Engineering
  loc.latitude    = 42.2929003;
  loc.pressure    = 101.0;      // Atmospheric pressure in kPa
  loc.temperature = 290.3;      // Atmospheric temperature in K

  delay(1000);
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

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
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

  /* Crystal must be configured AFTER loading calibration data into BNO055. */
  bno.setExtCrystalUse(true);
  // Serial.println("Ending Serial");
  // Serial.end();
  // delay(100);
  // Serial.begin(115200);
  // Serial.println("Restarting Serial");
  sensors_event_t event;
  bno.getEvent(&event);
  /* always recal the mag as It goes out of calibration very often */
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
    //TODO Commented out because of Arduino restarting issue
    //? Not sure if this is necessarily the problem, but it is a culprit
    // Serial.println("\n\nStoring calibration data to EEPROM...");
    // adafruit_bno055_offsets_t newCalib;
    // eeAddress = 0;
    // bno.getSensor(&sensor);
    // bnoID = sensor.sensor_id;

    // EEPROM.put(eeAddress, bnoID);

    // eeAddress += sizeof(long);
    // EEPROM.put(eeAddress, newCalib);
    // Serial.println("Data stored to EEPROM.");

    // Serial.println("\n--------------------------------\n");
    // delay(500);

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
  
  Serial.println("Starting Stepper Motor calibration");

  pinMode(panSwitchPin, INPUT_PULLUP);
  pinMode(tiltSwitchPin, INPUT_PULLUP);

  while (!Serial);
  AFMS.begin();  // Initialize motor shield

  while (!panSwitchEnabled || !tiltSwitchEnabled) {
    panSwitchState = digitalRead(panSwitchPin);
    tiltSwitchState = digitalRead(tiltSwitchPin);

    if (panSwitchState == HIGH) {  // Check if the pan switch is enabled
      if (!panSwitchEnabled) {
        panMotor->release();  // Release pan motor if switch is enabled
        panSwitchEnabled = true;
        currentPanOffset = 0.0;
        // Take note of the current cardinal direction of the IMU when the sensor is triggered.
        // Find the position of the sun, and use the cardinal direction given
      }
    } else {
      panSwitchEnabled = false;
      panMotor->step(1, FORWARD, SINGLE);  // Step pan motor forward
    }

    if (tiltSwitchState == HIGH) {  // Check if the tilt switch is enabled
      if (!tiltSwitchEnabled) {
        tiltMotor->release();  // Release tilt motor if switch is enabled
        tiltSwitchEnabled = true;
        // Perform tilt actions here
        currentTiltMotorPosition = 0.0;
      }
    } else {
      tiltSwitchEnabled = false;
      tiltMotor->step(1, FORWARD, SINGLE);  // Step tilt motor forward
    }
  }

  delay(500);
}

void loop() {
  // Serial.println("Starting Loop");
  unsigned long now = millis();

  // IMU Code
  uint8_t system, gyro, accel, mg = 0;
  
  if (now - lastIMURead > 100){
    //* SUN
    solTrack_computeSunPos(rtc.now(), azimuth, altitude);
    sensors_event_t event;
    bno.getEvent(&event);
    
    // Extracting values from the sensors on the IMU
    imu::Vector<3> acc =bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyr =bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag =bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    
    thetaM=-atan2(acc.x()/9.8,acc.z()/9.8)/2/3.141592654*360;
    phiM=-atan2(acc.y()/9.8,acc.z()/9.8)/2/3.141592654*360;
    phiFnew=.95*phiFold+.05*phiM;
    thetaFnew=.95*thetaFold+.05*thetaM;
    
    dt=(millis()-millisOld)/1000.;
    millisOld=millis();
    theta=(theta+gyr.y()*dt)*.95+thetaM*.05; // Degrees
    phi=(phi-gyr.x()*dt)*.95+ phiM*.05; // Degrees
    thetaG=thetaG+gyr.y()*dt;
    phiG=phiG-gyr.x()*dt;
    
    phiRad=phi/(360*2*3.141592654); // Radians
    thetaRad=theta/360*(2*3.141592654); // Radians

    // Compensated x-component of magnetometer
    Xm=mag.x()*cos(thetaRad)-mag.y()*sin(phiRad)*sin(thetaRad)+mag.z()*cos(phiRad)*sin(thetaRad);
    
    // Compenstated y-component of magnetometer
    Ym=mag.y()*cos(phiRad)+mag.z()*sin(phiRad);
    
    psi=((atan2(Ym,Xm)* 180) / 3.141592654); // Angle of Magnetometer Tilt (deg)
    
    // Apply the magnetic declination adjustment
    adjustedValue = psi + magneticDeclination;

    // Ensure the adjusted value stays within the 360-degree range and convert the adjusted value to the 360-degree scale
    convertedValue = convertTo360(adjustedValue);
    psi = convertTo360(psi);

    // Moving Average Filter
    float angle_deg = convertedValue;
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
    avg_deg = avg_rad * 180.0 / PI;
    if (avg_deg < 0) avg_deg += 360; // Ensure the angle is in the range [0, 360)

    // Output the average
    Serial.print('Cardinal Direction: ');
    Serial.println(avg_deg);

    // Update the index
    readIndex = (readIndex + 1) % numReadings;

    delay(1);


    // Serial.println("");
    // Serial.print("Cardinal Direction (True, Magnetic): ");
    //Serial.print(acc.x()/9.8);
    //Serial.print(",");
    //Serial.print(acc.y()/9.8);
    //Serial.print(",");
    //Serial.print(acc.z()/9.8);
    //Serial.print(",");
    //Serial.print(accel);
    //Serial.print(",");
    //Serial.print(gyro);
    //Serial.print(",");
    //Serial.print(mg);
    //Serial.print(",");
    //Serial.print(system);
    //Serial.print(",");
    //Serial.print(thetaM);
    //Serial.print(",");
    //Serial.print(phiM);
    //Serial.print(",");
    //Serial.print(thetaFnew);
    //Serial.print(","); 
    //Serial.print(phiFnew);
    //Serial.print(",");
    //Serial.print(thetaG);
    //Serial.print(",");
    //Serial.print(phiG);
    //Serial.print(",");
    //Serial.print(theta);
    //Serial.print(",");
    //Serial.print(phi);
  //  Serial.print(Xm);
  //  Serial.print(",");
    // Serial.print(convertedValue);
    // Serial.print(",");
    // Serial.println(psi);
    
    
    phiFold=phiFnew;
    thetaFold=thetaFnew;
    lastIMURead = now;
    
    /*
    *Given the current IMU cardinal direction reading, and the Azimuth/Altitude of the Sun, we have to determine the 0 point of where the
    *stepper motors are initially pointing. It is know that the azimuth of the panning motor will be equal to that of the IMU reading.
    *The altitude of the tilt motor depends on the default position set by the limit switch, which will have to be measured by hand.
    */
    //TODO: Move these initial positions to a place where they won't get reset at every step, or maybe create a panInitialPos and a panPos.
    int panPos = 0;
    int tiltPos = 90;
    //* The next step is to determine use the # degrees each step can take (1.8 degrees), and divide by 3, equivelent to the gear ratio
    int gearRatio = 1/3;

    int stepSize = 1.8;

    stepSize = stepSize * gearRatio;
    // TODO: All of the info between the TODO's is static information.

    // convertedValue is the current angle relative to True North
    

    /*
    *If the difference between the current angle (base angle is 0 deg), and the IMU reading is above 7 degrees, use that difference, and 
    *multiply it by the reciprocal of the stepper motor steps. This gives you the number of steps the stepper motor has to make. Round that
    * number to the nearest Then, have the stepper motor step in the direction.
    *If the difference is greater than 180, spin counter clockwise. Otherwise, spin clockwise the number of steps discovered in the last step.
    */

    // delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}

// Function to convert cardinal direction to 360-degree scale
float convertTo360(float value) {
  if (value < 0) {
    value += 360.0;
  } else if (value >= 360.0) {
    value -= 360.0;
  }
  return value;
}


void adjustPanMotorPosition(float azimuth) {
  // Calculate the difference between the target azimuth and the current motor position
  // The pan motor position will always be equal to the amount that the stepper motor moves
  // from the initial position, plus the 
  float azimuthDiff = azimuth - currentPanOffset;

  // Adjust the motor position by stepping in the appropriate direction
  if (azimuthDiff > 0) {
    panMotor->step(1, FORWARD, SINGLE);  // Step motor forward
    currentPanOffset++;
  } else if (azimuthDiff < 0) {
    panMotor->step(1, BACKWARD, SINGLE);  // Step motor backward
    currentPanOffset--;
  }
}

void adjustTiltMotorPosition(float altitude) {
  // Calculate the difference between the target azimuth and the current motor position
  float azimuthDiff = altitude - currentTiltMotorPosition;

  // Adjust the motor position by stepping in the appropriate direction
  if (azimuthDiff > 0) {
    panMotor->step(1, FORWARD, SINGLE);  // Step motor forward
    currentTiltMotorPosition++;
  } else if (azimuthDiff < 0) {
    panMotor->step(1, BACKWARD, SINGLE);  // Step motor backward
    currentTiltMotorPosition--;
  }
}

// Stepper motor library for potentially accelerating and decelerating
// Look for a stepper motor library that allows you to have a changing destination
// Potentially have a control system.

void solTrack_computeSunPos(DateTime dt, double& azimuth, double& altitude) {
  // Set (UT!) date and time:
  time.year   = dt.year();
  time.month  = dt.month();
  time.day    = dt.day();
  time.hour   = dt.hour();
  time.minute = dt.minute();
  time.second = dt.second();
  
  // Compute Sun position:
  struct STPosition pos;
  SolTrack(time, loc, &pos, useDegrees, useNorthEqualsZero, computeRefrEquatorial, computeDistance);

  //* Below is code used to print out the information that SolTrack computes.
  // // Write formatted output to serial connection:
  // char outputLine[256], secStr[7], JDstr[13], agstStr[11], azStr[9],altStr[9];
  // dtostrf(time.second,         6, 3, secStr);   //  6 digits, 3 decimals
  // dtostrf(pos.julianDay,      12, 6, JDstr);    // 14 digits, 6 decimals
  // dtostrf(pos.agst*R2H,       10, 7, agstStr);  // 10 digits, 7 decimals
  // dtostrf(pos.azimuthRefract,  8, 3, azStr);    //  8 digits, 3 decimals
  // dtostrf(pos.altitudeRefract, 8, 3, altStr);   //  8 digits, 3 decimals
  
  // sprintf(outputLine, "%d-%2.2d-%2.2d %2.2d:%2.2d:%s %s %s Azimuth: %s Altitude: %s %s",
  //         time.year, time.month, time.day, time.hour,time.minute,secStr,
  //         JDstr, agstStr, azStr,altStr, "     ");
  
  // Serial.println(outputLine);

  // Create a SunPosition object and populate it with azimuth and altitude values
  // Extract azimuth and altitude values
  azimuth = pos.azimuthRefract;
  altitude = pos.altitudeRefract;
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
void displaySensorStatus(void)
{
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