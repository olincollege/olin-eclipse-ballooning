#include <Arduino.h>
#include <SolTrack.h>
#include <RTClib.h>

RTC_DS3231 rtc;

// Global variables and structs:
int useDegrees = 1;             // Input (geographic position) and output are in degrees
int useNorthEqualsZero = 1;     // Azimuth: 0 = South, pi/2 (90deg) = West  ->  0 = North, pi/2 (90deg) = East
int computeRefrEquatorial = 0;  // Compure refraction-corrected equatorial coordinates (Hour angle, declination): 0-no, 1-yes
int computeDistance = 0;        // Compute the distance to the Sun in AU: 0-no, 1-yes

struct STTime time;               // Struct for date and time variables
struct STLocation loc;            // Struct for geographic location variables

void solTrack_computeAndPrintSunPos(DateTime dt);

void setup() {
    Serial.begin(9600);
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1) delay(10);
    }
    if (rtc.lostPower()) {
      Serial.println("RTC lost power, let's set the time!");
      // When time needs to be set on a new device, or after a power loss, the
      // following line sets the RTC to the date & time this sketch was compiled
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      // This line sets the RTC with an explicit date & time, for example to set
      // January 21, 2014 at 3am you would call:
      // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
}

void loop() {
  solTrack_computeAndPrintSunPos(rtc.now());

  // Serial.print(now.year(), DEC);
  // Serial.print('/');
  // Serial.print(now.month(), DEC);
  // Serial.print('/');
  // Serial.print(now.day(), DEC);
  // Serial.print(" ");
  // Serial.print(now.hour(), DEC);
  // Serial.print(':');
  // Serial.print(now.minute(), DEC);
  // Serial.print(':');
  // Serial.print(now.second(), DEC);
  // Serial.println();
}

void solTrack_computeAndPrintSunPos(DateTime dt) {
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
	
  // Write formatted output to serial connection:
	char outputLine[256], secStr[7], JDstr[13], agstStr[11], azStr[9],altStr[8];
	dtostrf(time.second,         6, 3, secStr);   //  6 digits, 3 decimals
	dtostrf(pos.julianDay,      12, 6, JDstr);    // 14 digits, 6 decimals
	dtostrf(pos.agst*R2H,       10, 7, agstStr);  // 10 digits, 7 decimals
	dtostrf(pos.azimuthRefract,  8, 3, azStr);    //  8 digits, 3 decimals
	dtostrf(pos.altitudeRefract, 7, 3, altStr);   //  8 digits, 3 decimals
	
	sprintf(outputLine, "%d-%2.2d-%2.2d %2.2d:%2.2d:%s %s %s %s %s %s",
					time.year, time.month, time.day, time.hour,time.minute,secStr,
					JDstr, agstStr, azStr,altStr, "     ");
	
	Serial.println(outputLine);
}
