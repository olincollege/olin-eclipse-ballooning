/*
Reads data from the PCB Geiger Counter over the serial port and saves data with timestamps to an SD card. 

Geiger counter connections: 
Serial pin 1 or 2 to GND
Serial pin 4 to pin 9
Serial pin 5 to pin 8

SD breakout:
5V to 5V
GND to GND
CLK to 13
D0 to 12
D1 to 11
CS to 10

RTC (right to left):
Vin to 3V3
GND to GND
SCL to SCL
SDA to SDA
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(8,9); // RX, TX
char receivedChars[100];      // an array to store the received data

#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;
String fileName = "datalog.csv";

#include <DS3231.h>
#include <Wire.h>
DS3231 myRTC;
bool century = false;
bool h12Flag;
bool pmFlag;

// used to find and save the desired data from the geiger counter output
int CPM_start = -1;
String CPM;
String dataString;

// used to time saving data every certain number of milliseconds
int millisBetween = 60000;
long lastTime;
long nowTime;

void setup() {
  // start the I2C interface
  Wire.begin();
  
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.print("Initializing SD card... ");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);  // UNCOMMENT WHEN SD CARD IS IN
  }
  Serial.println("card initialized.");

  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
}

void loop() { 
  // if complete data line is read
//  Serial.println("no data");
  if(mySerial.find("\n")){
    nowTime = millis();
    CPM_start = -1;
    CPM = "";
    mySerial.readBytesUntil('\n',receivedChars,100);
    Serial.println(receivedChars);
    
    // if it has been about a minute since last recording
    if(nowTime > lastTime + millisBetween){
      lastTime = millis();
      for(int i = 0; i < sizeof(receivedChars); i++){
        if(receivedChars[i] == 'M') {CPM_start = i+3;}
        if(CPM_start != -1 && i >= CPM_start){
          if(receivedChars[i] == ','){break;}
          else{CPM += receivedChars[i];}
        }
      }
      
      // organize desired data into a comma separated string
      dataString = "";
      dataString += myRTC.getHour(h12Flag, pmFlag);
      dataString += ":";
      dataString += myRTC.getMinute();
      dataString += ":";
      dataString += myRTC.getSecond();
      dataString += ",";
      dataString += CPM;
      Serial.println(dataString);

      // open the file to edit     UNCOMMENT WHEN SD CARD IS IN
      File dataFile = SD.open(fileName, FILE_WRITE);
      // write to the file if it is avaliable
      if (dataFile) {
        dataFile.println(dataString);
        dataFile.close();
      }
      // print an error if the file doesnt open
      else {
        Serial.print("error opening ");
        Serial.println(fileName);
      } 
    }
  }
}
