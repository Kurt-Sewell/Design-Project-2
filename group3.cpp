#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// HC-05 Bluetooth
#define HC05_RX 34
#define HC05_TX 35
SoftwareSerial hc05(HC05_RX, HC05_TX);

// GPS via hardware Serial1 (Teensy 4.1)
Adafruit_GPS GPS(&Serial1);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Gravity vector
float gravityX = 0, gravityY = 0, gravityZ = 0;
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("System initialization started...");

  // Initialize Bluetooth
  hc05.begin(9600);

  // Initialize GPS
  Serial.println("Starting GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  // Initialize SD Card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card ready.");
    // Create CSV header
    File dataFile = SD.open("data.csv", FILE_WRITE);
    if (dataFile) {
      dataFile.println("Time(s),AccelX,AccelY,AccelZ,Latitude,Longitude,Altitude,Satellites");
      dataFile.close();
    }
  }

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 IMU not detected!");
    while (1);
  }

  // Calibrate gravity vector
  calculateGravity();
  startTime = millis();
}

void calculateGravity() {
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t event;
  for (int i = 0; i < 100; i++) {
    bno.getEvent(&event);
    sumX += event.acceleration.x;
    sumY += event.acceleration.y;
    sumZ += event.acceleration.z;
    delay(10);
  }
  gravityX = sumX / 100;
  gravityY = sumY / 100;
  gravityZ = sumZ / 100;
}

void writeSensorData(File &dataFile, unsigned long elapsedTime, 
                    float linearAccelX, float linearAccelY, float linearAccelZ,
                    float lat, float lon, float alt, int sats) {
  // Time in seconds with 2 decimal places
  dataFile.print(elapsedTime / 1000.0, 2);
  dataFile.print(",");
  
  // Acceleration data
  dataFile.print(linearAccelX, 2);
  dataFile.print(",");
  dataFile.print(linearAccelY, 2);
  dataFile.print(",");
  dataFile.print(linearAccelZ, 2);
  dataFile.print(",");
  
  // GPS data
  dataFile.print(lat, 6);
  dataFile.print(",");
  dataFile.print(lon, 6);
  dataFile.print(",");
  dataFile.print(alt, 2);
  dataFile.print(",");
  dataFile.println(sats);
}

void loop() {
  unsigned long elapsedTime = millis() - startTime;
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
  int satellites = 0;

  // Read IMU data
  sensors_event_t event;
  bno.getEvent(&event);
  float linearAccelX = event.acceleration.x - gravityX;
  float linearAccelY = event.acceleration.y - gravityY;
  float linearAccelZ = event.acceleration.z - gravityZ;

  // Process GPS data
  GPS.read();
  if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA());
  }

  if (GPS.fix) {
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    altitude = GPS.altitude;
    satellites = GPS.satellites;

    // Bluetooth transmission
    hc05.print("POS|");
    hc05.print(latitude, 6);
    hc05.print("|");
    hc05.print(longitude, 6);
    hc05.print("|");
    hc05.println(altitude, 2);
  }

  // Save all data to SD card
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    writeSensorData(dataFile, elapsedTime, 
                   linearAccelX, linearAccelY, linearAccelZ,
                   latitude, longitude, altitude, satellites);
    dataFile.close();
    Serial.println("Data logged successfully");
  } else {
    Serial.println("Error opening data.csv");
  }

  // Serial monitor output
  Serial.print("Accel: ");
  Serial.print(linearAccelX, 2);
  Serial.print(", ");
  Serial.print(linearAccelY, 2);
  Serial.print(", ");
  Serial.print(linearAccelZ, 2);
  Serial.print(" | GPS: ");
  Serial.print(latitude, 6);
  Serial.print(", ");
  Serial.println(longitude, 6);

  delay(1000);  // Adjust sampling rate as needed
}