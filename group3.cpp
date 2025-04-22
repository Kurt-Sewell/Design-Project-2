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

// SD card chip select
#define SD_CS BUILTIN_SDCARD

// Gravity vector
float gravityX = 0, gravityY = 0, gravityZ = 0;
unsigned long startTime;

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Setup started...");

  // Bluetooth
  hc05.begin(9600);

  // GPS
  Serial.println("Initializing GPS...");
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // basic location data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);

  // SD Card
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card initialization failed!");
  } else {
    Serial.println("SD card initialized.");
  }

  // IMU
  if (!bno.begin()) {
    Serial.println("Failed to initialize BNO055!");
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

void writeSensorData(File &dataFile, unsigned long elapsedTime, float linearAccelX, float linearAccelY, float linearAccelZ) {
  dataFile.print("Time: ");
  dataFile.print(elapsedTime / 1000);
  dataFile.print(" s, Accel X: ");
  dataFile.print(linearAccelX, 2);
  dataFile.print(", Y: ");
  dataFile.print(linearAccelY, 2);
  dataFile.print(", Z: ");
  dataFile.println(linearAccelZ, 2);
}

void loop() {
  unsigned long elapsedTime = millis() - startTime;

  // Read IMU
  sensors_event_t event;
  bno.getEvent(&event);
  float linearAccelX = event.acceleration.x - gravityX;
  float linearAccelY = event.acceleration.y - gravityY;
  float linearAccelZ = event.acceleration.z - gravityZ;

  // Save to SD
  File dataFile = SD.open("Data.txt", FILE_WRITE);
  if (dataFile) {
    writeSensorData(dataFile, elapsedTime, linearAccelX, linearAccelY, linearAccelZ);
    dataFile.close();
  } else {
    Serial.println("Failed to open Data.txt for writing!");
  }

  // Read GPS
  GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
  }

  if (GPS.fix) {
    Serial.print("Latitude: "); Serial.println(GPS.latitudeDegrees, 6);
    Serial.print("Longitude: "); Serial.println(GPS.longitudeDegrees, 6);
    Serial.print("Altitude: "); Serial.print(GPS.altitude); Serial.println(" m");
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    // Send over Bluetooth
    hc05.print("Latitude: "); hc05.println(GPS.latitudeDegrees, 6);
    hc05.print("Longitude: "); hc05.println(GPS.longitudeDegrees, 6);
    hc05.print("Altitude: "); hc05.print(GPS.altitude); hc05.println(" m");
    hc05.print("Satellites: "); hc05.println((int)GPS.satellites);
  } else {
    Serial.println("Waiting for GPS fix...");
  }

  // IMU to Serial and Bluetooth
  Serial.println("IMU Data:");
  Serial.print("Accel X: "); Serial.print(linearAccelX, 2);
  Serial.print(", Y: "); Serial.print(linearAccelY, 2);
  Serial.print(", Z: "); Serial.println(linearAccelZ, 2);

  hc05.println("IMU Data:");
  hc05.print("Accel X: "); hc05.print(linearAccelX, 2);
  hc05.print(", Y: "); hc05.print(linearAccelY, 2);
  hc05.print(", Z: "); hc05.println(linearAccelZ, 2);

  delay(1000);
}
