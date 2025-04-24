// ECEN 4013, Project 2
// Final Program
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;

#define HC06_RX 34 // Teensy Pin RX2
#define HC06_TX 35 // Teensy Pin TX2
#define RX1_PIN 0 // Teensy RX1 (connect to GPS TX)
#define TX1_PIN 1 // Teensy TX1 (connect to GPS RX)
// LSM9DS1 object
// Initialize a serial connection for HC-06
SoftwareSerial hc06(HC06_RX, HC06_TX);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
HardwareSerial &gpsSerial = Serial1; // Use Serial1 for Teensy hardware serial

24

// SD card chip select pin
#define SD_CS BUILTIN_SDCARD
// Gravity calibration variables
float gravityX = 0, gravityY = 0, gravityZ = 0;
// Timer variable
unsigned long startTime;
void setup() {
Serial.begin(9600); // USB Serial
Serial.println("Initializing GPS...");
gpsSerial.begin(9600);
hc06.begin(9600);
Serial.println("Setup started...");
// Initialize SD card
if (!SD.begin(SD_CS)) {
Serial.println("SD card initialization failed!");
}
Serial.println("SD card initialized");
// Initialize LSM9DS1 sensor
if (!lsm.begin()) {
Serial.println("Failed to initialize LSM9DS1!");
while (1);
}
lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
// Calculate gravity vector
calculateGravity();
// Start timer
startTime = millis();

25

}
void calculateGravity() {
float sumX = 0, sumY = 0, sumZ = 0;
int samples = 100;
for (int i = 0; i < samples; i++) {
sensors_event_t accel, mag, gyro, temp;
lsm.getEvent(&accel, &mag, &gyro, &temp);
sumX += accel.acceleration.x;
sumY += accel.acceleration.y;
sumZ += accel.acceleration.z;
delay(10);
}
gravityX = sumX / samples;
gravityY = sumY / samples;
gravityZ = sumZ / samples;
}

void writeSensorData(File &dataFile, unsigned long elapsedTime, float linearAccelX, float linearAccelY, float linearAccelZ) {
dataFile.print("Time: ");
dataFile.print(elapsedTime / 1000);
dataFile.print(" s, Accel X: ");
dataFile.print(linearAccelX, 2);
dataFile.print(", Y: ");
dataFile.print(linearAccelY, 2);
dataFile.print(", Z: ");
dataFile.print(linearAccelZ, 2);
dataFile.println();
}
void loop() {
unsigned long elapsedTime = millis() - startTime;
Serial.println("Retrieving IMU Data");

26

// Read accelerometer data
sensors_event_t accel, mag, gyro, temp;
lsm.getEvent(&accel, &mag, &gyro, &temp);
// Calculate linear acceleration
float linearAccelX = accel.acceleration.x - gravityX;
float linearAccelY = accel.acceleration.y - gravityY;
float linearAccelZ = accel.acceleration.z - gravityZ;
delay(1000);
// Open SD card file
File dataFile = SD.open("Data.txt", FILE_WRITE);
if (dataFile) {
// Write accelerometer data
Serial.println("Printing IMU data to SD Card");
writeSensorData(dataFile, elapsedTime, linearAccelX, linearAccelY, linearAccelZ);
// Close the file
dataFile.close();
} else {
Serial.println("Failed to open Data.txt for writing!");
}
Serial.println("Beginning 10-Second GPS Delay");
delay(10000); // 10-Second delay to accomodate for GPS
if (gpsSerial.available() > 0) {
char c = gpsSerial.read();
gps.encode(c); // Send character to TinyGPS++ parser
if (gps.location.isUpdated()) {
Serial.print("Latitude: ");
Serial.println(gps.location.lat(), 6);
Serial.print("Longitude: ");
Serial.println(gps.location.lng(), 6);
}
// If GPS has valid altitude data, print it
if (gps.altitude.isUpdated()) {
Serial.print("Elevation: ");
Serial.print(gps.altitude.meters());

27

Serial.println(" meters");
}
// If GPS has valid satellite count, print it
if (gps.satellites.isUpdated()) {
Serial.print("Satellites: ");
Serial.println(gps.satellites.value());
}
}
else {
Serial.println("GPS Data Currently Unavailable");
}

Serial.println("Procedure Complete, Restarting...");

Serial.print("Latitude: ");
Serial.println(gps.location.lat(), 6);
Serial.print("Longitude: ");
Serial.println(gps.location.lng(), 6);
Serial.print("Elevation: ");
Serial.print(gps.altitude.meters());
Serial.println(" meters");
Serial.print("Satellites: ");
Serial.println(gps.satellites.value());

hc06.print("Latitude: ");
hc06.println(gps.location.lat(), 6);
hc06.print("Longitude: ");
hc06.println(gps.location.lng(), 6);
hc06.print("Elevation: ");
hc06.print(gps.altitude.meters());
hc06.println(" meters");
hc06.print("Satellites: ");
hc06.println(gps.satellites.value());

Serial.println("Retrieving IMU Data");
Serial.print("Acceleration X: ");
Serial.print(linearAccelX, 2);
Serial.print(", Y: ");
Serial.print(linearAccelY, 2);
Serial.print(", Z: ");
Serial.println(linearAccelZ, 2);
hc06.println("Retrieving IMU Data");
hc06.print("Acceleration X: ");
hc06.print(linearAccelX, 2);
hc06.print(", Y: ");
hc06.print(linearAccelY, 2);
hc06.print(", Z: ");
hc06.println(linearAccelZ, 2);

Serial.print("Angular Velocity X: ");
Serial.print(gyro.gyro.x, 2);
Serial.print(", Y: ");
Serial.print(gyro.gyro.y, 2);
Serial.print(", Z: ");
Serial.println(gyro.gyro.z, 2);
hc06.print("Angular Velocity X: ");
hc06.print(gyro.gyro.x, 2);
hc06.print(", Y: ");
hc06.print(gyro.gyro.y, 2);
hc06.print(", Z: ");
hc06.println(gyro.gyro.z, 2);

Serial.print("Magnetic Field X: ");
Serial.print(mag.magnetic.x, 2);
Serial.print(", Y: ");
Serial.print(mag.magnetic.y, 2);
Serial.print(", Z: ");
Serial.println(mag.magnetic.z, 2);
hc06.print("Magnetic Field X: ");
hc06.print(mag.magnetic.x, 2);
hc06.print(", Y: ");
hc06.print(mag.magnetic.y, 2);
hc06.print(", Z: ");
hc06.println(mag.magnetic.z, 2);
delay(1000); // 1-second loop delay
}
