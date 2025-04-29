#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <utility/imumaths.h>

// HC-05 Bluetooth
#define BT Serial6

// GPS via hardware Serial1 (Teensy 4.1)
Adafruit_GPS GPS(&Serial3);

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Gravity vector
float gravityX = 0, gravityY = 0, gravityZ = 0;
unsigned long startTime, t1, t2, tDel;
// void calculateGravity();

void setup()
{
  Serial.begin(9600);
  delay(1000);
  Serial.println("System initialization started...");

  // Initialize Bluetooth
  BT.begin(9600);

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
    // Write header only if file doesn't exist
    if (!SD.exists("data.csv")) {
      File dataFile = SD.open("data.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println("Time(s),Satellites,Latitude,Longitude,Elevation MSL (m),X Accel (m/s^2),Y Accel (m/s^2),Z Accel (m/s^2),X Mag (uT),Y Mag (uT),Z Mag (uT),X Gyro (rps),Y Gyro (rps),Z Gyro (rps)");
        dataFile.close();
      }
    }
  }

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not found!");
    while (1);
  }
  bno.setMode((adafruit_bno055_opmode_t)0x0C);

  // Calibrate gravity vector
  calculateGravity();
  startTime = millis();
}

void calculateGravity()
{
  float sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t event;
  for (int i = 0; i < 100; i++)
  {
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

void writeSensorData(File &dataFile, unsigned long elapsedTime, int sats,
                     float lat, float lon, float alt, float linearAccelX,
                     float linearAccelY, float linearAccelZ, float magX, 
                     float magY, float magZ, float gyrox, float gyroy, float gyroz)
{
  // Time in seconds with 2 decimal places
  dataFile.print(elapsedTime / 1000.0, 2);
  dataFile.print(",");

  dataFile.print(sats);
  dataFile.print(",");
  dataFile.print(lat, 6);
  dataFile.print(",");
  dataFile.print(lon, 6);
  dataFile.print(",");
  dataFile.print(alt, 2);
  dataFile.print(",");

  // Acceleration data
  dataFile.print(linearAccelX, 2);
  dataFile.print(",");
  dataFile.print(linearAccelY, 2);
  dataFile.print(",");
  dataFile.print(linearAccelZ, 2);
  dataFile.print(",");
  dataFile.print(magX, 2);
  dataFile.print(",");
  dataFile.print(magY, 2);
  dataFile.print(",");
  dataFile.print(magZ, 2);
  dataFile.print(",");
  dataFile.print(gyrox, 2);
  dataFile.print(",");
  dataFile.print(gyroy, 2);
  dataFile.print(",");
  dataFile.print(gyroz, 2);
  dataFile.println();

  // GPS data
}

static float velX = 0, velY = 0, velZ = 0;
static unsigned long lastTime = 0;

void loop()
{
  unsigned long elapsedTime = millis() - startTime;
  float latitude = 0.0, longitude = 0.0, altitude = 0.0;
  int satellites = 0;
  sensors_event_t accelEvent, magEvent, gyroEvent;

  // Read IMU data
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  delay(500);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // convert ms to seconds
  lastTime = currentTime;
  float linearAccelX = accelEvent.acceleration.x - gravityX;
  float linearAccelY = accelEvent.acceleration.y - gravityY;
  float linearAccelZ = accelEvent.acceleration.z - gravityZ;
  
  velX += linearAccelX * dt;
  velY += linearAccelY * dt;
  velZ += linearAccelZ * dt;
  
  float magX = magEvent.magnetic.x;
  float magY = magEvent.magnetic.y;
  float magZ = magEvent.magnetic.z;

  float gyrox = gyroEvent.gyro.x; // In radians per second (rps)
  float gyroy = gyroEvent.gyro.y;
  float gyroz = gyroEvent.gyro.z;

  // Process GPS data
  GPS.read();
  if (GPS.newNMEAreceived())
  {
    GPS.parse(GPS.lastNMEA());
  }

  if (GPS.fix)
  {
    latitude = GPS.latitudeDegrees;
    longitude = GPS.longitudeDegrees;
    altitude = GPS.altitude;
    satellites = GPS.satellites;

    // Bluetooth transmission
    BT.print("POS|");
    BT.print(latitude, 6);
    BT.print("|");
    BT.print(longitude, 6);
    BT.print("|");
    BT.println(altitude, 2);
  }

  // Save all data to SD card
  File dataFile = SD.open("data.csv", FILE_WRITE);
  if (dataFile) {
    writeSensorData(dataFile, elapsedTime, satellites, latitude, longitude,
                    altitude, linearAccelX, linearAccelY, linearAccelZ,
                    magX, magY, magZ, gyrox, gyroy, gyroz);
    dataFile.close();
  } else {
    Serial.println("Error opening file");
  }

  Serial.println("GPS Data:");
  Serial.print("Longitude: "); Serial.println(longitude, 2);
  Serial.print("Latitude: "); Serial.println(latitude, 2);
  Serial.print("Sattelites: "); Serial.println(satellites);


  // Serial monitor output
  Serial.println("IMU Data:");
  Serial.print("Accel X: ");
  Serial.print(linearAccelX, 2);
  Serial.print(", Y: ");
  Serial.print(linearAccelY, 2);
  Serial.print(", Z: ");
  Serial.println(linearAccelZ, 2);
  Serial.print("Magnetic X: ");
  Serial.print(magX, 2);
  Serial.print(", Y: ");
  Serial.print(magY, 2);
  Serial.print(", Z: ");
  Serial.println(magZ, 2);
  Serial.print("Velocity X: ");
  Serial.print(velX, 2);
  Serial.print(", Y: ");
  Serial.print(velY, 2);
  Serial.print(", Z: ");
  Serial.println(velZ, 2);
  Serial.print("Gyroscope X: ");
  Serial.print(gyrox, 2);
  Serial.print(", Y: ");
  Serial.print(gyroy, 2);
  Serial.print(", Z: ");
  Serial.println(gyroz, 2);

  BT.println("IMU Data:");
  BT.print("Velocity X: ");
  BT.print(velX, 2);
  BT.print(", Y: ");
  BT.print(velY, 2);
  BT.print(", Z: ");
  BT.println(velZ, 2);
  BT.print("Accel X: ");
  BT.print(linearAccelX, 2);
  BT.print(", Y: ");
  BT.print(linearAccelY, 2);
  BT.print(", Z: ");
  BT.println(linearAccelZ, 2);
  BT.print("Magnetic X: ");
  BT.print(magX, 2);
  BT.print(", Y: ");
  BT.print(magY, 2);
  BT.print(", Z: ");
  BT.println(magZ, 2);
  BT.print("Gyroscope X: ");
  BT.print(gyrox, 2);
  BT.print(", Y: ");
  BT.print(gyroy, 2);
  BT.print(", Z: ");
  BT.println(gyroz, 2);

  delay(1000); // Adjust sampling rate as needed
}