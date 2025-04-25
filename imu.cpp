#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//As of 04/25/2025

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

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
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 */
    Serial.print("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
   
  delay(1000);

  /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  bool Adafruit_BNO055::getEvent(sensors_event_t *event,
    adafruit_vector_type_t vec_type) {
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = _sensorID;
    event->timestamp = millis();

    // read the data according to vec_type
    imu::Vector<3> vec;

    if (vec_type == Adafruit_BNO055::VECTOR_ACCELEROMETER) {
    event->type = SENSOR_TYPE_ACCELEROMETER;
    vec = getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

    event->acceleration.x = vec.x();
    event->acceleration.y = vec.y();
    event->acceleration.z = vec.z();

    Serial.print(F("Acceleration: "));
    Serial.print(event.acceleration.x);
    Serial.print(F(" "));
    Serial.print(event.acceleration.y);
    Serial.print(F(" "));
    Serial.print(event.acceleration.z);
    Serial.println(F(""));
    } 

    else if (vec_type == Adafruit_BNO055::VECTOR_GYROSCOPE) {
    event->type = SENSOR_TYPE_GYROSCOPE;
    vec = getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    event->gyro.x = vec.x() * SENSORS_DPS_TO_RADS;
    event->gyro.y = vec.y() * SENSORS_DPS_TO_RADS;
    event->gyro.z = vec.z() * SENSORS_DPS_TO_RADS;

    Serial.print(F("Gyroscope: "));
    Serial.print(event.gyro.x);
    Serial.print(F(" "));
    Serial.print(event.gyro.y);
    Serial.print(F(" "));
    Serial.print(event.gyro.z);
    Serial.println(F(""));

    } 

    else if (vec_type == Adafruit_BNO055::VECTOR_MAGNETOMETER) {
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    vec = getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    event->magnetic.x = vec.x();
    event->magnetic.y = vec.y();
    event->magnetic.z = vec.z();

    Serial.print(F("Magnetic: "));
    Serial.print(event.magnetic.x);
    Serial.print(F(" "));
    Serial.print(event.magnetic.y);
    Serial.print(F(" "));
    Serial.print(event.magnetic.z);
    Serial.println(F(""));
    }

    return true;
    }

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
