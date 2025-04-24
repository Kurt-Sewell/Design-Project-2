#include <Adafruit_GPS.h>
#include <wxwidgets.h>

#define GPSRX 16
#define GPSTX 17
HardwareSerial &GPSserial = Serial1;

Adafruit_GPS MiniGPS(GPSserial);

void setup()
{
}