#include <HardwareSerial.h>

#define HC05_RX 1
#define HC05_TX 0

HardwareSerial bluetooth(HC05_RX, HC05_TX);

void setup() {
  Serial.begin(9600);

  bluetooth.begin(9600);

  Serial.println("HC-05 Bluetooth Test on Teensy");
}

void loop() {
  if (bluetooth.available()) {
    char c = bluetooth.read();
    Serial.print(c);
  }

  if (Serial.available()) {
    char c = Serial.read();
    bluetooth.print(c);
  }
}