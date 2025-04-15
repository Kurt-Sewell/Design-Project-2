#define HC05_RX 1 
#define HC05_TX 0

int state = 0;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  Serial.println("HC-05 Bluetooth Test on Teensy");
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);
  }

  if (Serial.available()) {
    char c = Serial.read();
    Serial1.print(c);
  }
}