#define HC05_BAUD 9600

void setup() {
  Serial.begin(9600);    
  Serial1.begin(HC05_BAUD); 
  Serial.println("HC-05 Bluetooth Test");
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