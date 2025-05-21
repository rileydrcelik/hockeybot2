const int sensorPin = 30; // OUT pin from sensor to digital pin 30

void setup() {
  pinMode(sensorPin, INPUT);    // No pullup needed, Pololu has one onboard
  Serial.begin(9600);
}

void loop() {
  int state = digitalRead(sensorPin);

  if (state == LOW) { // LOW = Object detected within 5 cm
    Serial.println("Object detected within 5 cm!");
  } else {
    Serial.println("No object.");
  }
  delay(200);
}
