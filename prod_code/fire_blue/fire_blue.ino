//THIS IS RECIEVER (RX) ARDUINO

// Motor driver pins
#define ENA 9   // Right motor speed
#define IN1 3   // Right motor dir 1
#define IN2 4   // Right motor dir 2
#define ENB 10  // Left motor speed
#define IN3 6   // Left motor dir 1
#define IN4 7   // Left motor dir 2

//max speed
const int baseSpeedL = 255;
const int baseSpeedR = 255;

void setup() {
  Serial.begin(9600);
  
  // Set up motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set direction forward for both motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void moveMotors(int speedL, int speedR){
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

void fire(int range){
  if (range == 2){
    moveMotors(255, 0);
    delay(3000);
  }
  else if (range == 1){
    moveMotors(255, 0);
    delay(2000);
  }
  else if (range == 0){
    moveMotors(255, 0);
    delay(1000);
  }
  moveMotors(255, 255);
  delay(500);
  moveMotors(0, 0);
}

void loop() {
  if (Serial.available() > 0) {
    String rx = Serial.readStringUntil('\n');
    int command = rx.toInt();
    if ((command == 1) || (command == 2) || (command == 0)) {
      fire(command);
    }
  }
}

