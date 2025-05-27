// Motor driver pins
#define ENA 9   // Right motor speed
#define IN1 3   // Right motor dir 1
#define IN2 4   // Right motor dir 2
#define ENB 10  // Left motor speed
#define IN3 6   // Left motor dir 1
#define IN4 7   // Left motor dir 2

//max speed
const int baseSpeedL = 45;
const int baseSpeedR = 45;


/*base speed
const int baseSpeedL = 30;
const int baseSpeedR = 60;
*/

void setup() {
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
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  
}

void moveMotors(int speedL, int speedR){
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

void stopMotors(int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  moveMotors(leftSpeed, rightSpeed);
  delay(50);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  moveMotors(0, 0);
}

void loop() {
  // Start motors at your desired speed
  analogWrite(ENA, baseSpeedL);  // Right motor speed
  analogWrite(ENB, baseSpeedR);  // Left motor speed

  delay(3000);

  stopMotors(baseSpeedL, baseSpeedR);
  delay(1000);

  // Nothing needed, just keep going straight
}
