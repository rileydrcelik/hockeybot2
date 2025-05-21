// Pin assignments
#define S_IN1 9     // Motor 1 direction
#define S_IN2 10
#define S_IN3 11     // Motor 2 direction
#define S_IN4 12

#define S_ENA 8     // Motor 1 speed
#define S_ENB 13    // Motor 2 speed

// Motor driver pins
#define ENA 2  // Right motor speed
#define IN1 3  // Right motor dir 1
#define IN2 4  // Right motor dir 2
#define ENB 5  // Left motor speed
#define IN3 6  // Left motor dir 1
#define IN4 7  // Left motor dir 2

const int baseSpeedL = 128;//128; // adjust if needed for straightness
const int baseSpeedR = 128;  // adjust if needed for straightness

void setup() {
  // Set all pins as OUTPUT
  pinMode(S_IN1, OUTPUT);
  pinMode(S_IN2, OUTPUT);
  pinMode(S_IN3, OUTPUT);
  pinMode(S_IN4, OUTPUT);
  pinMode(S_ENA, OUTPUT);
  pinMode(S_ENB, OUTPUT);

  // Motor 1 forward, Motor 2 backward
  digitalWrite(S_IN1, HIGH);
  digitalWrite(S_IN2, LOW);
  digitalWrite(S_IN3, LOW);
  digitalWrite(S_IN4, HIGH);

  // Start both motors at half speed
  analogWrite(S_ENA, 64);
  //analogWrite(S_ENB, 64);

  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Set direction forward for both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Start motors at your desired speed
  analogWrite(ENA, baseSpeedR); // Right motor speed
  analogWrite(ENB, baseSpeedR); // Left motor speed
}

void loop() {

  delay(2000);
}
