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
  // Set up motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Set direction forward for both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Start motors at your desired speed
  analogWrite(ENA, baseSpeedL); // Right motor speed
  analogWrite(ENB, baseSpeedR); // Left motor speed
}

void loop() {
  // Nothing needed, just keep going straight
}
