// Pin assignments
#define IN1 9     // Motor 1 direction
#define IN2 10
#define IN3 11     // Motor 2 direction
#define IN4 12

#define ENA 8     // Motor 1 speed
#define ENB 13    // Motor 2 speed

void setup() {
  // Set all pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Motor 1 forward, Motor 2 backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Start both motors at half speed
  analogWrite(ENA, 128);
  analogWrite(ENB, 128);
}

void loop() {
  // Run for 2 seconds
  delay(2000);

  // Stop both motors
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // Wait 1 second, then reverse directions
  delay(1000);

  // Reverse both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Run at half speed
  analogWrite(ENA, 128);
  analogWrite(ENB, 128);

  delay(2000);

  // Stop again
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  delay(1000);

  // Set back to original direction for loop
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // Loop continues...
  analogWrite(ENA, 128);
  analogWrite(ENB, 128);
  delay(2000);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  delay(1000);
}
