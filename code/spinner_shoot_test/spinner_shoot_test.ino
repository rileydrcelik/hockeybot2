// Pin assignments
#define IN1 3     // Motor 1 direction
#define IN2 4

#define IN3 6    // Motor 2 direction
#define IN4 7

#define ENA 9     // Motor 1 speed
#define ENB 10    // Motor 2 speed

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
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  // Start both motors at half speed
  //analogWrite(ENA, 64);
  //analogWrite(ENB, );
  pinMode(5, OUTPUT);

}

void loop() {
  //analogWrite(ENA, 64);
  analogWrite(ENB, 50);\
  
  digitalWrite(5, HIGH);
  
  delay(500);
}
