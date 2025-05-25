#include <QTRSensors.h>
#include <PID_v1.h>

// Motor driver pins
#define ENA 9  // Right motor speed (PWM)
#define IN1 3  // Right motor dir 1
#define IN2 4  // Right motor dir 2
#define ENB 10  // Left motor speed (PWM)
#define IN3 6  // Left motor dir 1
#define IN4 7  // Left motor dir 2

// QTR sensor array
const uint8_t SensorCount = 6;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// Motor speed limits and base speeds (tuned for your mismatched motors)
const int maxSpeedL = 64;
const int maxSpeedR = 200;
const int baseSpeedL = 30;
const int baseSpeedR = 60;

double input;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for serial to connect (especially useful on Leonardo)
  Serial.println("Master ready");





  // Set up motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A5, A4, A3, A2, A1, A0}, SensorCount);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // calibration mode

  // Calibration
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(5);
  }
  digitalWrite(LED_BUILTIN, LOW); // done calibrating

  delay(500);
}

void moveMotors(int leftSpeed, int rightSpeed) {
  // Forward direction for both motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, constrain(abs(leftSpeed), 0, maxSpeedL));
  analogWrite(ENB, constrain(abs(rightSpeed), 0, maxSpeedR));
}

void loop() {
  input = qtr.readLineBlack(sensorValues);
  if ((input < 250) || (input > 4500)){
    moveMotors(baseSpeedL + 10, baseSpeedR);
    Serial.println("Ping");
    delay(1000);
  }
  moveMotors(baseSpeedL, baseSpeedR);

  
  delay(2000);
}
