#include <QTRSensors.h>
#include <PID_v1.h>

// Pin assignments
#define IN1 3     // Motor 1 direction
#define IN2 4
#define IN3 6    // Motor 2 direction
#define IN4 7

#define ENA 9     // Motor 1 speed
#define ENB 10    // Motor 2 speed

// QTR sensor array
const uint8_t SensorCount = 6;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

double input;

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

void loop() {
  analogWrite(ENB, 0); //ENB for eating puck
  analogWrite(ENA, 0); //ENA for backshots
  // analogWrite(ENA, 0);
  // analogWrite(ENB, 0);
  // delay(5000);
  input = qtr.readLineBlack(sensorValues);
  if ((input < 250) || (input > 4500)){
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    delay(1000);
  }
}
