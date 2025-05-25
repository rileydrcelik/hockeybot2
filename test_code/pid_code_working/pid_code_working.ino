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
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

// Motor speed limits and base speeds (tuned for your mismatched motors)
const int maxSpeedL = 90;
const int maxSpeedR = 90;
const int baseSpeedL = 45;
const int baseSpeedR = 45;

// PID variables
double input, output, setpoint;
double Kp = 0.000075, Ki = 0.0, Kd = .006;  // Tune these!
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Set up motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
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

  Serial.println("Calibration done.");
  delay(500);

  // PID setup
  setpoint = 3500; // center of 0-5000 for 6 sensors
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);  // restrict correction range, tune if needed
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
  // Read QTR sensor position (0 = far left, 5000 = far right)
  input = qtr.readLineBlack(sensorValues);

  // Run PID
  pid.Compute();

  // Apply PID correction
  int leftSpeed  = baseSpeedL + output;
  int rightSpeed = baseSpeedR - output;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, maxSpeedL);
  rightSpeed = constrain(rightSpeed, 0, maxSpeedR);

  moveMotors(leftSpeed, rightSpeed); // swapped order (your setup)

  // Debugging output
  Serial.print("POS: "); Serial.print(input);
  Serial.print(" | OUT: "); Serial.print(output);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);

  delay(10);
}
