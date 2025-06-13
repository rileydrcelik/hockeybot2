#include <QTRSensors.h>
#include <PID_v1.h>
#include <EEPROM.h>

// Motor driver pins
#define ENA 9  // Right motor speed (PWM)
#define IN1 3  // Right motor dir 1
#define IN2 4  // Right motor dir 2
#define ENB 10  // Left motor speed (PWM)
#define IN3 6  // Left motor dir 1
#define IN4 7  // Left motor dir 2

//defining eeprom addr

// QTR sensor array
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[8] = {769, 757, 664, 689, 697, 715, 776, 796};
uint16_t sensorMax[8] = {872, 861, 789, 808, 813, 827, 866, 879};


// Motor speed limits and base speeds (tuned for your mismatched motors)

const int baseSpeedL = 30;
const int baseSpeedR = 20;
const int maxSpeedL = 2*baseSpeedL;
const int maxSpeedR = 2*baseSpeedR;

// PID variables
double input, output, setpoint;
double Kp = 0.0025, Ki = 0.0, Kd = .00185;  // Tune these!
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

  pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH); // calibration mode

  // // for (uint16_t i = 0; i < 200; i++) {
  // //   qtr.calibrate();
  // //   delay(5);
  // // }
  // digitalWrite(LED_BUILTIN, LOW); // done calibrating

  // for (uint16_t i = 0; i < 200; i++) {
  //   qtr.read(sensorValues);

  //   for (uint8_t j = 0; j < SensorCount; j++) {
  //     if (sensorValues[j] < sensorMin[j]) sensorMin[j] = sensorValues[j];
  //     if (sensorValues[j] > sensorMax[j]) sensorMax[j] = sensorValues[j];
  //   }

  //   delay(5);
  // }


  Serial.println("Calibration done.");
  

  // PID setup
  setpoint = 3500; // center of 0-5000 for 6 sensors
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);  // restrict correction range, tune if needed

  

  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print("Min["); Serial.print(i); Serial.print("] = "); Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(" | ");
  //   Serial.print("Max["); Serial.print(i); Serial.print("] = "); Serial.println(qtr.calibrationOn.maximum[i]);
  // }

  // Serial.print("uint16_t sensorMin[8] = {");
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(sensorMin[i]);
  //   if (i < SensorCount - 1) Serial.print(", ");
  // }
  // Serial.println("};");

  // Serial.print("uint16_t sensorMax[8] = {");
  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print(sensorMax[i]);
  //   if (i < SensorCount - 1) Serial.print(", ");
  // }
  // Serial.println("};");

}

void moveMotors(int leftSpeed, int rightSpeed) {
  // Forward direction for both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, constrain(abs(leftSpeed), 0, maxSpeedL));
  analogWrite(ENB, constrain(abs(rightSpeed), 0, maxSpeedR));
}

void loop() {
  // Read QTR sensor position (0 = far left, 5000 = far right)
  //qtr.readCalibrated(sensorValues);
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = constrain(sensorValues[i], sensorMin[i], sensorMax[i]);
    if (sensorMax[i] > sensorMin[i]) {
      sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    } else {
      sensorValues[i] = 0; // Prevent divide by zero
    }
  }
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
}
