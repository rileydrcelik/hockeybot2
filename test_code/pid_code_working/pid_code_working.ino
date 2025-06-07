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
const int EEPROM_ADDR = 0;

// QTR sensor array
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[8] = {667, 608, 484, 427, 502, 507, 499, 629};
uint16_t sensorMax[8] = {812, 756, 703, 643, 710, 757, 760, 798};

// Motor speed limits and base speeds (tuned for your mismatched motors)

const int baseSpeedL = 25;
const int baseSpeedR = 25;
const int maxSpeedL = 2*baseSpeedL;
const int maxSpeedR = 2*baseSpeedR;

// PID variables
double input, output, setpoint;
double Kp = 0.002, Ki = 0.0, Kd = .003;  // Tune these!
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void saveCalibration(){
  int addr = EEPROM_ADDR;
  for (uint8_t i = 0; i < SensorCount; i++){
    EEPROM.put(addr, qtr.calibrationOn.minimum[i]);
    addr+= sizeof(uint16_t);
  }
  for (uint8_t i = 0; i < SensorCount; i++){
    EEPROM.put(addr, qtr.calibrationOn.maximum[i]);
    addr+= sizeof(uint16_t);
  }
  Serial.println("calibration saved!!");
}

void loadCalibration() {
  int addr = EEPROM_ADDR;
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOn.minimum[i]);
    addr += sizeof(uint16_t);
  }
  for (uint8_t i = 0; i < SensorCount; i++) {
    EEPROM.get(addr, qtr.calibrationOn.maximum[i]);
    addr += sizeof(uint16_t);
  }
  Serial.println("Calibration loaded from EEPROM.");
}

void setup() {
  Serial.begin(9600);
  delay(2500);
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

  // Calibration... run ONCE
  // for (uint16_t i = 0; i < 200; i++) {
  //   qtr.calibrate();
  //   delay(5);
  // }
  // digitalWrite(LED_BUILTIN, LOW); // done calibrating
  // saveCalibration();

  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   qtr.calibrationOn.minimum[i] = sensorMin[i];
  //   qtr.calibrationOn.maximum[i] = sensorMax[i];
  // }
  loadCalibration();

  Serial.println("Calibration done.");
  delay(500);

  // PID setup
  setpoint = 3500; // center of 0-5000 for 6 sensors
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);  // restrict correction range, tune if needed

  // for (uint8_t i = 0; i < SensorCount; i++) {
  //   Serial.print("Min["); Serial.print(i); Serial.print("] = "); Serial.print(qtr.calibrationOn.minimum[i]); Serial.print(" | ");
  //   Serial.print("Max["); Serial.print(i); Serial.print("] = "); Serial.println(qtr.calibrationOn.maximum[i]);
  // }

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
  qtr.readCalibrated(sensorValues);
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
