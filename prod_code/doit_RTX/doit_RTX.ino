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
const int baseSpeedL = 25;
const int baseSpeedR = 25;
const int maxSpeedL = 2*baseSpeedL;
const int maxSpeedR = 2*baseSpeedR;


// PID variables
double input, output, setpoint;
double Kp = 0.002, Ki = 0.0, Kd = .003;  // Tune these!    //solid values:  Kp = 0.0078, Ki = 0.0, Kd = .0035  old kp value 0.002
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

  //qtr.setCalibration(sensorMin, sensorMax);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // calibration mode

  delay(2500);
  // Calibration
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    // delay(5);
  }
  digitalWrite(LED_BUILTIN, LOW); // done calibrating

  Serial.println("Calibration done.");

  //move wheel to show its done

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 30);
  analogWrite(ENB, 30);

  delay(1000);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  delay(3000);

  // PID setup
  setpoint = 3500; // center of 0-5000 for 6 sensors
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);  // restrict correction range, tune if needed
}

void moveMotors(int direction, int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR) {
  //0 = backwards, 1 = forwards, 2 = left, 3 = right
  if (direction == 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
      
  }
  else if (direction == 1){
    // Forward direction for both motors
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
  }
  else if (direction == 2){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction == 3){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  

  analogWrite(ENA, constrain(abs(leftSpeed), 0, maxSpeedL));
  analogWrite(ENB, constrain(abs(rightSpeed), 0, maxSpeedR));
}

void stopMotors(int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR){
  moveMotors(0, leftSpeed, rightSpeed);
  delay(50);
  moveMotors(1, 0, 0);
}

void turn(bool direction, int time){
  //0 = left, 1 = right
  if (direction == 0){
    moveMotors(2, maxSpeedL, maxSpeedR);
  }
  else{
    moveMotors(3, maxSpeedL, maxSpeedR);
  }
  delay(time);
  stopMotors();
}


void aimMotors(bool turnType){
  moveMotors(1, maxSpeedL, maxSpeedR);
  delay(350);
  stopMotors();

  if (turnType == 1){
    turn(1, 200);
  }
  else{
    turn(0, 500);
  }

  stopMotors();
  delay(500);


  moveMotors(1, maxSpeedL, maxSpeedR);
  delay(250);
  stopMotors();
  //delay(8000); use this when actually doit
  delay(1000);
}

bool crossCzech(){
  qtr.readCalibrated(sensorValues);
  int count = 0;
  for (int i = 0; i < 8; i++){
    if(sensorValues[i] > 950){
      count++;
      if(count == 6){
        count = 0;
        return 1;
      }
    }
  }
  return 0;
}

bool puckCzech(){
  qtr.readCalibrated(sensorValues);
  int count = 0;
  for (int i = 0; i < 8; i++){
    if( (sensorValues[i] < 50)){
      count++;
      if(count == 2){
        count = 0;
        return 1;
      }
    }
  }
  return 0;
}

void pidRun(){
// Read QTR sensor position (0 = far left, 5000 = far right)
  input = qtr.readLineBlack(sensorValues);
  //qtr.readCalibrated(sensorValues);
  pid.Compute();
  
  // Apply PID correction
  int leftSpeed  = baseSpeedL + output;
  int rightSpeed = baseSpeedR - output;

  // Constrain speeds
  leftSpeed = constrain(leftSpeed, 0, maxSpeedL);
  rightSpeed = constrain(rightSpeed, 0, maxSpeedR);

  moveMotors(1, leftSpeed, rightSpeed); // swapped order (your setup)
}

void pidGo(int mode) {
//0 is for puck, 1 is for cross
  int looper = 0;

  while (looper < 1){
    if (mode == 0){
      if (puckCzech() == 1){
        return;
      }
      else{
        pidRun();
      }
    }

    else if (mode == 1){
      if (crossCzech() == 1){
        stopMotors();
        return;
      }
      else{
        pidRun();
      }
    }
  }

  
  stopMotors();

}

void loop(){
  //pid until it seees puck
  pidGo(0);
  //once it sees the puck it turns right
  moveMotors(1, baseSpeedL/2, baseSpeedR*2);
  delay(850);

  //pid until it sees the cross
  pidGo(1);
  //aim then fire
  aimMotors(0);
  Serial.println(1);

  //back up, turn to face line, then pid until other puck
  delay(5000);
  moveMotors(0);
  delay(250);
  turn(1, 580);
  pidGo(0);
  moveMotors(1, (baseSpeedL/2)+10, baseSpeedR+10);
  delay(500);
  //aim then fire
  moveMotors(1);
  delay(125);
  aimMotors(1);
  Serial.println(1);

  // turn(1, 400);
  // stopMotors();
  // delay(250);

  // pidGo(1, 1);
  // Serial.println(1);
  // aimMotors(1);

  delay(5000000);
}
