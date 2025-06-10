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
uint16_t sensorMin[8] = {667, 608, 484, 427, 502, 507, 499, 629};
uint16_t sensorMax[8] = {812, 756, 703, 643, 710, 757, 760, 798};


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
  //move wheel to show its done

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, 30);
  analogWrite(ENB, 30);

  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  // PID setup
  setpoint = 3500; // center of 0-5000 for 6 sensors
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);  // restrict correction range, tune if needed
}

void moveMotors(int direction, int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR) {
  //0 = backwards, 1 = forwards, 2 = left, 3 = right
  if (direction == 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
      
  }
  else if (direction == 1){
    // Forward direction for both motors
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction == 2){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else if (direction == 3){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
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
  stopMotors();
  if (turnType == 1){
    turn(1, 100);
    stopMotors();
    delay(500);
    moveMotors(1, maxSpeedL, maxSpeedR);
    delay(250);
  }
  else{
    moveMotors(1, maxSpeedL, maxSpeedR);
    delay(350);
    stopMotors();
    delay(250);
    turn(0, 405);
    stopMotors();
    delay(500);
    moveMotors(1, maxSpeedL, maxSpeedR);
    delay(100);
  }
  stopMotors();
  //delay(8000); use this when actually doit
  delay(1000);
}

bool crossCzech(uint16_t* sensorValues){
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

bool puckCzech(uint16_t* sensorValues){
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

void pidRun(uint16_t* sensorValues){
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
  while (true){
    qtr.read(sensorValues);
    for (uint8_t i = 0; i < SensorCount; i++) {
      sensorValues[i] = constrain(sensorValues[i], sensorMin[i], sensorMax[i]);
      if (sensorMax[i] > sensorMin[i]) {
        sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
      } else {
        sensorValues[i] = 0; // Prevent divide by zero
      }
    }
    if (mode == 0){
      if (puckCzech(sensorValues) == 1){
        return;
      }
      else{
        pidRun(sensorValues);
      }
    }

    else if (mode == 1){
      if (crossCzech(sensorValues) == 1){
        stopMotors();
        return;
      }
      else{
        pidRun(sensorValues);
      }
    }
  }

  
  stopMotors();

}

void blackPuck(){
  //-------------BLACK PUCK--------------//
  moveMotors(1);
  delay(1500);
  stopMotors();
  delay(350);
  //turn right 90 deg then fire
  turn(1, 365);
  delay(1000);
  //go forward to 'eat puck' fully
  moveMotors(1);
  delay(500);
  stopMotors();
  delay(260);
  //fire
  Serial.println(1);
  delay(7500);
  moveMotors(1, 15, 15);
  delay(5000);
  //go back to line
  stopMotors();
  moveMotors(0);
  delay(800);
  turn(1, 330);
  moveMotors(1);
  delay(2100);
  stopMotors(250);
  turn(1, 280);
  stopMotors();
  delay(1000);
}

void yellowPuck(){
    //---------------PUCK ONE---------------//

  //pid until it seees puck
  pidGo(0);
  //once it sees the puck it turns right
  moveMotors(1, baseSpeedL/2, baseSpeedR*2);
  delay(550);
  //pid until it sees the cross
  pidGo(1);
  //aim then fire
  aimMotors(0);
  Serial.println(1);
  delay(7500);
  moveMotors(1, 15, 15);
  delay(5000);
}

void greenPuck(){
    //---------------------PUCK TWO---------------//
  //back up, turn to face line, then pid until other puck
  moveMotors(0);
  delay(400);
  turn(1, 420);
  pidGo(0);
  moveMotors(1, (baseSpeedL/2)+10, baseSpeedR+10);
  delay(500);
  //aim then fire
  moveMotors(1);
  delay(125);
  aimMotors(1);
  Serial.println(1);
  delay(7500);
  moveMotors(1, 15, 15);
  delay(5000);
}

void loop(){
  blackPuck();
  yellowPuck();
  greenPuck();
  delay(5000000);
}
