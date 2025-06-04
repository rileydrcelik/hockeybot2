#include <QTRSensors.h>
#include <PID_v1.h>

// Motor driver pins
#define ENA 9  // Right motor speed (PWM)
#define IN1 3  // Right motor dir 1

#define IN2 4  // Right motor dir 2
#define ENB 10  // Left motor speed (PWM)
#define IN3 6  // Left motor dir 1
#define IN4 7  // Left motor dir 2

int cn = 0;
//cn for crossczech

// QTR sensor array
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

uint16_t sensorMin[8] = {669, 590, 603, 594, 515, 545, 634, 658};
uint16_t sensorMax[8] = {750, 722, 725, 719, 675, 707, 749, 745};

// Motor speed limits and base speeds (tuned for your mismatched motors)
const int baseSpeedL = 22;
const int baseSpeedR = 22;
const int maxSpeedL = 2*baseSpeedL;
const int maxSpeedR = 2*baseSpeedR;


// PID variables
double input, output, setpoint;
double Kp = 0.002, Ki = 0.0, Kd = .003;  // Tune these!    //solid values:  Kp = 0.0078, Ki = 0.0, Kd = .0035
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
  for (uint16_t i = 0; i < 200; i++) {
    qtr.calibrate();
    delay(5);
  }
  digitalWrite(LED_BUILTIN, LOW); // done calibrating

  // for (uint8_t i = 0; i < SensorCount; i++){
  //   qtr.calibrationOn.minimum[i] = sensorMin[i];
  //   qtr.calibrationOn.maximum[i] = sensorMax[i];
  // }

  Serial.println("Calibration done.");
  delay(500);

  // PID setup
  setpoint = 3250; // center of 0-5000 for 6 sensors
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
  moveMotors(1, maxSpeedL, maxSpeedR);
  delay(350);
  stopMotors();

  if (turnType == 1){
    turn(0, 200);
  }
  else{
    turn(0, 400);
  }

  stopMotors();
  delay(500);


  moveMotors(1, maxSpeedL, maxSpeedR);
  delay(250);
  stopMotors();
  delay(8000);
}

bool crossCzech(){
  int count = 0;
  for (int i = 1; i < 7; i++){
    if(sensorValues[i] > 950){
      count++;
      if(count == 6){
        cn++;
        count = 0;
        return 1;
      }
    }
  }
  
  return 0;
}

bool puckCzech(){
  int count = 0;
  for (int i = 0; i < 8; i++){
    if( (sensorValues[i] < 500)){
      count++;
      if(count == 6){
        cn++;
        count = 0;
        return 1;
      }
    }
  }
  
  return 0;
}

void pidGo(int cnGoal, bool puckNo) {
//cnGoal is amount of pucks/crosses it will go over
//puckNo is which puck were targetting
  cn = 0;
  while(cn < cnGoal){
    // Read QTR sensor position (0 = far left, 5000 = far right)
    input = qtr.readLineBlack(sensorValues);
    //qtr.readCalibrated(sensorValues);

    // Run PID
    pid.Compute();

    //delay(10);
    bool x;
    if (cn == 0){
      x = puckCzech();
    }
    else{
      x = crossCzech();
    }
    //if a puck/line is detected...
    if (x==1){
      //cn == 1 means it passed the puck
      if(cn == 1){
        if (puckNo == 1){
          stopMotors();
        }
        else{
          //turn right when over the first puck
          stopMotors();
          delay(5000);
          moveMotors(1, baseSpeedL/2, baseSpeedR);
          delay(500);
        }
        }
      //else imples cn > 1 and its on the cross
      else{
        stopMotors();
      }
      
    }
    else{
      // Apply PID correction
      int leftSpeed  = baseSpeedL + output;
      int rightSpeed = baseSpeedR - output;

      // Constrain speeds
      leftSpeed = constrain(leftSpeed, 0, maxSpeedL);
      rightSpeed = constrain(rightSpeed, 0, maxSpeedR);

      moveMotors(1, leftSpeed, rightSpeed); // swapped order (your setup)

      // Debugging output
      Serial.print("POS: "); Serial.print(input);
      Serial.print(" | OUT: "); Serial.print(output);
      Serial.print(" | L: "); Serial.print(leftSpeed);
      Serial.print(" | R: "); Serial.println(rightSpeed);
    }

    
  }
  stopMotors();

}

void loop(){
  //go forward when starting from middle
  // moveMotors(1)
  // delay(3000);
  // turn(1, 250); //90 deg turn to face track
  // autoCalibrate();

  

  pidGo(2, 0);
  aimMotors(0);
  Serial.println(1);

  turn(1, 400);
  stopMotors();
  delay(250);

  pidGo(1, 1);
  aimMotors(1);
  Serial.println(1);
  // pidGo(1, 1);
  // aimMotors(1);

  // //TESTING DIRECRIONS
  // //forward
  // moveMotors(1);
  // delay(3000);
  // stopMotors();
  // delay(2500);
  // //back
  // moveMotors(0);
  // delay(3000);
  // stopMotors();
  // delay(2500);
  // //left turn
  // turn(0, 2500);
  // stopMotors();
  // delay(2500);
  // //right turn
  // turn(1, 2500);
  // stopMotors();
  // delay(2500);
  // //aim 1
  // aimMotors(0);
  // delay(2500);
  // //aim 2
  // aimMotors(1);
  // delay(1000000);


  delay(5000000);
  
}
