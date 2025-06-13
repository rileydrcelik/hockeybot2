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

const int debug = 2500;


// Motor speed limits and base speeds (tuned for your mismatched motors)
const int baseSpeedL = 19;
const int baseSpeedR = 31;
const int maxSpeedL = 20;
const int maxSpeedR = 32;


// PID variables
double input, output, setpoint;
//double Kp = 0.001, Ki = 0.0, Kd = .002;  // Tune these!    //solid values:  Kp = 0.0078, Ki = 0.0, Kd = .0035  old kp value 0.002
double Kp = .010, Kd = .00031, Ki = 0.0;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);

  // Set up motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Configure the sensors
  qtr.setTypeAnalog();

  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  qtr.setEmitterPin(12);

  // PID setup
  setpoint = 3500;
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-40, 40);
}


void moveMotors(int direction, int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR) {
  //0 = backwards, 1 = forwards, 2 = left, 3 = right
  if (direction == 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed-=3;
    rightSpeed-=8;
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
    leftSpeed+=4;
    rightSpeed-=4;
  }
  else if (direction == 3){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed-=3;
  }
  analogWrite(ENA, constrain(abs(leftSpeed), 0, maxSpeedL));
  analogWrite(ENB, constrain(abs(rightSpeed), 0, maxSpeedR));
}

void stopMotors(){
  moveMotors(0, baseSpeedL, baseSpeedR);
  delay(50);
  moveMotors(1, 0, 0);
}

void turn(bool direction, int time = 0){
  //0 = left, 1 = right
  if (time != 0){
    if (direction == 0){
      moveMotors(2);
    }
    else{
      moveMotors(3);
    }
    delay(time);
  }
  else{
    if (direction == 0){
      moveMotors(2, 0, baseSpeedR);
      delay(1950);
    }
    else{
      moveMotors(3, baseSpeedL+5, 0);
      delay(750);
    }
  }
  moveMotors(1, 0, 0);
}

bool crossCzech(uint16_t* sensorValues){
  int count = 0;
  for (int i = 0; i < 8; i++){
    if(sensorValues[i] > 700){
      count++;
      if(count == 5){
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
      if(count == 5){
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
  int startupDelay = 2500;
  unsigned long startTime = millis();

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
    //DEBUG CODEEEEEEE
    // Serial.print("Mapped sensor values: ");
    // for (uint8_t i = 0; i < SensorCount; i++) {
    //   Serial.print(sensorValues[i]);
    //   Serial.print("\t");
    // }


    Serial.println();
    if (mode == 0 && puckCzech(sensorValues)) return;
    if (mode == 1 && crossCzech(sensorValues)) {
      stopMotors();
      return;
    }
    pidRun(sensorValues);
  }
  stopMotors();
}

void fire(){
    //fire
  Serial.println(1);
  delay(3000);
  // moveMotors(1, 25, 20);
  delay(250);
  // stopMotors();
  delay(1000);
}

void blackPuck(){
  // pidGo(1);
  // pidGo(1);
  //-------------BLACK PUCK--------------//
  moveMotors(1);
  delay(750);

  //turn right 90 deg then fire
  turn(1);
  delay(debug);

  // //go forward to 'eat puck' fully
  // moveMotors(1, baseSpeedL, baseSpeedR);
  // delay(500);
  // stopMotors();
  // // delay(debug);

  fire();

  // stopMotors();
  // delay(debug);

  //go back to line
  moveMotors(0);
  delay(600);
  // stopMotors();
  // delay(debug);

  turn(1, 600);
  // delay(debug);

  moveMotors(1);
  delay(1200);
  // stopMotors();
  // delay(debug);

  turn(1, 550);
  stopMotors();
  delay(debug);
}

void yellowPuck(){
    //---------------YELLOW PUCK ONE---------------//

  //pid until it seees puck
  pidGo(0);
  moveMotors(1, baseSpeedL+1, 14);
  delay(1500);
  //once it sees the puck it turns right
  //moveMotors(1, baseSpeedL*0, baseSpeedR*2);
  //delay(200);
  //pid until it sees the cross
  pidGo(1);
  //aim then fire
  moveMotors(1);
  delay(250);
  
  moveMotors(2, 0, baseSpeedR);
  delay(1890);

  stopMotors();
  delay(debug);

  fire();

  // stopMotors();
  // delay(debug);

  //back up and return to line
  moveMotors(0);
  delay(900);
  turn(1, 800);
}

void greenPuck(){
  //---------------------GREEN PUCK TWO---------------//
  //go after green
  pidGo(0);
  moveMotors(1, baseSpeedL+3, 15);
  delay(850);
  moveMotors(1, 0, 0);
  delay(250);

  fire();

  stopMotors();
  delay(debug);
}

void loop(){
  blackPuck();
  //-------black done!!!
  yellowPuck();
  //-------yellow done!!!
  greenPuck();
  delay(5000000);
}
