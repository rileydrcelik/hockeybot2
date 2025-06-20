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

uint16_t sensorMin[8] = {703, 620, 507, 439, 496, 509, 501, 627};
uint16_t sensorMax[8] = {854, 803, 740, 700, 735, 755, 757,827};

// Motor speed limits and base speeds (tuned for your mismatched motors) and debug delay
const int baseSpeedL = 28; //19
const int baseSpeedR = 62; //31
const int maxSpeedL = 40; //20
const int maxSpeedR = 120; //32
const int debug = 2500;

// PID variables
double input, output, setpoint;
double Kp = .0035, Kd = .0002, Ki = 0.0; //Kp = .007, Kd = .0002, Ki = 0.0
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// === FUNCTION PROTOTYPES ===//
void moveMotors(int direction, int leftSpeed = -1, int rightSpeed = -1, int delayTime = 0, int stopAfter = 0);
void stopMotors(int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR);
bool czech(uint16_t* sensorValues, int mode);
void pidRun(uint16_t* sensorValues);
void pidGo(int mode);
void fire();
void blackPuck();
void yellowPuck();
void greenPuck();

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

void moveMotors(int direction, int leftSpeed = -1, int rightSpeed = -1, int delayTime = 0, int stopAfter = 0) {
  if (leftSpeed == -1) leftSpeed = baseSpeedL;
  if (rightSpeed == -1) rightSpeed = baseSpeedR;
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
  if (delayTime) delay(delayTime);
  if (stopAfter) stopMotors(leftSpeed, rightSpeed);
}

void stopMotors(int leftSpeed = baseSpeedL, int rightSpeed = baseSpeedR){
  moveMotors(0, leftSpeed, rightSpeed, 50);
  moveMotors(1, 0, 0);
}

bool czech(uint16_t* sensorValues, int mode){
  //mode 0 = line, mode 1 = puck
  int count = 0;
  if (mode == 0){
    //if it reads 5 sensors above 700 then we know were at a black line
    for (int i = 0; i < 8; i++){
      if(sensorValues[i] > 700){
        count++;
        if(count == 5){
          return 1;
        }
      }
    }
  }
  else{
    //likewise if we read 5 sensors below 50 we know theres a break in the line = puck
    for (int i = 0; i < 8; i++){
      if(sensorValues[i] < 50){
        count++;
        if(count == 3){
          return 1;
        }
      }
    }
  }
  return 0;
}

void pidRun(uint16_t* sensorValues){
// Read QTR sensor position (0 = far left, 7000 = far right)
  input = qtr.readLineBlack(sensorValues);
  pid.Compute();
  
  // Apply PID correction, and constrain speeds
  int leftSpeed  = constrain(baseSpeedL + output, 0, maxSpeedL);
  int rightSpeed = constrain(baseSpeedR - output, 0, maxSpeedR);

  moveMotors(1, leftSpeed, rightSpeed); 
}

void pidGo(int mode) {
//0 is line, 1 is for puck
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
    if (czech(sensorValues, mode)) return;
    pidRun(sensorValues);
  }
}

void fire(int range){
    //tell other nano to fire, then wait!!!
  Serial.println(range);
  delay(10);
  if (range == 0){
    delay(1000);
    moveMotors(1, -1, -1, 250, 1);
    delay(1000);
  }
  else if (range == 1){
    delay(2000);
    moveMotors(1, -1, -1, 250, 1);
    delay(1000);
  }
  else if (range == 2){
    delay(3000);
    moveMotors(1, -1, -1, 250, 1);
    delay(1000);
  }
}

void blackPuck(){
    //move up, turn, then fire
  moveMotors(1, -1, -1, 1100);
  moveMotors(3, baseSpeedL+10, 0, 1420, 1);
  fire(0);
    //go back to line
  moveMotors(0, -1, -1, 1200);
  moveMotors(3, -1, -1, 875);
  moveMotors(1, -1, -1, 2600);
  moveMotors(3, -1, -1, 875, 1);
}


void yellowPuck(){
    //pid until it seees puck, then turn
  moveMotors(1, -1, -1, 250);
  moveMotors(3, baseSpeedL+10, 0, 1400);
  stopMotors();
  delay(2500);

  //pidGo(1);
  //moveMotors(1, baseSpeedL+14, 20, 750);
    //pid until hits line, then aim and fire
  //pidGo(0);
  moveMotors(1, -1, -1, 2550);
  moveMotors(1, baseSpeedL+14, 15, 700);
  moveMotors(1, -1, -1, 600);
  moveMotors(2, 0, baseSpeedR, 1480, 1);
  fire(2);
    //back up and return to line
  moveMotors(0, -1, -1, 1000);
  moveMotors(3, -1, -1, 550, 1);
}

void greenPuck(){
    //pid until it sees green puck, aim, then fire
  //pidGo(1);
  moveMotors(1, -1, -1, 1000);
  moveMotors(1, baseSpeedL+3, 15, 900, 1);
  fire(0);
}

void loop(){
  //blackPuck();
  delay(500);
  yellowPuck();
  greenPuck();
  stopMotors();
  while (true); //STOP
}