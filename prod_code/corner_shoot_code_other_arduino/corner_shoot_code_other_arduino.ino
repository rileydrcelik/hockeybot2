#include <QTRSensors.h>

// Motor driver pins
#define ENA 9   // Right motor speed
#define IN1 3   // Right motor dir 1
#define IN2 4   // Right motor dir 2
#define ENB 10  // Left motor speed
#define IN3 6   // Left motor dir 1
#define IN4 7   // Left motor dir 2

//max speed
const int baseSpeedL = 255;
const int baseSpeedR = 255;

int cn = 0;
//cn for tarczech

// QTR sensor array
const uint8_t SensorCount = 8;
QTRSensors qtr;
uint16_t sensorValues[SensorCount];

/*base speed
const int baseSpeedL = 30;
const int baseSpeedR = 60;
*/

void setup() {
  Serial.begin(9600);
  
  // Set up motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set direction forward for both motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  

  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

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

  delay(2500);


}

bool puckCzech(){
  int count = 0;
  for (int i = 1; i < 7; i++){
    if(sensorValues[i] < 50){
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

bool crossCzech(){
  int count = 0;
  for (int i = 1; i < 7; i++){
    Serial.println(sensorValues[i]);
    if(sensorValues[i] > 950){
      count++;
      //Serial.println(count);
      if(count == 6){
        cn++;
        count = 0;
        return 1;
      }
    }
  }
  
  return 0;
}

void moveMotors(int speedL, int speedR){
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

void fire(int cnGoal){
  cn = 0;
  while (cn < cnGoal){
    bool x = crossCzech();
    //Serial.println(x);
    if (x==1){
      delay(2500);
    }
    Serial.println(cn);
  }
  moveMotors(255, 0);
  delay(3000);
  moveMotors(255, 255);
  delay(5000);
  moveMotors(0, 0);
  // while (cn < cnGoal){
  //   moveMotorsBack(0, 0);
  //   bool x = tarCzech();
  //   if (x==1){
  //     delay(3000);
  //   }
  // }
  // moveMotorsBack(255, 0);
  // delay(3000);
  // moveMotorsBack(255, 255);
  //bool x;
  // if (cn == 0) {
  //   x = puckCzech();
  //   if (x==1){
  //     delay(5000);
  //   }
  // }
  // x = crossCzech();
  // //Serial.println(x);
  // if (x==1){
  //   delay(2500);
  // }
  // Serial.println(cn);

}


void loop() {
  //moveMotors(255, 255);
  fire(1);
  delay(250000);
  // fire(1);
}
