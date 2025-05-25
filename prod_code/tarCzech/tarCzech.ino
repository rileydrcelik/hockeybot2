#include <QTRSensors.h>

// Motor driver pins
#define ENA 9   // Right motor speed
#define IN1 3   // Right motor dir 1
#define IN2 4   // Right motor dir 2
#define ENB 10  // Left motor speed
#define IN3 6   // Left motor dir 1
#define IN4 7   // Left motor dir 2

//max speed
const int baseSpeedL = 45;
const int baseSpeedR = 45;

int cn = 0;

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

/*base speed
const int baseSpeedL = 30;
const int baseSpeedR = 60;
*/

void setup() {
  
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

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

  // Calibration
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // done calibrating
  Serial.begin(9600);

  Serial.println("Calibration done.");
  delay(500);

}



void moveMotors(int speedL, int speedR){
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

bool tarCzech(){
  int count = 0;
  for (int i = 0; i < 8; i++){
    if(sensorValues[i] > 900){
      count++;
      if(count == 8){
        cn++;
        count = 0;
        return 1;
      }
    }
  }
  return 0;
}

void loop() {
  while(cn < 2){
    moveMotors(baseSpeedL, baseSpeedR);
    uint16_t position = qtr.readLineBlack(sensorValues);

    for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println();
    bool x = tarCzech();
    if (x==1){
      moveMotors(0, 0);
      delay(3000);
    }
  // Nothing needed, just keep going straight
  }
  moveMotors(0,0);
}
