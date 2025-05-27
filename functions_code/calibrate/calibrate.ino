/*

NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!
NOTE TO SELF ADD DELAY BETWEEN qtr.calibrate!!!!!!!!!


*/

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

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


/*base speed
const int baseSpeedL = 30;
const int baseSpeedR = 60;
*/

void setup() {
  Serial.begin(9600);
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

  calibrate();
}

void calibrate(){
  int x = 0;
  while (x < 3){
    //MOVE TO THE LEFT
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    moveMotors(baseSpeedL, baseSpeedR);

    for (uint16_t i = 0; i < 10; i++)
    {
      qtr.calibrate();
    }
    moveMotors(0, 0);


    //MOVE TO THE RIGHT
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    moveMotors(baseSpeedL, baseSpeedR);

    for (uint16_t i = 0; i < 18; i++)
    {
      qtr.calibrate();
    }

    moveMotors(0, 0);



    //MOVE BACK LEFT
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    moveMotors(baseSpeedL, baseSpeedR);

    for (uint16_t i = 0; i < 10; i++)
    {
      qtr.calibrate();
    }
    moveMotors(0, 0);
    x++;
  }


  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  //MOTORS IN FORWARDS MODE
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveMotors(int speedL, int speedR){
  analogWrite(ENA, speedL);
  analogWrite(ENB, speedR);
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);

  // Nothing needed, just keep going straight
}
