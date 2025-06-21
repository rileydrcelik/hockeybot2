#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[SensorCount];
uint16_t sensorMax[SensorCount];


void setup() {
  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  // Initialize min and max values
  for (int i = 0; i < SensorCount; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  Serial.println("=== QTR Manual Calibration ===");
  Serial.println("Wave the robot over the line for 200 iterations...");

  for (int i = 0; i < 200; i++) {
    qtr.read(sensorValues);

    for (int j = 0; j < SensorCount; j++) {
      if (sensorValues[j] < sensorMin[j]) sensorMin[j] = sensorValues[j];
      if (sensorValues[j] > sensorMax[j]) sensorMax[j] = sensorValues[j];
    }

    delay(20);  // Small delay for sensor stability and physical movement
  }

  Serial.println("\n--- Calibration Complete ---");

  Serial.print("uint16_t sensorMin[8] = {");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorMin[i]);
    if (i < SensorCount - 1) Serial.print(", ");
  }
  Serial.println("};");

  Serial.print("uint16_t sensorMax[8] = {");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorMax[i]);
    if (i < SensorCount - 1) Serial.print(", ");
  }
  Serial.println("};");

  while (true); // Halt after printing
}

void loop() {
  // No loop logic needed
}
