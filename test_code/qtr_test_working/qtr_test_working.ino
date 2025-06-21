#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t sensorMin[8] = {620, 558, 427, 372, 450, 462, 443, 572};
uint16_t sensorMax[8] = {765, 732, 635, 590, 643, 651, 647, 736};

void setup()
{
  Serial.begin(9600);
  
  // Configure QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12);

  // Setup LED indicator if needed for debugging
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // Read sensor values (calibrated based on manual values already stored within sensorMin and sensorMax)
  qtr.read(sensorValues);

  // Map each sensor value from the manual calibration range into 0–1000
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = constrain(sensorValues[i], sensorMin[i], sensorMax[i]);
    if (sensorMax[i] > sensorMin[i]) {
      sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
    } else {
      sensorValues[i] = 0; // Prevent divide-by-zero error
    }
  }

  // Print mapped sensor values, tab-separated
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  
  delay(200); // Slow down output for readability
}