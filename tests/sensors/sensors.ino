#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <QTRSensors.h>

#include "ColorDetection.h"

#define D8 34
#define D7 35
#define D6 32
#define D5 33
#define D4 25
#define D3 26
#define D2 27
#define D1 14

#define LED_PIN 23

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Create the TCS34725 instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {

  Serial.begin(115200);

  // Configurar array de sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){D1, D2, D3, D4, D5, D6, D7, D8}, SensorCount);
  qtr.setNonDimmable();

  delay(500);

  uint32_t startTime = millis();
  Serial.println("Calibration started.");
  // Calibrate for 10 s
  while (millis() - startTime < 10000)
  {
    qtr.calibrate();
  }
  Serial.println("Calibration done.");
  
  delay(1000);

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void loop() {
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000
  uint16_t position = qtr.readLineBlack(sensorValues);

  // print the sensor values as numbers from 0 to 1000, where 0 means maximum
  // reflectance and 1000 means minimum reflectance, followed by the line
  // position
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(position);

  uint8_t colorCode;
  getColorCode(&colorCode);
  printColorCode(colorCode);
  Serial.println();
  Serial.println();

  delay(500);
}
