#include "LineDetection.h"


const uint8_t sensorCount = 8;
uint8_t sensorPins[sensorCount] = { D1, D2, D3, D4, D5, D6, D7, D8 };

float weights[sensorCount] = { -1.0, -0.75, -0.50, -0.25, 0.25, 0.50, 0.75, 1.0 };

uint16_t sensorValues[sensorCount];
uint16_t minValues[sensorCount] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };
uint16_t maxValues[sensorCount] = { 0, 0, 0, 0, 0, 0, 0, 0 };

uint16_t lastPosition = 0;


void initLineDetectorPins() {
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

uint16_t* readArray() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  return sensorValues;
}

void printArray(uint16_t* values) {
  for (int i = 0; i < sensorCount; i++) {
    Serial.print(values[i]);
    Serial.print(" ");
  }

  Serial.println();
}

void calculateMaxMin() {
  readArray();

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] < minValues[i]) {
      minValues[i] = sensorValues[i];
    }

    if (sensorValues[i] > maxValues[i]) {
      maxValues[i] = sensorValues[i];
    }
  }
}

void calibrateLineDetector() {
  uint32_t startTime = millis();
  Serial.println("Calibration started.");
  // Calibrate for 10 s
  while (millis() - startTime < 10000) {
    calculateMaxMin();
  }
  Serial.println("Calibration done.");

  Serial.print("Maximum values: ");
  printArray(maxValues);
  Serial.print("Minimum values: ");
  printArray(minValues);
}

uint16_t* readArrayCalibrated() {
  readArray();

  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = map(sensorValues[i], minValues[i], maxValues[i], 0, 1023);
    if (sensorValues[i] > 1023) {
      sensorValues[i] = 1023;
    }
  }

  return sensorValues;
}

bool isOutOfBounds() {
  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] >= 512) {
      return false;
    }
  }

  return true;
}

int16_t getLinePosition() {
  readArrayCalibrated();
  printArray(sensorValues);
  float sum = 0;
  float weightedSum = 0;

  for (int i = 0; i < sensorCount; i++) {
    sum += sensorValues[i];
    weightedSum += weights[i] * sensorValues[i];
  }

  float position = weightedSum / sum;

  int16_t scaledPosition = position * 1023.0;

  if (isOutOfBounds()) {
    scaledPosition = lastPosition;
  } else {
    lastPosition = scaledPosition;
  }

  return scaledPosition;
}