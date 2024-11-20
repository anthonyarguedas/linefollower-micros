#include "LineDetection.h"


const unsigned short sensorCount = 8;
unsigned short sensorPins[sensorCount] = { D1, D2, D3, D4, D5, D6, D7, D8 };

float weights[sensorCount] = { -1.0, -0.75, -0.50, -0.25, 0.25, 0.50, 0.75, 1.0 };

unsigned int sensorValues[sensorCount];
unsigned int minValues[sensorCount] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };
unsigned int maxValues[sensorCount] = { 0, 0, 0, 0, 0, 0, 0, 0 };

unsigned int lastPosition = 0;


void initLineDetectorPins() {
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

unsigned int* readArray() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  return sensorValues;
}

void printArray(unsigned int* values) {
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
  unsigned long startTime = millis();
  // Calibrate for 10 s
  while (millis() - startTime < 10000) {
    calculateMaxMin();
  }
}

unsigned int* readArrayCalibrated() {
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

bool isOutOfBoundsRead() {
  readArrayCalibrated();

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] >= 512) {
      return false;
    }
  }

  return true;
}

int getLinePosition() {
  readArrayCalibrated();
  printArray(sensorValues);
  float sum = 0;
  float weightedSum = 0;

  for (int i = 0; i < sensorCount; i++) {
    sum += sensorValues[i];
    weightedSum += weights[i] * sensorValues[i];
  }

  float position = weightedSum / sum;

  int scaledPosition = position * 1023.0;

  if (isOutOfBounds()) {
    scaledPosition = lastPosition;
  } else {
    lastPosition = scaledPosition;
  }

  return scaledPosition;
}