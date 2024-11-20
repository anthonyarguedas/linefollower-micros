#include "LineDetection.h"


// Array FORWARD
unsigned short sensorPins[sensorCount] = { D1, D2, D3, D4, D5, D6, D7, D8 };

float weights[sensorCount] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

unsigned int sensorValues[sensorCount];
unsigned int minValues[sensorCount] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };
unsigned int maxValues[sensorCount] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int lastPosition = 0;

// Array BACKWARD
unsigned short sensorPinsBW[sensorCountBW] = { BW1, BW2, BW3, BW4, BW5, BW6, BW7, BW8 };

float weightsBW[sensorCountBW] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

unsigned int sensorValuesBW[sensorCountBW];

int lastPositionBW = 0;


void initLineDetectorPins() {
  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  for (int i = 0; i < sensorCountBW; i++) {
    pinMode(sensorPinsBW[i], INPUT);
  }
}

unsigned int* readArray() {
  for (int i = 0; i < sensorCount; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
  }

  return sensorValues;
}

unsigned int* readArrayBW() {
  for (int i = 0; i < sensorCountBW; i++) {
    sensorValuesBW[i] = (unsigned int)(digitalRead(sensorPinsBW[i])) * 1023;
  }

  return sensorValuesBW;
}

void printArray(unsigned int* values, unsigned short length) {
  for (int i = 0; i < length; i++) {
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

bool isOutOfBoundsBW() {
  for (int i = 0; i < sensorCountBW; i++) {
    if (sensorValuesBW[i] >= 512) {
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

int getLinePositionBW() {
  readArrayBW();

  float sum = 0;
  float weightedSum = 0;

  for (int i = 0; i < sensorCountBW; i++) {
    sum += sensorValuesBW[i];
    weightedSum += weightsBW[i] * sensorValuesBW[i];
  }

  float position = weightedSum / sum;

  int scaledPosition = position * 1023.0;

  if (isOutOfBoundsBW()) {
    scaledPosition = lastPositionBW;
  } else {
    lastPositionBW = scaledPosition;
  }

  return scaledPosition;
}