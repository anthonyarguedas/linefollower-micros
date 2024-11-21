#include "LineDetection.h"


// Array FORWARD
// De izquierda a derecha
unsigned short sensorPins[sensorCount] = { D8, D7, D6, D5, D4, D3, D2, D1 };

float weights[sensorCount] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

unsigned int sensorValues[sensorCount];
unsigned int minValues[sensorCount] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };
unsigned int maxValues[sensorCount] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int lastPosition = 0;

// Array BACKWARD
unsigned short sensorPinsBW[sensorCountBW] = { BWL, BWR, BWC };

//float weightsBW[sensorCountBW] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

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
    sensorValuesBW[i] = digitalRead(sensorPinsBW[i]) * 1023;
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
    if (sensorValues[i] >= OUT_OF_BOUNDS_THRESHOLD) {
      return false;
    }
  }

  return true;
}

bool isOutOfBoundsBW() {
  for (int i = 0; i < sensorCountBW; i++) {
    if (sensorValuesBW[i] >= OUT_OF_BOUNDS_THRESHOLD) {
      return false;
    }
  }

  return true;
}

bool isOutOfBoundsRead() {
  readArrayCalibrated();

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] >= OUT_OF_BOUNDS_THRESHOLD) {
      return false;
    }
  }

  return true;
}

bool isFork() {
  unsigned short blackCounter = 0;

  for (int i = 0; i < sensorCount; i++) {
    if (sensorValues[i] >= OUT_OF_BOUNDS_THRESHOLD) {
      blackCounter++;
    }
  }

  return (blackCounter >= 4) ? true : false;
}

int getLinePosition() {
  unsigned int* ptr = readArrayCalibrated();
  printArray(ptr, sensorCount);

  float sum = 0;
  float weightedSum = 0;

  for (int i = 0; i < sensorCount; i++) {
    sum += sensorValues[i];
    weightedSum += weights[i] * sensorValues[i];
  }

  float position = weightedSum / sum;

  int scaledPosition = position * 1023.0;

  if (isOutOfBounds()) {
    scaledPosition = -lastPosition;
  } else {
    lastPosition = scaledPosition;
  }

  return scaledPosition;
}

int getLinePositionBW() {
  readArrayBW();
  
  int scaledPosition;

  if (sensorValuesBW[LEFT] == 1023) {
      scaledPosition = -512;
      lastPositionBW = scaledPosition;
  } else if (sensorValuesBW[RIGHT] == 1023) {
      scaledPosition = 512;
      lastPositionBW = scaledPosition;
  } else if (!isOutOfBoundsBW()) {
      scaledPosition = 0;
      lastPositionBW = scaledPosition;
  } else {
      scaledPosition = lastPositionBW;
  }

  return scaledPosition;
}
