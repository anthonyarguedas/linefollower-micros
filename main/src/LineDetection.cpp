#include "LineDetection.h"

// Array FORWARD
//QTRSensorsAnalog qtra((unsigned char[]) {D6, D6, D5, D4, D3, D2, D1}, 
QTRSensorsAnalog qtra((unsigned char[]) {D8, D7, D6, D5, D4, D3, D2, D1}, 
  sensorCount, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[sensorCount];

// Array BACKWARD
unsigned short sensorPinsBW[sensorCountBW] = { BWR, BWC, BWL };

//float weightsBW[sensorCountBW] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

unsigned int sensorValuesBW[sensorCountBW];

int lastPositionBW = 0;

void initLineDetectorPins() {
    for (int i=0; i<sensorCountBW; i++) {
        pinMode(sensorPinsBW[i], INPUT);
    }

    pinMode(IR, OUTPUT);
    digitalWrite(IR, HIGH);
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

void printMeasurements() {
    printArray(sensorValues, sensorCount);
}

void calibrateLineDetector() {
  elapsedMillis startTime = 0;
  // Calibrate for 10 s
  while (startTime < 10000) {
    qtra.calibrate();
  }
}

unsigned int* readArrayCalibrated() {
  qtra.readCalibrated(sensorValues);

  return sensorValues;
}

bool isOutOfBounds() {
    bool allBlack = true;
    bool allWhite = true;

    for (int i=0; i<sensorCount; i++) {
        if (sensorValues[i] >= OUT_OF_BOUNDS_THRESHOLD) {allWhite = false;}
        else {allBlack = false;}
    }

    return allBlack || allWhite;
}

bool isOutOfBoundsRead() {
    qtra.readCalibrated(sensorValues);

    bool result = isOutOfBounds();
    return result;
}

bool isOutOfBoundsBW() {
  bool allWhite = true;
  
  for (int i = 0; i < sensorCountBW; i++) {
    if (sensorValuesBW[i] >= OUT_OF_BOUNDS_THRESHOLD) {
      allWhite = false;
    }
  }
  
  bool failure = (sensorValuesBW[BWL_VAL] == 1023) && (sensorValuesBW[BWR_VAL] == 1023) && (sensorValuesBW[BWC_VAL] == 0);

  return allWhite || failure;
}

bool isFork() {
  return false;
}

int getLinePosition(bool* isfork, unsigned short turnDirection) {
  *isfork = isFork();
  int position = qtra.readLine(sensorValues, *isfork, turnDirection);
  return position;
}

int getLinePositionBW() {
  unsigned int* ptr = readArrayBW();
  printArray(ptr, sensorCountBW);
  
  int scaledPosition;

  if (!isOutOfBoundsBW()) {
    if (sensorValuesBW[BWL_VAL] == 1023) {
      scaledPosition = 2000;
    } else if (sensorValuesBW[BWR_VAL] == 1023) {
      scaledPosition = 5000;
    } else {
      scaledPosition = 3500;
    }

    lastPositionBW = scaledPosition;
  }
  else {
      scaledPosition = lastPositionBW;
  }

  return scaledPosition;
}
