#include "LineDetection.h"

// Array FORWARD
QTRSensorsAnalog qtra((unsigned char[]) {D7, D6, D5, D4, D3, D2, D1}, 
  sensorCount, NUM_SAMPLES_PER_SENSOR, QTR_NO_EMITTER_PIN);
unsigned int sensorValues[sensorCount];

// Array BACKWARD
unsigned short sensorPinsBW[sensorCountBW] = { BWL, BWR, BWC };

//float weightsBW[sensorCountBW] = { -1.25, -1.0, -0.75, -0.5, 0.5, 0.75, 1.0, 1.25 };

unsigned int sensorValuesBW[sensorCountBW];

int lastPositionBW = 0;

void initLineDetectorPins() {
    for (int i=0; i<sensorCountBW; i++) {
        pinMode(sensorPinsBW[i], INPUT);
    }
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
  unsigned long startTime = millis();
  // Calibrate for 10 s
  while (millis() - startTime < 10000) {
    qtra.calibrate();
  }

  for (int i = 0; i < sensorCount; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
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
  for (int i = 0; i < sensorCountBW; i++) {
    if (sensorValuesBW[i] >= OUT_OF_BOUNDS_THRESHOLD) {
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

  return ((blackCounter >= 4) && (blackCounter < 7)) ? true : false;
}

int getLinePosition() {
  int position = qtra.readLine(sensorValues);

  Serial.print("Fork: ");
  Serial.println(isFork());

  return position;
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
