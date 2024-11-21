#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "pins.h"
#include "globals.h"

#define sensorCount 8
#define sensorCountBW 3

#define OUT_OF_BOUNDS_THRESHOLD 750

void initLineDetectorPins();
unsigned int* readArray();
unsigned int* readArrayBW();
void printArray(unsigned int* values);
void calculateMaxMin();
void calibrateLineDetector();
unsigned int* readArrayCalibrated();
bool isOutOfBounds();
bool isOutOfBoundsBW();
bool isOutOfBoundsRead();
int getLinePosition();
int getLinePositionBW();
bool isFork();

#endif