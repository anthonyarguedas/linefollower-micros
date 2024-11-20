#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "pins.h"

#define sensorCount 8
#define sensorCountBW 8

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

#endif