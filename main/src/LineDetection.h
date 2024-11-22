#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "QTRSensors.h"
#include "pins.h"
#include "globals.h"

#define sensorCount 8
#define NUM_SAMPLES_PER_SENSOR 4
#define sensorCountBW 3

void initLineDetectorPins();
unsigned int* readArrayBW();
void printArray(unsigned int* values);
void printMeasurements();
void calibrateLineDetector();
unsigned int* readArrayCalibrated();
bool isFork();
bool isOutOfBounds();
bool isOutOfBoundsRead();
bool isOutOfBoundsBW();
int getLinePosition(bool* isfork, unsigned short turnDirection);
int getLinePositionBW();

#endif
