#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "pins.h"
#include "globals.h"

#define sensorCount 8
#define NUM_SAMPLES_PER_SENSOR 4
#define sensorCountBW 8

#define OUT_OF_BOUNDS_THRESHOLD 750

void initLineDetectorPins();
unsigned int* readArrayBW();
void printArray(unsigned int* values);
void printMeasurements()
void calibrateLineDetector();
unsigned int* readArrayCalibrated();
bool isFork();
bool isOutOfBoundsBW();
int getLinePosition();
int getLinePositionBW();

#endif