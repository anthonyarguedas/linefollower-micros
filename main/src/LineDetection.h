#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "pins.h"

void initLineDetectorPins();
unsigned int* readArray();
void printArray(unsigned int* values);
void calculateMaxMin();
void calibrateLineDetector();
unsigned int* readArrayCalibrated();
bool isOutOfBounds();
bool isOutOfBoundsRead();
int getLinePosition();

#endif