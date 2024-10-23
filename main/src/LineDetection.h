#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#include "Arduino.h"

#include "pins.h"


void initLineDetectorPins();
uint16_t* readArray();
void printArray(uint16_t* values);
void calculateMaxMin();
void calibrateLineDetector();
uint16_t* readArrayCalibrated();
bool isOutOfBounds();
uint16_t getLinePosition();

#endif