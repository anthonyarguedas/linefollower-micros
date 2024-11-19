#include <Arduino.h>

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H

#define D8 23
#define D7 22
#define D6 21
#define D5 20
#define D4 17
#define D3 16
#define D2 15
#define D1 14

void initLineDetectorPins();
uint16_t* readArray();
void printArray(uint16_t* values);
void calculateMaxMin();
void calibrateLineDetector();
uint16_t* readArrayCalibrated();
bool isOutOfBounds();
uint16_t getLinePosition();

#endif