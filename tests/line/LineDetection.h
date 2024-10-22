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

const uint8_t sensorCount = 8;
uint8_t sensorPins[sensorCount] = { D1, D2, D3, D4, D5, D6, D7, D8 };

void initLineDetectorPins();
uint16_t* readArray();
void printArray(uint16_t* values);
void calculateMaxMin();
void calibrateLineDetector();
uint16_t* readArrayCalibrated();
bool isOutOfBounds();
uint16_t getLinePosition();

#endif