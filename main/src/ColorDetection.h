#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

#define OTHER_COLOR 0
#define RED 1
#define GREEN 2
#define BLUE 3

extern Adafruit_TCS34725 tcs;

unsigned short expectedRed[3] = {137, 68, 47};
unsigned short expectedGreen[3] = {73, 115, 55};
unsigned short expectedBlue[3] = {62, 82, 103};

#define AMOUNT_OF_COLORS 3
unsigned short* expectedColors[AMOUNT_OF_COLORS] = {expectedRed, expectedGreen, expectedBlue};
unsigned short colorCodes[AMOUNT_OF_COLORS] = {RED, GREEN, BLUE};

#define DIFF_THRESHOLD 10

void getColorCode(unsigned short *colorCode);
void printColorCode(unsigned short colorCode);

#endif