#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

#define OTHER_COLOR 0
#define RED 1
#define GREEN 2
#define BLUE 3

extern Adafruit_TCS34725 tcs;

uint8_t expectedRed[3] = {137, 68, 47};
uint8_t expectedGreen[3] = {73, 115, 55};
uint8_t expectedBlue[3] = {62, 82, 103};

#define AMOUNT_OF_COLORS 3
uint8_t* expectedColors[AMOUNT_OF_COLORS] = {expectedRed, expectedGreen, expectedBlue};
uint8_t colorCodes[AMOUNT_OF_COLORS] = {RED, GREEN, BLUE};

#define DIFF_THRESHOLD 10

void getColorCode(uint8_t *colorCode);
void printColorCode(uint8_t colorCode);

#endif