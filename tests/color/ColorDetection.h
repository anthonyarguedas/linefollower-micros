#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>

#define OTHER_COLOR 0
#define RED 1
#define GREEN 2
#define BLUE 3

extern Adafruit_TCS34725 tcs;

#define AMOUNT_OF_COLORS 3
#define DIFF_THRESHOLD 20

void getColorCode(uint8_t *colorCode);
void printColorCode(uint8_t colorCode);

#endif
