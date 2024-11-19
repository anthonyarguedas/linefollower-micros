#ifndef COLOR_DETECTION_H
#define COLOR_DETECTION_H

#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include "globals.h"


extern Adafruit_TCS34725 tcs;

#define AMOUNT_OF_COLORS 3
#define DIFF_THRESHOLD 10

void getColorCode(unsigned short *colorCode);
void printColorCode(unsigned short colorCode);

#endif