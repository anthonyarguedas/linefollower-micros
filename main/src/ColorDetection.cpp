#include "ColorDetection.h"


void getColorCode(unsigned short *colorCode) {
  float red, green, blue;

  unsigned long t0 = millis();
  while(millis() - t0 < 60);  // takes 50ms to read

  tcs.getRGB(&red, &green, &blue);

  bool colorFound = false;

  for (int i=0; i<AMOUNT_OF_COLORS; i++) {
    if (abs(red - expectedColors[i][0]) <= DIFF_THRESHOLD &&
        abs(green - expectedColors[i][1]) <= DIFF_THRESHOLD &&
        abs(blue - expectedColors[i][2]) <= DIFF_THRESHOLD) {
        *colorCode = colorCodes[i];
        colorFound = true;
        break;
    } 
  }

  if (!colorFound) {
    *colorCode = OTHER_COLOR;
  }
}

void printColorCode(unsigned short colorCode) {
    switch (colorCode) {
      case OTHER_COLOR:
        Serial.print("OTHER_COLOR");
        break;
      case RED:
        Serial.print("RED");
        break;
      case GREEN:
        Serial.print("GREEN");
        break;
      case BLUE:
        Serial.print("BLUE");
        break;
      case BLACK:
        Serial.print("BLACK");
        break;
    }
}