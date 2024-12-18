#include "ColorDetection.h"


unsigned short expectedRed[3] = {100, 80, 50};
unsigned short expectedGreen[3] = {73, 115, 55};
unsigned short expectedBlue[3] = {70, 85, 80};

unsigned short* expectedColors[AMOUNT_OF_COLORS] = {expectedRed, expectedGreen, expectedBlue};
unsigned short colorCodes[AMOUNT_OF_COLORS] = {RED, GREEN, BLUE};


void getColorCode(unsigned short *colorCode) {
  float red, green, blue;

  elapsedMillis t0 = 0;
  while(t0 < 60);  // takes 60ms to read

  tcs.getRGB(&red, &green, &blue);
  
  Serial.print("R: ");
  Serial.print(red);
  Serial.print(" G: ");
  Serial.print(green);
  Serial.print(" B:  ");
  Serial.println(blue);

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
    }
}
