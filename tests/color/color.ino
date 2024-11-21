#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include "ColorDetection.h"

// Create the TCS34725 instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void loop() {
  uint8_t colorCode;
  getColorCode(&colorCode);
  printColorCode(colorCode);
  Serial.println();
  Serial.println();

  delay(500);
}
