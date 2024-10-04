#include <Wire.h>
#include <Adafruit_TCS34725.h>

#include "ColorDetection.h"

#define D8 34
#define D7 35
#define D6 32
#define D5 33
#define D4 25
#define D3 26
#define D2 27
#define D1 14

#define LED_PIN 23

const int pins[8] = {D1, D2, D3, D4, D5, D6, D7, D8};

const int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

// Create the TCS34725 instance
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);
  //Wire.begin(13, 12); // SDA, SCL
 
  for (int i = 0; i < 8; i++) {
    pinMode(pins[i], INPUT);
  }

  if (tcs.begin()) {
    //Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  int readings[8];
  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    readings[i] = analogRead(pins[i]);
    Serial.print("D");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(readings[i]);
    Serial.print(", ");

    weightedSum += readings[i] * weights[i];
    total += readings[i];
  }

  Serial.println();

  // Evitar división por cero
  if (total == 0) {
    total = 1;
  }

  float position = (float)weightedSum / total;
  int normalizedPosition = (int)(position * 4096 / 4);

  Serial.print("Position: ");
  Serial.println(normalizedPosition);

  uint8_t colorCode;
  tcs.getColorCode(&colorCode);
  printColorCode(colorCode);
  Serial.println();

  delay(500);
}
