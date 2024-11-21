#include "LineDetection.h"
#include "pins.h"
#include "globals.h"

// Dirección de giro ante una bifurcación
unsigned short turnDirection = LEFT;

unsigned short sensorPins2[sensorCount] = { D8, D7, D6, D5, D4, D3, D2, D1 };

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < sensorCount; i++) {
    pinMode(sensorPins2[i], INPUT);
  }
  Serial.println("Calibration started.");
  calibrateLineDetector();
  Serial.println("Calibration done.");
}

void loop() {
  Serial.println(getLinePosition());

  Serial.print("BIfurcación: ");
  Serial.println(isFork());
  Serial.println();

  delay(500);
}
