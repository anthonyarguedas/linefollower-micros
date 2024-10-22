#include "LineDetection.h"


void setup() {
  Serial.begin(115200);

  initLineDetectorPins();
  calibrateLineDetector();
}

void loop() {
  Serial.print("Calibrated: ");
  printArray(readArrayCalibrated());

  Serial.print("Position: ");
  Serial.println(getLinePosition());
  Serial.println();

  delay(500);
}
