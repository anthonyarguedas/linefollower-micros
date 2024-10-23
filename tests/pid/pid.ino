#include "PID.h"


void setup() {
    Serial.begin(115200);

    initMotorPins();
}

void loop() {
    updatePID(0);

    delay(10);
}