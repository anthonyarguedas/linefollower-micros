#include "src/pins.h"
#include "src/PID.h"
#include "src/LineDetection.h"


String command = '.';
bool paused = false;

uint32_t lastUpdateTime = 0;



void setup() {
    initLineDetectorPins();
    initMotorPins();

    Serial.begin(115200);


    // while (command != "c") {
    //     Serial.println("Send 'c' to begin calibration.");
        
    //     while(!Serial.available()) {} // Wait for input

    //     command = Serial.readStringUntil('\n');
    // }

    Serial.println("Calibration started.");
    calibrateLineDetector();
    Serial.println("Calibration done.");
    Serial.println();


//     while (command != "s") {
//         Serial.println("Send 's' to start line follower.");
        
//         while(!Serial.available()) {} // Wait for input

//         command = Serial.readStringUntil('\n');
//     }

//     Serial.print("Line follower started.");
//     Serial.println("Send 'p' to pause.");
//     Serial.println();
}

void loop() {
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');

        if (command == "p") {
            Serial.println("Send 'r' to resume.");
            paused = true;
        } else if (command == "r") {
            Serial.println("Send 'p' to pause.");
            Serial.println();
            paused = false;
        } else {
            // updatePIDParams(command);
        }
    }

    if (!paused && (millis() - lastUpdateTime >= 50)) { // ms
        int16_t position = getLinePosition();
        delay(1);
        Serial.println(position);
        updatePID(position);
    }
} 