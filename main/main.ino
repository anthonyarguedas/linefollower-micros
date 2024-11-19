#include "src/pins.h"
#include "src/globals.h"
#include "src/PID.h"
#include "src/LineDetection.h"
#include "src/ReverseBuffer.h"


String command = '.';
unsigned short state = PAUSED;

unsigned long brakeTimer;

unsigned short redCounter = 0;
unsigned short greenCounter = 0;
unsigned short blueCounter = 0;


// Non-blocking delay
void delayNB(unsigned long time) {
    unsigned long lastUpdateTime = millis();
    while (millis() - lastUpdateTime < time) {} // delay(time)
}


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

    Serial.println("Send 'r' to start line follower.");
}

void loop() {
    if (Serial.available()) {
        command = Serial.readStringUntil('\n');

        if (command == "p") {
            Serial.print("Line follower paused.");
            Serial.println(" Send 'r' to resume.");
            state = PAUSED;
        } else if (command == "r") {
            Serial.print("Line follower started.");
            Serial.println(" Send 'p' to pause.");
            Serial.println();
            if (state == PAUSED) {state = FORWARD;}
        } else if (state == PAUSED) {
            // updatePIDParams(command);
        }
    }

    switch(state) {
        case BACKWARD:
            int position = readBuffer();
            Serial.print("state: ");
            Serial.println(state);
            updatePID(position, state);
            break;
        case BRAKE:
            Serial.print("state: ");
            Serial.println(state);
            updatePID(position, state);
            break;
        // El resto de casos se implementan igual
        default:
            int position = getLinePosition();
            delayNB(1);
            Serial.print(position);
            Serial.print(", state: ");
            Serial.println(state);
            updatePID(position, state);
            writeBuffer(position);
    }

    delayNB(50);
} 