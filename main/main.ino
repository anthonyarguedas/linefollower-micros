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

unsigned long timer;

int position;

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

    state = FORWARD;
    timer = millis();
}

void loop() {
    if (millis() - timer >= 3000) {
        if (state == PAUSED) {
            state = BACKWARD;
            timer = millis();
            refreshBuffer();
        } else if (state == FORWARD) {
            if (millis() - timer >= 3500) {
                state = BACKWARD;
                timer = millis();
            }
        }
    }

    switch(state) {
        case PAUSED:
            Serial.print("state: ");
            Serial.println(state);
            updatePID(position, state);
            break;
        case BACKWARD:
            position = readBuffer();
            Serial.print(position);
            Serial.print(", state: ");
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
            position = getLinePosition();
            delayNB(1);
            Serial.print(position);
            Serial.print(", state: ");
            Serial.println(state);
            updatePID(position, state);
            writeBuffer(position);
    }

    delayNB(50);
} 