#include "src/pins.h"
#include "src/globals.h"
#include "src/PID.h"
#include "src/LineDetection.h"
#include "src/ColorDetection.h"
#include <Adafruit_TCS34725.h>


String command = '.';
unsigned short state = PAUSED;

unsigned long brakeTimer;
unsigned long backwardTimer;

bool backwardLock = false;
bool brakeLock = false;

unsigned short redCounter = 0;
unsigned short greenCounter = 0;
unsigned short blueCounter = 0;

// Estados asignados a cada color
// Orden: RED, GREEN, BLUE
unsigned short colorStates[3] = {BACKWARD, FAST, BRAKE};

unsigned short colorCode;
int position;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


// Non-blocking delay
void delayNB(unsigned long time) {
    unsigned long lastUpdateTime = millis();
    while (millis() - lastUpdateTime < time) {} // delay(time)
}

void printState(unsigned short state) {
    switch (state) {
      case PAUSED:
        Serial.print("PAUSED");
        break;
      case FORWARD:
        Serial.print("FORWARD");
        break;
      case BACKWARD:
        Serial.print("BACKWARD");
        break;
      case BRAKE:
        Serial.print("BRAKE");
        break;
      case FAST:
        Serial.print("FAST");
        break;
    }
}


void setup() {
    initLineDetectorPins();
    initMotorPins();
    tcs.begin();

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
    /*** Actualizar estado ***/
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
        case PAUSED:
            break;
        case BACKWARD:
            if (millis() - backwardTimer >= 3000) {state = FORWARD;}
            break;
        case BRAKE:
            if (millis() - brakeTimer >= 10000) {state = FORWARD;}
            break;
        default:
            getColorCode(&colorCode);

            switch(colorCode) {
                case BLACK:
                    state = FORWARD;
                    break;
                case OTHER_COLOR:
                    break;
                default:
                    switch(colorStates[colorCode]) {
                        case FAST:
                            state = FAST;
                            backwardLock = false;
                            brakeLock = false;
                            break;
                        case BRAKE:
                            if (!brakeLock) {
                                state = BRAKE;
                                brakeLock = true;
                                brakeTimer = millis();
                            }
                            backwardLock = false;
                            break;
                        case BACKWARD:
                            if (!backwardLock) {
                                state = BACKWARD;
                                backwardLock = true;
                                backwardTimer = millis();
                            }
                            brakeLock = false;
                            break;
                    }
            }
    }


    
    /*** Controlar motores según la posición y el estado ***/
    switch (state) {
        case PAUSED:
            break;
        default:
            position = getLinePosition();
            delayNB(1);
            Serial.print(position);
            Serial.print(", ");
            printColorCode(colorCode);
            Serial.print(", ");
            printState(state);
            Serial.println();
            updatePID(position, state);
    }


    delayNB(50);
} 