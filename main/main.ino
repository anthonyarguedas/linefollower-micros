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

// 0: Se puede activar el estado
unsigned short backwardLock = 0;
unsigned short brakeLock = 0;

// Cantidad de veces que se ha encontrado cada color
// Orden: RED, GREEN, BLUE
unsigned short colorCounters[3] = {0, 0, 0};

// Estados asignados a cada color
// Orden: RED, GREEN, BLUE
unsigned short colorStates[3] = {BACKWARD, FAST, BRAKE};

// Se asume que el carro inicia sobre tape negro
unsigned short lastColorCode = OTHER_COLOR;
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
            getColorCode(&colorCode);

            if (colorCode != lastColorCode) {
                backwardLock++;
            }
            lastColorCode = colorCode;
            
            if (millis() - backwardTimer >= 3000) {state = FORWARD;}
            break;
        case BRAKE:
            if (millis() - brakeTimer >= 10000) {state = FORWARD;}
            break;
        default:
            getColorCode(&colorCode);

            if (colorCode != lastColorCode) {
                if (backwardLock > 0) {backwardLock--;}
            }
            lastColorCode = colorCode;

            switch(colorCode) {
                case OTHER_COLOR:
                    if (!isOutOfBoundsRead()) {
                        brakeLock = 0;
                        state = FORWARD;
                    }
                    break;
                default:
                    switch(colorStates[colorCode]) {
                        case FAST:
                            if (backwardLock == 0) {
                                state = FAST;
                                brakeLock = 0;
                            }
                            break;
                        case BRAKE:
                            if (brakeLock == 0 && backwardLock == 0) {
                                state = BRAKE;
                                brakeLock = 1;
                                brakeTimer = millis();
                            }
                            break;
                        case BACKWARD:
                            if (backwardLock == 0) {
                                state = BACKWARD;
                                backwardLock = 1;
                                backwardTimer = millis();
                                brakeLock = 0;
                            }
                            break;
                    }

                    colorCounters[colorCode]++;
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
    }

    printState(state);
    Serial.println();
    updatePID(position, state);

    delayNB(50);
} 