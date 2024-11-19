#include "src/pins.h"
#include "src/globals.h"
#include "src/PID.h"
#include "src/LineDetection.h"
#include "src/ColorDetection.h"
#include <Adafruit_TCS34725.h>
#include "src/ReverseBuffer.h"

// TODO: Remove
//#define USE_COLOR

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

// TODO: Remove
#ifndef USE_COLOR
unsigned long timer;
#endif


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

    Serial.println("Calibration started.");
    calibrateLineDetector();
    Serial.println("Calibration done.");
    Serial.println();
  
    // TODO: Remove
    #ifndef USE_COLOR
    state = FORWARD;
    timer = millis();
    #endif
}

void loop() {
    // TODO: Remove line below only
    #ifdef USE_COLOR
    switch(state) {
        case PAUSED:
            break;
        case BACKWARD:
            getColorCode(&colorCode);

            if (colorCode != lastColorCode) {
                backwardLock++;
            }
            lastColorCode = colorCode;
            
            if (millis() - backwardTimer >= 3000) {
                state = FORWARD;
                refreshBuffer();
            }
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
    // TODO: Remove
    #else
    if (millis() - timer >= 3000) {
        if (state == BACKWARD) {
            state = FORWARD;
            refreshBuffer();
            timer = millis();
        } else if (state == FORWARD) {
            if (millis() - timer >= 3500) {
                state = BACKWARD;
                timer = millis();
            }
        }
    }
    #endif

    /*** Controlar motores según la posición y el estado ***/
    switch (state) {
        case PAUSED:
            break;
        case BRAKE:
            break;
        case BACKWARD:
            position = readBuffer();
            delayNB(1);
            Serial.print(position);
            Serial.print(", ");
            break;
        case FORWARD:
            printColorCode(colorCode);
            Serial.print(", ");
            // Sin break para poder entrar al default
        default:
            position = getLinePosition();
            delayNB(1);
            Serial.print(position);
            Serial.print(", ");
            writeBuffer(position);
    }

    printState(state);
    Serial.println();
    updatePID(position, state);

    delayNB(50);
} 