#include <Adafruit_TCS34725.h>
#include <Arduino.h>

#include "src/pins.h"
#include "src/globals.h"
#include "src/PID.h"
#include "src/LineDetection.h"
#include "src/ColorDetection.h"
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

// Dirección de giro ante una bifurcación
unsigned short turnDirection = LEFT;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

volatile bool rxAvailable = false;
bool txAvailable = true;

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

void UARTRXISR() {
    rxAvailable = true;
}

void UARTRead() {
    // Leer los 7 bytes recibidos
    unsigned short byte1 = Serial2.read();
    unsigned int kp_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 2 y 3
    unsigned int ki_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 4 y 5
    unsigned int kd_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 6 y 7

    // Convertir a valores decimales (dividir por 100)
    float kp = kp_raw / 100.0;
    float ki = ki_raw / 100.0;
    float kd = kd_raw / 100.0;

    updatePIDParams(kp, kd, ki);

    // Extraer las secciones del byte1
    unsigned short redState = (byte1 & 0b00110000) >> 4;     // Bits 3 y 4
    unsigned short greenState = (byte1 & 0b00001100) >> 2;    // Bits 5 y 6
    unsigned short blueState = (byte1 & 0b00000011);          // Bits 7 y 8
    
    unsigned short newColorStates[3] = {redState, greenState, blueState};

    for (int i=0; i<3; i++) {
        switch (newColorStates[i]) {
            case 0: colorStates[i] = BRAKE; break;
            case 1: colorStates[i] = FAST; break;
            case 2: colorStates[i] = BACKWARD; break;
        }
    }

    turnDirection = (byte1 & 0b01000000) >> 6;       // Bit 2
    bool activate = (byte1 & 0b10000000) >> 7;  // Bit 1

    if (activate) {
        state = FORWARD;
        // TODO: Remove
        #ifndef USE_COLOR
        timer = millis();
        #endif
    } else {
        state = PAUSED;
        brakeLock = 1;
        backwardLock = 1;
    }
}


void setup() {
    initLineDetectorPins();
    initMotorPins();
    tcs.begin();

    Serial2.begin(115200);
    attachInterrupt(digitalPinToInterrupt(0), UARTRXISR, FALLING);

    /*
    Serial2.println("Calibration started.");
    calibrateLineDetector();
    Serial2.println("Calibration done.");
    Serial2.println();
    */
  
    // TODO: Remove
    #ifndef USE_COLOR
    timer = millis();
    #endif
}

void loop() {
    if (rxAvailable == true) {
        UARTRead();
        rxAvailable = false;
    }

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
                    txAvailable = true;
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

    if (txAvailable) {
        for (int i=0; i<3; i++) {
            Serial.write(colorCounters[i]);
        }
        txAvailable = false;
    }

    delayNB(10);
} 