#include <Adafruit_TCS34725.h>
#include <Arduino.h>

#include "src/pins.h"
#include "src/globals.h"
#include "src/PID.h"
#include "src/LineDetection.h"
#include "src/ColorDetection.h"

unsigned short state = PAUSED;
unsigned short calibrationState = UNCALIBRATED;

elapsedMillis brakeTimer;
elapsedMillis backwardTimer;

// 0: Se puede activar el estado
unsigned short backwardLock = 0;
unsigned short brakeLock = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Cantidad de veces que se ha encontrado cada color
// Orden: RED, GREEN, BLUE
unsigned short colorCounters[3] = {0, 0, 0};

// Estados asignados a cada color
// Orden: RED, GREEN, BLUE
unsigned short colorStates[3] = {BACKWARD, FAST, BRAKE};

// Se asume que el carro inicia sobre tape negro
unsigned short lastColorCode = OTHER_COLOR;
unsigned short colorCode;
bool avoidColor = false;

int position;
bool outOfBounds = false;
bool isfork = false;

// Dirección de giro ante una bifurcación
unsigned short turnDirection = LEFT;

elapsedMicros mainLoopTimer;


volatile bool rxAvailable = false;
bool txAvailable = true;
byte txBuffer[10];


// Non-blocking delay
void delayNB(unsigned long time) {
    elapsedMicros lastUpdateTime = 0;
    while (lastUpdateTime < time) {} // delay(time)
}

void UARTRXISR() {
    rxAvailable = true;
}

bool UARTRead() {
  if (Serial2.available() >= 7) {
    // Leer los 7 bytes recibidos
    unsigned short byte1 = Serial2.read();
    unsigned int kp_raw = (Serial2.read() << 8);
    kp_raw |= Serial2.read();
    int Speed = (Serial2.read() << 8);
    Speed |= Serial2.read();
    unsigned int kd_raw = (Serial2.read() << 8);
    kd_raw |= Serial2.read();

    // Convertir a valores decimales (dividir por 1000)
    float kp = kp_raw / 1000.0;
    float kd = kd_raw / 1000.0;

    updatePIDParams(kp, kd, Speed);

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
    unsigned short activate = (byte1 & 0b10000000) >> 7;  // Bit 1

    if (activate == 1) {
        state = FORWARD;
        // TODO: Remove
        calibrationState = CALIBRATING;
        UARTWrite();
        calibrateLineDetector();
        calibrationState = CALIBRATED;
        UARTWrite();
        brakeLock = 1;
        backwardLock = 1;
    } else {
        state = PAUSED;
        calibrationState = UNCALIBRATED;
    }

    return false;
  } else {
    return true;
  }
}

void UARTWrite() {
    for (int i=0; i<3; i++) {
        txBuffer[i] = colorCounters[i];
    }
    // TODO: Remove
    txBuffer[3] = state;
    txBuffer[4] = calibrationState;
    txBuffer[5] = colorCode;

    txBuffer[6] = (position >> 8) & 0xFF;
    txBuffer[7] = position & 0xFF;

    txBuffer[8] = outOfBounds;
    txBuffer[9] = isfork;

    Serial2.write(txBuffer, 10);
}


void setup() {
    initLineDetectorPins();
    initMotorPins();
    tcs.begin();

    Serial.begin(115200);
    Serial2.begin(115200);

    pinMode(SIGNAL, INPUT);
    attachInterrupt(digitalPinToInterrupt(SIGNAL), UARTRXISR, RISING);
}

void loop() {
    if (mainLoopTimer >= 5000) {
        if (rxAvailable == true) {
        rxAvailable = UARTRead();
        }

        switch(state) {
            case PAUSED:
                break;
            case BACKWARD:
                getColorCode(&colorCode);

                if (colorCode != lastColorCode) {
                    backwardLock++;

                    if ((colorCode != OTHER_COLOR) && (lastColorCode == OTHER_COLOR)) {avoidColor = true;}
                    else if ((colorCode == OTHER_COLOR) && (lastColorCode != OTHER_COLOR)) {avoidColor = false;}
                }
                lastColorCode = colorCode;

                if (backwardTimer >= 3000) {
                    state = FORWARD;
                    directionChange();
                }
                break;
            case BRAKE:
                if (brakeTimer >= 10000) {state = FORWARD;}
                break;
            default:
                getColorCode(&colorCode);
                if (colorCode != lastColorCode) {
                    if ((colorCode != OTHER_COLOR) && (lastColorCode == OTHER_COLOR)) {avoidColor = true;}
                    else if ((colorCode == OTHER_COLOR) && (lastColorCode != OTHER_COLOR)) {avoidColor = false;}

                    if (backwardLock > 0) {backwardLock--;}
                    if (colorCode != OTHER_COLOR) {
                    colorCounters[colorCode]++;
                    txAvailable = true;
                    }
                }
                lastColorCode = colorCode;

                switch(colorCode) {
                    case OTHER_COLOR:
                        outOfBounds = isOutOfBoundsRead();
                        if (!outOfBounds) {
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
                                    brakeTimer = 0;
                                }
                                break;
                            case BACKWARD:
                                if (backwardLock == 0) {
                                    state = BACKWARD;
                                    backwardLock = 1;
                                    brakeLock = 0;
                                    directionChange();
                                    backwardTimer = 0;
                                }
                                break;
                        }
                }
        }

        /*** Controlar motores según la posición y el estado ***/
        switch (state) {
            case PAUSED:
                turnMotorsOff();
                break;
            case BRAKE:
                turnMotorsOff();
                break;
            case BACKWARD:
                position = getLinePositionBW(avoidColor);
                delayNB(1000);
                updatePID(position, state);
                break;
            default:
                position = getLinePosition(&isfork, turnDirection);
                updatePID(position, state);
                delayNB(1000);
        }

        if (txAvailable == true) {
            UARTWrite();
            txAvailable = false;
        } else { // TODO: Remove
            UARTWrite();
        }

        mainLoopTimer = 0;
    }
} 
