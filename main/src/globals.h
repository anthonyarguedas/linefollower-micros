#ifndef GLOBALS_H
#define GLOBALS_H

// Estados
#define PAUSED 0
#define FORWARD 1
#define FAST 2
#define BACKWARD 3 
#define BRAKE 4

// Estados de calibración
#define UNCALIBRATED 0
#define CALIBRATING 1
#define CALIBRATED 2

// Colores
#define RED 0
#define GREEN 1
#define BLUE 2
#define OTHER_COLOR 3

// Direcciones de giro
#define LEFT 0
#define RIGHT 1

// Indices para leer array BW
#define BWL_VAL 0
#define BWC_VAL 1
#define BWR_VAL 2

#define OUT_OF_BOUNDS_THRESHOLD 750

#endif // GLOBALS_H
