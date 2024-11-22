#ifndef PID_H
#define PID_H

#include "Arduino.h"

#include "pins.h"
#include "globals.h"

#define SPEED_UPPER_LIMIT 255
#define SPEED_LOWER_LIMIT 0

void initMotorPins();
void setMotorPWM(int pwm, unsigned short IN1_PIN, unsigned short IN2_PIN);
void updatePID(int position, unsigned short state);
void updatePIDParams(float newKp, float newKd, int newSpeed);
void turnMotorsOff();
void directionChange();

#endif // PID_H
