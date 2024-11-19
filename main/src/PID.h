#ifndef PID_H
#define PID_H

#include "Arduino.h"

#include "pins.h"
#include "globals.h"

void initMotorPins();
void setMotorPWM(int pwm, unsigned short IN1_PIN, unsigned short IN2_PIN);
void updatePID(int position, unsigned short state);
void updatePIDParams(String &input);
void turnMotorsOff(); 

#endif // PID_H