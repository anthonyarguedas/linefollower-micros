#ifndef PID_H
#define PID_H

#include "Arduino.h"

#include "pins.h"


void initMotorPins();
void setMotorPWM(uint16_t pwm, uint8_t IN1_PIN, uint8_t IN2_PIN);
void updatePID(int16_t position);
void updatePIDParams(String &input);
void turnMotorsOff(); 

#endif // PID_H