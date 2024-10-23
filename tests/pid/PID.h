#ifndef PID_H
#define PID_H

#include "Arduino.h"


// Define control inputs
#define MOT_A1_PIN 10
#define MOT_A2_PIN 9
#define MOT_B1_PIN 6
#define MOT_B2_PIN 5

float Kp = 0.6;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.1;  // Derivative gain
uint16_t targetPosition = 0;  // Target position (centered on the line)

void setMotorPWM(uint16_t pwm, uint8_t IN1_PIN, uint8_t IN2_PIN);
void initMotorPins();
void updatePID(uint16_t position);

#endif // PID_H