#include "PID.h"


float Kp = 10;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0;  // Derivative gain

int targetPosition = 0;  // Target position (centered on the line)

float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;

void initMotorPins() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(SLP, OUTPUT);

  // Turn off motors - Initial state
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
  analogWriteFrequency(AIN1, 25000);
  analogWriteFrequency(AIN2, 25000);
  analogWriteFrequency(BIN1, 25000);
  analogWriteFrequency(BIN2, 25000);
  
  // Enable the driver
  digitalWrite(SLP, HIGH);

  // analogWriteResolution(10);
}

// Function to set motor speeds
void setMotorPWM(int pwm, unsigned short IN1_PIN, unsigned short IN2_PIN) {
  if (pwm < 0) {  // Reverse speeds
    analogWrite(IN2_PIN, -pwm);
    analogWrite(IN1_PIN, 0);
  } else {  // Stop or forward
    analogWrite(IN1_PIN, pwm);
    analogWrite(IN2_PIN, 0);
  }
}

void turnMotorsOff() {
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}

void updatePID(int position, unsigned short state) {
  // Proportional term
  float error = targetPosition - position;

  unsigned long timeDelta = millis() - lastTime;

  integral += error*timeDelta;

  // Derivative term
  float derivative = (error - previousError) / timeDelta;
  previousError = error;
  // PID output
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  /*
  int Speed = (state == FAST) ? 235 : 215;
  int motorA_speed = constrain(Speed - correction, 0, Speed);
  int motorB_speed = constrain(Speed + correction, 0, Speed);
  */
  int Speed = (state == FAST) ? 400 : 300;
  int motorA_speed = map(Speed - correction, 0, 1023, 0, 255);
  int motorB_speed = map(Speed + correction, 0, 1023, 0, 255);

  if (state != BACKWARD) {
    setMotorPWM(motorA_speed, AIN1, AIN2);
    setMotorPWM(motorB_speed, BIN1, BIN2);
  } else {
    setMotorPWM(motorA_speed, BIN1, BIN2);
    setMotorPWM(motorB_speed, AIN1, AIN2);
  }

  lastTime = millis();
}

void updatePIDParams(float newKp, float newKd, float newKi) {
  Kp = newKp;
  Kd = newKd;
  Ki = newKi;
}

void directionChange() {
  lastTime = millis();
  integral = 0;
  previousError = 0;
}