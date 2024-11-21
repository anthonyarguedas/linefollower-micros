#include "PID.h"


float Kp = 0.25;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0;  // Derivative gain

int targetPosition = 0;  // Target position (centered on the line)

float previousError = 0;
int Speed = 0;

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

  integral += error;

  // Derivative term
  float derivative = (error - previousError);
  previousError = error;
  // PID output
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  /*
  int Speed = (state == FAST) ? 235 : 215;
  int motorA_speed = constrain(Speed - correction, 0, Speed);
  int motorB_speed = constrain(Speed + correction, 0, Speed);
  */
  int maximumSpeed = (state == FAST) ? 255 : Speed;
  int motorA_speed = constrain(maximumSpeed - correction, 0, Speed);
  int motorB_speed = constrain(maximumSpeed + correction, 0, Speed);
  
  Serial.print("A Speed: ");
  Serial.print(motorA_speed);
  Serial.print("B Speed: ");
  Serial.println(motorB_speed);

  if (state != BACKWARD) {
    setMotorPWM(motorA_speed, AIN1, AIN2);
    setMotorPWM(motorB_speed, BIN1, BIN2);
  } else {
    setMotorPWM(-motorA_speed, BIN1, BIN2);
    setMotorPWM(-motorB_speed, AIN1, AIN2);
  }
}

void updatePIDParams(float newKp, float newKd, float newSpeed) {
  Kp = newKp;
  Kd = newKd;
  Speed = newSpeed;
}

void directionChange() {
  previousError = 0;
}
