#include "PID.h"


float Kp = 0.25;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 0;  // Derivative gain

int targetPosition = 3500;  // Target position (centered on the line)

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

  // Derivative term
  float derivative = (error - previousError);
  previousError = error;
  // PID output
  float correction = (Kp * error) + (Kd * derivative);

  int maximumSpeed = (state == FAST) ? 255 : Speed;
  int motorA_speed = constrain(maximumSpeed - correction, 0, maximumSpeed);
  int motorB_speed = constrain(maximumSpeed + correction, 0, maximumSpeed);

  if (state != BACKWARD) {
    // Check polarity
    setMotorPWM(motorA_speed, AIN1, AIN2);
    setMotorPWM(motorB_speed, BIN1, BIN2);
  } else {
    setMotorPWM(-motorA_speed, BIN1, BIN2);
    setMotorPWM(-motorB_speed, AIN1, AIN2);
  }
}

void updatePIDParams(float newKp, float newKd, int newSpeed) {
  Kp = newKp;
  Kd = newKd;
  Speed = newSpeed;
  
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(" Kd: ");
  Serial.print(Kd);
  Serial.print(" Speed: ");
  Serial.println(Speed);
}

void directionChange() {
  previousError = 0;
}
