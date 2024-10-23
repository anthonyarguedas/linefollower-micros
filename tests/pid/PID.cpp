#include "PID.h"


float previousError = 0;
float integral = 0;
uint32_t lastTime = 0;

// Function to set motor speeds
void setMotorPWM(uint16_t pwm, uint8_t IN1_PIN, uint8_t IN2_PIN) {
  if (pwm < 0) {  // Reverse speeds
    analogWrite(IN1_PIN, -pwm);
    digitalWrite(IN2_PIN, LOW);
  } else {  // Stop or forward
    digitalWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void initMotorPins() {
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);
  pinMode(MOT_B1_PIN, OUTPUT);
  pinMode(MOT_B2_PIN, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);
  digitalWrite(MOT_B1_PIN, LOW);
  digitalWrite(MOT_B2_PIN, LOW);
}

void updatePID(uint16_t position) {
  // Calculate PID control output
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
  
  // Proportional term
  float error = targetPosition - position;
  
  // Integral term
  integral += error * deltaTime;
  
  // Derivative term
  float derivative = (error - previousError) / deltaTime;
  
  // PID output
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Apply correction to motor speeds
  uint16_t motorA_speed = constrain(1023 - correction, -1023, 1023);  // Motor A speed
  uint16_t motorB_speed = constrain(1023 + correction, -1023, 1023);  // Motor B speed
  
  // Set motor speeds
  setMotorPWM(motorA_speed, MOT_A1_PIN, MOT_A2_PIN);  // Set motor A speed
  setMotorPWM(motorB_speed, MOT_B1_PIN, MOT_B2_PIN);  // Set motor B speed
  
  // Update variables for next loop
  previousError = error;
  lastTime = currentTime;
}