#include "PID.h"


float Kp = 0.6;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.1;  // Derivative gain

uint16_t targetPosition = 0;  // Target position (centered on the line)

float previousError = 0;
float integral = 0;
uint32_t lastTime = 0;

void initMotorPins() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);

  analogWriteResolution(10);
}

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
  setMotorPWM(motorA_speed, AIN1, AIN2);  // Set motor A speed
  setMotorPWM(motorB_speed, BIN1, BIN2);  // Set motor B speed
  
  // Update variables for next loop
  previousError = error;
  lastTime = currentTime;
}


void updatePIDParams(String &input) {
    if (input.indexOf("Kp=") >= 0) {
        int index = input.indexOf("Kp=") + 3;
        String newKp = input.substring(index);
        Kp = newKp.toFloat();
        Serial.print("Kp set to ");
        Serial.println(Kp);
    } else if (input.indexOf("Kd=") >= 0) {
        int index = input.indexOf("Kd=") + 3;
        String newKd = input.substring(index);
        Kd = newKd.toFloat();
        Serial.print("Kd set to ");
        Serial.println(Kd);
    } else if (input.indexOf("Ki=") >= 0) {
        int index = input.indexOf("Ki=") + 3;
        String newKi = input.substring(index);
        Ki = newKi.toFloat();
        Serial.print("Ki set to ");
        Serial.println(Ki);
    } else {
        Serial.println("Invalid input.");
    }
}