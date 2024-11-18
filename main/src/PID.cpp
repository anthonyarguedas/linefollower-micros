#include "PID.h"


float Kp = 5.5;  // Proportional gain
float Ki = 0;  // Integral gain
float Kd = 10;  // Derivative gain

int16_t targetPosition = 0;  // Target position (centered on the line)

float previousError = 0;
float integral = 0;
uint32_t lastTime = 0;

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

  Serial.print("Initial Kp = ");
  Serial.println(Kp);
  Serial.print("Initial Kd = ");
  Serial.println(Kd);
  Serial.print("Initial Ki = ");
  Serial.println(Ki);
  Serial.println();
}

// Function to set motor speeds
void setMotorPWM(uint16_t pwm, uint8_t IN1_PIN, uint8_t IN2_PIN) {
  if (pwm < 0) {  // Reverse speeds
    analogWrite(IN2_PIN, -pwm);
    digitalWrite(IN1_PIN, LOW);
  } else {  // Stop or forward
    analogWrite(IN1_PIN, pwm);
    digitalWrite(IN2_PIN, LOW);
  }
}

void turnMotorsOff() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void updatePID(int16_t position) {
  // Calculate PID control output
  uint32_t currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;  // Time difference in seconds
  
  // Proportional term
  float error = targetPosition - position;
  
  // Serial.print("error=");
  // Serial.println(error);

  // Integral term
  integral += error;
  
  // Derivative term
  // float derivative = (error - previousError) / deltaTime;
  float derivative = (error - previousError);
  // PID output
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Apply correction to motor speeds
  uint16_t motorA_speed = constrain(200 - correction, 0, 200);  // Motor A speed
  uint16_t motorB_speed = constrain(200 + correction, 0, 200);  // Motor B speed
  
  // Set motor speeds
  setMotorPWM(motorA_speed, AIN1, AIN2);  // Set motor A speed
  setMotorPWM(motorB_speed, BIN1, BIN2);  // Set motor B speed
  // Serial.print("VelA=");
  // Serial.print(motorA_speed);
  // Serial.println();
  // Serial.print("VelB=");
  // Serial.print(motorB_speed);
  // Serial.println();
  // Serial.print("Ki");
  // Serial.print(Ki);
  // Serial.println();
  // Serial.print("Kp");
  // Serial.print(Kp);
  // Serial.println();
  // Serial.print("Kd");
  // Serial.print(Kd);
  // Serial.println();
  // Serial.print("Correction");
  // Serial.print(correction);
  // Serial.println();
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