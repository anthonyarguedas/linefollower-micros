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

void updatePID(int position, unsigned short state) {
  int Speed = 215;
  // Apply correction to motor speeds
  int motorA_speed;
  int motorB_speed;

  switch (state) {
    case PAUSED:
      setMotorPWM(0, AIN1, AIN2);
      setMotorPWM(0, BIN1, BIN2);
      return;
    case BRAKE:
      setMotorPWM(0, AIN1, AIN2);
      setMotorPWM(0, BIN1, BIN2);
      return;
    case FAST:
      Speed = 235;
      break;
    default:
      Speed = 215;
  }

  // Proportional term
  float error = targetPosition - position;

  // Integral term
  integral += error;

  // Derivative term
  // float derivative = (error - previousError) / deltaTime;
  float derivative = (error - previousError);
  previousError = error;
  // PID output
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);

  motorA_speed = constrain(Speed - correction, 0, Speed);  // Motor A speed
  motorB_speed = constrain(Speed + correction, 0, Speed);  // Motor B speed

  setMotorPWM(motorA_speed, AIN1, AIN2);  // Set motor A speed
  setMotorPWM(motorB_speed, BIN1, BIN2);  // Set motor B speed
}
        
  
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


void updatePIDParams(float newKp, float newKd, float newKi) {
  Kp = newKp;
  Kd = newKd;
  Ki = newKi;
}