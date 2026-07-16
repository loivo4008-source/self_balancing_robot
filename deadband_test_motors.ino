#include <Arduino.h>

// Same pins as your main sketch
#define Motor_Left_IN1 17
#define Motor_Left_IN2 18
#define Motor_Left_PWM 16
#define Motor_Right_IN1 19
#define Motor_Right_IN2 23
#define Motor_Right_PWM 25

void setup() {
  pinMode(Motor_Left_IN1, OUTPUT);
  pinMode(Motor_Left_IN2, OUTPUT);
  pinMode(Motor_Left_PWM, OUTPUT);
  pinMode(Motor_Right_IN1, OUTPUT);
  pinMode(Motor_Right_IN2, OUTPUT);
  pinMode(Motor_Right_PWM, OUTPUT);

  Serial.begin(57600);
  delay(500);
  Serial.println("Deadband finder");
  Serial.println("Robot must be on a stand, wheels free to spin.");
  Serial.println("Send 'l' to test LEFT motor, 'r' to test RIGHT motor.");
  Serial.println("PWM will ramp up slowly. Note the value when the wheel FIRST starts turning.");
  Serial.println("Send 's' any time to stop.");
}

void driveMotor(bool isLeft, int pwm) {
  int in1 = isLeft ? Motor_Left_IN1 : Motor_Right_IN1;
  int in2 = isLeft ? Motor_Left_IN2 : Motor_Right_IN2;
  int pwmPin = isLeft ? Motor_Left_PWM : Motor_Right_PWM;

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  digitalWrite(Motor_Left_IN1, LOW);
  digitalWrite(Motor_Left_IN2, LOW);
  digitalWrite(Motor_Right_IN1, LOW);
  digitalWrite(Motor_Right_IN2, LOW);
  analogWrite(Motor_Left_PWM, 0);
  analogWrite(Motor_Right_PWM, 0);
}

void rampTest(bool isLeft) {
  Serial.println(isLeft ? "\nTesting LEFT motor..." : "\nTesting RIGHT motor...");
  for (int pwm = 0; pwm <= 255; pwm += 2) {
    driveMotor(isLeft, pwm);
    Serial.print("PWM: ");
    Serial.println(pwm);

    // Check for stop command mid-ramp
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == 's') {
        stopMotors();
        Serial.println("Stopped. Note the last PWM value printed before it started spinning.");
        return;
      }
    }

    delay(300);  // slow ramp — give yourself time to see/hear it start
  }
  stopMotors();
  Serial.println("Reached 255, stopping. If it never moved, check wiring.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'l') {
      rampTest(true);
    } else if (c == 'r') {
      rampTest(false);
    } else if (c == 's') {
      stopMotors();
      Serial.println("Stopped.");
    }
  }
}