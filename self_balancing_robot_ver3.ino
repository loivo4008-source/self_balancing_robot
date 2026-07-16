#include <Wire.h>

// ── Global variables ──────────────────────────────────────────────
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;

// ── Kalman filter variables ───────────────────────────────────────
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = { 0, 0 };

//PID Variables
float kp = 24;  //24
float ki = 12;  //12
float kd = 1.2;

float previousError = 0;
float error = 0;
float setpoint = 0;  //-2

float integral = 0;
float integralLimit = 200;
float derivative = 0;

float previousAngle = 0;
float alpha = 0.1;

//tuning:
void handleSerialTuning() {
  //See if there's anything first,
  //Serial.available() returns the number of bytes currently sitting in the serial receive buffer
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    //trim() is a method on Arduino's String class that removes whitespace from the start and end of a string
    //— spaces, tabs, \r, \n — leaving the middle untouched.
    input.trim();

    if (input.length() < 2) return;  // ignore garbage

    char command = input.charAt(0);
    float value = input.substring(1).toFloat();

    switch (command) {
      case 'p':
        kp = value;
        Serial.print("Kp set to: ");
        Serial.println(kp);
        break;
      case 'i':
        ki = value;
        Serial.print("Ki set to: ");
        Serial.println(ki);
        break;
      case 'd':
        kd = value;
        Serial.print("Kd set to: ");
        Serial.println(kd);
        break;
      default:
        Serial.println("Unknown command. Use p, i, or d followed by a number.");
    }
  }
}

//Motor variables:
// Nano
// #define Motor_Left_IN1 4
// #define Motor_Left_IN2 5
// #define Motor_Left_PWM 3
// #define Motor_Right_IN1 6
// #define Motor_Right_IN2 7
// #define Motor_Right_PWM 9

// ESP32
#define Motor_Left_IN1 17
#define Motor_Left_IN2 18
#define Motor_Left_PWM 16
#define Motor_Right_IN1 19
#define Motor_Right_IN2 23
#define Motor_Right_PWM 25

//Moving function + Stopping function
void setMotors(float output) {
  output = constrain(output, -255, 255);  // clamp range
  int speed = abs((int)output);           // 0-255 for PWM

  const int deadband = 80;
  if (speed > 0) {
    // tune this — min PWM that actually spins the wheels
    speed = map(speed, 0, 255, deadband, 255);
  }

  if (output > 0) {
    digitalWrite(Motor_Left_IN1, HIGH);
    digitalWrite(Motor_Left_IN2, LOW);
    digitalWrite(Motor_Right_IN1, HIGH);
    digitalWrite(Motor_Right_IN2, LOW);
  } else {
    digitalWrite(Motor_Left_IN1, LOW);
    digitalWrite(Motor_Left_IN2, HIGH);
    digitalWrite(Motor_Right_IN1, LOW);
    digitalWrite(Motor_Right_IN2, HIGH);
  }

  analogWrite(Motor_Left_PWM, speed);
  analogWrite(Motor_Right_PWM, speed);
}

void stop() {
  digitalWrite(Motor_Left_IN1, LOW);
  digitalWrite(Motor_Left_IN2, LOW);
  digitalWrite(Motor_Right_IN1, LOW);
  digitalWrite(Motor_Right_IN2, LOW);
  analogWrite(Motor_Left_PWM, 0);
  analogWrite(Motor_Right_PWM, 0);
}

// ── Kalman filter function ────────────────────────────────────────
void kalman_1d(float KalmanState, float KalmanUncertainty,
               float KalmanInput, float KalmanMeasurement, float dt) {

  //  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll, 0.004);

  // Predict step — integrate gyro rate to get new angle estimate
  //KalmanState is the angle (gyro), KalmanMeasurement is acce angle
  KalmanState = KalmanState + dt * KalmanInput;

  // Predict step — update uncertainty
  KalmanUncertainty = KalmanUncertainty + dt * dt * 4 * 4;

  // Update step — compute Kalman gain
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3);

  // Update step — correct state with accelerometer measurement
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);

  // Update step — update uncertainty
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  // Write outputs
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// ── Read all sensor data from MPU-6050 ───────────────────────────
void gyro_signals(void) {

  // 1. Configure low pass filter (10Hz bandwidth)
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // 2. Configure accelerometer range (±8g, 4096 LSB/g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // 3. Read accelerometer data (6 bytes from 0x3B)
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // 4. Configure gyroscope range (±500°/s, 65.5 LSB/°/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // 5. Read gyroscope data (6 bytes from 0x43)
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // 6. Convert raw gyro to degrees/second
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // 7. Convert raw accel to g with calibration offsets
  AccX = (float)AccXLSB / 4096 - 0.02;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096 - 0.02;

  // 8. Compute roll and pitch angles from accelerometer (degrees)
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
  //Motor setup:
  pinMode(Motor_Left_IN1, OUTPUT);
  pinMode(Motor_Left_IN2, OUTPUT);
  pinMode(Motor_Right_IN1, OUTPUT);
  pinMode(Motor_Right_IN2, OUTPUT);
  pinMode(Motor_Left_PWM, OUTPUT);
  pinMode(Motor_Right_PWM, OUTPUT);

  Serial.begin(57600);

  // Visual indicator that board is running
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // Start I2C at 400kHz fast mode
  Wire.setClock(400000);
  Wire.begin();
  delay(250);  // Wait for MPU-6050 to power up

  // Wake up MPU-6050 (starts in sleep mode by default)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyroscope calibration — sample 2000 times while perfectly still
  // Keep the robot/board completely stationary during this ~2 second window
  for (RateCalibrationNumber = 0;
       RateCalibrationNumber < 2000;
       RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  // Average the 2000 samples to get the bias offset
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;

  // Start the loop timer
  LoopTimer = micros();
}

// ── Main loop ─────────────────────────────────────────────────────
void loop() {
  float delta_T = (micros() - LoopTimer) / 1000000.0f;
  // float delta_T = 0.004;

  // Read sensor
  gyro_signals();

  // Subtract calibration bias from gyro readings
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Run Kalman filter for roll
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll, 0.004);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  // Run Kalman filter for pitch
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch, 0.004);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Handle tuning:
  handleSerialTuning();

  //PID Control:
  error = setpoint - KalmanAnglePitch;
  //P term
  float P_term = kp * error;
  P_term = constrain(P_term, -255, 255);
  //I term
  integral += error * delta_T;
  integral = constrain(integral, -integralLimit, integralLimit);
  float I_term = ki * integral;
  //D term
  // float rawDerivative = -(KalmanAnglePitch - previousAngle) / delta_T;
  // previousAngle = KalmanAnglePitch;
  // derivative = alpha * rawDerivative + (1 - alpha) * derivative;
  // float D_term = kd * derivative;
  float D_term = kd * (-RatePitch);  // negative sign depends on your gyro orientation/convention

  float result = P_term + I_term + D_term;

  if (abs(KalmanAnglePitch) > 30) {
    stop();
    integral = 0;  // ← prevents windup while lying on the floor
    previousAngle = KalmanAnglePitch;
  } else {
    setMotors(result);
    // stop();
    // integral = 0;
  }

  // // Print filtered angles
  // // Throttled print — only every 25th loop (~every 100ms at 4ms/loop)
  //   static uint32_t printCounter = 0;
  //   if (++printCounter >= 25) {
  //     printCounter = 0;
  //     Serial.print("Roll Angle [°] ");
  //     Serial.print(KalmanAngleRoll);
  //     Serial.print("  Pitch Angle [°] ");
  //     Serial.print(KalmanAnglePitch);
  //     Serial.print(" P: ");
  //     Serial.print(P_term);
  //     Serial.print(" I: ");
  //     Serial.print(I_term);
  //     Serial.print(" D: ");
  //     Serial.println(D_term);
  //   }

  // Print filtered angles as CSV for the Python logger
  // Throttled print — only every 25th loop (~every 100ms at 4ms/loop)
  static uint32_t printCounter = 0;
  if (++printCounter >= 25) {
    printCounter = 0;
    Serial.print(millis());
    Serial.print(",");
    Serial.print(KalmanAngleRoll);
    Serial.print(",");
    Serial.print(KalmanAnglePitch);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(P_term);
    Serial.print(",");
    Serial.print(I_term);
    Serial.print(",");
    Serial.print(D_term);
    Serial.print(",");
    Serial.println(result);  // result is your motor output
  }

  // Hold loop to exactly 4ms (250Hz)
  while (micros() - LoopTimer < 4000)
    ;
  LoopTimer = micros();
}
