# Self-Balancing Robot

A two-wheel self-balancing robot built on an ESP32, using an MPU-6050 IMU
for angle sensing, a Kalman filter for angle estimation, and PID
controller algorithm to keep it upright.

## Overview

- **Microcontroller:** ESP32
- **IMU:** MPU-6050 (accelerometer + gyroscope)
- **Angle estimation:** Kalman filter combining gyro rate integration with
  accelerometer tilt correction
- **Control:** PID loop running at 250Hz (4ms fixed loop timing)
- **Tuning tools:** live serial tuning (`p`, `i`, `d` commands) plus a
  Python data logger/plotter for offline analysis

## Hardware

- ESP32 dev board
- MPU-6050 IMU
- Motor driver L298N
- 2x 3-6V DC TT motor motors + wheels
- 2x 18650 Lithium-ion batteries 

## How it works

### Angle estimation

Angle can be calculated using pure gyroscope by integrating the rate angle is changing. 
However, the gyroscope always has offsets so this method will be off overtime, accumulating 
a lot of errors when running for a while. Angle can also be calculated using accelerometer, 
but it is extremely prone to noise (error is huge even when just shaking, not good for robot). 
Therefore, gyroscope is inaccurate but stable, accelerometer is accurate but unstable 
=> Combining the two, we have a good angle measurement using Kalman filter. 

Kalman filter will take angle measurement from the gyroscope, and based on the uncertainty of the gyro, 
it will partially take in the measurement of the accelerometer. 
If the gyro is far off, the angle will mostly from the accelerometer. 
If the gyro is working well and the uncertainty is low, the angle will mostly be from the gyro. 

### Control

A PID control algorithm is applied to the robot. P means proportional, 
which will evaluate the error and respond proportionally to it (if error is high, P will be big). 
I is integral, which fixes the error over time (if the error is constant, I will be a positive number 
added to the output to lessen the error).D is derivative (rate of change of error), 
which predicts the error and fix it (if the robot is falling fast, the rate of change will be positive, 
P will add/ if the robot is recovering, the rate of change is negative, P will be subtracted from the output).

- P will help the robot catch itself
- I will fix error accumulated over time
- D will prevent overshooting from P when robot recovers

Tune P, then D, then I. 

### Live tuning

`p<value>`, `i<value>`, `d<value>` typed into Serial Monitor update the
PID gains in real time without reflashing, e.g. `p30` sets Kp to 30.

## Repo structure

```
firmware/       Arduino sketch (.ino)
logging/        Python serial logger + offline plotting tool
data/           Sample logged run (CSV)
```
 
