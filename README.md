# Self-Balancing Robot

A two-wheel self-balancing robot built on an ESP32, using an MPU-6050 IMU
for angle sensing, a Kalman filter for angle estimation, and a PID
controller to keep it upright.

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
- [Motor driver model, e.g. L298N / TB6612FNG]
- 2x 3-6V DC TT motor motors + wheels
- [Battery, chassis, wheel specs, etc.]

## How it works

### Angle estimation

Raw gyro rate is integrated over time to estimate angle, but drifts over
long periods. Raw accelerometer readings give an absolute tilt angle
(from the gravity vector) but are noisy from vibration and motor kicks.
A 1D Kalman filter fuses both: predicting the next angle from the gyro,
then correcting that prediction using the accelerometer measurement,
weighted by each sensor's uncertainty.

The MPU-6050's onboard digital low-pass filter (DLPF) is set to
[44Hz / DLPF_CFG=3] to suppress motor vibration noise while keeping
group delay low (~5ms) — an earlier version used a tighter 10Hz filter,
which introduced ~14ms of lag and visibly destabilized the control loop
under fast corrections.

### Control

A standard PID loop computes P/I/D terms from the pitch angle error
against a setpoint (default 0°, upright). The D-term reads the gyro's
angular rate directly rather than differentiating the filtered angle,
avoiding the noise amplification that differentiation introduces.
Integral windup is clamped, and the robot cuts motor output entirely
past ±30° tilt (fallen state).

### Live tuning

`p<value>`, `i<value>`, `d<value>` typed into Serial Monitor update the
PID gains in real time without reflashing, e.g. `p30` sets Kp to 30.

## Repo structure

```
firmware/       Arduino sketch (.ino)
logging/        Python serial logger + offline plotting tool
data/           Sample logged run (CSV)
media/          Demo GIF and analysis charts
```

## Building and flashing the firmware

1. Open `firmware/self_balancing_robot.ino` in the Arduino IDE
2. Select your ESP32 board under **Tools > Board**
3. Select the correct COM port under **Tools > Port**
4. Upload

## Logging and analyzing data

Install dependencies:
```powershell
pip install -r requirements.txt
```

List available serial ports:
```powershell
python logging/robot_logger.py --list
```

Log a run, with a live plot:
```powershell
python logging/robot_logger.py --port COM5 --plot
```

Generate a static, annotated chart from a logged run:
```powershell
python logging/plot_robot_log.py data/sample_run.csv --out media/pid_response_chart.png
```

## Results

![PID response chart](media/pid_response_chart.png)

[A sentence or two describing what the chart shows — e.g. recovery
behavior after a manual push, or before/after tuning comparison.]

## Future improvements

- [e.g. tune I-term more carefully / add complementary filter comparison /
  wireless telemetry / etc.]
