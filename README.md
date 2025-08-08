# SelfBalancingPIDBot

## Overview
This Arduino project implements a **closed-loop PID controller** to balance a ball on a beam using:
- An **ultrasonic sensor** to provide feedback on the ball’s position
- A **servo motor** to tilt the beam and keep the ball at a desired position (setpoint)
- **PID control** with **anti-windup** and **median filtering** for stable, accurate control

The goal is to automatically balance the ball at a specific distance from the ultrasonic sensor by adjusting the beam angle in real time.

![IMG_4332](https://github.com/user-attachments/assets/6dacecbe-0613-41aa-a814-da82ac1d02f5)

---

## Hardware Requirements
- Arduino Uno
- HC-SR04 ultrasonic sensor
- Standard servo motor (SG90, MG996R, etc.)
- Ball-on-beam mechanical setup
- Jumper wires and breadboard
- USB power supply (servo needs separate battery supply, maybe 6V depending on model)

---

## How It Works

### 1. Position Measurement
- The **HC-SR04** ultrasonic sensor measures the distance from the sensor to the ball.
- **Median filtering** is applied to a sample of 13 readings to remove noise and outliers.
- Extreme jumps (difference > 30 cm) or impossible readings (> 35 cm) are rejected.

### 2. PID Control
The servo motor angle is computed as:
output = servoCenter + Kp * error + Ki * integral (sum) + Kd * derivative
Where:
- **error** = `setpoint - measured_position`
- **Kp, Ki, Kd** are tunable values
- **servoCenter** is the neutral position that keeps the beam level
- Output is constrained between **servoMin** and **servoMax** to set limits for the beam's angle

### 3. Anti-Windup
- Before updating the integral term, the code tests if the PID output would saturate the servo.
- If saturated, the integral is **not updated**, preventing **integral windup**.

### 4. Stability Mode
- If the ball is **close to the setpoint** and **moving slowly** (low derivative), the servo is automatically centered to hold position without unnecessary movement.

---

## 🔧 Tuning Parameters
- `setpoint` → Desired ball position in cm
- `Kp` → Proportional gain (main responsiveness)
- `Ki` → Integral gain (steady-state accuracy, keep small to avoid windup)
- `Kd` → Derivative gain (damping, reduces overshoot)
- `servoMin` / `servoMax` → Mechanical tilt limits
- `servoCenter` → Beam level servo value
