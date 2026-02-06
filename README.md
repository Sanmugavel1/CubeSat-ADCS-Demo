# CubeSat ADCS Demo 🚀

This repository presents a **low-cost CubeSat Attitude Determination and Control System (ADCS)** prototype developed using ESP32, IMU sensors, actuator-based stabilization, and a real-time web-based ground station.

---

## 🔍 Project Overview

Attitude Determination and Control System (ADCS) is a critical subsystem of a CubeSat responsible for:
- Determining satellite orientation
- Stabilizing orientation against disturbances
- Providing real-time telemetry to a ground station

This project demonstrates these concepts using affordable hardware and open-source tools.

---

## 🧠 System Architecture

- **Onboard Computer:** ESP32
- **IMU Sensor:** MPU6050 / MPU6500
- **Actuator:** Servo / BLDC-based reaction wheel (prototype)
- **Communication:** Wi-Fi (Ground station)
- **Ground Station:** Web-based 3D visualization
- **Simulation:** MATLAB-based ADCS model

---

## 📡 Features

- Real-time attitude estimation (roll, pitch, yaw)
- Sensor fusion using filtering techniques
- Closed-loop stabilization logic
- FREE and STABILIZE modes
- 3D CubeSat orientation visualization
- Hardware + simulation validation

---

## 🖥️ Ground Station Preview

![Dashboard](media/images/dashboard_ui.png)

---

## 🎥 Demo Video

📺 See the system in action:  
`media/videos/adcs_demo.mp4`

---

## 📁 Repository Structure

- `firmware/` → ESP32 source code  
- `ground_station/` → Web dashboard  
- `simulation/` → MATLAB ADCS simulation  
- `media/` → Images & videos  
- `docs/` → Reports & documentation  

---

## 🚀 Motivation

This project was developed as part of a **CubeSat Space Technology challenge**, focusing on realistic ADCS concepts using minimal cost hardware.

---

## 📌 Future Work

- Reaction wheel using BLDC motor
- SDR-based telemetry
- Multi-axis stabilization
- Extended Kalman Filter (EKF)

---
