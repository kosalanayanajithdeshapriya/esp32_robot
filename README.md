# Line Following Robot with Robotic Arm & Color Detection
<img width="960" height="1151" alt="IMG_4957" src="https://github.com/user-attachments/assets/3d69683f-9f26-4125-8d72-0dc2a8e4d689" />

An ESP32-based autonomous robot that follows a track using 
5-sensor PID control, detects colored obstacles via a TCS34725 
color sensor, and operates a 2-servo robotic arm to pick up 
GREEN objects and avoid RED obstacles — all managed through a 
Finite State Machine (FSM).

## Features
- PID line following with 5 IR sensors (Kp=30, Ki=0, Kd=18)
- Sharp 90° turn detection using outer sensors (S6, S7)
- Real-time color classification using rN-gN separator threshold
- Ultrasonic obstacle detection (HC-SR04)
- Robotic arm with pick, carry, and deposit sequence
- Web-based color calibration tool over ESP32 WiFi AP
- End-of-path detection with 1500ms sustained confirmation

## Hardware
- ESP32 DevKit
- TCS34725 RGB Color Sensor
- HC-SR04 Ultrasonic Sensor
- 7× IR Line Sensors
- 2× Servo Motors (Arm + Gripper)
- L298N Motor Driver + 2× DC Motors

## Libraries
- ESP32Servo
- Adafruit_TCS34725
- Wire
