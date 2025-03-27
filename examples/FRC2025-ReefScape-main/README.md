# FRC 2025 ReefScape Robot

## 🤖 Overview

This repository contains the robot code for the 2025 FIRST Robotics Competition game "ReefScape". The robot is designed to collect, transport, and place game elements (coral and algae) in an underwater-themed competition environment.

## 🌊 Game Elements

- **Coral**: Game pieces that must be collected and scored at various heights
- **Algae**: Elements that must be removed from reef structures

## 🔧 Robot Capabilities

- **Drivetrain**: Swerve drive system for precise movement and positioning
- **Vision System**: AprilTag detection for autonomous alignment and positioning
- **End Effector**: Specialized mechanism for handling coral and removing algae
- **Lift System**: Multi-position elevator for scoring at different heights
- **Climbing System**: For endgame climbing points

## 🧠 Key Features

- Command-based robot architecture
- Advanced autonomous routines with PathPlanner integration
- Vision-assisted driver controls for precise alignment
- Comprehensive telemetry and logging
- Simulation support for testing without physical hardware

## 📊 Technology Stack

- **Framework**: WPILib 2025 Framework
- **Language**: Java
- **Libraries**:
  - PathPlanner for autonomous path planning
  - AdvantageKit for robust logging and replay
  - Phoenix for motor control
  - REV Robotics Color Sensor for game piece detection

## ⚙️ Development Setup

1. Clone the repository
2. Open in WPILib VS Code
3. Build with Gradle: `./gradlew build`
4. Deploy to robot: `./gradlew deploy`

## 🎮 Controls

### Driver Controller
- Left Stick: Robot translation
- Right Stick: Robot rotation
- Triggers: Auto-align to reef positions
- Y Button: Auto-align to coral station
- B Button: Reset Gyro

### Operator Controller
- A Button: Capture game piece
- B Button: Remove algae
- Y Button: Shoot/deposit game piece
- D-Pad: Lift position control
- Bumpers: Ramp mechanism control
- Triggers: Climb control

## 🔄 Simulation

The code includes a complete simulation environment with:
- Physics-based drivetrain simulation
- Game element interactions
- Vision simulation

Run in simulation mode with: `./gradlew simulateJava`
