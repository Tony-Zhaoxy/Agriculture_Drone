# Agricultural UAV System – SCI Lab, UCLA

## Introduction

This repository hosts the system integration codebase for an **agricultural unmanned aerial vehicle (UAV)** platform developed at the **SCI Lab, University of California, Los Angeles (UCLA)**.

The platform is designed to support onboard perception, state estimation, and autonomous decision-making using a Jetson-based companion computer, enabling low-latency and network-independent operation in field environments.

---

## System Overview

The system integrates the following components:

- **Flight Control**  
  A mature autopilot stack (e.g., PX4 or ArduPilot) is used for low-level stabilization, flight modes, and fail-safe handling.

- **Onboard Compute**  
  An **NVIDIA Jetson** companion computer executes perception, state estimation, task logic, and data logging fully onboard.

- **Perception & Sensors**  
  The UAV integrates onboard cameras (RGB and/or depth), IMU, GNSS/RTK, and optional altitude sensors (LiDAR or radar) to provide robust environmental observation and flight-state feedback.

- **State Estimation**  
  Existing open-source visual-inertial odometry (VIO) and/or GNSS–INS fusion pipelines are reused to generate unified pose and velocity estimates.

- **Middleware**  
  **ROS 2** is used as the middleware for data handling, visualization, and modular control logic, with standardized topics, timestamps, and coordinate frames (TF).

- **Task Policy & Control Interface**  
  Task-level logic or learning-based policies run onboard and output high-level commands (e.g., velocity and yaw-rate setpoints).  
  These commands are passed through a safety wrapper before being sent to the autopilot.
