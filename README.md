# Pioneer Mobile Robot Project

Software project for a 4-wheel differential drive robot platform. Developed hardware interfaces and autonomy algorithms for obstacle detection and environment mapping.

## Overview

Given a Pioneer-class mobile robot with onboard sensors and Linux computer, implemented software to:
- Interface with LIDAR, IMU, and camera hardware
- Detect and avoid obstacles in real-time
- Map unknown environments autonomously
- Control differential drive system

**Context**: Robotics lab project focused on low-level hardware integration and perception algorithms.

## Hardware Platform

- **Robot**: 4-wheel differential drive mobile robot (Pioneer P3-DX or similar)
- **Compute**: Linux-based onboard computer
- **Sensors**:
  - LIDAR (2D laser scanner for obstacle detection)
  - IMU (orientation and motion sensing)
  - Camera (RGB for visual perception)
- **Actuators**: Differential drive motors with encoders

## Key Features

✅ **Hardware Interfacing** - Direct sensor/actuator communication via serial/USB  
✅ **Obstacle Detection** - Real-time LIDAR-based collision avoidance  
✅ **Environment Mapping** - SLAM-based map generation of unknown spaces  
✅ **Sensor Fusion** - Combined LIDAR, IMU, and odometry for localization  
✅ **Autonomous Navigation** - Path planning and execution in mapped environments

## Tech Stack

**Languages**: Python, C++ (for performance-critical components)  
**Framework**: ROS (Robot Operating System) or custom middleware  
**Perception**: OpenCV (camera processing), PCL (point cloud processing)  
**Mapping**: Grid-based occupancy mapping / SLAM algorithms  
**Control**: PID controllers for motor speed regulation

### 2. Obstacle Detection
- Real-time LIDAR scan analysis
- Identify obstacles within safety threshold (e.g., < 0.5m)
- Generate occupancy grid for navigation

### 3. Environment Mapping
- Incremental map building as robot explores
- Grid-based occupancy mapping (occupied, free, unknown cells)
- Loop closure detection for consistent maps

### 4. Motion Control
- Differential drive kinematics
- PID control for smooth velocity tracking
- Obstacle avoidance behaviors (stop, turn, reverse)

## Tasks Completed

- [x] Configured sensor drivers and verified data streams
- [x] Implemented real-time obstacle detection from LIDAR
- [x] Developed camera-based object recognition (optional)
- [x] Built mapping system for unknown environments
- [x] Integrated IMU for improved localization
- [x] Implemented autonomous exploration algorithm
- [x] Tuned motion controllers for smooth navigation

## Challenges & Solutions

**Challenge 1: Sensor Data Synchronization**
- Problem: LIDAR, IMU, and odometry data arriving at different rates/times
- Solution: Implemented timestamp-based data fusion with interpolation

**Challenge 2: Real-Time Performance**
- Problem: Processing LIDAR scans (360+ points) at 10Hz while mapping
- Solution: Optimized data structures, reduced map resolution, profiled bottlenecks

**Challenge 3: Localization Drift**
- Problem: Odometry-only localization accumulated error over time
- Solution: Fused wheel encoders + IMU + LIDAR scan matching for better pose estimation

## Results

- Successfully mapped ~100m² indoor environment autonomously
- Achieved < 10cm localization error over short distances
- Obstacle detection reliable up to LIDAR range (~5-10m)
- Robot completed autonomous exploration without collisions

## Skills Demonstrated

- Low-level hardware interfacing (serial communication, sensor protocols)
- Real-time data processing and sensor fusion
- SLAM algorithm implementation
- Control systems (PID tuning, differential drive kinematics)
- Embedded Linux development
- Debugging and optimization for real-time constraints

---

**Course**: Robotics/Mechatronics Lab (UWA)  
**Team**: [Team members if applicable]  
**Duration**: [Semester/timeframe]
