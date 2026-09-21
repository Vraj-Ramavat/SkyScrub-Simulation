# SkyScrub Simulation

Simulation repository for the SkyScrub autonomous glass-facade cleaning hexacopter.

## V0.1 Status

The initial six-motor SkyScrub flight model has been validated using:

- Ubuntu 22.04
- PX4 v1.17
- Gazebo Harmonic 8
- ROS 2 Humble

## Validated Features

- Custom SkyScrub PX4 airframe
- Six-motor control allocation
- Six-rotor Gazebo model
- IMU
- Accelerometer
- Gyroscope
- Magnetometer
- Barometer
- GPS / EKF2
- PX4 preflight check
- Arming
- Takeoff
- Hold mode
- Landing
- Automatic disarm after landing

## Current Architecture

SkyScrub is being developed as an E610-class hexacopter with:

- 6 × Hobbywing X6 180KV motors
- 12S 16000 mAh battery
- 8 L onboard water tank
- Low-flow mist cleaning system
- Raspberry Pi 5 + camera
- Radar-based facade standoff
- Blower drying system

## Development Roadmap

### V0.1
Basic six-motor PX4 + Gazebo flight validation.

### V0.2
Realistic mass and payload model:

- Component masses
- 8 L water payload
- Center of gravity
- Moment of inertia
- Dynamic water depletion
- Loaded hover validation

### V0.3
Facade environment and radar-based standoff control.

### V0.4
Window detection and segment-based autonomous path planning.

### V0.5
Wash → rinse → dry cleaning simulation.

## Project

**SkyScrub — Autonomous Drone-Based Glass Facade Cleaning System**
