# Design of a Detailed Dynamic Positioning System - Control System Design

This repository contains the project **"Design of a Detailed Dynamic Positioning System"**, which involves designing and implementing a dynamic positioning (DP) system for a marine supply vessel using MATLAB and Simulink with the MSS Toolbox. The project encompasses the development of mathematical models, control system design, observer implementation, and thrust allocation strategies to ensure precise vessel positioning under various environmental conditions.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
   - [Reference Frames](#reference-frames)
   - [Process Plant Model](#process-plant-model)
   - [Control Plant Model](#control-plant-model)
   - [Environmental Loads](#environmental-loads)
   - [Reference Model](#reference-model)
   - [Observer Design](#observer-design)
   - [Controller Design](#controller-design)
   - [Thrust Allocation](#thrust-allocation)
3. [Simulations](#simulations)
4. [Results](#results)
5. [Conclusion](#conclusion)
6. [Usage](#usage)
7. [Dependencies](#dependencies)
8. [License](#license)

---

## Introduction

Dynamic positioning systems are crucial for maintaining the position and heading of marine vessels without the need for anchors, using thrusters and propellers. This project aims to design a detailed DP system that can handle environmental disturbances such as wind, waves, and currents.

---

## Project Overview

### Reference Frames

- **Earth-Fixed Frame (NED)**: North-East-Down frame used for positioning.
- **Body-Fixed Frame**: Attached to the vessel, used for control inputs.

### Process Plant Model

Developed a comprehensive model describing the actual vessel dynamics, including mass, damping, and environmental forces.

### Control Plant Model

Simplified model used for controller design, focusing on essential behaviors and reduced-order representations. The model is divided into:

- **Low-Frequency (LF) Model**: Captures slow vessel motions due to environmental forces.
- **Wave-Frequency (WF) Model**: Represents the vessel's response to wave-induced motions.
- **Bias Model**: Accounts for slowly varying forces and model uncertainties.
- **Measurement Model**: Incorporates sensor measurements and noise.

### Environmental Loads

Implemented models for:

- **Water Currents**: Surface current modeling using a 2D approach.
- **Wind**: Modeled mean wind and gusts using the Harris spectrum.
- **Waves**: Used ITTC spectrum to simulate wave effects.

### Reference Model

Created a reference trajectory generator to ensure smooth transitions between setpoints, including acceleration, constant velocity, deceleration, and constant position phases.

### Observer Design

Implemented two observers:

- **Extended Kalman Filter (EKF)**
- **Nonlinear Passive Observer (NPO)**

After comparison, the NPO was selected for its better performance and easier tuning.

### Controller Design

Designed and compared:

- **Proportional-Integral-Derivative (PID) Controller**
- **Linear Quadratic Gaussian (LQG) Controller**

The LQG controller was chosen due to its optimal performance and ease of tuning.

### Thrust Allocation

Developed a thrust allocation algorithm to distribute control efforts among the vessel's thrusters, considering limitations and optimizing performance. The system handles:

- **Overactuation**: More thrusters than control degrees of freedom.
- **Thruster Constraints**: Physical limitations like maximum thrust and rotation speeds.
- **Optimization**: Minimizes power consumption and wear.

---

## Simulations

Conducted several simulations to validate the system:

1. **Environmental Loads**: Vessel behavior under environmental forces without control inputs.
2. **DP and Thrust Allocation**: Four-corner test to assess controller and thrust allocation performance.
3. **DP and Environmental Forces**: Tested the DP system under environmental disturbances.
4. **Observer Selection**: Compared EKF and NPO observers.
5. **Complete Simulation**: Validated the full DP system with selected observer and controller.
6. **Capability Plot**: Assessed the system's capability under varying environmental conditions.
7. **Observer Robustness**: Tested observer performance under extreme conditions.

---

## Results

- The **LQG controller** and **NPO observer** provided optimal performance in maintaining vessel position and heading.
- The **thrust allocation algorithm** effectively managed thruster outputs, even under thruster failure scenarios.
- The system demonstrated robustness under various environmental conditions, including extreme weather.

---

## Conclusion

The project successfully developed a detailed DP system for a marine vessel, incorporating advanced control strategies, observer designs, and thrust allocation methods. The system was validated through extensive simulations, demonstrating its effectiveness and robustness.
