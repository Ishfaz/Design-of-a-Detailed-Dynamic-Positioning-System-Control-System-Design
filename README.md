# Design of a Detailed Dynamic Positioning System - Control System Design

This repository contains the project "Design of a Detailed Dynamic Positioning System," which focuses on designing and implementing a dynamic positioning (DP) system for a marine supply vessel using MATLAB and Simulink with the MSS Toolbox. The project explores mathematical modeling, control design, observer selection, and thrust allocation to ensure robust vessel positioning under varying environmental conditions.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
   - [Reference Frames](#reference-frames)
   - [Process Plant Model](#process-plant-model)
   - [Control Plant Model](#control-plant-model)
   - [Environmental Loads](#environmental-loads)
   - [Reference Model](#reference-model)
3. [Observer Design](#observer-design)
4. [Controller Design](#controller-design)
5. [Thrust Allocation](#thrust-allocation)
6. [Simulations](#simulations)
7. [Results](#results)
8. [Conclusion](#conclusion)
9. [Usage](#usage)
10. [Dependencies](#dependencies)
11. [License](#license)

---

## Introduction

Dynamic positioning systems are essential for maintaining a vessel's position and heading without requiring anchors. This project aims to design a robust DP system capable of compensating for environmental disturbances such as wind, waves, and currents.

---

## Project Overview

### Reference Frames

1. **Earth-Fixed Frame (NED)**:
   - Used for positioning relative to a global reference.
   - Defined as North-East-Down coordinates.

2. **Body-Fixed Frame**:
   - Attached to the vessel.
   - Defined as Surge-Sway-Yaw for horizontal plane dynamics.

**Transformation Matrix**:
\[
R(\psi) = 
\begin{bmatrix} 
\cos\psi & -\sin\psi & 0 \\ 
\sin\psi & \cos\psi & 0 \\ 
0 & 0 & 1 
\end{bmatrix}
\]

This matrix transforms between the NED and body-fixed frames.

---

### Process Plant Model

The process plant model represents the full vessel dynamics using the 6 DOF motion equation:

\[
M\dot{\nu} + CRB(\nu)\nu + CA(\nu_r)\nu_r + D(\nu_r) + G(\eta) = \tau_{env} + \tau_{thr}
\]

Where:
- \( M \): Inertia matrix (includes added mass).
- \( CRB \), \( CA \): Coriolis and centripetal matrices.
- \( D \): Damping forces.
- \( G \): Restoring forces.
- \( \tau_{env}, \tau_{thr} \): Environmental and thruster forces.

---

### Control Plant Model

The control plant model simplifies the process plant for controller design, focusing on low-frequency (LF) and wave-frequency (WF) dynamics:

\[
\dot{\nu} = M^{-1}(-D\nu - G(\eta) + \tau_{env} + \tau_{thr})
\]

Key components:
- **Low-Frequency Model**: Describes slow vessel motions due to environmental forces.
- **Wave-Frequency Model**: Represents the vessel's response to wave excitation.

---

### Environmental Loads

Simulated using models for:
- **Water Currents**: Based on 2D surface flow with Gauss-Markov processes.
- **Wind**: Includes mean wind velocity and gust modeled using the Harris spectrum.
- **Waves**: Modeled using the ITTC spectrum for significant wave height and peak wave period.

---

### Reference Model

A reference trajectory generator ensures smooth transitions between setpoints. It includes:
- Acceleration phase
- Constant velocity phase
- Deceleration phase
- Constant position phase

---

## Observer Design

Two observers were implemented and compared:
1. **Extended Kalman Filter (EKF)**: Accurate but complex to tune.
2. **Nonlinear Passive Observer (NPO)**: Easier to tune, globally convergent, and chosen for the final system.

---

## Controller Design

Two control strategies were implemented and compared:
1. **Proportional-Integral-Derivative (PID) Controller**:
   - Easy to implement but limited for complex dynamics.
2. **Linear Quadratic Gaussian (LQG) Controller**:
   - Optimal performance with easier tuning, selected as the primary controller.

---

## Thrust Allocation

The thrust allocation algorithm distributes control efforts among thrusters, accounting for:
- Overactuation: More thrusters than degrees of freedom.
- Physical Constraints: Includes thruster rotation speed and maximum thrust limits.
- Optimization: Minimizes power consumption and improves response times.

---

## Simulations

Several simulations were conducted to validate the system:
1. **Environmental Loads**: Vessel response to wind, waves, and currents without control.
2. **DP and Thrust Allocation**: Four-corner tests to assess controller and allocation strategies.
3. **Observer Comparison**: EKF vs. NPO.
4. **Complete System**: Full DP system validation with selected observer and controller.
5. **Capability Plot**: Thrust utilization under varying environmental conditions.
6. **Observer Robustness**: Performance under extreme environmental conditions.

---

## Results

- The LQG controller and NPO observer demonstrated superior performance.
- The thrust allocation strategy effectively managed thruster outputs under normal and failure scenarios.
- The system showed robustness in maintaining vessel position and heading under extreme conditions.

---

## Conclusion

This project successfully developed a detailed DP system for marine vessels, incorporating advanced modeling, control strategies, and optimization techniques. The system was validated through simulations, demonstrating its effectiveness and robustness under various conditions.

---


## Dependencies

- MATLAB (R2020 or later)
- MSS Toolbox
- Simulink

---

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---
