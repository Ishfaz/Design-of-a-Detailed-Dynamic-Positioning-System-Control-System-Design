# Design-of-a-Detailed-Dynamic-Positioning-System

This repository contains the project **"Design of a Detailed Dynamic Positioning System"**, focused on designing and implementing a dynamic positioning (DP) control system for marine vessels using MATLAB, Simulink, and the MSS Toolbox. This project explores mathematical modeling, control system design, observer implementation, thrust allocation, and environmental load simulations.

---

## Table of Contents
1. [Introduction](#introduction)
2. [Reference Frames](#reference-frames)
3. [Process Plant Model](#process-plant-model)
4. [Control Plant Model](#control-plant-model)
5. [Environmental Loads](#environmental-loads)
6. [Reference Model](#reference-model)
7. [Observer Design](#observer-design)
8. [Controller Design](#controller-design)
9. [Thrust Allocation](#thrust-allocation)
10. [Simulations](#simulations)
11. [Results](#results)
12. [Conclusion](#conclusion)
13. [Usage](#usage)
14. [Dependencies](#dependencies)
15. [License](#license)

---

## Introduction

Dynamic Positioning (DP) systems are critical for maintaining a vessel's position and heading without the need for anchors. This project details the design of a DP system capable of counteracting environmental disturbances like wind, waves, and currents.

Key components include:
- **Observer Design**: Extended Kalman Filter (EKF) and Nonlinear Passive Observer (NPO).
- **Controller Design**: Proportional-Integral-Derivative (PID) and Linear Quadratic Gaussian (LQG).
- **Thrust Allocation**: Optimization for overactuated systems.

---

## Reference Frames

Two main reference frames are used:
1. **Earth-Fixed Frame (NED)**: Tracks vessel's position relative to a fixed point.
2. **Body-Fixed Frame**: Coordinates attached to the vessel's body, facilitating control actions.

### Equation for Transformation:
\[
R(\psi) = 
\begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi  & 0 \\
0        & 0         & 1
\end{bmatrix}
\]
This rotation matrix transforms between the NED and body-fixed frames.

---

## Process Plant Model

The process plant represents high-fidelity vessel dynamics. The 6 DOF motion equation is:

\[
M\nu̇ + CRB(\nu)\nu + CA(\nu_r)\nu_r + D(\nu_r) + G(\eta) = τ_{env} + τ_{thr}
\]

Where:
- \( M \): Inertia matrix (includes added mass).
- \( CRB \), \( CA \): Coriolis and centripetal matrices.
- \( D \): Damping forces.
- \( G \): Restoring forces.
- \( τ_{env}, τ_{thr} \): Environmental and thruster forces.

---

## Control Plant Model

The reduced-order control model simplifies the process plant model for controller design, splitting motions into:
1. **Low-Frequency (LF)**: Slow motions due to environmental forces.
2. **Wave-Frequency (WF)**: Oscillations induced by waves.
3. **Bias Model**: Represents slow-varying forces and model uncertainties.

---

## Environmental Loads

Models for environmental disturbances include:
- **Current**: Surface current modeled as a vector in the NED frame:
  \[
  \nu_c = [V_c\cos\psi_c, V_c\sin\psi_c, 0]^T
  \]
- **Wind**: Calculated using the Harris spectrum.
- **Waves**: Simulated using the ITTC spectrum:
  \[
  S(\omega) = A \omega^{-5} \exp\left(-\frac{B}{\omega^4}\right)
  \]

---

## Reference Model

The reference model generates smooth transitions between setpoints using a four-phase trajectory:
1. **Acceleration**
2. **Constant Velocity**
3. **Deceleration**
4. **Constant Position**

---

## Observer Design

### 1. Extended Kalman Filter (EKF)
A nonlinear filter estimating states from noisy measurements. Prediction and correction stages handle state estimation and noise filtering.

### 2. Nonlinear Passive Observer (NPO)
A simpler alternative to the EKF, tuned using Lyapunov analysis to ensure global stability.

---

## Controller Design

### PID Controller
The control law:
\[
τ_{PID} = K_pR^T(\eta_d - \eta) + K_i \int R^T(\eta_d - \eta) dt + K_d(\nu_d - \nu)
\]

### LQG Controller
Minimizes the quadratic cost:
\[
J = E\left[\int_0^\infty (x^TQx + u^TRu) dt\right]
\]

---

## Thrust Allocation

Handles overactuated systems using:
- **Moore-Penrose Pseudoinverse**:
  \[
  u = B^T (BB^T)^{-1} τ
  \]
- **Optimization**: Minimizes power consumption and ensures operational constraints.

---

## Simulations

1. **Environmental Loads**: Vessel response to currents, wind, and waves.
2. **DP and Thrust Allocation**: Four-corner test to validate thrust allocation and controller performance.
3. **Observer Comparison**: Evaluated EKF vs. NPO.
4. **Complete DP System**: Integration of all subsystems under varying conditions.

---

## Results

- The **LQG controller** outperformed PID in stability and response time.
- The **NPO observer** offered robust state estimation with simpler tuning compared to EKF.
- Thrust allocation effectively managed overactuation and minimized power usage.

---

## Usage

Clone the repository and open the MATLAB Simulink files. Ensure the MSS Toolbox is installed for simulation.

```bash
git clone https://github.com/your-repo-link.git
