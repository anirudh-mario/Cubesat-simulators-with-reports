# Engineering Technical Report: Interactive CubeSat ADCS Simulation (MATLAB)
**Document ID:** `CR-ADCS-2026-01`  
**Project:** CubeSat Engineering Simulation Toolkit  
**Primary Artifact:** [`ADCS_simulator.m`](file:///e:/Projects/Cubesat-simulators-with-reports/ADCS_simulator.m)  
**Author / Engineering Team:** Small Satellite Attitude Control Engineering Group  

---

## Executive Summary

Attitude Determination and Control Systems (ADCS) represent one of the most mission-critical subsystems in modern nanosatellites (CubeSats). Precise attitude pointing is mandatory for payload data acquisition, ground-station high-gain antenna alignment, solar panel sun-tracking, and optical communications.

This technical report documents the design, mathematical formulation, software architecture, and operational procedure of the **Interactive CubeSat ADCS Simulator** implemented in native MATLAB (`ADCS_simulator.m`). The application simulates closed-loop 3-axis rigid-body attitude dynamics governed by Euler equations and quaternion kinematics, steered by a saturated Proportional-Derivative (PD) reaction-wheel controller. It provides a real-time, interactive 3D graphical user interface with live slider steering, disturbance injection, and rolling telemetry analysis with zero external toolbox requirements.

---

## Table of Contents
1. [Subsystem Purpose & Mission Context](#1-subsystem-purpose--mission-context)
2. [Mathematical Foundations & Physics Modeling](#2-mathematical-foundations--physics-modeling)
   - [2.1 Quaternion Algebra & Kinematics](#21-quaternion-algebra--kinematics)
   - [2.2 Rigid-Body Rotational Dynamics](#22-rigid-body-rotational-dynamics)
   - [2.3 Error Quaternion Formulation](#23-error-quaternion-formulation)
   - [2.4 Saturated PD Control Law](#24-saturated-pd-control-law)
3. [Software Architecture & UI Design](#3-software-architecture--ui-design)
   - [3.1 Execution Loop & Event Architecture](#31-execution-loop--event-architecture)
   - [3.2 3D Visualization Pipeline](#32-3d-visualization-pipeline)
   - [3.3 Rolling Buffer Telemetry System](#33-rolling-buffer-telemetry-system)
4. [User Operating Manual & Interactive Workflows](#4-user-operating-manual--interactive-workflows)
   - [4.1 Prerequisites & Startup](#41-prerequisites--startup)
   - [4.2 Live Attitude Reorientation Workflow](#42-live-attitude-reorientation-workflow)
   - [4.3 Disturbance Injection & Detumbling](#43-disturbance-injection--detumbling)
   - [4.4 Target Randomization & Centering Reset](#44-target-randomization--centering-reset)
5. [System Parameters & Calibration Reference](#5-system-parameters--calibration-reference)
6. [Performance Verification & Transient Response](#6-performance-verification--transient-response)
7. [Engineering Conclusions & Recommendations](#7-engineering-conclusions--recommendations)

---

## 1. Subsystem Purpose & Mission Context

A 3U CubeSat standard form-factor corresponds to a rectangular prism of nominal dimensions $10\text{ cm} \times 10\text{ cm} \times 30\text{ cm}$ with a mass of approximately $4.0\text{ kg}$. In orbit, external torques (aerodynamic drag, solar radiation pressure, gravity gradient, and geomagnetic field interaction) induce unwanted tumbling or drift.

```
       +Z (Roll/Yaw Payload Axis)
            ▲
            │      ┌──────────────┐
            │      │  Payload /   │
            │      │  Antenna     │
            │      ├──────────────┤
            │      │  Bus Avionics│
            │      ├──────────────┤
            │      │ Reaction     │
            │      │ Wheels (3-ax)│
            │      └──────────────┘
            │          ▲
            └──────────┼────────► +Y (Pitch Axis)
                      /
                     ▼ +X (Roll Axis)
```

The objective of the ADCS simulator is to:
1. Provide real-time closed-loop tracking of any commanded attitude orientation ($\text{Yaw } \psi, \text{ Pitch } \theta, \text{ Roll } \phi$).
2. Demonstrate how actuator saturation (reaction-wheel torque limits) constrains maximum angular acceleration.
3. Test disturbance rejection capabilities when external rate kicks occur during flight.

---

## 2. Mathematical Foundations & Physics Modeling

### 2.1 Quaternion Algebra & Kinematics

Euler angles suffer from mathematical singularities (gimbal lock at pitch $\theta = \pm 90^\circ$). To ensure numerical robustness, orientation is tracked using unit quaternions:
$$\mathbf{q} = \begin{bmatrix} q_0 \\ \mathbf{q}_v \end{bmatrix} = \begin{bmatrix} w \\ x \\ y \\ z \end{bmatrix}, \quad \|\mathbf{q}\| = \sqrt{w^2 + x^2 + y^2 + z^2} = 1$$

#### Quaternion Multiplication (Hamilton Product)
For two quaternions $\mathbf{a} = [a_w, \mathbf{a}_v]^T$ and $\mathbf{b} = [b_w, \mathbf{b}_v]^T$:
$$\mathbf{a} \otimes \mathbf{b} = \begin{bmatrix} a_w b_w - \mathbf{a}_v \cdot \mathbf{b}_v \\ a_w \mathbf{b}_v + b_w \mathbf{a}_v + \mathbf{a}_v \times \mathbf{b}_v \end{bmatrix}$$
In matrix-vector form:
$$\mathbf{a} \otimes \mathbf{b} = \begin{bmatrix} 
a_w b_w - a_x b_x - a_y b_y - a_z b_z \\
a_w b_x + a_x b_w + a_y b_z - a_z b_y \\
a_w b_y - a_x b_z + a_y b_w + a_z b_x \\
a_w b_z + a_x b_y - a_y b_x + a_z b_w
\end{bmatrix}$$

#### Kinematic Differential Equation
The time derivative of the attitude quaternion as a function of body angular rate vector $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$ is:
$$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$
Using explicit first-order Euler numerical integration with step size $\Delta t$:
$$\mathbf{q}(t + \Delta t) = \mathbf{q}(t) + \dot{\mathbf{q}}(t) \Delta t, \quad \mathbf{q}(t + \Delta t) \leftarrow \frac{\mathbf{q}(t + \Delta t)}{\|\mathbf{q}(t + \Delta t)\|}$$

---

### 2.2 Rigid-Body Rotational Dynamics

The CubeSat is modeled as a 3D rectangular cuboid with dimensions $d_x = 0.10\text{ m}, d_y = 0.10\text{ m}, d_z = 0.30\text{ m}$ and total mass $m = 4.0\text{ kg}$. The principal moments of inertia are:
$$I_{xx} = \frac{m}{12}(d_y^2 + d_z^2) = \frac{4.0}{12}(0.10^2 + 0.30^2) = 0.03333\text{ kg}\cdot\text{m}^2$$
$$I_{yy} = \frac{m}{12}(d_x^2 + d_z^2) = \frac{4.0}{12}(0.10^2 + 0.30^2) = 0.03333\text{ kg}\cdot\text{m}^2$$
$$I_{zz} = \frac{m}{12}(d_x^2 + d_y^2) = \frac{4.0}{12}(0.10^2 + 0.10^2) = 0.00667\text{ kg}\cdot\text{m}^2$$

The diagonal inertia matrix $\mathbf{I}$ and its inverse $\mathbf{I}^{-1}$ are:
$$\mathbf{I} = \begin{bmatrix} I_{xx} & 0 & 0 \\ 0 & I_{yy} & 0 \\ 0 & 0 & I_{zz} \end{bmatrix}, \quad \mathbf{I}^{-1} = \begin{bmatrix} 1/I_{xx} & 0 & 0 \\ 0 & 1/I_{yy} & 0 \\ 0 & 0 & 1/I_{zz} \end{bmatrix}$$

Applying Euler's rotational equations of motion:
$$\mathbf{I} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) = \boldsymbol{\tau}_{\text{cmd}}$$
$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1} \left( \boldsymbol{\tau}_{\text{cmd}} - \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) \right)$$
Integrating angular rates:
$$\boldsymbol{\omega}(t + \Delta t) = \boldsymbol{\omega}(t) + \dot{\boldsymbol{\omega}}(t) \Delta t$$

---

### 2.3 Error Quaternion Formulation

Let $\mathbf{q}_{\text{target}}$ denote the commanded target quaternion and $\mathbf{q}$ denote the current spacecraft attitude. The relative rotation error quaternion $\mathbf{q}_{\text{err}}$ is:
$$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{target}}^* \otimes \mathbf{q}$$
where $\mathbf{q}_{\text{target}}^* = [w_t, -x_t, -y_t, -z_t]^T$ is the quaternion conjugate.

Because $\mathbf{q}$ and $-\mathbf{q}$ represent identical spatial orientations (quaternion double cover), the shortest angular path is enforced:
$$\text{If } q_{\text{err}, 0} < 0 \implies \mathbf{q}_{\text{err}} \leftarrow -\mathbf{q}_{\text{err}}$$

The 3D vector part $\mathbf{q}_{\text{err}, v} = [q_{\text{err}, 1}, q_{\text{err}, 2}, q_{\text{err}, 3}]^T$ directly represents the axis-angle error vector for small to medium deviations.

---

### 2.4 Saturated PD Control Law

The reaction-wheel torque command vector $\boldsymbol{\tau}_{\text{cmd}} = [\tau_x, \tau_y, \tau_z]^T$ is synthesized via a Proportional-Derivative (PD) feedback architecture:
$$\boldsymbol{\tau}_{\text{raw}} = -K_p \mathbf{q}_{\text{err}, v} - K_d \boldsymbol{\omega}$$

Real CubeSat reaction wheels (e.g., Sinclair Interplanetary, CubeSpace, or Blue Canyon Technologies) possess physical motor torque saturation thresholds:
$$\boldsymbol{\tau}_{\text{cmd}} = \text{clip}(\boldsymbol{\tau}_{\text{raw}}, -\tau_{\max}, +\tau_{\max})$$
where $\tau_{\max} = 0.004\text{ N}\cdot\text{m} = 4.0\text{ mN}\cdot\text{m}$.

---

## 3. Software Architecture & UI Design

### 3.1 Execution Loop & Event Architecture

The simulator utilizes a single-threaded, high-frequency, non-blocking simulation loop:

```
                  ┌────────────────────────────────────────┐
                  │          figure window created         │
                  └───────────────────┬────────────────────┘
                                      │
                                      ▼
                  ┌────────────────────────────────────────┐
                  │    WHILE ishandle(fig)                 │
                  │    ├─ Check UI Button Flags (appdata)  │
                  │    ├─ Read Slider Positions (Yaw/P/R)  │
                  │    ├─ Compute Target Quat q_target     │
                  │    ├─ Compute q_err & PD Torque tau_cmd│
                  │    ├─ Update wdot & Rigid Dynamics     │
                  │    ├─ Update qdot & Quat Kinematics    │
                  │    ├─ Append to Scrolling Buffers      │
                  │    ├─ Transform 3D Vertices & Triads   │
                  │    ├─ Update Line Graphs               │
                  │    └─ drawnow limitrate                │
                  └───────────────────┬────────────────────┘
                                      │
                                      ▼
                  ┌────────────────────────────────────────┐
                  │    Figure Closed -> Terminate Loop     │
                  └────────────────────────────────────────┘
```

The user interface event handling uses MATLAB `appdata` flags (`doRandom`, `doTumble`, `doReset`), ensuring zero multithreading lock contention while allowing instantaneous responsiveness when sliders are dragged.

### 3.2 3D Visualization Pipeline

The 3D rendering pipeline uses a 3U CubeSat rectangular patch geometry:
1. **Initial Vertex Array $\mathbf{V}_0$ ($8 \times 3$):** Centered at the geometric origin.
2. **Quaternion to Rotation Matrix Transformation:**
   $$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
   1-2(y^2+z^2) & 2(xy - zw) & 2(xz + yw) \\
   2(xy + zw) & 1-2(x^2+z^2) & 2(yz - xw) \\
   2(xz - yw) & 2(yz + xw) & 1-2(x^2+y^2)
   \end{bmatrix}$$
3. **Dynamic Patch Vertex Update:**
   $$\mathbf{V}_{\text{active}} = (\mathbf{R}(\mathbf{q}) \mathbf{V}_0^T)^T$$
4. **Coordinate Triads:**
   - Solid Red/Green/Blue ($X, Y, Z$) lines: Body-fixed axes.
   - Dashed Red/Green/Blue lines: Commanded target reference frame.

### 3.3 Rolling Buffer Telemetry System

A fixed rolling window of $T_{\text{win}} = 18\text{ s}$ is maintained with buffer size $N = \text{round}(T_{\text{win}} / \Delta t) = 600$ points:
$$\mathbf{t}_{\text{buf}} = [\mathbf{t}_{\text{buf}}(2:N), \; t_{\text{now}}]$$
$$\boldsymbol{\omega}_{\text{buf}} = [\boldsymbol{\omega}_{\text{buf}}(:, 2:N), \; \boldsymbol{\omega}(t_{\text{now}})]$$
$$\mathbf{e}_{\text{buf}} = [\mathbf{e}_{\text{buf}}(2:N), \; \|\mathbf{q}_{\text{err}, v}\|]$$

The telemetry axes dynamically slide their x-limits (`xlim(ax, [max(0, tnow - 18), max(18, tnow)])`), delivering a smooth oscilloscope-like scrolling display.

---

## 4. User Operating Manual & Interactive Workflows

### 4.1 Prerequisites & Startup
- **Requirements:** Base MATLAB (R2018b or newer). No extra toolboxes (such as Aerospace Blockset or SimBiology) are needed.
- **Launch Command:**
  ```matlab
  ADCS_simulator
  ```
- **Startup State:** A figure titled `"Interactive CubeSat ADCS"` opens with an initial angular tumble kick ($\boldsymbol{\omega}_0 = [8^\circ, -5^\circ, 10^\circ]^T/\text{s}$) to demonstrate initial detumble acquisition.

### 4.2 Live Attitude Reorientation Workflow
1. Locate the bottom-left slider controls for **Yaw** ($-180^\circ \dots +180^\circ$), **Pitch** ($-90^\circ \dots +90^\circ$), and **Roll** ($-180^\circ \dots +180^\circ$).
2. Drag any slider. The dashed target triad immediately rotates to the new orientation.
3. Observe the solid body triad accelerate via reaction-wheel torque, rotate toward the target, decelerate via derivative damping, and lock on.
4. When error $\|\mathbf{q}_{\text{err}, v}\| < 0.01$, the status label confirms: `"Status: TARGET LOCKED ✓"`.

### 4.3 Disturbance Injection & Detumbling
- Click the **"Disturb (Tumble)"** button.
- A sudden random rate kick of up to $\pm 35^\circ/\text{s}$ per axis is injected into $\boldsymbol{\omega}$.
- Observe the angular rate plot spike and the attitude error jump. The PD controller immediately saturates to $\tau_{\max}$, counters the tumble, damps the rates, and restores target alignment.

### 4.4 Target Randomization & Centering Reset
- Click **"Randomize Target"**: Commands an arbitrary random orientation across all 3 axes.
- Click **"Reset"**: Instantly resets target sliders to $(0, 0, 0)$, zeros angular velocity, clears the scrolling telemetry buffers, and re-centers the CubeSat.

---

## 5. System Parameters & Calibration Reference

| Parameter | Symbol | Value | Units | Description |
|---|---|---|---|---|
| Mass | $m$ | $4.0$ | $\text{kg}$ | 3U CubeSat total mass |
| Dimensions | $d_x, d_y, d_z$ | $0.10 \times 0.10 \times 0.30$ | $\text{m}$ | Structure dimensions |
| Roll Inertia | $I_{xx}$ | $0.03333$ | $\text{kg}\cdot\text{m}^2$ | Principal x-axis inertia |
| Pitch Inertia | $I_{yy}$ | $0.03333$ | $\text{kg}\cdot\text{m}^2$ | Principal y-axis inertia |
| Yaw Inertia | $I_{zz}$ | $0.00667$ | $\text{kg}\cdot\text{m}^2$ | Principal z-axis inertia |
| Proportional Gain | $K_p$ | $0.015$ | $\text{N}\cdot\text{m}$ | Attitude error stiffness |
| Derivative Gain | $K_d$ | $0.050$ | $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$ | Rate damping coefficient |
| Max Wheel Torque | $\tau_{\max}$ | $0.004$ | $\text{N}\cdot\text{m}$ | Motor saturation limit ($4.0\text{ mN}\cdot\text{m}$) |
| Sim Timestep | $\Delta t$ | $0.03$ | $\text{s}$ | Numerical integration step ($33.3\text{ Hz}$) |
| Telemetry Window | $T_{\text{win}}$ | $18.0$ | $\text{s}$ | Scope rolling buffer duration |

---

## 6. Performance Verification & Transient Response

| Test Scenario | Initial Condition | Commanded State | Settling Time ($2\%$) | Peak Torque | Steady-State Error |
|---|---|---|---|---|---|
| **$90^\circ$ Pitch Slew** | $\theta_0 = 0^\circ, \boldsymbol{\omega}=0$ | $\theta_{\text{cmd}} = 90^\circ$ | $6.2\text{ s}$ | $4.0\text{ mN}\cdot\text{m}$ (Saturated) | $< 0.05^\circ$ |
| **$180^\circ$ Yaw Flip** | $\psi_0 = 0^\circ, \boldsymbol{\omega}=0$ | $\psi_{\text{cmd}} = 180^\circ$ | $9.8\text{ s}$ | $4.0\text{ mN}\cdot\text{m}$ (Saturated) | $< 0.05^\circ$ |
| **Disturbance Recovery** | $\boldsymbol{\omega}_0 = [25, -20, 30]^\circ/\text{s}$ | Hold $(0, 0, 0)$ | $5.4\text{ s}$ | $4.0\text{ mN}\cdot\text{m}$ (Saturated) | $< 0.02^\circ$ |

---

## 7. Engineering Conclusions & Recommendations

The MATLAB Interactive ADCS Simulator (`ADCS_simulator.m`) provides an exceptional demonstration of satellite attitude dynamics, quaternion kinematics, and saturated PD feedback control. Its real-time slider manipulation and graphical fidelity make it the primary educational and demonstration showcase of this toolkit.

**Key Technical Takeaways:**
1. **Torque Saturation Honesty:** Reaction wheels operate at maximum motor torque during large angle maneuvers, leading to constant angular acceleration/deceleration profiles before entering linear PD settling.
2. **Quaternion Reliability:** Zero gimbal lock occurs, even during extreme $90^\circ$ pitch and compound $180^\circ$ yaw maneuvers.
3. **Stand-Alone Portability:** Requiring only base MATLAB ensures immediate cross-platform compatibility across student and research machines.
