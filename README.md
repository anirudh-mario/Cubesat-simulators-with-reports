# CubeSat Engineering Simulation Toolkit
### Interactive ADCS Simulation (MATLAB Showcase) & Space Mission Degradation Suite (Python)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2018b%2B-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![Python](https://img.shields.io/badge/Python-3.8%2B-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-orange.svg)]()

A comprehensive small-satellite (CubeSat) engineering simulation toolkit developed as an engineering capstone project. The toolkit features a **flagship interactive 3D Attitude Determination and Control System (ADCS) simulator** in MATLAB, complemented by an **integrated orbital flight and space material degradation simulator** in Python.

---

## Table of Contents
1. [Toolkit Overview & Architecture](#1-toolkit-overview--architecture)
2. [Primary Showcase: Interactive ADCS Simulator (MATLAB)](#2-primary-showcase-interactive-adcs-simulator-matlab)
   - [2.1 Core Features & Highlights](#21-core-features--highlights)
   - [2.2 Mathematical Foundations](#22-mathematical-foundations)
   - [2.3 User Interface & Interaction Guide](#23-user-interface--interaction-guide)
   - [2.4 Running the MATLAB Simulation](#24-running-the-matlab-simulation)
3. [Secondary Module: Integrated Flight & Material Degradation Simulator (Python)](#3-secondary-module-integrated-flight--material-degradation-simulator-python)
   - [3.1 Purpose & Role in the Toolkit](#31-purpose--role-in-the-toolkit)
   - [3.2 Physics Coupling: Material Degradation & Flight Dynamics](#32-physics-coupling-material-degradation--flight-dynamics)
   - [3.3 Live Dashboard & Telemetry Visualizer](#33-live-dashboard--telemetry-visualizer)
   - [3.4 Running the Python Simulation](#34-running-the-python-simulation)
4. [Toolkit Cross-Comparison Matrix](#4-toolkit-cross-comparison-matrix)
5. [Candidate Space Materials Reference](#5-candidate-space-materials-reference)
6. [Documentation & Engineering Reports](#6-documentation--engineering-reports)
7. [Physical & Control Constants Reference](#7-physical--control-constants-reference)

---

## 1. Toolkit Overview & Architecture

Modern satellite development requires cross-disciplinary analysis spanning **attitude control dynamics**, **orbital mechanics**, and **environmental material degradation**. This toolkit brings these facets together into a cohesive simulation suite:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                    CUBESAT ENGINEERING SIMULATION TOOLKIT                   │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
         ┌─────────────────────────────┴─────────────────────────────┐
         ▼                                                           ▼
┌───────────────────────────────────┐       ┌───────────────────────────────────┐
│       PRIMARY FLAGSHIP MODULE     │       │     SECONDARY INTEGRATED MODULE   │
│       Interactive ADCS Sim        │       │    Flight & Material Sim          │
│       (ADCS_simulator.m)          │       │(flight_and_material_simulator.py) │
├───────────────────────────────────┤       ├───────────────────────────────────┤
│ • Platform: Base MATLAB           │       │ • Platform: Python 3 + Tkinter    │
│ • Focus: Real-time 3D control     │       │ • Focus: Mission orbital flight   │
│ • Live slider orientation steering│       │ • RK4 + J2 orbit propagation      │
│ • Saturated PD reaction wheel law │       │ • Multi-hazard space degradation  │
│ • Live tumble disturbance kick    │       │ • SRP & power degradation coupling│
│ • Rolling telemetry buffers       │       │ • Full CSV + TXT data logging     │
└───────────────────────────────────┘       └───────────────────────────────────┘
```

---

## 2. Primary Showcase: Interactive ADCS Simulator (MATLAB)

The **Interactive CubeSat ADCS Simulator** (`ADCS_simulator.m`) is the centerpiece of this engineering project. It delivers an intuitive, visually stunning, real-time closed-loop attitude determination and control environment with zero external toolbox dependencies.

### 2.1 Core Features & Highlights
- **Live Target Attitude Dragging:** Adjust Yaw, Pitch, and Roll sliders at any time while the simulation is actively computing dynamics; the target orientation updates immediately, and the CubeSat executes a real-time reorientation maneuver.
- **Dual 3D Triad Rendering:** Displays the active CubeSat 3D body triad (solid red/green/blue lines) continuously chasing the commanded target reference triad (dashed lines).
- **Physical Reaction Wheel Model:** Implements realistic torque saturation limits ($\tau_{\max} = 0.004\text{ N}\cdot\text{m}$) preventing unrealistic instant pointing.
- **Perturbation & Disturbance Injection:**
  - **"Disturb (Tumble)" Button:** Injects an abrupt random angular velocity disturbance kick ($\pm 35^\circ/\text{s}$), challenging the PD controller to detumble and re-acquire pointing.
  - **"Randomize Target" Button:** Commands an instantaneous random target attitude.
  - **"Reset" Button:** Re-centers state, zeroes angular rates, and resets scrolling buffers.
- **Rolling Real-Time Telemetry:** Scrolling history plots for 3-axis angular rates ($\omega_x, \omega_y, \omega_z$), attitude error norm $\|q_{\text{err}}\|$, and lock-in indicator status.

### 2.2 Mathematical Foundations

#### Quaternion Kinematics
Spacecraft attitude is represented using unit quaternions $q = [q_0, \mathbf{q}_v]^T = [w, x, y, z]^T$, completely eliminating gimbal lock singularities:
$$\dot{q} = \frac{1}{2} q \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix} = \frac{1}{2} \begin{bmatrix} -x\omega_x - y\omega_y - z\omega_z \\ w\omega_x + y\omega_z - z\omega_y \\ w\omega_y - x\omega_z + z\omega_x \\ w\omega_z + x\omega_y - y\omega_x \end{bmatrix}$$

#### Euler Rigid-Body Dynamics
Rotational motion obeys Euler's equations with reaction wheel control torque $\boldsymbol{\tau}_{\text{cmd}}$:
$$\mathbf{I} \dot{\boldsymbol{\omega}} + \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) = \boldsymbol{\tau}_{\text{cmd}} \implies \dot{\boldsymbol{\omega}} = \mathbf{I}^{-1} \left( \boldsymbol{\tau}_{\text{cmd}} - \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) \right)$$
where $\mathbf{I} = \text{diag}(I_{xx}, I_{yy}, I_{zz})$ is the 3U CubeSat inertia tensor calculated from mass ($4.0\text{ kg}$) and dimensions ($0.1\text{m} \times 0.1\text{m} \times 0.3\text{m}$).

#### Saturated PD Quaternion Tracking Control Law
Given target quaternion $q_{\text{target}}$ and current quaternion $q$, the error quaternion is computed via quaternion conjugation:
$$q_{\text{err}} = q_{\text{target}}^* \otimes q$$
If the scalar component $q_{\text{err}, 0} < 0$, the quaternion is negated to enforce the shortest rotation path. The control torque command is then:
$$\boldsymbol{\tau}_{\text{cmd}} = \text{clip}\left( -K_p \mathbf{q}_{\text{err}, v} - K_d \boldsymbol{\omega}, \; -\tau_{\max}, \; +\tau_{\max} \right)$$
*(Default tuning: $K_p = 0.015$, $K_d = 0.050$, $\tau_{\max} = 0.004\text{ N}\cdot\text{m}$, $\Delta t = 0.03\text{ s}$)*.

### 2.3 User Interface & Interaction Guide

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          INTERACTIVE MATLAB ADCS UI                         │
├──────────────────────────────────────┬──────────────────────────────────────┤
│               3D VIEW                │          BODY ANGULAR VELOCITY       │
│                                      │  ┌────────────────────────────────┐  │
│        Z ▲ (Blue)                    │  │ ~~~~\omega_x  ~~~~\omega_y     │  │
│          │                           │  └────────────────────────────────┘  │
│          │  Solid = Body             │             ATTITUDE ERROR           │
│          ├─────► Y (Green)           │  ┌────────────────────────────────┐  │
│         /   Dashed = Target          │  │ \--------                      │  │
│        ▼ X (Red)                     │  └────────────────────────────────┘  │
├──────────────────────────────────────┴──────────────────────────────────────┤
│  [Yaw Slider -180..+180]   [Pitch Slider -90..+90]   [Roll Slider -180..+180]│
│  [ Randomize Target ]       [ Disturb (Tumble) ]      [ Reset ]              │
│  Status: TARGET LOCKED ✓                                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 2.4 Running the MATLAB Simulation

1. Open MATLAB (R2018b or later; standard base installation, no toolboxes needed).
2. Navigate to the project root directory.
3. Run the script from the MATLAB Command Window:
   ```matlab
   ADCS_simulator
   ```
4. A $1250 \times 700$ figure window will appear with the active 3D visualization and sliders. Drag sliders at will to observe real-time attitude tracking.

---

## 3. Secondary Module: Integrated Flight & Material Degradation Simulator (Python)

The **Integrated Flight & Material Simulator** (`flight_and_material_simulator.py`) is a multi-threaded Python application combining full orbital mechanics, attitude control, and space material degradation.

### 3.1 Purpose & Role in the Toolkit
While the MATLAB application focuses on real-time attitude interaction, the Python simulation answers mission-level questions:
- *How do orbital disturbance torques (gravity gradient, magnetic dipole, aerodynamic drag, solar radiation pressure) challenge attitude control over multi-orbit passes?*
- *How does the space environment (UV radiation, atomic oxygen, thermal cycles, ionizing radiation) degrade satellite materials over time?*
- *How does material degradation feedback into attitude control disturbances and solar power loss?*

### 3.2 Physics Coupling: Material Degradation & Flight Dynamics

The simulation actively couples physical material degradation to satellite dynamics:
1. **Solar Radiation Pressure (SRP) Drift:** As outer surface materials darken and erode from UV and atomic oxygen (AO), their solar absorptivity $\alpha(t)$ increases and optical reflectivity $\rho(t) = 1 - \alpha(t)$ drops. This alters the SRP force magnitude:
   $$\mathbf{F}_{\text{SRP}}(t) = P_{\text{sun}} A (1 + \rho(t)) \hat{\mathbf{s}}_{\text{body}}$$
   The resulting disturbance torque $\boldsymbol{\tau}_{\text{SRP}} = \mathbf{r}_{\text{cp}} \times \mathbf{F}_{\text{SRP}}$ shifts dynamically over the mission lifetime.
2. **Solar Power Generation & Bus Voltage Decay:** As solar cell cover glass (Fused Silica / polymers) darkens from ionizing radiation and UV exposure, solar array efficiency $\eta_{\text{solar}}(t)$ drops:
   $$P_{\text{gen}}(t) = P_{\text{base}} \cdot \text{Integrity}_{\text{cover}}(t), \quad V_{\text{bus}}(t) = 28.0 \cdot \left(0.60 + 0.40 \frac{P_{\text{gen}}(t)}{P_{\text{base}}}\right)$$
   When bus voltage falls below $22.0\text{ V}$, the flight software automatically transitions into `LOW VOLTAGE` safe mode.
3. **Thermal Radiative Equilibrium:** Computes instantaneous equilibrium temperatures between solar absorption and Stefan-Boltzmann radiative cooling:
   $$T_{\text{eq}} = \left( \frac{\alpha(t) \cdot \Phi_{\text{solar}}}{\epsilon(t) \cdot \sigma} \right)^{1/4}$$

### 3.3 Live Dashboard & Telemetry Visualizer

The multi-threaded Tkinter dashboard features:
- **12 Live Telemetry Tiles:** Roll, Pitch, Yaw, Attitude Error, Altitude, Orbital Velocity, Reaction Wheel Saturation %, Fuel Remaining, Surface Material Temperature, Material Integrity %, Bus Voltage, Expended $\Delta V$.
- **6 Synchronized Real-Time Charts:** Euler angles, body rates, attitude error & wheel saturation, material degradation breakdown (UV, Rad, AO), thermal temperature curve, and bus power/voltage telemetry.
- **Automatic Mission Persistence:** Generates `flight_material_report.txt` and full per-timestep `telemetry_data.csv`.

### 3.4 Running the Python Simulation

Ensure dependencies are installed:
```bash
pip install numpy matplotlib
```
Run the simulator:
```bash
python flight_and_material_simulator.py
```

---

## 4. Toolkit Cross-Comparison Matrix

| Metric / Aspect | Primary Showcase: ADCS Simulator | Secondary Module: Flight & Material Simulator |
|---|---|---|
| **Environment / Language** | MATLAB (Base only, no toolboxes) | Python 3 (NumPy, Matplotlib, Tkinter) |
| **Primary Focus** | Live 3D closed-loop attitude tracking | Mission flight dynamics & material aging |
| **Attitude Math** | Quaternions ($w, x, y, z$), Hamilton product | Quaternions ($w, x, y, z$), Hamilton product |
| **Control Law** | Saturated PD on quaternion error | Saturated PD on quaternion error + RW Momentum |
| **Interaction Model** | Live slider drag & perturb mid-flight | Pre-launch mission config dialog + live dashboard |
| **Orbital Mechanics** | None (pure rigid-body attitude dynamics) | RK4 + $J_2$ Earth oblateness perturbation |
| **Disturbances Modeled** | User-injected tumble kick ($\pm 35^\circ/\text{s}$) | Gravity gradient, aero drag, magnetic, coupled SRP |
| **Material Degradation** | N/A | UV photolysis, AO erosion, ionizing radiation, thermal |
| **Degradation Coupling** | N/A | Shifts SRP disturbance torque & solar power voltage |
| **Threading Model** | Single-threaded `while ishandle(fig)` loop | Background physics thread + lock-protected GUI |
| **Persisted Output** | In-memory rolling telemetry buffers | CSV full time-series + TXT engineering report |

---

## 5. Candidate Space Materials Reference

The toolkit models 5 spaceflight-grade materials:

| Material | Density ($\text{kg/m}^3$) | Thermal Cond. ($\text{W/m}\cdot\text{K}$) | Solar Absorptivity $\alpha_0$ | IR Emissivity $\epsilon_0$ | Primary Space Application |
|---|---|---|---|---|---|
| **Aluminum 6061-T6** | 2,700 | 167.0 | 0.09 | 0.05 | Primary CubeSat chassis, structural ribs |
| **Carbon Fiber / Epoxy** | 1,600 | 5.0 | 0.92 | 0.85 | Solar array substrates, high-stiffness panels |
| **Titanium Ti-6Al-4V** | 4,430 | 6.7 | 0.40 | 0.10 | High-stress fasteners, thruster brackets |
| **Kapton Polyimide Film** | 1,420 | 0.12 | 0.30 | 0.86 | Multi-Layer Insulation (MLI) thermal blankets |
| **Fused Silica (Quartz)** | 2,203 | 1.38 | 0.07 | 0.93 | Solar cell cover glass, optical sensor lenses |

*For complete characteristics, degradation curves, and engineering trade-offs, refer to [FLIGHT_MATERIAL_SIMULATOR_REPORT.md](FLIGHT_MATERIAL_SIMULATOR_REPORT.md).*

---

## 6. Documentation & Engineering Reports

Comprehensive standalone engineering reports are provided for detailed academic and technical evaluation:
- [ADCS_SIMULATOR_REPORT.md](ADCS_SIMULATOR_REPORT.md) — Comprehensive technical report on the MATLAB interactive ADCS simulator, mathematical derivations, controller design, and validation.
- [FLIGHT_MATERIAL_SIMULATOR_REPORT.md](FLIGHT_MATERIAL_SIMULATOR_REPORT.md) — Comprehensive technical report on the integrated Python flight and material degradation simulator, orbital mechanics, physics coupling, and the exhaustive Space Material Catalog.

---

## 7. Physical & Control Constants Reference

| Symbol | Parameter | Nominal Value | Units |
|---|---|---|---|
| $\mu_{\text{Earth}}$ | Earth Gravitational Parameter | $3.986004418 \times 10^{14}$ | $\text{m}^3/\text{s}^2$ |
| $R_{\text{Earth}}$ | Earth Mean Radius | $6.371 \times 10^6$ | $\text{m}$ |
| $J_2$ | Earth Oblateness Perturbation | $1.08263 \times 10^{-3}$ | — |
| $\Omega_{\text{Earth}}$ | Earth Angular Rotation Rate | $7.292115 \times 10^{-5}$ | $\text{rad/s}$ |
| $B_0$ | Reference Earth Magnetic Field | $3.12 \times 10^{-4}$ | $\text{T}$ |
| $P_{\text{sun}}$ | Solar Radiation Pressure (1 AU) | $4.56 \times 10^{-6}$ | $\text{N/m}^2$ |
| $\sigma$ | Stefan-Boltzmann Constant | $5.670374 \times 10^{-8}$ | $\text{W}/(\text{m}^2\cdot\text{K}^4)$ |
| $\Phi_{\text{solar}}$ | Solar Constant (1 AU) | $1361.0$ | $\text{W/m}^2$ |
| $g_0$ | Standard Gravity | $9.80665$ | $\text{m/s}^2$ |
