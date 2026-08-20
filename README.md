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
Spacecraft attitude is represented using unit quaternion.
#### Euler Rigid-Body Dynamics
Rotational motion obeys Euler's equations with reaction wheel control torque .
#### Saturated PD Quaternion Tracking Control Law
Given target quaternion  and current quaternion, the error quaternion is computed via quaternion conjugation.

### 2.3 User Interface & Interaction Guide

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          INTERACTIVE MATLAB ADCS UI                         │
├──────────────────────────────────────┬──────────────────────────────────────┤
│               3D VIEW                │          BODY ANGULAR VELOCITY       │
│                                      │  ┌────────────────────────────────┐  │
│        Z ▲ (Blue)                    │  │ ~~~~\omega_x  ~~~~\omega_y     │  │
│          │                           │  └────────────────────────────────┘  │
│          │  Solid = Spacecraft Body  │             ATTITUDE ERROR           │
│          ├─────► Y (Green)           │  ┌────────────────────────────────┐  │
│         /   Dashed = Target Attitude │  │ \--------                      │  │
│        ▼ X (Red)                     │  └────────────────────────────────┘  │
├──────────────────────────────────────┴──────────────────────────────────────┤
│  [Yaw Slider: -180°..+180°]  [Pitch: -90°..+90°]  [Roll: -180°..+180°]       │
│  [ Randomize Target ]        [ Disturb (Tumble) ]  [ Reset ]                │
│  Status: TARGET LOCKED ✓                                                    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.1 Why It's Built This Way
- **Zero Toolboxes Needed:** Works out-of-the-box on standard, base MATLAB (R2018b or newer). No expensive add-on toolboxes required.
- **Drag Sliders Anytime:** Move the Yaw, Pitch, or Roll sliders mid-simulation. The dashed target triad instantly moves, and the solid CubeSat body immediately pivots and fires its reaction wheels to chase it down.
- **Real Reaction Wheel Saturation:** Motors can't produce infinite torque. We cap motor torque at realistic CubeSat limits ($4\text{ mN}\cdot\text{m}$), so you see authentic acceleration curves and realistic settling times.
- **Disturbance Injection:** Hit the **"Disturb (Tumble)"** button to kick the satellite with a random rate spike (up to $\pm 35^\circ/\text{s}$) and watch the controller automatically detumble and recover.

---

### 1.2 How the Physics & Controls Work

The simulation loop computes three main steps every 30 milliseconds ($\Delta t = 0.03\text{ s}$):

#### Step A: Kinematics (Updating Orientation with Quaternions)
We use unit quaternions $\mathbf{q} = [w, x, y, z]^T$ so the simulation never suffers from gimbal lock (the mathematical singularity Euler angles face at $90^\circ$ pitch).

$$\dot{\mathbf{q}} = \frac{1}{2} \, \mathbf{q} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}$$

*What this means:* As the satellite spins with angular rate $\boldsymbol{\omega} = [\omega_x, \omega_y, \omega_z]^T$, the quaternion rate of change $\dot{\mathbf{q}}$ updates our 3D orientation smoothly.

---

#### Step B: Rotational Dynamics (Euler's Equations)
Newton's second law applied to rotating bodies in space:

$$\mathbf{I} \dot{\boldsymbol{\omega}} = \boldsymbol{\tau}_{\text{cmd}} - \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega})$$

$$\dot{\boldsymbol{\omega}} = \mathbf{I}^{-1} \left( \boldsymbol{\tau}_{\text{cmd}} - \boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega}) \right)$$

*What this means:*
- $\mathbf{I}$ is the satellite's inertia matrix (how hard it is to spin around each axis). For a $4\text{ kg}$, $3\text{U}$ CubeSat ($10 \times 10 \times 30\text{ cm}$), the long axis ($Z$) is much easier to spin than the lateral axes ($X$ and $Y$).
- $\boldsymbol{\tau}_{\text{cmd}}$ is the reaction wheel torque pushing the satellite.
- $\boldsymbol{\omega} \times (\mathbf{I} \boldsymbol{\omega})$ is the gyroscopic cross-coupling that happens when an object spins on multiple axes at once.

---

#### Step C: The Saturated PD Controller (The "Brain")
To steer the satellite toward the target quaternion $\mathbf{q}_{\text{target}}$, the controller calculates the error quaternion:

$$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{target}}^* \otimes \mathbf{q}$$

The Proportional-Derivative (PD) control law calculates the required motor torque:

$$\boldsymbol{\tau}_{\text{raw}} = -K_p \, \mathbf{q}_{\text{err}, v} - K_d \, \boldsymbol{\omega}$$

$$\boldsymbol{\tau}_{\text{cmd}} = \text{clamp}(\boldsymbol{\tau}_{\text{raw}}, \; -\tau_{\max}, \; +\tau_{\max})$$

*What this means:*
- **Proportional term ($-K_p \, \mathbf{q}_{\text{err}, v}$):** Acts like a virtual spring, pulling the satellite toward the target orientation.
- **Derivative term ($-K_d \, \boldsymbol{\omega}$):** Acts like a shock absorber or damper, slowing down rotation so the satellite doesn't overshoot and oscillate forever.
- **Torque clamp ($\pm \tau_{\max}$):** Enforces the physical $4\text{ mN}\cdot\text{m}$ motor limit.

---

### 1.3 How to Run & Play with It

1. Open MATLAB.
2. Open the project folder.
3. In the MATLAB Command Window, type:
   ```matlab
   ADCS_simulator
   ```
4. **Try these experiments:**
   - **Smooth Slew:** Drag the Pitch slider to $+45^\circ$ and release. Watch the CubeSat smoothly pitch up and lock into place.
   - **Tumble Recovery:** Click **"Disturb (Tumble)"**. Notice how the angular rate plot spikes and the reaction wheels immediately fight back to restore pointing.
   - **Agility Test:** Click **"Randomize Target"** several times in a row to see how fast the control loop re-settles.

---

## 2. Mission Module: Python Flight & Material Degradation Simulator

While the MATLAB program focuses on real-time hand control, the **Python Simulator** (`flight_and_material_simulator.py`) zooms out to the full mission scale. It simulates hours of orbital flight, tracking how space hazards wear down the satellite's materials over time.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                 PYTHON FLIGHT & MATERIAL SIMULATOR DASHBOARD                │
├────────────────────────┬────────────────────────────────────────────────────┤
│  ● NOMINAL  T+00:18:30 │  EULER ANGLES (°): Roll, Pitch, Yaw                │
│  Roll / Pitch / Yaw    │  ┌──────────────────────────────────────────────┐  │
│  Attitude Error: 0.02° │  └──────────────────────────────────────────────┘  │
│  Altitude: 400.1 km    │  BODY RATES (°/s): ω_x, ω_y, ω_z                   │
│  Velocity: 7668 m/s    │  ┌──────────────────────────────────────────────┐  │
│  RW Saturation: 14.2%  │  └──────────────────────────────────────────────┘  │
│  Fuel Left: 0.500 kg   │  CONTROL: Attitude Error & Wheel Saturation        │
│  Mat Temp: 293.4 K     │  ┌──────────────────────────────────────────────┐  │
│  Integrity: 99.8%      │  └──────────────────────────────────────────────┘  │
│  Bus Voltage: 27.96 V  │  MATERIAL DEGRADATION: UV, Radiation, Atomic O     │
│  ΔV Used: 0.00 m/s     │  ┌──────────────────────────────────────────────┐  │
│                        │  └──────────────────────────────────────────────┘  │
│  [ATTITUDE ERROR BAR]  │  MATERIAL TEMPERATURE & EQUILIBRIUM CURVE          │
│  [RW SATURATION BAR]   │  ┌──────────────────────────────────────────────┐  │
│                        │  └──────────────────────────────────────────────┘  │
│  [ PAUSE ]   [ STOP ]  │  ELECTRICAL BUS POWER (W) & VOLTAGE (V)            │
│                        │  ┌──────────────────────────────────────────────┐  │
│                        │  └──────────────────────────────────────────────┘  │
└────────────────────────┴────────────────────────────────────────────────────┘
```

### 2.1 What Makes This Simulator Unique
- **High-Fidelity Orbit Physics:** Uses 4th-order Runge-Kutta (RK4) integration with Earth's $J_2$ oblateness perturbation, modeling real LEO ground tracks and altitude changes.
- **Realistic Orbital Disturbance Torques:** Models gravity gradient torque, aerodynamic drag in the upper atmosphere, residual magnetic dipole torque, and solar radiation pressure.
- **Multi-Hazard Environmental Engine:** Tracks 5 space degradation mechanisms simultaneously across LEO, GEO, or Deep Space:
  1. Solar Vacuum Ultraviolet (UV) photolysis (darkening surfaces).
  2. Hyperthermal Atomic Oxygen (AO) erosion (stripping polymer surfaces in LEO).
  3. Ionizing particle radiation dose (Van Allen belts and cosmic rays).
  4. Day/Night thermal cycling between $+120^\circ\text{C}$ sunlit passes and $-150^\circ\text{C}$ eclipse shadow.
  5. Poisson-distributed micrometeorite impacts and vacuum outgassing.

---

### 2.2 The Physics Coupling (Materials $\leftrightarrow$ Flight)

In most simulators, material science and flight dynamics live in separate worlds. Here, they talk to each other directly:

1. **Surface Degradation $\rightarrow$ Disturbance Torque Shifts:**
   As outer coatings darken from UV and erode from atomic oxygen, solar absorptivity $\alpha(t)$ goes up and reflectivity $\rho(t)$ drops. This directly alters the **Solar Radiation Pressure (SRP)** force:
   $$\mathbf{F}_{\text{SRP}}(t) = P_{\text{sun}} \, A \, \big(1 + \rho(t)\big) \, \hat{\mathbf{s}}_{\text{body}}$$
   The resulting torque $\boldsymbol{\tau}_{\text{SRP}} = \mathbf{r}_{\text{cp}} \times \mathbf{F}_{\text{SRP}}$ shifts over time, placing changing demands on the reaction wheels.

2. **Solar Cover Glass Degradation $\rightarrow$ Bus Voltage Loss $\rightarrow$ Safe Mode:**
   As the quartz cover glass on the solar arrays darkens from radiation dose, solar power drops:
   $$P_{\text{gen}}(t) = P_{\text{base}} \times \text{Integrity}_{\text{cover}}(t)$$
   $$V_{\text{bus}}(t) = 28.0\text{ V} \times \left(0.60 + 0.40 \frac{P_{\text{gen}}(t)}{P_{\text{base}}}\right)$$
   If bus voltage drops below $22.0\text{ V}$, the flight software automatically demotes itself into `LOW VOLTAGE` safe mode to shed non-essential loads.

---

### 2.3 How to Run the Python Simulator

1. Make sure dependencies are installed:
   ```bash
   pip install numpy matplotlib
   ```
2. Run the application:
   ```bash
   python flight_and_material_simulator.py
   ```
3. Click **"⚙ CONFIGURE & LAUNCH MISSION"**, pick your spacecraft preset, environment, and surface material, and watch the live telemetry stream.
4. When finished, it automatically saves `flight_material_report.txt` and a full `telemetry_data.csv` spreadsheet.

---

## 3. Space Materials Quick Reference

The toolkit models 5 industry-standard space materials:

| Material | What It Is | Why Spacecraft Use It | Main Space Vulnerability | Typical Subsystem Role |
|---|---|---|---|---|
| **Aluminum 6061-T6** | Lightweight alloy | Strong, easy to machine, excellent heat spreader | Thermal expansion stress, minor surface pitting | CubeSat chassis skeleton, rails, structural ribs |
| **Carbon Fiber / Epoxy** | Composite laminate | Extremely stiff, $40\%$ lighter than Al, near-zero thermal expansion | Atomic oxygen in LEO aggressively erodes epoxy matrix | Deployable solar panel substrates, bus shear panels |
| **Titanium Ti-6Al-4V** | High-strength alloy | High strength, high melting point, thermal barrier | High density and higher machining cost | Thruster mounts, launch-load fasteners, hinge pins |
| **Kapton Polyimide** | Flexible amber film | Outstanding thermal insulator, handles $-269^\circ\text{C}$ to $+400^\circ\text{C}$ | Severe erosion under LEO atomic oxygen; UV darkening | Multi-Layer Insulation (MLI) thermal blankets |
| **Fused Silica (Quartz)** | Ultra-pure glass | High optical clarity, radiation-hard, inert to atomic oxygen | Radiation browning (color centers) under high doses | Solar cell cover glass, optical sensor lenses |

*For complete property tables, degradation curves, and engineering trade-offs, see the [FLIGHT_MATERIAL_SIMULATOR_REPORT.md](FLIGHT_MATERIAL_SIMULATOR_REPORT.md).*

---

## 4. Side-by-Side Comparison

| Feature | MATLAB ADCS Simulator (Showcase) | Python Flight & Material Simulator |
|---|---|---|
| **Primary Focus** | Live, hands-on 3D attitude control | Mission orbital flight & material degradation |
| **Core Language** | Base MATLAB (Zero toolboxes needed) | Python 3 (NumPy, Matplotlib, Tkinter) |
| **User Interaction** | Drag sliders live mid-simulation | Configure launch dialog $\rightarrow$ watch live dashboard |
| **Attitude Math** | Quaternions ($w, x, y, z$) | Quaternions ($w, x, y, z$) |
| **Control Law** | Saturated PD ($4\text{ mN}\cdot\text{m}$ wheel limit) | Saturated PD with wheel momentum tracking |
| **Orbit Model** | None (pure attitude dynamics) | RK4 + $J_2$ Earth oblateness propagation |
| **Disturbances** | User-injected tumble kicks ($\pm 35^\circ/\text{s}$) | Gravity gradient, aero drag, magnetic, coupled SRP |
| **Material Physics** | N/A | UV, Atomic O, Radiation, Thermal, Impacts |
| **Degradation Coupling** | N/A | Alters SRP disturbance torque & solar power |
| **Output Data** | Rolling in-memory oscilloscope buffers | CSV full time-series + TXT engineering report |

---

## 5. Detailed Engineering Reports

We have written two comprehensive, in-depth reports for this project:

- 📄 **[ADCS_SIMULATOR_REPORT.md](ADCS_SIMULATOR_REPORT.md)**: Complete mathematical derivations, quaternion kinematics, inertia tensor calculations, PD tuning, UI architecture, and transient response benchmarks for the MATLAB simulator.
- 📄 **[FLIGHT_MATERIAL_SIMULATOR_REPORT.md](FLIGHT_MATERIAL_SIMULATOR_REPORT.md)**: Full orbital mechanics equations, sensor noise models, environmental hazard math, degradation-to-flight coupling equations, and an exhaustive 5-material reference catalog with complete thermophysical property sheets.
