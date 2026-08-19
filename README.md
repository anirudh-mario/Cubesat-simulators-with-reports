# CubeSat Engineering Simulation Toolkit
### Interactive ADCS Simulation (MATLAB Showcase) & Space Mission Degradation Suite (Python)

Welcome to the **CubeSat Engineering Simulation Toolkit**! This project was built as an engineering project to explore and replicate satellite dynamics in software. It brings together two core areas of space engineering that are usually separated: **how a satellite points and controls its orientation (ADCS)**, and **how harsh space environments degrade its materials and affect its flight**.

The toolkit is split into two complementary tools:
1. **Interactive 3D ADCS Simulator (MATLAB)** — *The Flagship Showcase*: A real-time, interactive 3D simulation where you can grab sliders mid-flight, command new orientations, inject tumble disturbances, and watch reaction wheels stabilize the spacecraft in real time.
2. **Flight & Space Material Degradation Simulator (Python)** — *The Mission Toolkit Module*: A high-fidelity mission simulation that runs orbital mechanics (RK4 + $J_2$) alongside material degradation models (UV, atomic oxygen, radiation, thermal cycling) to see how material aging physically alters flight disturbance torques and solar power generation.

---

## Quick Navigation
- [1. Flagship Showcase: Interactive MATLAB ADCS Simulator](#1-flagship-showcase-interactive-matlab-adcs-simulator)
  - [1.1 Why It's Built This Way](#11-why-its-built-this-way)
  - [1.2 How the Physics & Controls Work](#12-how-the-physics--controls-work)
  - [1.3 How to Run & Play with It](#13-how-to-run--play-with-it)
- [2. Mission Module: Python Flight & Material Degradation Simulator](#2-mission-module-python-flight--material-degradation-simulator)
  - [2.1 What Makes This Simulator Unique](#21-what-makes-this-simulator-unique)
  - [2.2 The Physics Coupling (Materials $\leftrightarrow$ Flight)](#22-the-physics-coupling-materials-leftrightarrow-flight)
  - [2.3 How to Run the Python Simulator](#23-how-to-run-the-python-simulator)
- [3. Space Materials Quick Reference](#3-space-materials-quick-reference)
- [4. Side-by-Side Comparison](#4-side-by-side-comparison)
- [5. Detailed Engineering Reports](#5-detailed-engineering-reports)

---

## 1. Flagship Showcase: Interactive MATLAB ADCS Simulator

The **MATLAB ADCS Simulator** (`ADCS_simulator.m`) is the main highlight of this toolkit. Instead of running a static script and waiting for plots at the end, this simulator runs a live physics loop in real time. You can interact with the CubeSat while it flies, just like an operator at a ground station commanding a spacecraft.

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
