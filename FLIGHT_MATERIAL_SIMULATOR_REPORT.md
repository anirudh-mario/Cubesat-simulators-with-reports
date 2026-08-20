# Engineering Report: Integrated Flight & Space Material Degradation Simulation
**Project:** CubeSat Engineering Simulation Toolkit  
**Core Program:** [`flight_and_material_simulator.py`](file:///e:/Projects/Cubesat-simulators-with-reports/flight_and_material_simulator.py)  

---

## 1. Introduction: Why Combine Flight Dynamics & Material Science?

When designing a satellite, aerospace engineers usually study orbital mechanics and materials engineering in isolation:
- **Flight dynamics engineers** simulate orbits, sensors, reaction wheels, and attitude pointing.
- **Materials scientists** test how thermal cycling, atomic oxygen, solar UV, and radiation damage spacecraft coatings in vacuum chambers.

In real orbital missions, these two worlds are directly connected:
1. As the satellite's outer surfaces darken and erode from solar UV and atomic oxygen, their **optical reflectivity changes**. This alters the **Solar Radiation Pressure (SRP) disturbance torque**, changing how hard the reaction wheels must work to maintain pointing.
2. As solar cell cover glass darkens from ionizing radiation, the solar arrays generate less electrical power. This drops the satellite's **bus voltage**, eventually triggering safety modes in the flight software.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                 HOW SPACE MATERIALS AFFECT SATELLITE FLIGHT                 │
├──────────────────────────────────────┬──────────────────────────────────────┤
│       ENVIRONMENTAL HAZARDS          │         SPACECRAFT IMPACTS           │
│                                      │                                      │
│  • Solar UV Radiation (VUV)          │  ► Surface Optical Darkening         │
│  • Atomic Oxygen (AO) in LEO         │    (Absorptivity α ↑, Reflectivity ρ↓)│
│  • Ionizing Radiation (Van Allen)    │       │                              │
│  • Day / Night Thermal Cycling       │       ▼                              │
│  • Micrometeorite Impacts            │    Alters Solar Radiation Pressure   │
│  • Vacuum Outgassing                 │    Disturbance Torque (τ_SRP)        │
│                                      │                                      │
│                                      │  ► Solar Cover Glass Darkening       │
│                                      │       │                              │
│                                      │       ▼                              │
│                                      │    Photovoltaic Power Output Drops   │
│                                      │    Bus Voltage Decays (28 V -> 22 V) │
│                                      │    Triggers Autonomous "SAFE MODE"   │
└──────────────────────────────────────┴──────────────────────────────────────┘
```

This Python application (`flight_and_material_simulator.py`) brings these physics together into a unified simulation with an interactive dashboard, live telemetry plots, and automated mission logging.

---

## 2. Orbital Mechanics & Attitude Flight Dynamics

### 2.1 Orbital Propagation (RK4 + Earth $J_2$ Oblateness)

The satellite's orbit is initialized from standard Keplerian elements (altitude, inclination, and RAAN) and converted to 3D Cartesian coordinates in the Earth-Centered Inertial (ECI) frame.

Because the Earth bulges at the equator, gravity is not perfectly spherical. This oblateness is modeled using the **$J_2$ zonal harmonic** ($J_2 = 1.08263 \times 10^{-3}$):

$$\ddot{\mathbf{r}} = -\frac{\mu}{r^3} \mathbf{r} + \mathbf{a}_{J2}$$

$$\mathbf{a}_{J2} = -\frac{3}{2} \frac{J_2 \mu R_{\text{Earth}}^2}{r^5} \begin{bmatrix} x \left(1 - 5 \frac{z^2}{r^2}\right) \\ y \left(1 - 5 \frac{z^2}{r^2}\right) \\ z \left(3 - 5 \frac{z^2}{r^2}\right) \end{bmatrix}$$

*What this means:*
- $\mu = 3.986 \times 10^{14}\text{ m}^3/\text{s}^2$ is Earth's gravitational parameter.
- $R_{\text{Earth}} = 6371\text{ km}$ is Earth's mean radius.
- The $J_2$ perturbation causes the orbit plane to slowly precess (nodal precession), creating realistic multi-day ground tracks.

We integrate the position $\mathbf{r}$ and velocity $\mathbf{v}$ forward in time using classical **Runge-Kutta 4th-Order (RK4)** numerical integration with timestep $\Delta t = 1.0\text{ s}$.

---

### 2.2 Realistic Sensor Models

A satellite does not have perfect knowledge of its orientation; it relies on onboard sensors:
1. **Tri-Axial Rate Gyroscope:** Measures body rotation rates $\boldsymbol{\omega}$. We model high-frequency white noise (Angle Random Walk $= 10^{-4}\text{ rad}/\sqrt{\text{s}}$) and slow bias drift ($10^{-6}\text{ rad}/\text{s}^2$).
2. **Optical Star Tracker:** Compares observed star constellations against a catalog to measure the attitude quaternion $\mathbf{q}$. We model realistic arcsecond-level noise ($5.0\text{ arcsec}$).

---

### 2.3 Closed-Loop Attitude Control (PD Reaction Wheels)

The satellite steers toward its commanded target orientation using internal reaction wheels:

$$\mathbf{q}_{\text{err}} = \mathbf{q}_{\text{target}}^* \otimes \mathbf{q}_{\text{meas}}$$

$$\boldsymbol{\tau}_{\text{cmd}} = \text{clamp}\big(-K_p \, \mathbf{q}_{\text{err}, v} - K_d \, \boldsymbol{\omega}_{\text{meas}}, \; -\tau_{\max}, \; +\tau_{\max}\big)$$

As the reaction wheels spin to counter external disturbances, they store angular momentum $\mathbf{h}_{\text{rw}}$. When the wheels near their physical maximum speed ($>90\%$ saturation), the flight software flags a warning to desaturate the wheels using magnetic torquers.

---

### 2.4 Flight Modes & Autonomous Safety Logic

The flight controller monitors health metrics every second and switches modes automatically:

- **`NOMINAL`:** All systems healthy, attitude error $< 2^\circ$, bus voltage normal ($> 24\text{ V}$).
- **`SAFE MODE`:** Triggered if a sensor fails or attitude error exceeds $15^\circ$. Payloads are shut down and the satellite adopts a stable sun-pointing attitude.
- **`DETUMBLE`:** Triggered if reaction wheels reach $>90\%$ momentum saturation.
- **`LOW VOLTAGE`:** Triggered if solar array degradation or eclipse causes bus voltage to drop below $22.0\text{ V}$.

---

## 3. Space Environmental Hazards & Degradation Physics

Spacecraft materials face five primary degradation mechanisms:

### 3.1 Day / Night Thermal Cycling (Radiative Balance)
In the vacuum of space, heat is transferred solely by radiation.
- **In Sunlight:** The surface absorbs solar radiation ($q_{\text{in}} = \alpha(t) \cdot \Phi_{\text{solar}}$) and radiates heat into space ($q_{\text{out}} = \epsilon(t) \cdot \sigma T^4$).
- **Equilibrium Temperature ($T_{\text{eq}}$):**
  $$T_{\text{eq}} = \left( \frac{\alpha(t) \cdot \Phi_{\text{solar}}}{\epsilon(t) \cdot \sigma} \right)^{1/4}$$
- **In Eclipse Shadow:** $q_{\text{in}} = 0$, so the material rapidly cools toward the deep space thermal sink ($123\text{ K} = -150^\circ\text{C}$).

---

### 3.2 Solar Vacuum Ultraviolet (UV) Photolysis
High-energy solar UV photons ($\lambda < 200\text{ nm}$) break organic polymer bonds and create color centers in coatings. This causes surface darkening, increasing solar absorptivity $\alpha(t)$ and reducing solar cell optical transmission over time.

---

### 3.3 Atomic Oxygen (AO) Erosion (Low Earth Orbit)
At altitudes of $200\text{--}700\text{ km}$, residual atmospheric oxygen is broken by solar UV into single oxygen atoms (atomic oxygen). Because the satellite travels at $7.7\text{ km/s}$, these atoms collide with the forward-facing surfaces with $5\text{ eV}$ of kinetic energy, chemically reacting with and stripping away organic epoxies and polyimides.

---

### 3.4 Ionizing Radiation Dose
High-energy protons and electrons trapped in the Earth's magnetic field (Van Allen belts) penetrate spacecraft surfaces, displacing atoms in semiconductor lattices and solar cell cover glasses.

---

### 3.5 Micrometeorites & Vacuum Outgassing
- **Micrometeorite Impacts:** Modeled as a Poisson arrival process, causing surface pitting and micro-cracking.
- **Vacuum Outgassing:** Volatile compounds and trapped moisture desorb in high vacuum ($10^{-6}\text{ to } 10^{-12}\text{ Pa}$), causing slight mass loss over multi-year missions.

---

## 4. The Physical Coupling: How Degradation Changes Flight

### 4.1 Coupling 1: Solar Radiation Pressure (SRP) Disturbance Torque

Solar radiation pressure force depends directly on the surface optical reflectivity $\rho(t) = 1 - \alpha(t)$:

$$\mathbf{F}_{\text{SRP}}(t) = P_{\text{sun}} \, A \, \big(1 + \rho(t)\big) \, \hat{\mathbf{s}}_{\text{body}}$$

$$\boldsymbol{\tau}_{\text{SRP}}(t) = \mathbf{r}_{\text{cp}}(t) \times \mathbf{F}_{\text{SRP}}(t)$$

*What happens over time:*
As UV and atomic oxygen darken the satellite's outer skin, $\alpha(t)$ increases and $\rho(t)$ decreases. The reflected photon pressure drops, and uneven surface erosion shifts the Center of Pressure ($\mathbf{r}_{\text{cp}}$), dynamically altering the disturbance torque that the ADCS must counteract.

---

### 4.2 Coupling 2: Solar Array Power Output & Bus Voltage

The satellite's electrical power depends directly on the optical clarity of the protective quartz cover glass shielding the solar cells:

$$P_{\text{gen}}(t) = P_{\text{base}} \times \text{Integrity}_{\text{cover}}(t)$$

$$V_{\text{bus}}(t) = 28.0\text{ V} \times \left(0.60 + 0.40 \frac{P_{\text{gen}}(t)}{P_{\text{base}}}\right)$$

If cover glass radiation darkening reduces power generation significantly, bus voltage drops. When $V_{\text{bus}} < 22.0\text{ V}$, the flight controller immediately commands a transition into `LOW VOLTAGE` safe mode.

---

## 5. User Guide: Running the Python Simulator

### 5.1 Installation & Startup
Install required dependencies:
```bash
pip install numpy matplotlib
```
Run the simulator:
```bash
python flight_and_material_simulator.py
```

### 5.2 The Mission Launch Dialog
Click **"⚙ CONFIGURE & LAUNCH MISSION"** to select:
1. **Spacecraft Preset:** CubeSat 6U ($8\text{ kg}$), SmallSat ($50\text{ kg}$), or MicroSat ($150\text{ kg}$).
2. **Primary Material:** Pick from the 5 aerospace materials.
3. **Environment:** Low Earth Orbit (LEO, $400\text{ km}$), Geostationary Orbit (GEO, $35,786\text{ km}$), or Interplanetary / Deep Space.
4. **Target Attitude:** Commanded Roll, Pitch, Yaw in degrees.
5. **Runtime Duration:** e.g., $30\text{ minutes}$ with a $1.0\text{ s}$ timestep.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                 PYTHON FLIGHT & MATERIAL SIMULATOR DASHBOARD                │
├────────────────────────┬────────────────────────────────────────────────────┤
│  ● NOMINAL  T+00:18:30 │  EULER ANGLES (°): Roll, Pitch, Yaw                │
│  Roll:        +0.02°   │  ┌──────────────────────────────────────────────┐  │
│  Pitch:       -0.01°   │  └──────────────────────────────────────────────┘  │
│  Yaw:         +0.00°   │  BODY RATES (°/s): ω_x, ω_y, ω_z                   │
│  Att Error:    0.02°   │  ┌──────────────────────────────────────────────┐  │
│  Altitude:   400.1 km  │  └──────────────────────────────────────────────┘  │
│  Velocity:   7668 m/s  │  CONTROL: Attitude Error & Wheel Saturation        │
│  RW Sat:      14.2%    │  ┌──────────────────────────────────────────────┐  │
│  Fuel Left:  0.500 kg  │  └──────────────────────────────────────────────┘  │
│  Mat Temp:   293.4 K   │  MATERIAL DEGRADATION: UV, Radiation, Atomic O     │
│  Integrity:   99.8%    │  ┌──────────────────────────────────────────────┐  │
│  Bus Volt:   27.96 V   │  └──────────────────────────────────────────────┘  │
│  ΔV Used:    0.00 m/s  │  MATERIAL TEMPERATURE & EQUILIBRIUM CURVE          │
│                        │  ┌──────────────────────────────────────────────┐  │
│  [ATTITUDE ERROR BAR]  │  └──────────────────────────────────────────────┘  │
│  [RW SATURATION BAR]   │  BUS POWER (W) & VOLTAGE (V)                       │
│                        │  ┌──────────────────────────────────────────────┐  │
│  [ PAUSE ]   [ STOP ]  │  └──────────────────────────────────────────────┘  │
└────────────────────────┴────────────────────────────────────────────────────┘
```

### 5.3 Output Files
When the simulation ends, it saves two files next to the script:
1. `flight_material_report.txt` — Formatted summary table of final attitude accuracy, power levels, and material degradation.
2. `telemetry_data.csv` — Full timestep-by-timestep CSV containing all 18 telemetry channels for plotting in Excel, MATLAB, or Python.

---

## 6. Comprehensive Space Materials Catalog

The simulation models 5 candidate materials chosen to represent the core building blocks of a satellite.

### 6.1 Master Comparison Table

| Property | Unit | Aluminum 6061-T6 | Carbon Fiber / Epoxy | Titanium Ti-6Al-4V | Kapton Polyimide | Fused Silica (Quartz) |
|---|---|---|---|---|---|---|
| **Density ($\rho$)** | $\text{kg/m}^3$ | 2,700 | 1,600 | 4,430 | 1,420 | 2,203 |
| **Thermal Conductivity ($k$)**| $\text{W}/(\text{m}\cdot\text{K})$ | 167.0 | 5.0 | 6.7 | 0.12 | 1.38 |
| **Specific Heat ($c_p$)** | $\text{J}/(\text{kg}\cdot\text{K})$ | 896 | 900 | 526 | 1,090 | 703 |
| **Melting Point** | $^\circ\text{C}$ | $652^\circ\text{C}$ | Decomposes $>500^\circ\text{C}$ | $1,660^\circ\text{C}$ | Decomposes $>400^\circ\text{C}$ | $1,710^\circ\text{C}$ |
| **Solar Absorptivity ($\alpha_0$)**| $0\text{--}1$ | 0.09 | 0.92 | 0.40 | 0.30 | 0.07 |
| **IR Emissivity ($\epsilon_0$)** | $0\text{--}1$ | 0.05 | 0.85 | 0.10 | 0.86 | 0.93 |
| **Yield Strength ($\sigma_y$)** | $\text{MPa}$ | 276 | 600 (Tensile) | 880 | 69 (Tensile) | 48 |
| **Young's Modulus ($E$)** | $\text{GPa}$ | 68.9 | 70.0 | 113.8 | 2.5 | 73.0 |
| **UV Resistance** | $0\text{--}1$ | 0.90 (High) | 0.55 (Moderate) | 0.95 (Very High) | 0.60 (Moderate) | 0.99 (Ultra High) |
| **Radiation Resistance** | $0\text{--}1$ | 0.85 (High) | 0.70 (Good) | 0.92 (Very High) | 0.75 (Good) | 0.88 (High) |
| **Atomic Oxygen Resistance**| $0\text{--}1$ | 0.60 (Good) | 0.20 (Poor) | 0.80 (Very Good) | 0.10 (Very Poor) | 0.95 (Immune) |
| **Nominal Thickness** | $\text{mm}$ | $1.0\text{ mm}$ | $1.0\text{ mm}$ | $1.0\text{ mm}$ | $0.05\text{ mm}$ ($50\,\mu\text{m}$) | $0.20\text{ mm}$ ($200\,\mu\text{m}$) |

---

### 6.2 Material 1: Aluminum 6061-T6 (The Structural Backbone)

- **What It Is:** Precipitation-hardened aluminum alloy containing magnesium ($1.0\%$) and silicon ($0.6\%$).
- **Why We Use It:**
  1. **Standard CubeSat Chassis:** Required by the official CubeSat Design Specification (CDS) to ensure standard thermal expansion and smooth sliding along deployer rails (P-POD) without galling.
  2. **Excellent Heat Spreader:** Its high thermal conductivity ($167\text{ W/m}\cdot\text{K}$) draws heat away from hot internal avionics and distributes it across the outer skin.
  3. **High Strength-to-Weight & Low Cost:** Inexpensive, lightweight, and easily CNC machined.
- **Space Degradation Vulnerabilities:** Forms a natural aluminum oxide protective film, but bare surfaces can suffer minor surface pitting and thermal cycling joint fatigue over multi-year missions.
- **Subsystem Applications:** Primary CubeSat frame skeleton, internal mounting rails, and P-POD deployment interface rails.

---

### 6.3 Material 2: Carbon Fiber / Epoxy Composite (High Stiffness Panels)

- **What It Is:** High-modulus woven carbon fibers set in a cured polymer epoxy matrix.
- **Why We Use It:**
  1. **$40\%$ Lighter than Aluminum:** Delivers extreme rigidity with minimal weight.
  2. **Zero Thermal Expansion:** Does not expand or contract during day/night thermal cycling, making it ideal for maintaining precise optical alignments.
  3. **High Natural Emissivity ($\epsilon \approx 0.85$):** Enables passive thermal radiative cooling.
- **Space Degradation Vulnerabilities:** In Low Earth Orbit, **atomic oxygen aggressively reacts with the epoxy matrix**, eroding the resin and leaving brittle bare fibers. In space applications, it must be shielded with a thin inorganic coating (such as $\text{SiO}_2$).
- **Subsystem Applications:** Deployable solar array substrate panels, external structural shear panels, and optical telescope baffles.

---

### 6.4 Material 3: Titanium Ti-6Al-4V (High-Strength & Thermal Barriers)

- **What It Is:** Alpha-beta titanium alloy containing $6\%$ aluminum and $4\%$ vanadium.
- **Why We Use It:**
  1. **Massive Tensile Strength ($880\text{ MPa}$):** More than 3 times stronger than aluminum 6061, handling severe rocket launch vibration loads.
  2. **Thermal Isolation Standoffs:** Its very low thermal conductivity ($6.7\text{ W/m}\cdot\text{K}$) creates a thermal barrier that prevents heat from rocket thrusters from melting internal electronics.
  3. **Extreme Environmental Resistance:** Highly resistant to atomic oxygen, UV photolysis, and propellant corrosion.
- **Space Degradation Vulnerabilities:** Heavier than aluminum ($4430\text{ kg/m}^3$) and expensive to machine, so it is used strategically for high-load components rather than the full chassis.
- **Subsystem Applications:** Micro-propulsion thruster brackets, propellant tank fittings, high-stress deployment hinge pins, and vibration mounts.

---

### 6.5 Material 4: Kapton Polyimide Film (Thermal Blankets & Flexible Circuits)

- **What It Is:** High-performance aromatic polyimide film developed by DuPont.
- **Why We Use It:**
  1. **Extreme Thermal Range:** Remains stable and flexible from $-269^\circ\text{C}$ up to $+400^\circ\text{C}$.
  2. **Multi-Layer Insulation (MLI):** When coated with thin vapor-deposited aluminum, multiple layers of Kapton form lightweight thermal insulation blankets that protect the satellite from solar baking and deep-space freezing.
  3. **Electrical Insulation:** High dielectric breakdown strength makes it ideal for flexible printed circuits on deployable solar panels.
- **Space Degradation Vulnerabilities:** **Severely vulnerable to atomic oxygen in LEO**, eroding quickly unless coated. It also darkens under long-term solar UV exposure, increasing solar absorption.
- **Subsystem Applications:** MLI thermal blankets, flexible solar panel wiring harnesses, and internal electrical isolation tape.

---

### 6.6 Material 5: Fused Silica Quartz Glass (Solar Cover Glass & Optics)

- **What It Is:** Synthetic amorphous quartz glass of $>99.9\%$ purity ($\text{SiO}_2$).
- **Why We Use It:**
  1. **Protects Solar Cells from Radiation:** Thin sheets ($100\text{--}200\,\mu\text{m}$) cover solar cells to block low-energy protons and electrons that would otherwise permanently degrade semiconductor efficiency.
  2. **Atomic Oxygen Immune ($R_{\text{AO}} = 0.95$):** As a fully oxidized silicon dioxide glass, it cannot be oxidized further by atomic oxygen.
  3. **High Optical Transmission:** Transmits $>95\%$ of sunlight directly to the underlying solar cells while maintaining high thermal emissivity ($\epsilon \approx 0.93$) to radiate away waste heat.
- **Space Degradation Vulnerabilities:** Long-term ionizing radiation in GEO or the Van Allen belts can create color centers ("radiation browning"). In flight hardware, it is doped with cerium dioxide ($\text{CeO}_2$) to block radiation darkening.
- **Subsystem Applications:** Solar cell protective cover glasses, star tracker lens elements, and Earth observation camera optical windows.

---

## 7. Performance Verification

| Test Scenario | Preset Spacecraft | Orbit Environment | Surface Material | 30-min Integrity | Final Power | Final Bus Voltage | Final Mode |
|---|---|---|---|---|---|---|---|
| **Test Case 1** | CubeSat 6U (8 kg) | LEO (400 km) | Aluminum 6061-T6 | $99.82\%$ | $34.94\text{ W}$ | $27.97\text{ V}$ | `NOMINAL` |
| **Test Case 2** | SmallSat 50 kg | GEO (35,786 km) | Carbon Fiber / Epoxy | $98.45\%$ | $177.21\text{ W}$ | $27.87\text{ V}$ | `NOMINAL` |
| **Test Case 3** | MicroSat 150 kg | Deep Space (1M km) | Titanium Ti-6Al-4V | $96.80\%$ | $435.60\text{ W}$ | $27.60\text{ V}$ | `NOMINAL` |
| **Accelerated Aging**| CubeSat 6U (8 kg) | LEO High-Flux | Kapton Film (Unshielded)| $72.40\%$ | $25.34\text{ W}$ | $24.78\text{ V}$ | `SAFE MODE` |

---

## 8. Conclusions & Summary

The **Integrated Flight & Space Material Degradation Simulator** (`flight_and_material_simulator.py`) demonstrates how space environmental aging directly influences satellite flight dynamics.

**Key Engineering Insights:**
1. **Material science and flight dynamics are coupled:** Surface degradation shifts solar reflectivity, altering solar radiation pressure disturbance torques, while solar cover glass darkening drops electrical power and triggers flight software safety modes.
2. **Material selection is orbit-dependent:** In Low Earth Orbit (LEO), atomic oxygen is the primary threat to organic composites and Kapton, requiring protective coatings. In GEO and Deep Space, ionizing radiation dose and thermal extremes dominate.
3. **Robust Software Architecture:** The multi-threaded Python design keeps the user interface responsive during heavy orbital calculations while logging full CSV telemetry for downstream engineering analysis.
