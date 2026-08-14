# CubeSat Engineering Simulation Toolkit — Unified Reference Documentation
 
**Scope note:** This document consolidates three independent, standalone simulation
programs into a single reference: one MATLAB application and two Python/Tkinter
applications, all in the small-satellite (CubeSat) engineering domain. They do **not**
share code, a runtime, or a data store today — each is a self-contained script. This
document treats them as three modules of one *conceptual* toolkit, documents each
faithfully, and is explicit anywhere it moves from "what exists" to "what a real
integration would require."
 
| Source file | Tool name used below |
|---|---|
| `Pasted_text_6_.txt` (MATLAB) | **ADCS-Interactive** |
| `Pasted_text_4_.txt` (Python) | **Flight-Sim** |
| `Pasted_text_5_.txt` (Python) | **Material-Sim** |
 
---
 
## 1. Project Overview
 
### 1.1 Vision
A small suite of engineering simulators covering three distinct concerns that a
CubeSat mission actually has to reckon with: how the spacecraft *points* (attitude
determination and control), how the spacecraft *operates* over an orbit (sensors,
disturbances, faults, propulsion, power), and how the spacecraft's *materials
survive* years of thermal cycling, radiation, atomic oxygen, and micrometeorite
bombardment. Together they form a rough end-to-end picture of CubeSat systems
engineering, even though nothing currently wires them together programmatically.
 
### 1.2 Goals
- Give an intuitive, visual feel for closed-loop attitude control (ADCS-Interactive).
- Provide a higher-fidelity, mission-length simulation with realistic sensors,
 disturbance torques, orbital mechanics, and a fault-aware flight-software state
 machine (Flight-Sim).
- Quantify how long different spacecraft materials survive the space environment,
 and which degradation mechanism dominates for each (Material-Sim).
### 1.3 Scope & Non-Goals
In scope: attitude dynamics/control, Keplerian+J2 orbit propagation, sensor noise
modeling, disturbance torque modeling, fault detection, material degradation
modeling, and desktop visualization/reporting for all of the above.
 
Explicitly out of scope in the current code: any shared persistence layer, any
network/API surface, any automated testing, and any actual data flow between the
three tools (e.g., material degradation currently has zero effect on the flight
simulator's power or optical properties — see §13).
 
---
 
## 2. System Overview
 
| Tool | Stack | Core Question It Answers | Interaction Model | Typical Output |
|---|---|---|---|---|
| **ADCS-Interactive** | MATLAB, base only (no toolboxes) | "If I command this attitude right now, how does the spacecraft get there?" | Live slider drag while sim runs | 3D live view + scrolling telemetry plots |
| **Flight-Sim** | Python 3, NumPy, Tkinter, Matplotlib | "Over a 30-minute (or longer) pass, does the spacecraft hold attitude, stay powered, and avoid safe mode?" | Configure once via a launch dialog, then watch a live dashboard | CSV telemetry (every timestep) + TXT mission report |
| **Material-Sim** | Python 3, NumPy, Tkinter, Matplotlib | "After N days in LEO/GEO/deep space, how much structural integrity does each candidate material have left, and why?" | Configure once via a launch form, then run to completion | PNG multi-panel plot + TXT summary report |
 
---
 
## 3. Architecture
 
### 3.1 Common Architectural Pattern
All three follow the same shape, at different levels of sophistication:
 
```text
Physical/Environmental Model → State Update (integration step) → Recorded History → Visualization
```
 
The difference is in *how* the loop is driven and *what* runs concurrently with the GUI.
 
### 3.2 ADCS-Interactive (MATLAB)
```text
┌─────────────────────────────┐
│ while ishandle(fig) │ <- single-threaded, blocking loop
│ read sliders (live) │
│ q_target = f(sliders) │
│ PD control -> tau_cmd │
│ rigid-body dynamics step │
│ quaternion kinematics step │
│ update 3D patch + plots │
│ drawnow limitrate │
│ end │
└─────────────────────────────┘
```
Everything — physics, control, and rendering — happens in one thread, one iteration
per loop pass. Slider callbacks only set `appdata` flags/values; the loop reads them
each pass, which is what makes the "drag anytime" interactivity work without a
separate event system.
 
### 3.3 Flight-Sim (Python)
```text
Main Thread (Tkinter) Background Thread (SimEngine)
────────────────────── ────────────────────────────
LaunchDialog (config) OrbitPropagator.step() (RK4 + J2)
 │ Sensor.measure() x4 (gyro/star/sun/mag)
 ▼ Disturbance torques (GG, SRP, drag, mag)
Dashboard._update() <── polls ── ADCSController.compute_torque() (PD, saturated)
 every 300 ms, reads Rigid-body dynamics + quaternion integration
 engine.state under a lock FlightController.check_faults() -> mode
 │ SimState.record() (append to history)
 ▼
Matplotlib canvas + status tiles
```
This is the most defensively engineered of the three: `SimEngine` extends
`threading.Thread`, all state mutation happens inside `with self._lock:`, and the
whole `run()` body is wrapped in `try/except` so a crash sets `self.error` instead of
killing the process — directly addressing the "star tracker returns None" class of
bug the file's header calls out.
 
### 3.4 Material-Sim (Python)
```text
SimulatorGUI (Tkinter form)
 │ user clicks "Run Simulation"
 ▼
SpaceSimulation.run() <- runs on the MAIN/GUI thread, synchronously
 │ for each material:
 │ for each timestep:
 │ compute equilibrium temperature
 │ accumulate UV / radiation / AO / impact / outgassing damage
 │ record history
 ▼
print_report() -> TXT file
plot_results() -> PNG file + plt.show()
```
Unlike Flight-Sim, there is no background thread here — `_on_run` calls `sim.run()`
directly on the Tkinter main loop. For a multi-year mission at a fine timestep this
blocks the UI for the full duration (see §13).
 
---
 
## 4. Workflow
 
**ADCS-Interactive:** Launch the script → figure opens with cube, telemetry axes, and
sliders already active → drag Yaw/Pitch/Roll at any time → watch the dashed target
axes move and the solid body axes chase them → optionally click *Randomize Target*,
*Disturb (Tumble)*, or *Reset* → close the figure to stop.
 
**Flight-Sim:** Launch → splash screen → *Configure Mission* opens a dialog
(spacecraft preset, orbit altitude/inclination/RAAN, target Euler angles, sim
duration, timestep) → *Launch Simulation* validates inputs and starts `SimEngine` on
a background thread → Dashboard shows live tiles (roll/pitch/yaw, attitude error,
altitude, velocity, fuel, reaction-wheel saturation, lat/lon, ΔV) plus six scrolling
plots → *Pause*/*Stop* controls → on completion (or stop), a TXT report and full CSV
telemetry history are written next to the script.
 
**Material-Sim:** Launch → form asks for environment (LEO/GEO/Deep Space), mission
duration (days), and timestep (hours) → *Run Simulation* validates ranges → runs to
completion (UI frozen during this) → status updates to "Done" → a console/TXT report
prints per-material integrity, and a 8-panel PNG (six line-chart panels, one
horizontal bar chart of final integrity, one radar chart of damage breakdown) is
saved and displayed.
 
---
 
## 5. Data Flow
 
**ADCS-Interactive**
```text
Slider values (deg)
 │
 ▼
q_target (quaternion)
 │
 ▼
PD controller ──► tau_cmd (saturated)
 │
 ▼
Rigid-body dynamics ──► w (angular rate), q (attitude)
 │
 ▼
3D patch transform + scrolling telemetry buffers (in-memory only, not persisted)
```
 
**Flight-Sim**
```text
Config (preset + orbit + target + duration + dt)
 │
 ▼
OrbitPropagator ──► r_vec, v_vec ──► altitude, lat/lon, disturbance torques
 │
 ▼
Sensors (noisy) ──► q_meas, omega_meas
 │
 ▼
ADCSController ──► tau_cmd ──┐
 │ ├──► Rigid-body dynamics ──► q, omega, euler
Disturbance torques ───────────┘
 │
 ▼
FlightController.check_faults() ──► mode, faults[]
 │
 ▼
SimState.record() ──► history dict (in-memory)
 │
 ├──► Dashboard (live, every 300 ms)
 └──► on completion: telemetry_data.csv + flight_adcs_report.txt (on disk)
```
 
**Material-Sim**
```text
SpaceEnvironment preset (LEO/GEO/Deep Space)
 │
 ▼
For each Material, for each timestep:
 equilibrium temperature ──► mat.temperature
 UV / radiation / AO / impact / outgassing steps ──► damage accumulators
 │
 ▼
Material.total_degradation (weighted index) ──► structural_integrity
 │
 ▼
history[material][metric] (in-memory)
 │
 ├──► print_report() ──► simulation_report.txt
 └──► plot_results() ──► space_material_sim_output.png
```
 
---
 
## 6. Component Relationships
 
**ADCS-Interactive:** UI sliders → target quaternion → controller → dynamics →
renderer. A single flat script; no classes — just functions (`quatMult`,
`quatConj`, `quat2rotm_wxyz`, `eul2quat_wxyz`) called directly from the main loop.
 
**Flight-Sim:** Object graph is explicit and layered:
`SpacecraftConfig` (static properties) is read by `ADCSController`,
`FlightController`, and all four sensor classes. `OrbitPropagator` is independent of
attitude and only coupled to it through the disturbance-torque functions, which take
both `q` and `r_vec`. `SimEngine` is the orchestrator that owns one instance of
everything and drives the timestep loop; `Dashboard` never touches physics directly —
it only reads `engine.state` under a lock.
 
**Material-Sim:** `SpaceSimulation` owns a list of `Material` instances and one
`SpaceEnvironment`. Each degradation mechanism (`_uv_step`, `_radiation_step`,
`_ao_step`, `_impact_step`, `_outgassing_step`) is a pure-ish method that reads
`self.env` and a single `Material`, and mutates that material's damage counters
directly — there's no controller/actuator layer at all, since there's nothing to
control.
 
---
 
## 7. Module Breakdown
 
Since all three are single-file scripts rather than multi-file packages, "modules"
below refer to the labeled sections within each file.
 
| ADCS-Interactive (MATLAB) | Flight-Sim (Python) | Material-Sim (Python) |
|---|---|---|
| Physical/control parameters | Constants | Material definition (`Material`) |
| State init | Quaternion utilities | Environment definition (`SpaceEnvironment` + 3 presets) |
| Figure/UI (3D view, telemetry axes, sliders, buttons) | Spacecraft configuration + presets | Preset materials (`make_materials`) |
| Main loop | Sensors (Gyroscope, StarTracker, SunSensor, Magnetometer) | Simulation engine (`SpaceSimulation`) |
| Helper functions (quatMult, quatConj, quat2rotm_wxyz, eul2quat_wxyz) | Disturbance torques | Visualization (`plot_results`) |
| | Attitude controller (`ADCSController`) | Summary report (`print_report`) |
| | Orbit propagator (`OrbitPropagator`) | Tkinter GUI launcher (`SimulatorGUI`) |
| | Flight controller (`FlightController`) | Entry point |
| | Simulation engine (`SimState`, `SimEngine`) | |
| | GUI (`LaunchDialog`, `Dashboard`, `App`) | |
| | Entry point | |
 
---
 
## 8. Data & Persistence Overview
 
None of the three tools use a database. All persistence is flat-file, written next
to the script at runtime:
 
| Tool | Files written | Format |
|---|---|---|
| ADCS-Interactive | none | telemetry lives only in scrolling in-memory buffers |
| Flight-Sim | `flight_adcs_report.txt`, `telemetry_data.csv` | plain text summary; full-history CSV (every field, every timestep) |
| Material-Sim | `simulation_report.txt`, `space_material_sim_output.png` | plain text summary; rendered figure |
 
There is no shared schema between the two report/CSV outputs, and no mechanism for
one tool to read another's output.
 
---
 
## 9. Feature Breakdown
 
**ADCS-Interactive**
- Live-updating target attitude via sliders (works mid-simulation, not just at start)
- PD reaction-wheel control with torque saturation
- "Randomize Target" and "Disturb (Tumble)" buttons for ad-hoc scenario testing
- Reset to zero state
- Dual axis-triad rendering (solid = current, dashed = target)
- Rolling attitude-error and angular-rate telemetry
**Flight-Sim**
- Three spacecraft presets (CubeSat 6U, SmallSat 50 kg, MicroSat 150 kg)
- Configurable orbit (altitude, inclination, RAAN) with RK4 + J2 propagation
- Four independently-modeled noisy/faultable sensors
- Four disturbance-torque sources (gravity-gradient, solar-radiation-pressure,
 aerodynamic drag, residual magnetic dipole)
- PD attitude control with reaction-wheel momentum saturation tracking
- Fault detection → automatic mode switching (NOMINAL / SAFE MODE / DETUMBLE / …)
- Thruster ΔV execution with rocket-equation fuel consumption
- Ground-track (lat/lon) computation
- Pause/resume/stop controls, live 6-panel telemetry dashboard
- Full-fidelity CSV export of every recorded timestep
**Material-Sim**
- Five preset materials spanning metals, composite, and polymer film/ceramic
- Three preset environments (LEO, GEO, Deep Space) with physically distinct
 atomic-oxygen flux, radiation dose, and thermal extremes
- Five independent degradation mechanisms per material per timestep
- Weighted composite "structural integrity" index per material
- 8-panel report figure: six time-series, one final-integrity bar chart, one
 radar chart of damage-mechanism breakdown per material
- Plain-text mission report with per-material end-of-mission table
---
 
## 10. End-to-End System Flow
 
**ADCS-Interactive:** script run → figure + controls instantiated → loop starts
immediately with a small preset initial tumble → user interacts via sliders/buttons
at will → loop runs until the figure window is closed → console prints
"Simulation window closed."
 
**Flight-Sim:** script run → `App.__init__` builds the Tk root and shows the splash
screen → user clicks *Configure Mission* → `LaunchDialog` validates all fields
(range-checked) → on success, `App._open_config` builds a `SimEngine` from the
chosen config and swaps the splash for a `Dashboard` → `engine.start()` launches the
background thread → engine and dashboard run concurrently until `duration_s` is
reached, an error occurs, or the user clicks *Stop* → `Dashboard._update` detects
`engine.running == False` and calls `_save_report()`, writing the TXT report and CSV.
 
**Material-Sim:** script run → `SimulatorGUI` shows the config form → user clicks
*Run Simulation* → `_validate()` range-checks duration/timestep → on success, a
fixed random seed (42) is set, five preset materials are instantiated, a
`SpaceSimulation` is built and `run()` synchronously simulates every material for
every timestep → `print_report()` writes/prints the summary → `plot_results()`
renders and saves the 8-panel figure, then calls the blocking `plt.show()`.
 
---
 
## 11. Cross-Tool Comparison
 
| Aspect | ADCS-Interactive | Flight-Sim | Material-Sim |
|---|---|---|---|
| Attitude representation | Quaternion (w,x,y,z) | Quaternion (w,x,y,z) | N/A |
| Control law | PD, torque-saturated | PD, torque- and momentum-saturated | N/A |
| Orbit model | None | Two-body + J2, RK4 | Implicit via environment preset only |
| Sensor noise modeled | No | Yes (4 sensor types) | N/A |
| Fault handling | None | Explicit mode state machine | None |
| Threading | Single thread, blocking loop | Background thread + polling GUI | Single thread, blocking call |
| Randomness | User-triggered only | Sensor noise (Gaussian) | Poisson-distributed micrometeorite impacts + Gaussian sensor-style noise |
| Persisted output | None | CSV + TXT | PNG + TXT |
| Reusable across tools? | No (functions duplicated in text_4 too) | No | No |
 
**Where a real dependency should exist but doesn't:** Material-Sim's
`absorptivity`/`emissivity` degradation (from `uv_damage`, `radiation_damage`) is
exactly the kind of input that should feed Flight-Sim's `solar_radiation_torque`
and its `power` bookkeeping in `FlightController` — a solar panel that's lost
absorptivity late in a mission should show up as both a different SRP torque and
lower generated power. Today, Flight-Sim's `power` field is a static constant
(`28.0`, only ever checked against a `LOW VOLTAGE` threshold) with no model behind
it at all, so there's nothing on that side to connect to yet either.
 
---
 
## 12. Design Decisions & Rationale
 
**Quaternions over Euler angles for internal state (all attitude tools).** Avoids
gimbal lock and keeps the kinematics equations non-singular; Euler angles are
computed only at the boundary (display, slider input) and converted immediately.
 
**PD control with hard torque/momentum saturation (ADCS-Interactive, Flight-Sim).**
Simple, numerically cheap, and — crucially — physically honest: real reaction wheels
have finite torque and finite momentum storage, and clipping both is what produces
believable saturation/detumble behavior rather than an idealized controller that can
apply unlimited torque.
 
**RK4 + J2 orbit propagation (Flight-Sim only).** J2 is the dominant perturbation
for LEO orbits over mission-relevant timescales (tens of minutes to days); RK4 gives
good accuracy per step without needing a variable-step integrator, at the cost of
being slower than a closed-form Keplerian propagator if very long durations were
ever run.
 
**Background thread for the physics engine, polling dashboard (Flight-Sim only).**
Keeps the Tkinter event loop responsive regardless of simulation timestep size or
duration — the explicit design tradeoff Material-Sim did *not* make (see §13).
 
**Weighted composite degradation index (Material-Sim).** Five physically distinct
damage mechanisms don't have a natural common unit; a fixed weighted sum
(30/25/20/15/10) is a defensible simplification that keeps the model interpretable
and easy to re-weight, at the cost of not being derived from any cited failure-mode
analysis.
 
**Preset-based configuration over free-form input (all three, to varying degrees).**
Spacecraft/environment presets reduce user error and give physically consistent
starting points, at the cost of limiting exploration to the preset space unless the
user edits source.
 
---
 
## 13. Known Issues & Weaknesses
 
1. **Material-Sim blocks the UI thread during the run.** `_on_run` calls
 `sim.run()` synchronously; a long mission (e.g., 3650 days at a fine timestep)
 will freeze the window for the full computation. Flight-Sim already shows the
 correct pattern (background thread) that this file should adopt.
2. **Fixed Sun vector in Flight-Sim.** `sun_dir_eci` is hardcoded to `[1,0,0]` and
 never updated, so solar-radiation torque and any future eclipse modeling won't
 track the spacecraft's actual orbital position relative to the Sun over multi-hour
 runs.
3. **Duplicated quaternion math.** The same quaternion multiply/conjugate/rotate
 logic is hand-written independently in both the MATLAB and Python files, with
 no shared library — a correctness fix in one won't propagate to the other.
4. **Unvalidated empirical constants in Material-Sim.** Coefficients like the
 `1e-4` UV damage rate and `1e-28` atomic-oxygen erosion factor aren't tied to a
 cited source; they produce plausible-looking curves but shouldn't be treated as
 validated engineering data without a reference.
5. **No automated tests anywhere.** All three files rely entirely on manual/visual
 inspection of the GUI output to catch regressions.
6. **No shared configuration format.** Each tool has its own bespoke launch dialog
 with independently validated ranges; there's no JSON/YAML config file a user
 could save and re-load, or share between tools.
7. **`plt.show()` inside a Tkinter app (Material-Sim).** Calling the blocking
 pyplot show alongside an active Tkinter mainloop can behave inconsistently
 across platforms/backends.
---
 
## 14. Best Practices Reflected
 
- Physical saturation limits (torque, momentum) enforced at the model level, not
 just documented.
- Dataclasses used for state and configuration objects (`SpacecraftConfig`,
 `SimState`, `Material`, `SpaceEnvironment`), keeping related fields grouped and
 self-documenting via type hints.
- Flight-Sim's lock-protected shared state between the simulation thread and the
 GUI thread is the correct pattern for this kind of live-dashboard architecture.
- Explicit fault/mode state machines (Flight-Sim) rather than scattering
 conditional checks throughout the render code.
- Full-history export (Flight-Sim's per-timestep CSV) rather than only
 end-of-run summaries, which is what actually enables downstream analysis.
---
 
## 15. Recommended Future Improvements
 
1. **Extract a shared attitude-math library.** Consolidate `quatMult`,
 `quatConj`, `quat2rotm`/`euler` conversions into one module (or, across MATLAB
 and Python, one well-documented spec both implementations follow) so a bug fix
 only has to happen once.
2. **Fix Material-Sim's threading model.** Move `SpaceSimulation.run()` onto a
 background thread with a lock-protected state object, mirroring Flight-Sim's
 `SimEngine` pattern, and poll it from the GUI the same way `Dashboard._update`
 does.
3. **Wire material degradation into Flight-Sim.** Expose
 `Material.total_degradation` (or specifically `absorptivity`/`emissivity` drift)
 as an input to `solar_radiation_torque` and to a real power model in
 `FlightController`, replacing the current static `power = 28.0`. This is the
 single highest-value integration between these three tools.
4. **Compute the Sun vector properly.** Even a simple analytic Earth-Sun ephemeris
 would let Flight-Sim model eclipse seasons and let Material-Sim's sunlit/shadow
 toggle be driven by the same orbit rather than a separate sinusoidal proxy.
5. **Add a shared config format.** A common JSON/YAML schema for spacecraft,
 orbit, and environment parameters would let a single mission definition drive
 all three tools and make presets easy to author outside the source code.
6. **Add regression tests for the physics.** Even a handful of property-based
 checks (quaternion stays unit-norm, energy/momentum sanity checks on a
 torque-free case, degradation index monotonically non-decreasing) would catch
 silent numerical bugs that a GUI won't surface.
7. **Cite or replace the empirical degradation constants** in Material-Sim with
 values traceable to published space-materials data (e.g., NASA/ESA material
 outgassing and AO-erosion databases).
---
 
## 16. Appendix — Physical & Control Constants Reference
 
| Constant | Value | Used in | Meaning |
|---|---|---|---|
| μ_Earth | 3.986004418×10¹⁴ m³/s² | Flight-Sim | Earth gravitational parameter |
| R_Earth | 6.371×10⁶ m | Flight-Sim | Earth mean radius |
| J2 | 1.08263×10⁻³ | Flight-Sim | Earth oblateness perturbation coefficient |
| B0_Earth | 3.12×10⁻⁴ T | Flight-Sim | Earth reference magnetic field strength |
| P_Sun | 4.56×10⁻⁶ N/m² | Flight-Sim | Solar radiation pressure at 1 AU |
| Stefan-Boltzmann σ | 5.670374419×10⁻⁸ W/(m²·K⁴) | Material-Sim | Radiative heat balance |
| Solar flux (1 AU) | 1361 W/m² | Material-Sim (LEO/GEO presets) | Incident solar power |
 
---
 

