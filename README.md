# CubeSat Simulators with Reports

A multi-module CubeSat simulation project developed using Python and MATLAB. The project provides software-based simulations for spacecraft flight behavior, space-environment material degradation, and attitude determination and control.

The simulators are designed to provide visualized simulation results and generated reports that can be used to study different aspects of CubeSat operation and design.

# 1. Program Requirements

The project uses Python for the Flight Simulator and Material Simulator, while the ADCS Simulator is developed using MATLAB.

Python Requirements
Python 3
NumPy
Matplotlib
Tkinter
SciPy, where required by the Flight Simulator

Install the Python libraries using:

pip install numpy matplotlib scipy

Tkinter is included with most Windows Python installations. On Linux, it may need to be installed separately.

MATLAB Requirements
MATLAB
Base MATLAB functionality

The current ADCS simulator is designed to run using base MATLAB without additional toolboxes.

# 2. About the Project

CubeSat Simulators with Reports is a collection of simulation programs designed to study different aspects of CubeSat systems through software.

The project consists of three main simulators:

                    CubeSat Simulators with Reports
                               │
          ┌────────────────────┼────────────────────┐
          │                    │                    │
          ▼                    ▼                    ▼
   Flight Simulator     Material Simulator    ADCS Simulator
       Python                Python                MATLAB
          │                    │                    │
          ▼                    ▼                    ▼
   Flight & Spacecraft    Environmental &       Attitude &
       Behavior             Material Effects      Control
          │                    │                    │
          └────────────────────┼────────────────────┘
                               ▼
                     Simulation Results
                               │
                               ▼
                        Reports / Graphs
Flight Simulator

The Python Flight Simulator provides a virtual environment for simulating CubeSat flight-related behavior.

It incorporates spacecraft and orbital simulation concepts along with attitude dynamics, sensors, disturbances, control, telemetry, and visualization.

The simulator allows spacecraft behavior to be observed without requiring physical flight hardware.

Material Simulator

The Python Material Simulator studies material degradation under simulated space-environment conditions.

The simulation includes:

Thermal cycling
Solar and UV radiation
High-energy radiation
Atomic oxygen erosion
Micrometeorite impacts
Vacuum outgassing

It provides predefined environments for:

Low Earth Orbit (LEO)
Geostationary Orbit (GEO)
Interplanetary / Deep Space

The simulator compares materials such as:

Aluminum 6061-T6
Carbon Fiber / Epoxy
Titanium Ti-6Al-4V
Kapton Polyimide Film
Fused Silica

The resulting material degradation and structural integrity are visualized and included in a generated simulation report.

ADCS Simulator

The MATLAB simulator focuses on the Attitude Determination and Control System (ADCS) of a 3U CubeSat.

The simulation models:

CubeSat dimensions and mass
Moment of inertia
Quaternion-based attitude
Angular velocity
Rigid-body rotational dynamics
PD attitude control
Reaction-wheel torque
Actuator torque saturation

The simulator provides an interactive interface where the user can change the desired:

Yaw
Pitch
Roll

while the simulation is running.

A disturbance can also be introduced to simulate a tumble, allowing the controller's recovery behavior to be observed.

# 3. How the Simulators Are Developed

The project combines mathematical modeling, numerical simulation, control algorithms, visualization, and report generation.

Flight Simulation

The Flight Simulator is implemented in Python and combines multiple spacecraft-related models.

The simulation processes the spacecraft state and environmental conditions, updates the spacecraft behavior, and produces telemetry and visualization.

The general process is:

Input Parameters
      │
      ▼
Initialize Spacecraft
      │
      ▼
Initialize Orbital & Attitude State
      │
      ▼
Calculate Spacecraft Environment
      │
      ▼
Update Orbit and Attitude
      │
      ▼
Apply Sensors / Disturbances / Control
      │
      ▼
Update Telemetry
      │
      ▼
Visualize Simulation
      │
      ▼
Generate Results
Material Simulation

The Material Simulator represents each material using physical and degradation-related properties.

Examples include:

Thermal Conductivity
Specific Heat
Density
Melting Point
Emissivity
Solar Absorptivity
UV Resistance
Radiation Resistance
Atomic Oxygen Resistance
Yield Strength
Young's Modulus

The selected space environment supplies conditions such as:

Solar Flux
UV Flux
Radiation Dose Rate
Atomic Oxygen Flux
Micrometeorite Flux
Sunlit Temperature
Shadow Temperature
Vacuum Pressure

During the simulation, these conditions are used to calculate different degradation mechanisms.

The simulator maintains a degradation state for each material and calculates a combined total degradation and structural integrity.

The results are then plotted and a text-based simulation report is generated.

ADCS Simulation

The ADCS simulator represents the CubeSat's orientation using a quaternion.

The target Yaw, Pitch, and Roll values are converted into a target quaternion.

The difference between the current and target quaternion is then used by the PD controller.

Current Attitude
       │
       ▼
Quaternion Representation
       │
       ▼
Quaternion Error
       │
       ▼
PD Controller
       │
       ▼
Control Torque
       │
       ▼
Torque Saturation
       │
       ▼
Rigid-Body Dynamics
       │
       ▼
Angular Velocity
       │
       ▼
Quaternion Update
       │
       └──────────► Feedback

The simulator continuously updates the CubeSat model and telemetry.

The 3D visualization shows both the current spacecraft orientation and the commanded target orientation.

# 4. Simulation Plan & Workflow

The project follows a modular approach where each simulator focuses on a particular aspect of CubeSat development.

The general workflow begins with defining the spacecraft or simulation conditions, performing numerical calculations, updating the simulated state, and finally presenting the results through graphs, telemetry, or reports.

Overall Project Workflow
                    ┌────────────────────────┐
                    │       Start Project    │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Select Simulation       │
                    │                        │
                    │ Flight / Material /    │
                    │ ADCS                   │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Configure Parameters    │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Run Mathematical Model  │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Update Simulation State │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Collect Simulation Data │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Visualize Results       │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │ Generate Reports        │
                    └────────────┬───────────┘
                                 │
                                 ▼
                    ┌────────────────────────┐
                    │         Results         │
                    └────────────────────────┘
Flight Simulator Workflow
        Start
          │
          ▼
 Initialize Spacecraft
          │
          ▼
 Initialize Orbit & State
          │
          ▼
 Calculate Environment
          │
          ▼
 Update Spacecraft State
          │
     ┌────┴─────┐
     ▼          ▼
   Orbit     Attitude
   Model      Model
     │          │
     └────┬─────┘
          ▼
     Sensor Models
          │
          ▼
   Control Algorithms
          │
          ▼
   Update Telemetry
          │
          ▼
 Visualize Simulation
          │
          ▼
   Continue Running
          │
          └──────────► Repeat
Material Simulation Workflow
             Start
               │
               ▼
      Select Environment
               │
               ▼
      Set Mission Duration
               │
               ▼
       Initialize Materials
               │
               ▼
      Calculate Space Effects
               │
      ┌────────┼─────────┐
      ▼        ▼         ▼
   Thermal     UV     Radiation
      │        │         │
      └────────┼─────────┘
               │
        ┌──────┴───────┐
        ▼              ▼
 Atomic Oxygen    Micrometeorites
        │              │
        └──────┬───────┘
               ▼
          Outgassing
               │
               ▼
      Update Material State
               │
               ▼
    Calculate Degradation
               │
               ▼
    Calculate Structural
           Integrity
               │
               ▼
       Store Simulation Data
               │
               ▼
        Generate Graphs
               │
               ▼
        Generate Report
               │
               ▼
              End
ADCS Simulation Workflow
                 Start
                   │
                   ▼
          Initialize CubeSat
                   │
                   ▼
          Set Initial Attitude
                   │
                   ▼
          Set Target Attitude
                   │
                   ▼
       Read Yaw / Pitch / Roll
                   │
                   ▼
        Calculate Quaternion
              Error
                   │
                   ▼
           PD Controller
                   │
                   ▼
        Apply Torque Limit
                   │
                   ▼
       Rigid-Body Dynamics
                   │
                   ▼
        Update Angular Rate
                   │
                   ▼
         Update Quaternion
                   │
                   ▼
       Update 3D Visualization
                   │
                   ▼
        Update Telemetry
                   │
                   ▼
        Calculate Error
                   │
                   ▼
          Target Reached?
             /        \
           No          Yes
           │            │
           └─────┬──────┘
                 ▼
          Continue Loop
# 5. Expected Outcomes

The main outcome of the project is a software environment in which different CubeSat-related systems can be simulated and analyzed.

The Flight Simulator is expected to provide a virtual representation of spacecraft flight behavior, system states, telemetry, and simulation responses.

The Material Simulator is expected to show how different materials respond to simulated space environments over a selected mission duration. It provides degradation curves, structural-integrity results, material comparisons, and a generated simulation report.

The ADCS Simulator is expected to demonstrate closed-loop attitude control. A user can change the commanded orientation and observe the CubeSat attempt to reach the new target. A simulated tumble disturbance can also be introduced to observe the controller's recovery behavior.

The project can produce outputs such as:

Simulation graphs
Telemetry data
Attitude response
Angular velocity response
Attitude-error plots
Material degradation curves
Structural-integrity comparisons
Space-environment analysis
Simulation reports

These results can be used for understanding spacecraft behavior, comparing design choices, studying control algorithms, and identifying areas that could later be tested using physical hardware.

The simulations are intended as engineering and educational models. Their results depend on the assumptions, parameters, and simplified models implemented in the respective simulators and should not be treated as exact predictions of an actual spacecraft mission.

# 6. Conclusion

CubeSat Simulators with Reports provides a modular software-based approach to studying important aspects of CubeSat design and operation.

The project combines a Python Flight Simulator, a Python Space Material Degradation Simulator, and a MATLAB ADCS Simulator. Each component focuses on a different part of the spacecraft system while providing visual or report-based results.

The Material Simulator demonstrates the effects of space-like environmental conditions on candidate materials. The Flight Simulator provides a virtual environment for studying spacecraft flight behavior and telemetry. The ADCS Simulator demonstrates attitude dynamics and closed-loop control using quaternion-based representation and PD control.

Together, these simulators provide a foundation for understanding spacecraft systems before moving toward physical implementation.

Future development can expand the project by increasing the fidelity of the physical models, integrating the individual simulators, adding additional CubeSat subsystems, improving visualization, and eventually connecting the simulation environment with hardware-in-the-loop testing.

The overall development path can therefore progress from:

Simulation
     ↓
Analysis
     ↓
Algorithm Development
     ↓
Testing
     ↓
Hardware Integration
     ↓
CubeSat System Development
