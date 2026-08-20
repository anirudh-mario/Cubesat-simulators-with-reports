"""
CubeSat Flight & Space Material Degradation Simulator
======================================================
Unified Engineering Toolkit: Flight Dynamics + Space Material Degradation

This unified simulation couples orbital mechanics, quaternion-based attitude
determination and control (ADCS), and multi-hazard space material degradation.
As spacecraft materials degrade under harsh orbital conditions (UV, atomic oxygen,
ionizing radiation, thermal cycling), optical and physical properties drift, directly
impacting Solar Radiation Pressure (SRP) disturbance torques, solar array power output,
thermal balance, and flight software mode transitions.

Key Capabilities:
  • Orbit Propagation: Runge-Kutta 4th-Order (RK4) with Earth J2 oblateness perturbation.
  • Attitude Dynamics & Kinematics: Quaternion Hamilton representation, Euler rigid-body equations.
  • Closed-Loop ADCS: PD reaction-wheel control with torque & momentum saturation.
  • Disturbance Torques: Gravity gradient, aerodynamic drag, residual magnetic dipole, and SRP.
  • Material Degradation Engine: Tracks 5 aerospace materials across LEO, GEO, and Deep Space
    (Thermal cycling, UV darkening, AO erosion, Ionizing radiation, Micrometeorites, Outgassing).
  • Physics Coupling:
      - Material surface degradation -> shifts absorptivity/reflectivity -> alters SRP disturbance torque.
      - Solar cover degradation -> reduces solar power generation -> drops bus voltage -> triggers Safe Mode.
  • Responsive UI: Multi-threaded Tkinter dashboard with 12 telemetry tiles & 6 live-updating plots.
  • Automatic Data Persistence: Full-mission CSV telemetry export + detailed TXT engineering summary.
"""

# Standard library imports
import csv
import os
import threading
import time
import traceback
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Tuple

# Third-party imports
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

# Matplotlib configuration for Tkinter backend
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ═══════════════════════════════════════════════════════════════════════════════
# 1. PHYSICAL & ASTRONOMICAL CONSTANTS
# ═══════════════════════════════════════════════════════════════════════════════
MU_EARTH = 3.986004418e14       # Earth's gravitational parameter [m^3/s^2]
R_EARTH = 6.371e6                # Earth's mean radius [m]
J2 = 1.08263e-3                  # Earth's oblateness coefficient (J2 perturbation)
OMEGA_EARTH = 7.292115e-5       # Earth's rotation rate [rad/s]
B0_EARTH = 3.12e-4               # Earth's surface magnetic field [T]
P_SUN = 4.56e-6                  # Solar radiation pressure at 1 AU [N/m^2]
STEFAN_BOLTZMANN = 5.670374e-8   # Stefan-Boltzmann constant [W/(m^2·K^4)]
G0 = 9.80665                     # Standard gravity [m/s^2]


# ═══════════════════════════════════════════════════════════════════════════════
# 2. QUATERNION & ATTITUDE MATHEMATICAL UTILITIES
# ═══════════════════════════════════════════════════════════════════════════════

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton quaternion product q = q1 ⊗ q2, where q = [w, x, y, z]."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat_conj(q: np.ndarray) -> np.ndarray:
    """Quaternion conjugate q* = [w, -x, -y, -z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Convert quaternion [w, x, y, z] to 3-2-1 Euler angles [Roll, Pitch, Yaw] in degrees."""
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.zeros(3)
    w, x, y, z = q / norm
    roll = np.degrees(np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w*y - z*x), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)))
    return np.array([roll, pitch, yaw])

def euler_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """Convert Euler angles [Roll, Pitch, Yaw] in degrees to unit quaternion [w, x, y, z]."""
    roll, pitch, yaw = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    q = np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])
    return q / np.linalg.norm(q)

def quat_to_rot_matrix(q: np.ndarray) -> np.ndarray:
    """Convert unit quaternion to 3x3 direction cosine rotation matrix (ECI to Body)."""
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ])


# ═══════════════════════════════════════════════════════════════════════════════
# 3. SPACE MATERIAL & ENVIRONMENTAL MODELS
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class Material:
    """Defines physical, optical, mechanical, and space resistance properties of materials."""
    name: str
    thermal_conductivity: float       # [W/(m·K)]
    specific_heat: float              # [J/(kg·K)]
    density: float                    # [kg/m^3]
    melting_point: float              # [K]
    emissivity_init: float            # Initial IR emissivity [0..1]
    absorptivity_init: float          # Initial solar absorptivity [0..1]
    uv_resistance: float              # Resistance to UV photolysis [0..1]
    radiation_resistance: float       # Resistance to ionizing dose [0..1]
    atomic_oxygen_resistance: float   # Resistance to AO erosion in LEO [0..1]
    yield_strength: float             # [MPa]
    young_modulus: float              # [GPa]
    thickness: float = 1e-3           # Sample thickness [m]

    # Dynamic state variables updated during simulation
    temperature: float = 293.15       # [K]
    uv_damage: float = 0.0            # Cumulative damage [0..1]
    radiation_damage: float = 0.0     # Cumulative radiation damage [0..1]
    ao_erosion: float = 0.0           # Cumulative depth eroded [m]
    impact_damage: float = 0.0        # Cumulative impact damage [0..1]
    outgassing_loss: float = 0.0      # Mass lost per unit area [kg/m^2]

    @property
    def mass_per_area(self) -> float:
        return self.density * self.thickness

    @property
    def total_degradation(self) -> float:
        """Composite degradation index: 0.0 (pristine) to 1.0 (failed)."""
        ao_term = self.ao_erosion / max(self.thickness, 1e-6)
        outgas_term = min(self.outgassing_loss / (0.01 * max(self.mass_per_area, 1e-6)), 1.0)
        return float(np.clip(
            0.30 * self.uv_damage +
            0.25 * self.radiation_damage +
            0.20 * ao_term +
            0.15 * self.impact_damage +
            0.10 * outgas_term,
            0.0, 1.0
        ))

    @property
    def structural_integrity(self) -> float:
        """Remaining structural & functional integrity fraction [1.0 -> 0.0]."""
        return max(0.0, 1.0 - self.total_degradation)

    @property
    def current_absorptivity(self) -> float:
        """Degraded solar absorptivity (UV and radiation cause surface darkening)."""
        drift = 0.25 * (self.uv_damage + self.radiation_damage)
        return float(np.clip(self.absorptivity_init + drift, 0.01, 0.98))

    @property
    def current_emissivity(self) -> float:
        """Degraded thermal emissivity."""
        drift = -0.10 * self.uv_damage
        return float(np.clip(self.emissivity_init + drift, 0.02, 0.98))

    @property
    def current_reflectivity(self) -> float:
        """Degraded optical reflectivity rho = 1 - alpha."""
        return float(np.clip(1.0 - self.current_absorptivity, 0.02, 0.99))


def get_material_catalog() -> Dict[str, Material]:
    """Catalog of standard aerospace materials used in CubeSats."""
    return {
        "Aluminum 6061-T6": Material(
            name="Aluminum 6061-T6",
            thermal_conductivity=167.0, specific_heat=896.0, density=2700.0,
            melting_point=925.0, emissivity_init=0.05, absorptivity_init=0.09,
            uv_resistance=0.90, radiation_resistance=0.85, atomic_oxygen_resistance=0.60,
            yield_strength=276.0, young_modulus=68.9
        ),
        "Carbon Fiber / Epoxy": Material(
            name="Carbon Fiber / Epoxy",
            thermal_conductivity=5.0, specific_heat=900.0, density=1600.0,
            melting_point=800.0, emissivity_init=0.85, absorptivity_init=0.92,
            uv_resistance=0.55, radiation_resistance=0.70, atomic_oxygen_resistance=0.20,
            yield_strength=600.0, young_modulus=70.0
        ),
        "Titanium Ti-6Al-4V": Material(
            name="Titanium Ti-6Al-4V",
            thermal_conductivity=6.7, specific_heat=526.0, density=4430.0,
            melting_point=1933.0, emissivity_init=0.10, absorptivity_init=0.40,
            uv_resistance=0.95, radiation_resistance=0.92, atomic_oxygen_resistance=0.80,
            yield_strength=880.0, young_modulus=113.8
        ),
        "Kapton Polyimide Film": Material(
            name="Kapton Polyimide Film",
            thermal_conductivity=0.12, specific_heat=1090.0, density=1420.0,
            melting_point=673.0, emissivity_init=0.86, absorptivity_init=0.30,
            uv_resistance=0.60, radiation_resistance=0.75, atomic_oxygen_resistance=0.10,
            yield_strength=69.0, young_modulus=2.5, thickness=0.05e-3
        ),
        "Fused Silica (Quartz)": Material(
            name="Fused Silica (Quartz)",
            thermal_conductivity=1.38, specific_heat=703.0, density=2203.0,
            melting_point=1983.0, emissivity_init=0.93, absorptivity_init=0.07,
            uv_resistance=0.99, radiation_resistance=0.88, atomic_oxygen_resistance=0.95,
            yield_strength=48.0, young_modulus=73.0, thickness=0.2e-3
        ),
    }


@dataclass
class SpaceEnvironment:
    """Orbital environmental hazard profile."""
    name: str
    orbit_altitude_km: float
    solar_flux: float              # [W/m^2]
    uv_flux: float                 # Relative factor (1.0 = 1 AU Earth orbit)
    radiation_dose_rate: float     # [Gy/day]
    atomic_oxygen_flux: float      # [atoms/(cm^2·s)]
    micrometeorite_flux: float     # [impacts/(m^2·day)]
    temperature_sunlit: float      # [K]
    temperature_shadow: float      # [K]
    vacuum_pressure: float         # [Pa]


ENVIRONMENT_PRESETS: Dict[str, SpaceEnvironment] = {
    "Low Earth Orbit (LEO)": SpaceEnvironment(
        name="Low Earth Orbit (LEO)",
        orbit_altitude_km=400.0,
        solar_flux=1361.0,
        uv_flux=1.0,
        radiation_dose_rate=5.0,
        atomic_oxygen_flux=1.0e15,
        micrometeorite_flux=0.10,
        temperature_sunlit=393.15,
        temperature_shadow=123.15,
        vacuum_pressure=1.0e-6,
    ),
    "Geostationary Orbit (GEO)": SpaceEnvironment(
        name="Geostationary Orbit (GEO)",
        orbit_altitude_km=35786.0,
        solar_flux=1361.0,
        uv_flux=1.0,
        radiation_dose_rate=50.0,
        atomic_oxygen_flux=0.0,
        micrometeorite_flux=0.05,
        temperature_sunlit=423.15,
        temperature_shadow=93.15,
        vacuum_pressure=1.0e-9,
    ),
    "Interplanetary / Deep Space": SpaceEnvironment(
        name="Interplanetary / Deep Space",
        orbit_altitude_km=1.0e6,
        solar_flux=500.0,
        uv_flux=0.5,
        radiation_dose_rate=200.0,
        atomic_oxygen_flux=0.0,
        micrometeorite_flux=0.005,
        temperature_sunlit=350.0,
        temperature_shadow=40.0,
        vacuum_pressure=1.0e-12,
    ),
}


# ═══════════════════════════════════════════════════════════════════════════════
# 4. SPACECRAFT CONFIGURATION & PRESETS
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SpacecraftConfig:
    """Satellite physical, structural, and actuator parameters."""
    name: str = "CubeSat-6U"
    mass: float = 8.0               # [kg]
    Ixx: float = 0.06               # [kg·m^2]
    Iyy: float = 0.06               # [kg·m^2]
    Izz: float = 0.09               # [kg·m^2]
    cross_section: float = 0.06     # [m^2]
    solar_array_area: float = 0.08  # [m^2]
    base_power_w: float = 35.0      # Base electrical power [W]
    drag_coeff: float = 2.2         # Dimensionless drag coefficient
    Isp: float = 220.0              # Specific impulse [s]
    fuel_mass: float = 0.50         # [kg]
    rw_max_torque: float = 0.005    # Reaction wheel max torque [N·m]
    rw_max_momentum: float = 0.010  # Reaction wheel max momentum [N·m·s]
    mag_dipole: float = 0.05        # Residual magnetic dipole [A·m^2]
    surface_material_name: str = "Aluminum 6061-T6"
    solar_cover_material_name: str = "Fused Silica (Quartz)"

    @property
    def inertia(self) -> np.ndarray:
        return np.diag([self.Ixx, self.Iyy, self.Izz])


SPACECRAFT_PRESETS: Dict[str, SpacecraftConfig] = {
    "CubeSat 6U (8 kg)": SpacecraftConfig(
        name="CubeSat-6U", mass=8.0, Ixx=0.06, Iyy=0.06, Izz=0.09,
        cross_section=0.06, solar_array_area=0.08, base_power_w=35.0,
        fuel_mass=0.50, rw_max_torque=0.005, rw_max_momentum=0.010,
        surface_material_name="Aluminum 6061-T6",
        solar_cover_material_name="Fused Silica (Quartz)"
    ),
    "SmallSat 50 kg": SpacecraftConfig(
        name="SmallSat-50", mass=50.0, Ixx=3.5, Iyy=3.5, Izz=5.0,
        cross_section=0.50, solar_array_area=0.60, base_power_w=180.0,
        fuel_mass=5.0, rw_max_torque=0.05, rw_max_momentum=0.10,
        surface_material_name="Carbon Fiber / Epoxy",
        solar_cover_material_name="Fused Silica (Quartz)"
    ),
    "MicroSat 150 kg": SpacecraftConfig(
        name="MicroSat-150", mass=150.0, Ixx=20.0, Iyy=20.0, Izz=30.0,
        cross_section=1.5, solar_array_area=1.8, base_power_w=450.0,
        fuel_mass=20.0, rw_max_torque=0.20, rw_max_momentum=0.50,
        surface_material_name="Titanium Ti-6Al-4V",
        solar_cover_material_name="Fused Silica (Quartz)"
    ),
}


# ═══════════════════════════════════════════════════════════════════════════════
# 5. SENSOR MODELS
# ═══════════════════════════════════════════════════════════════════════════════

class Gyroscope:
    """Tri-axial rate gyro with angle random walk and bias drift."""
    def __init__(self, arw: float = 1e-4, drift_rate: float = 1e-6):
        self.arw = arw
        self.drift_rate = drift_rate
        self.bias = np.zeros(3)
        self.healthy = True

    def measure(self, omega_true: np.ndarray, dt: float) -> np.ndarray:
        if not self.healthy:
            return np.zeros(3)
        self.bias += self.drift_rate * dt * np.random.randn(3)
        noise = self.arw * np.random.randn(3) / np.sqrt(max(dt, 1e-4))
        return omega_true + self.bias + noise


class StarTracker:
    """High-accuracy optical star tracker providing attitude quaternions."""
    def __init__(self, accuracy_arcsec: float = 5.0):
        self.sigma = np.radians(accuracy_arcsec / 3600.0)
        self.healthy = True

    def measure(self, q_true: np.ndarray) -> np.ndarray:
        if not self.healthy:
            return q_true
        noise_axis = np.random.randn(3)
        noise_axis /= (np.linalg.norm(noise_axis) + 1e-12)
        angle = self.sigma * np.random.randn()
        dq = np.array([np.cos(angle / 2.0), *(np.sin(angle / 2.0) * noise_axis)])
        return quat_mult(dq, q_true)


# ═══════════════════════════════════════════════════════════════════════════════
# 6. ORBIT PROPAGATOR (RK4 + J2 PERTURBATION)
# ═══════════════════════════════════════════════════════════════════════════════

class OrbitPropagator:
    """Numerical orbit propagator with Runge-Kutta 4th-order integration and J2 oblateness."""
    def __init__(self, altitude_km: float = 400.0, inclination_deg: float = 51.6,
                 raan_deg: float = 0.0, true_anomaly_deg: float = 0.0):
        self.a = R_EARTH + altitude_km * 1e3
        self.inc = np.radians(inclination_deg)
        self.raan = np.radians(raan_deg)
        self.nu = np.radians(true_anomaly_deg)
        self.e = 0.0
        self.r_vec, self.v_vec = self._keplerian_to_cartesian()
        self.t = 0.0

    def _keplerian_to_cartesian(self) -> Tuple[np.ndarray, np.ndarray]:
        p = self.a * (1.0 - self.e**2)
        r = p / (1.0 + self.e * np.cos(self.nu))
        r_pqw = r * np.array([np.cos(self.nu), np.sin(self.nu), 0.0])
        v_pqw = np.sqrt(MU_EARTH / p) * np.array([-np.sin(self.nu), self.e + np.cos(self.nu), 0.0])
        cW, sW = np.cos(self.raan), np.sin(self.raan)
        ci, si = np.cos(self.inc), np.sin(self.inc)
        R = np.array([
            [cW, -sW*ci,  sW*si],
            [sW,  cW*ci, -cW*si],
            [0,   si,     ci],
        ])
        return R @ r_pqw, R @ v_pqw

    def step(self, dt: float) -> None:
        def deriv(rv: np.ndarray) -> np.ndarray:
            r_v, v_v = rv[:3], rv[3:]
            r = np.linalg.norm(r_v)
            x, y, z = r_v
            fac_j2 = 1.5 * J2 * (R_EARTH / r)**2
            ax = -MU_EARTH / r**3 * x * (1.0 - fac_j2 * (5.0 * z*z / r/r - 1.0))
            ay = -MU_EARTH / r**3 * y * (1.0 - fac_j2 * (5.0 * z*z / r/r - 1.0))
            az = -MU_EARTH / r**3 * z * (1.0 - fac_j2 * (5.0 * z*z / r/r - 3.0))
            return np.array([*v_v, ax, ay, az])

        rv = np.concatenate([self.r_vec, self.v_vec])
        k1 = deriv(rv)
        k2 = deriv(rv + dt/2 * k1)
        k3 = deriv(rv + dt/2 * k2)
        k4 = deriv(rv + dt * k3)
        rv_new = rv + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.r_vec = rv_new[:3]
        self.v_vec = rv_new[3:]
        self.t += dt

    @property
    def altitude_km(self) -> float:
        return (np.linalg.norm(self.r_vec) - R_EARTH) / 1e3

    @property
    def velocity_ms(self) -> float:
        return np.linalg.norm(self.v_vec)

    @property
    def orbital_period(self) -> float:
        return 2.0 * np.pi * np.sqrt(self.a**3 / MU_EARTH)

    @property
    def ground_track(self) -> Tuple[float, float]:
        r = self.r_vec
        lat = np.degrees(np.arcsin(r[2] / np.linalg.norm(r)))
        lon_rad = np.arctan2(r[1], r[0]) - OMEGA_EARTH * self.t
        lon = np.degrees((lon_rad + np.pi) % (2 * np.pi) - np.pi)
        return lat, lon


# ═══════════════════════════════════════════════════════════════════════════════
# 7. COUPLED DISTURBANCE TORQUES & MATERIAL DEGRADATION PHYSICS
# ═══════════════════════════════════════════════════════════════════════════════

def compute_disturbances(q: np.ndarray, r_vec: np.ndarray, v_vec: np.ndarray,
                         sc: SpacecraftConfig, surface_mat: Material,
                         sun_dir_eci: np.ndarray) -> np.ndarray:
    """Calculate the four major environmental disturbance torques with material coupling."""
    r = np.linalg.norm(r_vec)
    r_hat = r_vec / r
    R_eci2b = quat_to_rot_matrix(q)
    c_nadir = R_eci2b @ r_hat

    # 1. Gravity gradient torque
    n2 = MU_EARTH / (r**3)
    I = sc.inertia
    T_gg = 3.0 * n2 * np.cross(c_nadir, I @ c_nadir)

    # 2. Aerodynamic drag torque (LEO altitude dependency)
    alt = r - R_EARTH
    if alt < 800e3:
        rho = 1.225 * np.exp(-alt / 8500.0)
        v = np.linalg.norm(v_vec)
        if v > 1e-3:
            v_b = R_eci2b @ (v_vec / v)
            F_drag = -0.5 * rho * sc.drag_coeff * sc.cross_section * (v**2) * v_b
            arm_drag = np.array([0.02, -0.01, 0.015])
            T_aero = np.cross(arm_drag, F_drag)
        else:
            T_aero = np.zeros(3)
    else:
        T_aero = np.zeros(3)

    # 3. Residual magnetic dipole torque
    B_mag = B0_EARTH * (R_EARTH / r)**3
    B_body = R_eci2b @ (B_mag * np.array([0.0, 0.0, 1.0]))
    m_dipole = np.array([sc.mag_dipole * 0.5, sc.mag_dipole * 0.2, sc.mag_dipole * 0.1])
    T_mag = np.cross(m_dipole, B_body)

    # 4. MATERIAL-COUPLED Solar Radiation Pressure (SRP) torque
    # As surface material degrades (darkens / erodes), reflectivity shifts
    sun_body = R_eci2b @ sun_dir_eci
    refl = surface_mat.current_reflectivity
    F_srp = P_SUN * sc.cross_section * (1.0 + refl) * sun_body
    # Center-of-pressure offset shifts slightly as asymmetric degradation accumulates
    cp_offset = np.array([0.03 + 0.01 * surface_mat.uv_damage, 0.015, -0.01])
    T_srp = np.cross(cp_offset, F_srp)

    return T_gg + T_aero + T_mag + T_srp


def update_material_degradation(mat: Material, env: SpaceEnvironment,
                                sunlit: bool, dt_s: float) -> None:
    """Integrate material degradation mechanisms over time step dt_s [seconds]."""
    dt_days = dt_s / 86400.0

    # 1. Thermal balance & temperature drift
    if sunlit:
        q_in = mat.current_absorptivity * env.solar_flux
        t_target = (q_in / (mat.current_emissivity * STEFAN_BOLTZMANN + 1e-12))**0.25
    else:
        t_target = env.temperature_shadow

    thermal_time_const = max(mat.specific_heat * mat.density * mat.thickness /
                             (4.0 * mat.current_emissivity * STEFAN_BOLTZMANN * (mat.temperature**3) + 1e-6), 1.0)
    mat.temperature += (t_target - mat.temperature) * min(dt_s / thermal_time_const, 1.0)

    # 2. UV photolysis damage (sunlit only)
    if sunlit:
        uv_rate = 5.0e-5 * env.uv_flux * dt_days * (1.0 - mat.uv_resistance)
        mat.uv_damage = float(np.clip(mat.uv_damage + uv_rate * (1.0 - mat.uv_damage), 0.0, 1.0))

    # 3. Ionizing radiation damage
    dose = env.radiation_dose_rate * dt_days
    rad_rate = min((1.0 - mat.radiation_resistance) * dose * 1e-3, 0.05)
    mat.radiation_damage = float(np.clip(mat.radiation_damage + rad_rate * (1.0 - mat.radiation_damage), 0.0, 1.0))

    # 4. Atomic oxygen erosion (LEO only)
    if env.atomic_oxygen_flux > 0:
        ao_sens = 1.0 - mat.atomic_oxygen_resistance
        erosion_m = ao_sens * env.atomic_oxygen_flux * dt_s * 1e4 * 1e-28
        mat.ao_erosion += min(erosion_m, mat.thickness * 0.001)

    # 5. Micrometeorite impact
    impact_prob = env.micrometeorite_flux * dt_days
    n_impacts = np.random.poisson(impact_prob)
    if n_impacts > 0:
        dmg = n_impacts * 1e-3 / (mat.yield_strength / 100.0)
        mat.impact_damage = float(np.clip(mat.impact_damage + dmg, 0.0, 1.0))

    # 6. Vacuum outgassing
    outgas_rate = 1e-9 * (mat.temperature / 300.0) * (-np.log10(env.vacuum_pressure + 1e-20))
    mat.outgassing_loss += outgas_rate * dt_days * mat.mass_per_area


# ═══════════════════════════════════════════════════════════════════════════════
# 8. ADCS ATTITUDE CONTROLLER & FLIGHT SOFTWARE STATE MACHINE
# ═══════════════════════════════════════════════════════════════════════════════

class ADCSController:
    """PD attitude controller with reaction wheel torque and momentum limits."""
    def __init__(self, sc: SpacecraftConfig, Kp: float = 0.04, Kd: float = 0.06):
        self.sc = sc
        self.Kp = Kp
        self.Kd = Kd
        self.rw_momentum = np.zeros(3)

    def compute_torque(self, q_meas: np.ndarray, q_target: np.ndarray,
                       omega_meas: np.ndarray, dt: float) -> np.ndarray:
        q_err = quat_mult(quat_conj(q_target), q_meas)
        if q_err[0] < 0:
            q_err = -q_err
        err_vec = q_err[1:] * np.sign(q_err[0])
        tau_cmd = -self.Kp * err_vec - self.Kd * omega_meas
        tau_sat = np.clip(tau_cmd, -self.sc.rw_max_torque, self.sc.rw_max_torque)
        self.rw_momentum = np.clip(
            self.rw_momentum + tau_sat * dt,
            -self.sc.rw_max_momentum,
            self.sc.rw_max_momentum
        )
        return tau_sat

    @property
    def rw_saturation(self) -> float:
        return float(np.linalg.norm(self.rw_momentum) / (self.sc.rw_max_momentum * np.sqrt(3)))


class FlightController:
    """Mission supervisor: power management, fault monitoring, and mode switching."""
    MODES = ["NOMINAL", "SAFE MODE", "DETUMBLE", "LOW VOLTAGE"]

    def __init__(self, sc: SpacecraftConfig, orbit: OrbitPropagator):
        self.sc = sc
        self.orbit = orbit
        self.mode = "NOMINAL"
        self.fuel = sc.fuel_mass
        self.dv_total = 0.0
        self.faults: List[str] = []
        self.power_gen_w = sc.base_power_w
        self.bus_voltage_v = 28.0

    def update_power_and_faults(self, gyro: Gyroscope, star: StarTracker,
                                solar_cover_mat: Material, att_err_deg: float,
                                rw_sat: float) -> None:
        """Update solar power decay and check health/modes."""
        self.faults = []

        # Power generation coupled with solar cover glass degradation
        solar_efficiency = solar_cover_mat.structural_integrity
        self.power_gen_w = self.sc.base_power_w * solar_efficiency
        # Bus voltage drops when power generation degrades significantly
        self.bus_voltage_v = 28.0 * (0.60 + 0.40 * (self.power_gen_w / self.sc.base_power_w))

        if not gyro.healthy:
            self.faults.append("GYRO FAIL")
        if not star.healthy:
            self.faults.append("STAR TRACKER FAIL")
        if att_err_deg > 15.0:
            self.faults.append(f"ATT ERR {att_err_deg:.1f}°")
        if rw_sat > 0.90:
            self.faults.append("RW SATURATION")
        if self.fuel < 0.05:
            self.faults.append("FUEL LOW")
        if self.bus_voltage_v < 22.0:
            self.faults.append("LOW VOLTAGE")

        # Mode determination
        if self.bus_voltage_v < 22.0:
            self.mode = "LOW VOLTAGE"
        elif "RW SATURATION" in self.faults:
            self.mode = "DETUMBLE"
        elif any("FAIL" in f or "ATT ERR" in f for f in self.faults):
            self.mode = "SAFE MODE"
        else:
            self.mode = "NOMINAL"


# ═══════════════════════════════════════════════════════════════════════════════
# 9. SIMULATION STATE & MULTI-THREADED ENGINE
# ═══════════════════════════════════════════════════════════════════════════════

@dataclass
class SimState:
    """Thread-safe snapshot of spacecraft telemetry and material state."""
    t: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))
    att_err_deg: float = 0.0
    rw_sat: float = 0.0
    torques: np.ndarray = field(default_factory=lambda: np.zeros(3))
    altitude_km: float = 400.0
    velocity_ms: float = 7660.0
    lat: float = 0.0
    lon: float = 0.0
    fuel_kg: float = 0.50
    dv_total: float = 0.0
    mode: str = "NOMINAL"
    faults: List[str] = field(default_factory=list)
    power_w: float = 35.0
    bus_voltage_v: float = 28.0

    # Material telemetry
    mat_temp_k: float = 293.15
    mat_integrity_pct: float = 100.0
    mat_uv_damage: float = 0.0
    mat_rad_damage: float = 0.0
    mat_ao_erosion_um: float = 0.0

    history: Dict[str, List[float]] = field(default_factory=lambda: {
        "t_min": [], "roll": [], "pitch": [], "yaw": [],
        "att_err": [], "rw_sat": [], "alt": [], "vel": [],
        "omega_x": [], "omega_y": [], "omega_z": [],
        "fuel": [], "power_w": [], "voltage_v": [],
        "mat_temp": [], "mat_integrity": [], "mat_uv": [], "mat_rad": [], "mat_ao": [],
    })

    def record(self) -> None:
        h = self.history
        h["t_min"].append(self.t / 60.0)
        h["roll"].append(self.euler[0])
        h["pitch"].append(self.euler[1])
        h["yaw"].append(self.euler[2])
        h["att_err"].append(self.att_err_deg)
        h["rw_sat"].append(self.rw_sat * 100.0)
        h["alt"].append(self.altitude_km)
        h["vel"].append(self.velocity_ms)
        h["omega_x"].append(np.degrees(self.omega[0]))
        h["omega_y"].append(np.degrees(self.omega[1]))
        h["omega_z"].append(np.degrees(self.omega[2]))
        h["fuel"].append(self.fuel_kg)
        h["power_w"].append(self.power_w)
        h["voltage_v"].append(self.bus_voltage_v)
        h["mat_temp"].append(self.mat_temp_k)
        h["mat_integrity"].append(self.mat_integrity_pct)
        h["mat_uv"].append(self.mat_uv_damage)
        h["mat_rad"].append(self.mat_rad_damage)
        h["mat_ao"].append(self.mat_ao_erosion_um)


class SimEngine(threading.Thread):
    """Background simulation thread driving coupled orbit, ADCS, and degradation physics."""
    def __init__(self, sc: SpacecraftConfig, env: SpaceEnvironment,
                 orbit_params: dict, surface_mat: Material, solar_cover_mat: Material,
                 duration_s: float, dt: float = 1.0, target_euler: Tuple[float, float, float] = (0, 0, 0)):
        super().__init__(daemon=True)
        self.sc = sc
        self.env = env
        self.surface_mat = surface_mat
        self.solar_cover_mat = solar_cover_mat
        self.duration = duration_s
        self.dt = dt
        self.target_euler = target_euler
        self.q_target = euler_to_quat(*target_euler)

        self.orbit = OrbitPropagator(**orbit_params)
        self.adcs = ADCSController(sc)
        self.fc = FlightController(sc, self.orbit)
        self.gyro = Gyroscope()
        self.star = StarTracker()

        self.state = SimState()
        self.state.q = euler_to_quat(10.0, -5.0, 15.0)
        self.state.omega = np.radians([0.5, 0.3, -0.2])

        self.sun_dir_eci = np.array([1.0, 0.0, 0.0])
        self.running = False
        self.paused = False
        self._lock = threading.Lock()
        self.error: str = None

    def run(self) -> None:
        try:
            self.running = True
            I = self.sc.inertia
            I_inv = np.linalg.inv(I)

            while self.running and self.state.t < self.duration:
                if self.paused:
                    time.sleep(0.05)
                    continue

                dt = self.dt

                # 1. Orbit Step
                self.orbit.step(dt)

                # 2. Sensors
                q_true = self.state.q / np.linalg.norm(self.state.q)
                omega_m = self.gyro.measure(self.state.omega, dt)
                q_m = self.star.measure(q_true)

                # 3. Environment & Material Degradation Update
                # Sunlit check: LEO periodic eclipse vs GEO/Deep Space
                if self.env.orbit_altitude_km < 2000.0:
                    sunlit = (np.sin(2.0 * np.pi * (self.state.t / self.orbit.orbital_period)) > 0)
                else:
                    sunlit = True

                update_material_degradation(self.surface_mat, self.env, sunlit, dt)
                update_material_degradation(self.solar_cover_mat, self.env, sunlit, dt)

                # 4. Environmental Disturbance Torques (Coupled with Material State)
                T_dist = compute_disturbances(q_true, self.orbit.r_vec, self.orbit.v_vec,
                                             self.sc, self.surface_mat, self.sun_dir_eci)

                # 5. ADCS Control Torque
                T_ctrl = self.adcs.compute_torque(q_m, self.q_target, omega_m, dt)
                T_total = T_ctrl + T_dist

                # 6. Rigid Body Dynamics & Kinematics
                omega = self.state.omega
                alpha = I_inv @ (T_total - np.cross(omega, I @ omega))
                omega_new = omega + alpha * dt

                w_quat = np.array([0.0, *omega_new])
                q_dot = 0.5 * quat_mult(q_true, w_quat)
                q_new = q_true + q_dot * dt
                q_new /= np.linalg.norm(q_new)

                # 7. Attitude Error
                q_err = quat_mult(quat_conj(self.q_target), q_new)
                if q_err[0] < 0:
                    q_err = -q_err
                att_err_deg = 2.0 * np.degrees(np.arccos(np.clip(q_err[0], -1.0, 1.0)))

                # 8. Flight Software & Health Check
                self.fc.update_power_and_faults(self.gyro, self.star, self.solar_cover_mat,
                                                att_err_deg, self.adcs.rw_saturation)

                # 9. Update Shared Thread-Safe State
                with self._lock:
                    self.state.t += dt
                    self.state.q = q_new
                    self.state.omega = omega_new
                    self.state.euler = quat_to_euler(q_new)
                    self.state.att_err_deg = att_err_deg
                    self.state.rw_sat = self.adcs.rw_saturation
                    self.state.altitude_km = self.orbit.altitude_km
                    self.state.velocity_ms = self.orbit.velocity_ms
                    self.state.lat, self.state.lon = self.orbit.ground_track
                    self.state.fuel_kg = self.fc.fuel
                    self.state.dv_total = self.fc.dv_total
                    self.state.mode = self.fc.mode
                    self.state.faults = list(self.fc.faults)
                    self.state.power_w = self.fc.power_gen_w
                    self.state.bus_voltage_v = self.fc.bus_voltage_v

                    # Material telemetry
                    self.state.mat_temp_k = self.surface_mat.temperature
                    self.state.mat_integrity_pct = self.surface_mat.structural_integrity * 100.0
                    self.state.mat_uv_damage = self.surface_mat.uv_damage
                    self.state.mat_rad_damage = self.surface_mat.radiation_damage
                    self.state.mat_ao_erosion_um = self.surface_mat.ao_erosion * 1e6

                    self.state.record()

                time.sleep(max(dt * 0.0005, 0.0001))

            self.running = False

        except Exception as e:
            traceback.print_exc()
            self.error = str(e)
            self.running = False

    def stop(self) -> None:
        self.running = False


# ═══════════════════════════════════════════════════════════════════════════════
# 10. GRAPHICAL USER INTERFACE & TELEMETRY DASHBOARD
# ═══════════════════════════════════════════════════════════════════════════════

# UI Color Palette & Typography
COLOR_BG = "#0A0E17"
COLOR_PANEL = "#111827"
COLOR_BORDER = "#1F2937"
COLOR_ACCENT = "#38BDF8"
COLOR_GREEN = "#34D399"
COLOR_YELLOW = "#FBBF24"
COLOR_RED = "#F87171"
COLOR_TEXT = "#F3F4F6"
COLOR_SUBTEXT = "#9CA3AF"
FONT_TITLE = ("Segoe UI", 13, "bold")
FONT_MONO = ("Courier New", 9)
FONT_BOLD = ("Segoe UI", 9, "bold")
FONT_NORM = ("Segoe UI", 9)


class LaunchDialog(tk.Toplevel):
    """Mission and simulation configuration dialog."""
    def __init__(self, parent):
        super().__init__(parent)
        self.title("CubeSat Mission & Degradation Setup")
        self.configure(bg=COLOR_BG)
        self.resizable(False, False)
        self.result = None

        w, h = 640, 720
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self._build_ui()
        self.grab_set()
        self.wait_window()

    def _build_ui(self):
        hdr = tk.Frame(self, bg=COLOR_ACCENT, height=5)
        hdr.pack(fill="x")

        tk.Label(self, text="🛰️  CUBESAT FLIGHT & MATERIAL SIMULATOR",
                 bg=COLOR_BG, fg=COLOR_ACCENT, font=("Segoe UI", 14, "bold"), pady=10).pack()
        tk.Label(self, text="Configure spacecraft, orbital environment, and surface materials",
                 bg=COLOR_BG, fg=COLOR_SUBTEXT, font=FONT_NORM).pack(pady=(0, 8))

        frm = tk.Frame(self, bg=COLOR_BG, padx=25, pady=5)
        frm.pack(fill="both", expand=True)

        row = 0
        # Spacecraft selection
        tk.Label(frm, text="▸ SPACECRAFT & MATERIALS", bg=COLOR_BG, fg=COLOR_ACCENT, font=FONT_BOLD).grid(row=row, column=0, columnspan=2, sticky="w", pady=(6, 2))
        row += 1
        tk.Label(frm, text="Spacecraft Preset:", bg=COLOR_BG, fg=COLOR_TEXT, font=FONT_NORM).grid(row=row, column=0, sticky="w", pady=3)
        self.sc_var = tk.StringVar(value=list(SPACECRAFT_PRESETS.keys())[0])
        ttk.Combobox(frm, textvariable=self.sc_var, values=list(SPACECRAFT_PRESETS.keys()), state="readonly", width=28).grid(row=row, column=1, sticky="w", padx=8)

        row += 1
        materials = list(get_material_catalog().keys())
        tk.Label(frm, text="Primary Structure Material:", bg=COLOR_BG, fg=COLOR_TEXT, font=FONT_NORM).grid(row=row, column=0, sticky="w", pady=3)
        self.mat_var = tk.StringVar(value=materials[0])
        ttk.Combobox(frm, textvariable=self.mat_var, values=materials, state="readonly", width=28).grid(row=row, column=1, sticky="w", padx=8)

        row += 1
        # Environment selection
        tk.Label(frm, text="▸ SPACE ENVIRONMENT", bg=COLOR_BG, fg=COLOR_ACCENT, font=FONT_BOLD).grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 2))
        row += 1
        tk.Label(frm, text="Orbit Environment:", bg=COLOR_BG, fg=COLOR_TEXT, font=FONT_NORM).grid(row=row, column=0, sticky="w", pady=3)
        self.env_var = tk.StringVar(value=list(ENVIRONMENT_PRESETS.keys())[0])
        ttk.Combobox(frm, textvariable=self.env_var, values=list(ENVIRONMENT_PRESETS.keys()), state="readonly", width=28).grid(row=row, column=1, sticky="w", padx=8)

        row += 1
        # Orbit Parameters
        tk.Label(frm, text="▸ ORBIT PARAMETERS", bg=COLOR_BG, fg=COLOR_ACCENT, font=FONT_BOLD).grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 2))
        row += 1
        self.alt_e = self._entry(frm, "Altitude (km):", "400.0", row)
        row += 1
        self.inc_e = self._entry(frm, "Inclination (°):", "51.6", row)
        row += 1
        self.raan_e = self._entry(frm, "RAAN (°):", "0.0", row)

        row += 1
        # Attitude Target
        tk.Label(frm, text="▸ TARGET ATTITUDE (EULER ANGLES)", bg=COLOR_BG, fg=COLOR_ACCENT, font=FONT_BOLD).grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 2))
        row += 1
        self.tr_e = self._entry(frm, "Target Roll (°):", "0.0", row)
        row += 1
        self.tp_e = self._entry(frm, "Target Pitch (°):", "0.0", row)
        row += 1
        self.ty_e = self._entry(frm, "Target Yaw (°):", "0.0", row)

        row += 1
        # Simulation settings
        tk.Label(frm, text="▸ SIMULATION RUNTIME", bg=COLOR_BG, fg=COLOR_ACCENT, font=FONT_BOLD).grid(row=row, column=0, columnspan=2, sticky="w", pady=(10, 2))
        row += 1
        self.dur_e = self._entry(frm, "Duration (minutes):", "30.0", row)
        row += 1
        self.dt_e = self._entry(frm, "Time-step dt (seconds):", "1.0", row)

        row += 1
        self.err_lbl = tk.Label(frm, text="", bg=COLOR_BG, fg=COLOR_RED, font=FONT_NORM)
        self.err_lbl.grid(row=row, column=0, columnspan=2, pady=6)

        btn_f = tk.Frame(self, bg=COLOR_BG, pady=12, padx=25)
        btn_f.pack(fill="x")
        tk.Button(btn_f, text="Cancel", bg=COLOR_PANEL, fg=COLOR_SUBTEXT, relief="flat", font=FONT_NORM,
                  padx=12, pady=6, cursor="hand2", command=self.destroy).pack(side="left")
        tk.Button(btn_f, text="▶ LAUNCH SIMULATION", bg=COLOR_ACCENT, fg=COLOR_BG, relief="flat",
                  font=("Segoe UI", 10, "bold"), padx=18, pady=8, cursor="hand2", command=self._launch).pack(side="right")

    def _entry(self, parent, label: str, default: str, row: int) -> tk.Entry:
        tk.Label(parent, text=label, bg=COLOR_BG, fg=COLOR_TEXT, font=FONT_NORM).grid(row=row, column=0, sticky="w", pady=2)
        e = tk.Entry(parent, bg=COLOR_PANEL, fg=COLOR_TEXT, insertbackground=COLOR_TEXT,
                     relief="flat", highlightthickness=1, highlightbackground=COLOR_BORDER,
                     highlightcolor=COLOR_ACCENT, font=FONT_MONO, width=16)
        e.insert(0, default)
        e.grid(row=row, column=1, sticky="w", padx=8, pady=2)
        return e

    def _get_val(self, entry: tk.Entry, lo: float, hi: float, name: str) -> float:
        try:
            v = float(entry.get())
            if not (lo <= v <= hi):
                raise ValueError
            return v
        except ValueError:
            raise ValueError(f"{name} must be in range [{lo}, {hi}]")

    def _launch(self):
        try:
            alt = self._get_val(self.alt_e, 100.0, 40000.0, "Altitude")
            inc = self._get_val(self.inc_e, 0.0, 180.0, "Inclination")
            raan = self._get_val(self.raan_e, 0.0, 360.0, "RAAN")
            tr = self._get_val(self.tr_e, -180.0, 180.0, "Target Roll")
            tp = self._get_val(self.tp_e, -90.0, 90.0, "Target Pitch")
            ty = self._get_val(self.ty_e, -180.0, 180.0, "Target Yaw")
            dur = self._get_val(self.dur_e, 1.0, 10080.0, "Duration")
            dt = self._get_val(self.dt_e, 0.1, 60.0, "Time-step")
        except ValueError as e:
            self.err_lbl.config(text=f"✘ {e}")
            return

        catalog = get_material_catalog()
        surface_mat = catalog[self.mat_var.get()]
        solar_cover_mat = catalog["Fused Silica (Quartz)"]

        self.result = {
            "sc": SPACECRAFT_PRESETS[self.sc_var.get()],
            "env": ENVIRONMENT_PRESETS[self.env_var.get()],
            "surface_mat": surface_mat,
            "solar_cover_mat": solar_cover_mat,
            "orbit": {"altitude_km": alt, "inclination_deg": inc, "raan_deg": raan},
            "target": (tr, tp, ty),
            "duration": dur * 60.0,
            "dt": dt,
        }
        self.destroy()


class Dashboard(tk.Frame):
    """Main live telemetry and visualization dashboard."""
    def __init__(self, parent, engine: SimEngine):
        super().__init__(parent, bg=COLOR_BG)
        self.engine = engine
        self._build_ui()
        self._update()

    def _tile(self, parent, label: str, row: int, col: int) -> tk.Label:
        f = tk.Frame(parent, bg=COLOR_PANEL, highlightthickness=1,
                     highlightbackground=COLOR_BORDER, padx=6, pady=4)
        f.grid(row=row, column=col, sticky="nsew", padx=3, pady=3)
        tk.Label(f, text=label, bg=COLOR_PANEL, fg=COLOR_SUBTEXT, font=("Courier New", 7)).pack(anchor="w")
        val = tk.Label(f, text="---", bg=COLOR_PANEL, fg=COLOR_ACCENT, font=("Courier New", 12, "bold"))
        val.pack(anchor="w")
        return val

    def _build_ui(self):
        # Top Bar
        top = tk.Frame(self, bg=COLOR_PANEL, pady=6, padx=14)
        top.pack(fill="x")
        self.mode_lbl = tk.Label(top, text="● NOMINAL", bg=COLOR_PANEL, fg=COLOR_GREEN, font=("Segoe UI", 11, "bold"))
        self.mode_lbl.pack(side="left")

        self.fault_lbl = tk.Label(top, text="", bg=COLOR_PANEL, fg=COLOR_RED, font=FONT_MONO)
        self.fault_lbl.pack(side="left", padx=20)

        self.time_lbl = tk.Label(top, text="T+00:00:00", bg=COLOR_PANEL, fg=COLOR_SUBTEXT, font=FONT_MONO)
        self.time_lbl.pack(side="right")

        # Main Body
        mid = tk.Frame(self, bg=COLOR_BG)
        mid.pack(fill="both", expand=True, padx=8, pady=4)

        # Telemetry Tiles Panel
        tiles = tk.Frame(mid, bg=COLOR_BG, width=240)
        tiles.pack(side="left", fill="y", padx=(0, 6))
        tiles.pack_propagate(False)
        tiles.columnconfigure(0, weight=1)
        tiles.columnconfigure(1, weight=1)

        self.t_roll  = self._tile(tiles, "ROLL (°)",        0, 0)
        self.t_pitch = self._tile(tiles, "PITCH (°)",       0, 1)
        self.t_yaw   = self._tile(tiles, "YAW (°)",         1, 0)
        self.t_err   = self._tile(tiles, "ATT ERROR (°)",   1, 1)
        self.t_alt   = self._tile(tiles, "ALTITUDE (km)",   2, 0)
        self.t_vel   = self._tile(tiles, "VELOCITY (m/s)",  2, 1)
        self.t_rw    = self._tile(tiles, "RW SAT (%)",      3, 0)
        self.t_fuel  = self._tile(tiles, "FUEL (kg)",       3, 1)
        self.t_temp  = self._tile(tiles, "MAT TEMP (K)",    4, 0)
        self.t_integ = self._tile(tiles, "MAT INTEGRITY",   4, 1)
        self.t_volt  = self._tile(tiles, "BUS VOLTAGE (V)", 5, 0)
        self.t_dv    = self._tile(tiles, "ΔV USED (m/s)",   5, 1)

        # Progress Bars
        pb_f1 = tk.Frame(tiles, bg=COLOR_BG)
        pb_f1.grid(row=6, column=0, columnspan=2, sticky="ew", padx=3, pady=(6, 2))
        tk.Label(pb_f1, text="ATTITUDE ERROR", bg=COLOR_BG, fg=COLOR_SUBTEXT, font=("Courier New", 7)).pack(anchor="w")
        self.err_bar = ttk.Progressbar(pb_f1, length=220, mode="determinate", maximum=180)
        self.err_bar.pack(fill="x")

        pb_f2 = tk.Frame(tiles, bg=COLOR_BG)
        pb_f2.grid(row=7, column=0, columnspan=2, sticky="ew", padx=3, pady=(2, 6))
        tk.Label(pb_f2, text="REACTION WHEEL SATURATION", bg=COLOR_BG, fg=COLOR_SUBTEXT, font=("Courier New", 7)).pack(anchor="w")
        self.rw_bar = ttk.Progressbar(pb_f2, length=220, mode="determinate", maximum=100)
        self.rw_bar.pack(fill="x")

        # Action Buttons
        btn_f = tk.Frame(tiles, bg=COLOR_BG)
        btn_f.grid(row=8, column=0, columnspan=2, pady=6)
        self.pause_btn = tk.Button(btn_f, text="⏸ PAUSE", bg=COLOR_YELLOW, fg=COLOR_BG,
                                   relief="flat", font=FONT_BOLD, padx=8, pady=4, cursor="hand2",
                                   command=self._toggle_pause)
        self.pause_btn.pack(side="left", padx=4)
        tk.Button(btn_f, text="⏹ STOP", bg=COLOR_RED, fg="white",
                  relief="flat", font=FONT_BOLD, padx=8, pady=4, cursor="hand2",
                  command=self._stop).pack(side="left", padx=4)

        # Matplotlib Plots Panel
        self._build_plots(mid)

    def _build_plots(self, parent):
        fig = plt.Figure(figsize=(9.5, 6.8), facecolor=COLOR_BG)
        gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.48, wspace=0.32)

        self._axes = {}
        self._lines = {}

        specs = [
            ("euler",    gs[0, 0], "Euler Angles (°)",             ["roll", "pitch", "yaw"],           ["#FF7043", "#42A5F5", "#66BB6A"]),
            ("omega",    gs[0, 1], "Angular Rates (°/s)",          ["omega_x", "omega_y", "omega_z"],   ["#FF7043", "#42A5F5", "#66BB6A"]),
            ("control",  gs[1, 0], "Attitude Error (°) & RW Sat",  ["att_err", "rw_sat"],               [COLOR_YELLOW, COLOR_RED]),
            ("material", gs[1, 1], "Material Degradation Index",   ["mat_uv", "mat_rad", "mat_ao"],     ["#E879F9", "#FB923C", "#38BDF8"]),
            ("thermal",  gs[2, 0], "Material Temperature (K)",     ["mat_temp"],                        [COLOR_ACCENT]),
            ("power",    gs[2, 1], "Bus Power (W) & Voltage (V)",  ["power_w", "voltage_v"],            [COLOR_GREEN, COLOR_YELLOW]),
        ]

        for key, spec, title, series, colors in specs:
            ax = fig.add_subplot(spec)
            ax.set_facecolor(COLOR_PANEL)
            ax.tick_params(colors=COLOR_SUBTEXT, labelsize=7)
            ax.set_title(title, color=COLOR_TEXT, fontsize=8, pad=3, fontweight="bold")
            ax.grid(color=COLOR_BORDER, linewidth=0.5, linestyle="--")
            for spine in ax.spines.values():
                spine.set_edgecolor(COLOR_BORDER)

            self._lines[key] = []
            for s, c in zip(series, colors):
                ln, = ax.plot([], [], color=c, linewidth=1.2, label=s)
                self._lines[key].append((s, ln))

            if len(series) > 1:
                ax.legend(fontsize=6, facecolor=COLOR_PANEL, edgecolor=COLOR_BORDER,
                          labelcolor=COLOR_TEXT, loc="upper right")
            self._axes[key] = ax

        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    def _update(self):
        if self.engine.error:
            self.mode_lbl.config(text="⚠️ SIMULATION CRASHED", fg=COLOR_RED)
            self.fault_lbl.config(text=f"ERR: {self.engine.error}")

        if not self.engine.running and self.engine.state.t == 0:
            self.after(200, self._update)
            return

        with self.engine._lock:
            s = self.engine.state
            e = s.euler

            self.t_roll.config(text=f"{e[0]:+6.2f}")
            self.t_pitch.config(text=f"{e[1]:+6.2f}")
            self.t_yaw.config(text=f"{e[2]:+6.2f}")

            err_col = COLOR_GREEN if s.att_err_deg < 2.0 else (COLOR_YELLOW if s.att_err_deg < 10.0 else COLOR_RED)
            self.t_err.config(text=f"{s.att_err_deg:5.2f}", fg=err_col)
            self.t_alt.config(text=f"{s.altitude_km:6.1f}")
            self.t_vel.config(text=f"{s.velocity_ms:6.1f}")
            self.t_rw.config(text=f"{s.rw_sat*100:5.1f}", fg=COLOR_RED if s.rw_sat > 0.85 else COLOR_ACCENT)
            self.t_fuel.config(text=f"{s.fuel_kg:.3f}")
            self.t_temp.config(text=f"{s.mat_temp_k:5.1f}")
            self.t_integ.config(text=f"{s.mat_integrity_pct:5.1f}%",
                                fg=COLOR_GREEN if s.mat_integrity_pct > 80 else (COLOR_YELLOW if s.mat_integrity_pct > 50 else COLOR_RED))
            self.t_volt.config(text=f"{s.bus_voltage_v:5.2f}",
                               fg=COLOR_GREEN if s.bus_voltage_v > 24 else COLOR_RED)
            self.t_dv.config(text=f"{s.dv_total:5.2f}")

            mode_col = COLOR_GREEN if s.mode == "NOMINAL" else (COLOR_YELLOW if s.mode == "DETUMBLE" else COLOR_RED)
            self.mode_lbl.config(text=f"● {s.mode}", fg=mode_col)
            self.fault_lbl.config(text="  ".join(s.faults))

            sec = int(s.t)
            h, m, sc = sec // 3600, (sec % 3600) // 60, sec % 60
            self.time_lbl.config(text=f"T+{h:02d}:{m:02d}:{sc:02d}")

            self.err_bar["value"] = min(s.att_err_deg, 180)
            self.rw_bar["value"] = min(s.rw_sat * 100, 100)

            hist = s.history
            if len(hist["t_min"]) > 1:
                t_arr = hist["t_min"]
                for key, ax in self._axes.items():
                    for s_name, ln in self._lines[key]:
                        ln.set_data(t_arr, hist[s_name])
                    ax.relim()
                    ax.autoscale_view()
                self._canvas.draw_idle()

        if self.engine.running:
            self.after(300, self._update)
        else:
            if not self.engine.error:
                self.mode_lbl.config(text="● SIMULATION COMPLETE", fg=COLOR_ACCENT)
            self._save_reports()

    def _toggle_pause(self):
        self.engine.paused = not self.engine.paused
        self.pause_btn.config(text="▶ RESUME" if self.engine.paused else "⏸ PAUSE")

    def _stop(self):
        self.engine.stop()

    def _save_reports(self):
        s = self.engine.state
        sc = self.engine.sc
        env = self.engine.env
        mat = self.engine.surface_mat

        report_lines = [
            "=" * 70,
            "  CUBESAT FLIGHT & MATERIAL DEGRADATION SIMULATION REPORT",
            f"  Generated : {datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}",
            "=" * 70,
            f"  Spacecraft Preset    : {sc.name}",
            f"  Total Mass           : {sc.mass} kg",
            f"  Inertia (kg·m^2)     : Ixx={sc.Ixx}, Iyy={sc.Iyy}, Izz={sc.Izz}",
            f"  Surface Material     : {mat.name}",
            f"  Solar Cover Material : {self.engine.solar_cover_mat.name}",
            "-" * 70,
            f"  Environment Profile  : {env.name}",
            f"  Orbit Altitude       : {s.altitude_km:.2f} km",
            f"  Simulated Duration   : {s.t/60.0:.2f} minutes ({s.t/3600.0:.2f} hours)",
            "-" * 70,
            "  END-OF-SIMULATION TELEMETRY & ATTITUDE PERFORMANCE",
            "-" * 70,
            f"  Final Euler Angles   : Roll={s.euler[0]:+.2f}°, Pitch={s.euler[1]:+.2f}°, Yaw={s.euler[2]:+.2f}°",
            f"  Attitude Error       : {s.att_err_deg:.3f}°",
            f"  Reaction Wheel Sat.  : {s.rw_sat*100:.1f}%",
            f"  Delta-V Expended     : {s.dv_total:.3f} m/s",
            f"  Propellant Left      : {s.fuel_kg:.4f} kg",
            f"  Final Flight Mode    : {s.mode}",
            f"  Active Faults        : {', '.join(s.faults) if s.faults else 'None'}",
            "-" * 70,
            "  SPACE MATERIAL DEGRADATION & POWER SUMMARY",
            "-" * 70,
            f"  Material Temperature: {s.mat_temp_k:.1f} K ({s.mat_temp_k - 273.15:+.1f} °C)",
            f"  Structural Integrity: {mat.structural_integrity*100:.2f}%",
            f"  Total Degradation    : {mat.total_degradation*100:.2f}%",
            f"  UV Solar Damage      : {mat.uv_damage:.4f}",
            f"  Radiation Damage     : {mat.radiation_damage:.4f}",
            f"  AO Surface Erosion   : {mat.ao_erosion*1e6:.3f} µm",
            f"  Outgassing Loss      : {mat.outgassing_loss:.4e} kg/m^2",
            f"  Final Solar Power    : {s.power_w:.2f} W (Base: {sc.base_power_w:.1f} W)",
            f"  Final Bus Voltage    : {s.bus_voltage_v:.2f} V",
            "=" * 70,
        ]

        report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "flight_material_report.txt")
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("\n".join(report_lines))
        print(f"\n✅ Summary report saved to: {report_path}")

        # Save CSV telemetry
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "telemetry_data.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(s.history.keys())
            writer.writerows(zip(*s.history.values()))
        print(f"✅ CSV Telemetry saved to: {csv_path}")


class App:
    """Main Application Entry Point."""
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("CubeSat Flight & Space Material Simulation Toolkit")
        self.root.configure(bg=COLOR_BG)

        w, h = 1280, 780
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self._splash()
        self.root.mainloop()

    def _splash(self):
        for widget in self.root.winfo_children():
            widget.destroy()

        splash = tk.Frame(self.root, bg=COLOR_BG)
        splash.place(relx=0.5, rely=0.5, anchor="center")

        tk.Label(splash, text="🛰️ CUBESAT ENGINEERING SIMULATOR",
                 bg=COLOR_BG, fg=COLOR_ACCENT, font=("Segoe UI", 22, "bold")).pack()
        tk.Label(splash, text="Integrated Flight Dynamics & Space Material Degradation Toolkit",
                 bg=COLOR_BG, fg=COLOR_TEXT, font=("Segoe UI", 13)).pack(pady=4)
        tk.Label(splash, text="Coupled ADCS • Multi-Hazard Environmental Degradation • Live Telemetry Logging",
                 bg=COLOR_BG, fg=COLOR_SUBTEXT, font=FONT_NORM).pack(pady=8)

        tk.Button(splash, text="⚙ CONFIGURE & LAUNCH MISSION",
                  bg=COLOR_ACCENT, fg=COLOR_BG, relief="flat",
                  font=("Segoe UI", 12, "bold"), padx=22, pady=10, cursor="hand2",
                  command=self._open_config).pack(pady=20)

    def _open_config(self):
        dlg = LaunchDialog(self.root)
        if dlg.result is None:
            return

        cfg = dlg.result
        engine = SimEngine(
            sc=cfg["sc"],
            env=cfg["env"],
            orbit_params=cfg["orbit"],
            surface_mat=cfg["surface_mat"],
            solar_cover_mat=cfg["solar_cover_mat"],
            duration_s=cfg["duration"],
            dt=cfg["dt"],
            target_euler=cfg["target"],
        )

        for widget in self.root.winfo_children():
            widget.destroy()

        dash = Dashboard(self.root, engine)
        dash.pack(fill="both", expand=True)
        engine.start()


if __name__ == "__main__":
    App()
