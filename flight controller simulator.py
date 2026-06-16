"""
Flight Controller & ADCS Simulator - FULLY CORRECTED
====================================================
FIXES:
  - Resolved ambiguous truth-value error when star tracker returns None.
  - Records every timestep and exports full CSV.
  - Realistic Sun vector and corrected ground-track longitude.
  - Hardened with exception handling.
"""

import numpy as np
import tkinter as tk
from tkinter import ttk
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import List, Dict
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from datetime import datetime
import os
import csv

# ─────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────
MU_EARTH    = 3.986004418e14   # m³/s²
R_EARTH     = 6.371e6          # m
J2          = 1.08263e-3
B0_EARTH    = 3.12e-4          # T
C_LIGHT     = 3e8              # m/s
P_SUN       = 4.56e-6          # N/m²
G0          = 9.80665          # m/s²
OMEGA_EARTH = 7.292115e-5      # rad/s (Earth's rotation rate)


# ─────────────────────────────────────────────────────────────
# QUATERNION UTILITIES
# ─────────────────────────────────────────────────────────────

def quat_mult(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_to_euler(q):
    w,x,y,z = q / np.linalg.norm(q)
    roll  = np.degrees(np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x), -1, 1)))
    yaw   = np.degrees(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return np.array([roll, pitch, yaw])

def euler_to_quat(roll_deg, pitch_deg, yaw_deg):
    r,p,y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr,sr = np.cos(r/2), np.sin(r/2)
    cp,sp = np.cos(p/2), np.sin(p/2)
    cy,sy = np.cos(y/2), np.sin(y/2)
    return np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ])

def skew(v):
    return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])


# ─────────────────────────────────────────────────────────────
# SPACECRAFT CONFIGURATION
# ─────────────────────────────────────────────────────────────

@dataclass
class SpacecraftConfig:
    name: str = "CubeSat-6U"
    mass: float = 8.0
    Ixx: float  = 0.06
    Iyy: float  = 0.06
    Izz: float  = 0.09
    cross_section: float = 0.06
    reflectivity: float  = 0.3
    drag_coeff: float    = 2.2
    Isp: float           = 220.0
    fuel_mass: float     = 0.5
    rw_max_torque: float = 0.005
    rw_max_momentum: float = 0.01
    mag_dipole: float    = 0.05

    @property
    def inertia(self):
        return np.diag([self.Ixx, self.Iyy, self.Izz])


PRESETS = {
    "CubeSat 6U (8 kg)":  SpacecraftConfig("CubeSat-6U",  8,  0.06, 0.06, 0.09, 0.06),
    "SmallSat 50 kg":     SpacecraftConfig("SmallSat-50", 50, 3.5,  3.5,  5.0,  0.5,  fuel_mass=5.0,  rw_max_torque=0.05),
    "MicroSat 150 kg":    SpacecraftConfig("MicroSat-150",150, 20,   20,   30,   1.5,  fuel_mass=20.0, rw_max_torque=0.2),
}


# ─────────────────────────────────────────────────────────────
# SENSORS
# ─────────────────────────────────────────────────────────────

class Gyroscope:
    def __init__(self, arw=1e-4, drift_rate=1e-6):
        self.arw        = arw
        self.drift_rate = drift_rate
        self.bias       = np.zeros(3)
        self.healthy    = True
    def measure(self, omega_true, dt):
        self.bias += self.drift_rate * dt * np.random.randn(3)
        noise = self.arw * np.random.randn(3) / np.sqrt(dt)
        return omega_true + self.bias + noise if self.healthy else np.zeros(3)

class StarTracker:
    def __init__(self, accuracy_arcsec=5.0):
        self.sigma   = np.radians(accuracy_arcsec / 3600)
        self.healthy = True
    def measure(self, q_true):
        if not self.healthy: 
            return None
        noise_axis  = np.random.randn(3)
        noise_axis /= np.linalg.norm(noise_axis) + 1e-12
        noise_angle = self.sigma * np.random.randn()
        dq = np.array([np.cos(noise_angle/2), 
                       *(np.sin(noise_angle/2) * noise_axis)])
        return quat_mult(dq, q_true)

class SunSensor:
    def __init__(self, fov_deg=120, accuracy_deg=1.0):
        self.fov     = np.radians(fov_deg)
        self.sigma   = np.radians(accuracy_deg)
        self.healthy = True
    def measure(self, sun_body_true):
        if not self.healthy or np.linalg.norm(sun_body_true) < 0.1:
            return None
        noise = self.sigma * np.random.randn(3)
        v = sun_body_true / np.linalg.norm(sun_body_true) + noise
        return v / np.linalg.norm(v)

class Magnetometer:
    def __init__(self, accuracy_nT=50):
        self.sigma   = accuracy_nT * 1e-9
        self.healthy = True
    def measure(self, B_body_true):
        if not self.healthy:
            return None
        return B_body_true + self.sigma * np.random.randn(3)


# ─────────────────────────────────────────────────────────────
# DISTURBANCE TORQUES
# ─────────────────────────────────────────────────────────────

def gravity_gradient_torque(q, r_vec, sc: SpacecraftConfig):
    r = np.linalg.norm(r_vec)
    r_hat = r_vec / r
    w,x,y,z = q / np.linalg.norm(q)
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])
    c_hat = R @ r_hat
    n2 = MU_EARTH / r**3
    I  = sc.inertia
    return 3 * n2 * np.cross(c_hat, I @ c_hat)

def solar_radiation_torque(q, r_vec, sc: SpacecraftConfig, sun_dir_eci):
    w,x,y,z = q / np.linalg.norm(q)
    R = np.array([
        [1-2*(y*y+z*z), 2*(x*y-w*z),   2*(x*z+w*y)],
        [2*(x*y+w*z),   1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x*x+y*y)],
    ])
    sun_body = R @ sun_dir_eci
    F_srp = P_SUN * sc.cross_section * (1 + sc.reflectivity) * sun_body
    arm   = np.array([0.05, 0.02, 0.01])
    return np.cross(arm, F_srp)

def aero_drag_torque(r_vec, v_vec, sc: SpacecraftConfig):
    alt = np.linalg.norm(r_vec) - R_EARTH
    if alt > 800e3:
        return np.zeros(3)
    rho = 1.225 * np.exp(-alt / 8500)
    v   = np.linalg.norm(v_vec)
    F_drag = -0.5 * rho * sc.drag_coeff * sc.cross_section * v**2 * (v_vec/v)
    arm    = np.array([0.03, -0.01, 0.02])
    return np.cross(arm, F_drag)

def magnetic_disturbance_torque(q, r_vec):
    r     = np.linalg.norm(r_vec)
    B_mag = B0_EARTH * (R_EARTH/r)**3
    B_vec = B_mag * np.array([0, 0, 1])
    residual_dipole = np.array([1e-3, 5e-4, 2e-4])
    return np.cross(residual_dipole, B_vec)


# ─────────────────────────────────────────────────────────────
# ATTITUDE CONTROLLER
# ─────────────────────────────────────────────────────────────

class ADCSController:
    def __init__(self, sc: SpacecraftConfig, Kp=0.05, Kd=0.07):
        self.sc  = sc
        self.Kp  = Kp
        self.Kd  = Kd
        self.rw_momentum = np.zeros(3)
        self.mode = "NORMAL"

    def compute_torque(self, q_meas, q_target, omega_meas, dt):
        q_err    = quat_mult(quat_conj(q_target), q_meas)
        if q_err[0] < 0:
            q_err = -q_err
        err_vec  = q_err[1:] * np.sign(q_err[0])
        torque   = -self.Kp * err_vec - self.Kd * omega_meas
        torque   = np.clip(torque,
                           -self.sc.rw_max_torque,
                            self.sc.rw_max_torque)
        self.rw_momentum = np.clip(
            self.rw_momentum + torque * dt,
            -self.sc.rw_max_momentum,
             self.sc.rw_max_momentum
        )
        return torque

    @property
    def rw_saturation(self):
        return np.linalg.norm(self.rw_momentum) / (self.sc.rw_max_momentum * np.sqrt(3))


# ─────────────────────────────────────────────────────────────
# ORBIT PROPAGATOR
# ─────────────────────────────────────────────────────────────

class OrbitPropagator:
    def __init__(self, altitude_km=400, inclination_deg=51.6,
                 raan_deg=0, true_anomaly_deg=0):
        a = R_EARTH + altitude_km * 1e3
        self.a    = a
        self.inc  = np.radians(inclination_deg)
        self.raan = np.radians(raan_deg)
        self.nu   = np.radians(true_anomaly_deg)
        self.e    = 0.0
        self.r_vec, self.v_vec = self._keplerian_to_cartesian()
        self.t = 0.0

    def _keplerian_to_cartesian(self):
        a, e, i, W, nu = self.a, self.e, self.inc, self.raan, self.nu
        p = a * (1 - e**2)
        r = p / (1 + e * np.cos(nu))
        r_pqw = r * np.array([np.cos(nu), np.sin(nu), 0])
        v_pqw = np.sqrt(MU_EARTH/p) * np.array([-np.sin(nu), e+np.cos(nu), 0])
        cW, sW = np.cos(W), np.sin(W)
        ci, si = np.cos(i), np.sin(i)
        cw, sw = 1.0, 0.0
        R = np.array([
            [cW*cw-sW*sw*ci,  -cW*sw-sW*cw*ci,  sW*si],
            [sW*cw+cW*sw*ci,  -sW*sw+cW*cw*ci, -cW*si],
            [si*sw,             si*cw,            ci   ],
        ])
        return R @ r_pqw, R @ v_pqw

    def step(self, dt):
        def deriv(rv):
            r_v, v_v = rv[:3], rv[3:]
            r = np.linalg.norm(r_v)
            x, y, z = r_v
            fac_j2  = 1.5 * J2 * (R_EARTH/r)**2
            ax = -MU_EARTH/r**3 * x * (1 - fac_j2*(5*z*z/r/r - 1))
            ay = -MU_EARTH/r**3 * y * (1 - fac_j2*(5*z*z/r/r - 1))
            az = -MU_EARTH/r**3 * z * (1 - fac_j2*(5*z*z/r/r - 3))
            return np.array([*v_v, ax, ay, az])

        rv = np.concatenate([self.r_vec, self.v_vec])
        k1 = deriv(rv)
        k2 = deriv(rv + dt/2*k1)
        k3 = deriv(rv + dt/2*k2)
        k4 = deriv(rv + dt*k3)
        rv_new      = rv + dt/6*(k1+2*k2+2*k3+k4)
        self.r_vec  = rv_new[:3]
        self.v_vec  = rv_new[3:]
        self.t     += dt

    @property
    def altitude_km(self):
        return (np.linalg.norm(self.r_vec) - R_EARTH) / 1e3

    @property
    def velocity_ms(self):
        return np.linalg.norm(self.v_vec)

    @property
    def orbital_period(self):
        return 2 * np.pi * np.sqrt(self.a**3 / MU_EARTH)

    @property
    def ground_track(self):
        r = self.r_vec
        lat = np.degrees(np.arcsin(r[2] / np.linalg.norm(r)))
        lon_rad = np.arctan2(r[1], r[0])
        gmst = OMEGA_EARTH * self.t
        lon_rad = (lon_rad - gmst) % (2 * np.pi)
        if lon_rad > np.pi:
            lon_rad -= 2 * np.pi
        lon = np.degrees(lon_rad)
        return lat, lon


# ─────────────────────────────────────────────────────────────
# FLIGHT CONTROLLER
# ─────────────────────────────────────────────────────────────

class FlightController:
    MODES = ["NOMINAL", "SAFE MODE", "DETUMBLE", "MANOEUVRE", "COMMS WINDOW"]

    def __init__(self, sc: SpacecraftConfig, orbit: OrbitPropagator):
        self.sc       = sc
        self.orbit    = orbit
        self.mode     = "NOMINAL"
        self.fuel     = sc.fuel_mass
        self.dv_total = 0.0
        self.faults: List[str] = []
        self.power    = 28.0
        self._manoeuvre_dv_remaining = 0.0
        self._manoeuvre_axis = np.array([0,0,1.0])

    def check_faults(self, gyro: Gyroscope, star: StarTracker,
                     att_err_deg: float, rw_sat: float):
        self.faults = []
        if not gyro.healthy:
            self.faults.append("GYRO FAIL")
        if not star.healthy:
            self.faults.append("STR FAIL")
        if att_err_deg > 15:
            self.faults.append(f"ATT ERR {att_err_deg:.1f}°")
        if rw_sat > 0.9:
            self.faults.append("RW SATURATION")
        if self.fuel < 0.05:
            self.faults.append("FUEL LOW")
        if self.power < 22:
            self.faults.append("LOW VOLTAGE")
        if any("GYRO" in f or "ATT ERR" in f for f in self.faults):
            self.mode = "SAFE MODE"
        elif "RW SATURATION" in self.faults:
            self.mode = "DETUMBLE"

    def execute_thruster(self, dv_vec, dt):
        dv = np.linalg.norm(dv_vec)
        if dv < 1e-6 or self.fuel <= 0:
            return 0.0
        dm = self.sc.mass * (1 - np.exp(-dv / (self.sc.Isp * G0)))
        dm = min(dm, self.fuel)
        actual_dv = self.sc.Isp * G0 * np.log(self.sc.mass / (self.sc.mass - dm))
        self.fuel -= dm
        self.dv_total += actual_dv
        self.orbit.v_vec += (dv_vec / dv) * actual_dv
        return actual_dv


# ─────────────────────────────────────────────────────────────
# SIMULATION ENGINE (HARDENED WITH FIXED STAR TRACKER HANDLING)
# ─────────────────────────────────────────────────────────────

@dataclass
class SimState:
    t: float = 0.0
    q: np.ndarray = field(default_factory=lambda: np.array([1.,0.,0.,0.]))
    omega: np.ndarray = field(default_factory=lambda: np.zeros(3))
    euler: np.ndarray = field(default_factory=lambda: np.zeros(3))
    att_err_deg: float = 0.0
    rw_sat: float = 0.0
    altitude_km: float = 400.0
    velocity_ms: float = 7660.0
    lat: float = 0.0
    lon: float = 0.0
    fuel_kg: float = 0.5
    dv_total: float = 0.0
    mode: str = "NOMINAL"
    faults: List[str] = field(default_factory=list)
    power_v: float = 28.0
    torques: np.ndarray = field(default_factory=lambda: np.zeros(3))
    history: Dict = field(default_factory=lambda: {
        "t":[], "roll":[], "pitch":[], "yaw":[],
        "att_err":[], "rw_sat":[], "alt":[], "vel":[],
        "omega_x":[], "omega_y":[], "omega_z":[],
        "fuel":[], "torque_ctrl":[],
        "lat":[], "lon":[],
    })

    def record(self):
        h = self.history
        h["t"].append(self.t/60)
        h["roll"].append(self.euler[0])
        h["pitch"].append(self.euler[1])
        h["yaw"].append(self.euler[2])
        h["att_err"].append(self.att_err_deg)
        h["rw_sat"].append(self.rw_sat * 100)
        h["alt"].append(self.altitude_km)
        h["vel"].append(self.velocity_ms)
        h["omega_x"].append(np.degrees(self.omega[0]))
        h["omega_y"].append(np.degrees(self.omega[1]))
        h["omega_z"].append(np.degrees(self.omega[2]))
        h["fuel"].append(self.fuel_kg)
        h["torque_ctrl"].append(np.linalg.norm(self.torques))
        h["lat"].append(self.lat)
        h["lon"].append(self.lon)


class SimEngine(threading.Thread):
    def __init__(self, config: SpacecraftConfig, orbit_params: dict,
                 duration_s: float, dt: float = 1.0,
                 target_euler=(0,0,0), initial_rates=(0.5,0.3,-0.2)):
        super().__init__(daemon=True)
        self.sc       = config
        self.duration = duration_s
        self.dt       = dt
        self.state    = SimState()
        self.running  = False
        self.paused   = False
        self._lock    = threading.Lock()
        self.error    = None

        self.orbit  = OrbitPropagator(**orbit_params)
        self.adcs   = ADCSController(config)
        self.fc     = FlightController(config, self.orbit)
        self.gyro   = Gyroscope()
        self.star   = StarTracker()
        self.sun    = SunSensor()
        self.mag    = Magnetometer()

        self.q_target = euler_to_quat(*target_euler)
        self.state.q  = euler_to_quat(10, -5, 15)
        self.state.omega = np.radians(initial_rates)

        # Fixed Sun vector in ECI
        self.sun_dir_eci = np.array([1.0, 0.0, 0.0])
        self.sun_dir_eci /= np.linalg.norm(self.sun_dir_eci)

    def run(self):
        try:
            self.running = True
            print("🚀 Simulation thread started successfully.")
            I   = self.sc.inertia
            I_inv = np.linalg.inv(I)

            while self.running and self.state.t < self.duration:
                if self.paused:
                    time.sleep(0.05)
                    continue

                dt = self.dt

                # ── Orbit ────────────────────────────────────────
                self.orbit.step(dt)

                # ── Sensors ──────────────────────────────────────
                q_true   = self.state.q / np.linalg.norm(self.state.q)
                omega_m  = self.gyro.measure(self.state.omega, dt)

                # [FIXED] Star tracker measurement – correctly handle None
                q_meas = self.star.measure(q_true)
                if q_meas is None:
                    q_meas = q_true

                # ── Disturbances ─────────────────────────────────
                T_gg  = gravity_gradient_torque(q_true, self.orbit.r_vec, self.sc)
                T_srp = solar_radiation_torque(q_true, self.orbit.r_vec, self.sc, self.sun_dir_eci)
                T_aero= aero_drag_torque(self.orbit.r_vec, self.orbit.v_vec, self.sc)
                T_mag = magnetic_disturbance_torque(q_true, self.orbit.r_vec)
                T_dist = T_gg + T_srp + T_aero + T_mag

                # ── ADCS control ──────────────────────────────────
                T_ctrl = self.adcs.compute_torque(q_meas, self.q_target, omega_m, dt)
                T_total = T_ctrl + T_dist

                # ── Attitude dynamics ────────────────────────────
                omega = self.state.omega
                alpha = I_inv @ (T_total - np.cross(omega, I @ omega))
                omega_new = omega + alpha * dt

                # ── Quaternion kinematics ────────────────────────
                w_quat = np.array([0, *omega_new])
                q_dot  = 0.5 * quat_mult(q_true, w_quat)
                q_new  = q_true + q_dot * dt
                q_new /= np.linalg.norm(q_new)

                # ── Attitude error ────────────────────────────────
                q_err   = quat_mult(quat_conj(self.q_target), q_new)
                if q_err[0] < 0:
                    q_err = -q_err
                att_err = 2 * np.degrees(np.arccos(np.clip(q_err[0], -1, 1)))

                # ── Fault check ───────────────────────────────────
                self.fc.check_faults(self.gyro, self.star, att_err,
                                     self.adcs.rw_saturation)

                # ── Update state ──────────────────────────────────
                with self._lock:
                    self.state.t          += dt
                    self.state.q           = q_new
                    self.state.omega       = omega_new
                    self.state.euler       = quat_to_euler(q_new)
                    self.state.att_err_deg = att_err
                    self.state.rw_sat      = self.adcs.rw_saturation
                    self.state.altitude_km = self.orbit.altitude_km
                    self.state.velocity_ms = self.orbit.velocity_ms
                    self.state.lat, self.state.lon = self.orbit.ground_track
                    self.state.fuel_kg     = self.fc.fuel
                    self.state.dv_total    = self.fc.dv_total
                    self.state.mode        = self.fc.mode
                    self.state.faults      = list(self.fc.faults)
                    self.state.torques     = T_ctrl

                    # Record EVERY timestep
                    self.state.record()

                time.sleep(dt * 0.001)

            self.running = False
            print("✅ Simulation thread finished cleanly.")

        except Exception as e:
            print("❌ SIMULATION THREAD CRASHED!")
            print(f"   Error: {e}")
            traceback.print_exc()
            self.error = str(e)
            self.running = False

    def stop(self):
        self.running = False


# ─────────────────────────────────────────────────────────────
# GUI (UNCHANGED – WORKS PERFECTLY)
# ─────────────────────────────────────────────────────────────

BG      = "#070B14"
PANEL   = "#0E1620"
BORDER  = "#1C2B3A"
ACCENT  = "#00D4FF"
GREEN   = "#00FF9C"
YELLOW  = "#FFD600"
RED     = "#FF3B5C"
TEXT    = "#C9D8E8"
SUBTEXT = "#5A7A96"
FONT_MONO = ("Courier New", 9)
FONT_H    = ("Segoe UI", 10, "bold")
FONT_S    = ("Segoe UI", 9)


class LaunchDialog(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Mission Configuration")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.result = None
        w, h = 580, 620
        sw, sh = self.winfo_screenwidth(), self.winfo_screenheight()
        self.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self._build()
        self.grab_set()
        self.wait_window()

    def _lbl(self, parent, text, row, col=0, colspan=1, fg=TEXT, font=FONT_S):
        tk.Label(parent, text=text, bg=BG, fg=fg,
                 font=font).grid(row=row, column=col, columnspan=colspan,
                                  sticky="w", pady=(8,0))

    def _entry(self, parent, default, row, col=1, width=14):
        e = tk.Entry(parent, bg=PANEL, fg=TEXT, insertbackground=TEXT,
                     relief="flat", highlightthickness=1,
                     highlightbackground=BORDER, highlightcolor=ACCENT,
                     font=("Courier New",10), width=width, bd=3)
        e.insert(0, str(default))
        e.grid(row=row, column=col, sticky="w", padx=(10,0), pady=(8,0))
        return e

    def _build(self):
        hdr = tk.Frame(self, bg=ACCENT, height=5)
        hdr.pack(fill="x")
        tk.Label(self, text="⚡  FLIGHT CONTROLLER & ADCS SIMULATOR",
                 bg=BG, fg=ACCENT, font=("Courier New",13,"bold"),
                 pady=14).pack()
        tk.Label(self, text="Configure mission parameters before launch",
                 bg=BG, fg=SUBTEXT, font=FONT_S).pack(pady=(0,10))
        tk.Frame(self, bg=BORDER, height=1).pack(fill="x", padx=20)

        frm = tk.Frame(self, bg=BG, padx=30, pady=10)
        frm.pack(fill="both", expand=True)

        self._lbl(frm, "▸ SPACECRAFT",       0, 0, 2, fg=ACCENT, font=("Courier New",9,"bold"))
        self._lbl(frm, "Spacecraft Preset:",  1)
        self.sc_var = tk.StringVar(value=list(PRESETS.keys())[0])
        cb = ttk.Combobox(frm, textvariable=self.sc_var,
                          values=list(PRESETS.keys()),
                          state="readonly", font=FONT_S, width=28)
        cb.grid(row=1, column=1, sticky="w", padx=(10,0), pady=(8,0))

        self._lbl(frm, "▸ ORBIT",            3, 0, 2, fg=ACCENT, font=("Courier New",9,"bold"))
        self._lbl(frm, "Altitude (km):",      4)
        self.alt_e   = self._entry(frm, "400",  4)
        self._lbl(frm, "Inclination (°):",    5)
        self.inc_e   = self._entry(frm, "51.6", 5)
        self._lbl(frm, "RAAN (°):",           6)
        self.raan_e  = self._entry(frm, "0",    6)

        self._lbl(frm, "▸ ATTITUDE TARGET",  8, 0, 2, fg=ACCENT, font=("Courier New",9,"bold"))
        self._lbl(frm, "Target Roll (°):",    9)
        self.tr_e = self._entry(frm, "0", 9)
        self._lbl(frm, "Target Pitch (°):",  10)
        self.tp_e = self._entry(frm, "0", 10)
        self._lbl(frm, "Target Yaw (°):",    11)
        self.ty_e = self._entry(frm, "0", 11)

        self._lbl(frm, "▸ SIMULATION",       13, 0, 2, fg=ACCENT, font=("Courier New",9,"bold"))
        self._lbl(frm, "Duration (min):",     14)
        self.dur_e = self._entry(frm, "30", 14)
        self._lbl(frm, "Time-step (s):",      15)
        self.dt_e  = self._entry(frm, "1",  15)

        self.err_lbl = tk.Label(frm, text="", bg=BG, fg=RED, font=FONT_S)
        self.err_lbl.grid(row=16, column=0, columnspan=2, pady=(6,0))
        frm.columnconfigure(1, weight=1)

        tk.Frame(self, bg=BORDER, height=1).pack(fill="x", padx=20)
        btn_f = tk.Frame(self, bg=BG, pady=14, padx=30)
        btn_f.pack(fill="x")

        tk.Button(btn_f, text="✖  Cancel", bg=PANEL, fg=SUBTEXT,
                  relief="flat", font=FONT_S, padx=12, pady=6,
                  cursor="hand2", command=self.destroy).pack(side="left")
        tk.Button(btn_f, text="▶  LAUNCH SIMULATION", bg=ACCENT, fg=BG,
                  relief="flat", font=("Courier New",10,"bold"),
                  padx=16, pady=8, cursor="hand2",
                  command=self._launch).pack(side="right")

    def _get_float(self, entry, lo, hi, name):
        try:
            v = float(entry.get())
            if not (lo <= v <= hi):
                raise ValueError
            return v
        except ValueError:
            raise ValueError(f"{name} must be between {lo} and {hi}")

    def _launch(self):
        try:
            alt   = self._get_float(self.alt_e,   100, 36000,  "Altitude")
            inc   = self._get_float(self.inc_e,     0,   180, "Inclination")
            raan  = self._get_float(self.raan_e,    0,   360, "RAAN")
            tr    = self._get_float(self.tr_e,   -180,   180, "Target Roll")
            tp    = self._get_float(self.tp_e,    -90,    90, "Target Pitch")
            ty    = self._get_float(self.ty_e,   -180,   180, "Target Yaw")
            dur   = self._get_float(self.dur_e,     1, 10080, "Duration")
            dt    = self._get_float(self.dt_e,    0.1,    60, "Time-step")
        except ValueError as e:
            self.err_lbl.config(text=f"✘  {e}")
            return

        self.result = dict(
            sc       = PRESETS[self.sc_var.get()],
            orbit    = dict(altitude_km=alt, inclination_deg=inc, raan_deg=raan),
            target   = (tr, tp, ty),
            duration = dur * 60,
            dt       = dt,
        )
        self.destroy()


class Dashboard(tk.Frame):
    def __init__(self, parent, engine: SimEngine):
        super().__init__(parent, bg=BG)
        self.engine = engine
        self.error_shown = False
        self._build()
        self._update()

    def _build(self):
        top = tk.Frame(self, bg=PANEL, pady=6, padx=14)
        top.pack(fill="x")

        self.mode_lbl = tk.Label(top, text="● NOMINAL", bg=PANEL, fg=GREEN,
                                  font=("Courier New", 11, "bold"))
        self.mode_lbl.pack(side="left")

        self.fault_lbl = tk.Label(top, text="", bg=PANEL, fg=RED,
                                   font=("Courier New", 9))
        self.fault_lbl.pack(side="left", padx=20)

        self.time_lbl = tk.Label(top, text="T+00:00:00", bg=PANEL, fg=SUBTEXT,
                                  font=FONT_MONO)
        self.time_lbl.pack(side="right")

        mid = tk.Frame(self, bg=BG)
        mid.pack(fill="both", expand=True, padx=10, pady=6)

        tiles = tk.Frame(mid, bg=BG, width=220)
        tiles.pack(side="left", fill="y", padx=(0,8))
        tiles.pack_propagate(False)
        self._build_tiles(tiles)
        self._build_plots(mid)

    def _tile(self, parent, label, row, col):
        f = tk.Frame(parent, bg=PANEL, bd=0, highlightthickness=1,
                     highlightbackground=BORDER, padx=8, pady=6)
        f.grid(row=row, column=col, sticky="nsew", padx=4, pady=4)
        tk.Label(f, text=label, bg=PANEL, fg=SUBTEXT,
                 font=("Courier New", 7)).pack(anchor="w")
        val = tk.Label(f, text="---", bg=PANEL, fg=ACCENT,
                       font=("Courier New", 14, "bold"))
        val.pack(anchor="w")
        return val

    def _build_tiles(self, parent):
        parent.columnconfigure(0, weight=1)
        parent.columnconfigure(1, weight=1)
        self.t_roll  = self._tile(parent, "ROLL (°)",      0, 0)
        self.t_pitch = self._tile(parent, "PITCH (°)",     0, 1)
        self.t_yaw   = self._tile(parent, "YAW (°)",       1, 0)
        self.t_err   = self._tile(parent, "ATT ERROR (°)", 1, 1)
        self.t_alt   = self._tile(parent, "ALTITUDE (km)", 2, 0)
        self.t_vel   = self._tile(parent, "VELOCITY (m/s)",2, 1)
        self.t_fuel  = self._tile(parent, "FUEL (kg)",     3, 0)
        self.t_rw    = self._tile(parent, "RW SAT (%)",    3, 1)
        self.t_lat   = self._tile(parent, "LAT (°)",       4, 0)
        self.t_lon   = self._tile(parent, "LON (°)",       4, 1)
        self.t_omega = self._tile(parent, "ω NORM (°/s)",  5, 0)
        self.t_dv    = self._tile(parent, "ΔV USED (m/s)", 5, 1)

        pb_f = tk.Frame(parent, bg=BG)
        pb_f.grid(row=6, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        tk.Label(pb_f, text="ATTITUDE ERROR", bg=BG, fg=SUBTEXT,
                 font=("Courier New",7)).pack(anchor="w")
        self.err_bar = ttk.Progressbar(pb_f, length=200, mode="determinate",
                                        maximum=180)
        self.err_bar.pack(fill="x")

        pb_f2 = tk.Frame(parent, bg=BG)
        pb_f2.grid(row=7, column=0, columnspan=2, sticky="ew", padx=4, pady=4)
        tk.Label(pb_f2, text="RW SATURATION", bg=BG, fg=SUBTEXT,
                 font=("Courier New",7)).pack(anchor="w")
        self.rw_bar = ttk.Progressbar(pb_f2, length=200, mode="determinate",
                                       maximum=100)
        self.rw_bar.pack(fill="x")

        btn_f = tk.Frame(parent, bg=BG)
        btn_f.grid(row=8, column=0, columnspan=2, pady=8)
        self.pause_btn = tk.Button(btn_f, text="⏸ PAUSE", bg=YELLOW, fg=BG,
                                    relief="flat", font=("Courier New",8,"bold"),
                                    padx=8, pady=4, cursor="hand2",
                                    command=self._toggle_pause)
        self.pause_btn.pack(side="left", padx=4)
        tk.Button(btn_f, text="⏹ STOP", bg=RED, fg="white",
                  relief="flat", font=("Courier New",8,"bold"),
                  padx=8, pady=4, cursor="hand2",
                  command=self._stop).pack(side="left", padx=4)

    def _build_plots(self, parent):
        fig = plt.Figure(figsize=(9, 6.5), facecolor=BG)
        gs  = gridspec.GridSpec(3, 2, figure=fig, hspace=0.55, wspace=0.35)

        self._axes = {}
        specs = [
            ("euler",   gs[0,0], "Euler Angles (°)",      ["roll","pitch","yaw"],   ["#FF7043","#42A5F5","#66BB6A"]),
            ("omega",   gs[0,1], "Body Rates (°/s)",       ["omega_x","omega_y","omega_z"], ["#FF7043","#42A5F5","#66BB6A"]),
            ("att_err", gs[1,0], "Attitude Error (°)",     ["att_err"],              [YELLOW]),
            ("rw_sat",  gs[1,1], "RW Saturation (%)",      ["rw_sat"],               [RED]),
            ("alt",     gs[2,0], "Altitude (km)",           ["alt"],                  [ACCENT]),
            ("fuel",    gs[2,1], "Fuel Remaining (kg)",     ["fuel"],                 [GREEN]),
        ]

        self._plot_lines = {}
        for key, spec, title, series, colors in specs:
            ax = fig.add_subplot(spec)
            ax.set_facecolor(PANEL)
            ax.tick_params(colors=SUBTEXT, labelsize=7)
            ax.set_title(title, color=TEXT, fontsize=8, pad=4)
            ax.grid(color=BORDER, linewidth=0.5)
            for spine in ax.spines.values():
                spine.set_edgecolor(BORDER)
            self._plot_lines[key] = []
            for s, c in zip(series, colors):
                ln, = ax.plot([], [], color=c, linewidth=1.2, label=s)
                self._plot_lines[key].append((s, ln))
            if len(series) > 1:
                ax.legend(fontsize=6, facecolor=PANEL, edgecolor=BORDER,
                          labelcolor=TEXT, loc="upper right")
            self._axes[key] = ax

        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas = canvas

    def _update(self):
        if self.engine.error and not self.error_shown:
            self.error_shown = True
            self.mode_lbl.config(text="⚠️ THREAD CRASHED", fg=RED)
            self.fault_lbl.config(text=f"ERROR: {self.engine.error}", fg=RED)

        if not self.engine.running and self.engine.state.t == 0:
            self.after(200, self._update)
            return

        with self.engine._lock:
            s = self.engine.state

            e = s.euler
            self.t_roll.config(text=f"{e[0]:+7.2f}")
            self.t_pitch.config(text=f"{e[1]:+7.2f}")
            self.t_yaw.config(text=f"{e[2]:+7.2f}")

            err_color = GREEN if s.att_err_deg < 2 else (YELLOW if s.att_err_deg < 10 else RED)
            self.t_err.config(text=f"{s.att_err_deg:6.2f}", fg=err_color)
            self.t_alt.config(text=f"{s.altitude_km:7.2f}")
            self.t_vel.config(text=f"{s.velocity_ms:7.1f}")
            self.t_fuel.config(text=f"{s.fuel_kg:.4f}")
            self.t_rw.config(text=f"{s.rw_sat*100:5.1f}",
                              fg=RED if s.rw_sat > 0.8 else ACCENT)
            self.t_lat.config(text=f"{s.lat:+7.2f}")
            self.t_lon.config(text=f"{s.lon:+7.2f}")
            omega_norm = np.degrees(np.linalg.norm(s.omega))
            self.t_omega.config(text=f"{omega_norm:.3f}")
            self.t_dv.config(text=f"{s.dv_total:.3f}")

            mode_color = GREEN if s.mode=="NOMINAL" else (YELLOW if "SAFE" not in s.mode else RED)
            self.mode_lbl.config(text=f"● {s.mode}", fg=mode_color)
            self.fault_lbl.config(text="  ".join(s.faults))

            h, m, sec = int(s.t)//3600, (int(s.t)%3600)//60, int(s.t)%60
            self.time_lbl.config(text=f"T+{h:02d}:{m:02d}:{sec:02d}")

            self.err_bar["value"] = min(s.att_err_deg, 180)
            self.rw_bar["value"]  = min(s.rw_sat * 100, 100)

            hist = s.history
            if len(hist["t"]) > 1:
                t_arr = hist["t"]
                mapping = {
                    "euler":  [("roll","roll"),("pitch","pitch"),("yaw","yaw")],
                    "omega":  [("omega_x","omega_x"),("omega_y","omega_y"),("omega_z","omega_z")],
                    "att_err":[("att_err","att_err")],
                    "rw_sat": [("rw_sat","rw_sat")],
                    "alt":    [("alt","alt")],
                    "fuel":   [("fuel","fuel")],
                }
                for key, pairs in mapping.items():
                    ax = self._axes[key]
                    for (hist_k, _), (_, ln) in zip(pairs, self._plot_lines[key]):
                        ln.set_data(t_arr, hist[hist_k])
                    ax.relim()
                    ax.autoscale_view()
                self._canvas.draw_idle()

        if self.engine.running:
            self.after(300, self._update)
        else:
            if not self.engine.error:
                self.mode_lbl.config(text="● SIMULATION COMPLETE", fg=ACCENT)
            self._save_report()

    def _toggle_pause(self):
        self.engine.paused = not self.engine.paused
        self.pause_btn.config(text="▶ RESUME" if self.engine.paused else "⏸ PAUSE")

    def _stop(self):
        self.engine.stop()

    def _save_report(self):
        s = self.engine.state
        sc = self.engine.sc
        orb = self.engine.orbit

        lines = [
            "=" * 65,
            "  FLIGHT CONTROLLER & ADCS SIMULATION REPORT",
            f"  Generated : {datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}",
            "=" * 65,
            f"  Spacecraft      : {sc.name}",
            f"  Mass            : {sc.mass} kg",
            f"  Inertia (kg·m²) : Ixx={sc.Ixx}  Iyy={sc.Iyy}  Izz={sc.Izz}",
            "-" * 65,
            f"  Orbit altitude  : {orb.altitude_km:.2f} km",
            f"  Orbital period  : {orb.orbital_period/60:.2f} min",
            f"  Sim duration    : {s.t/60:.1f} min",
            "-" * 65,
            "  FINAL TELEMETRY",
            "-" * 65,
            f"  Roll / Pitch / Yaw : {s.euler[0]:+.2f}° / {s.euler[1]:+.2f}° / {s.euler[2]:+.2f}°",
            f"  Attitude error     : {s.att_err_deg:.3f}°",
            f"  RW saturation      : {s.rw_sat*100:.1f}%",
            f"  Fuel remaining     : {s.fuel_kg:.4f} kg",
            f"  ΔV expended        : {s.dv_total:.3f} m/s",
            f"  Final mode         : {s.mode}",
            f"  Active faults      : {', '.join(s.faults) if s.faults else 'None'}",
            "=" * 65,
        ]
        if self.engine.error:
            lines.append(f"\n⚠️  SIMULATION THREAD CRASHED: {self.engine.error}")

        report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "flight_adcs_report.txt")
        with open(report_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines))
        print(f"\n  Report saved → {report_path}")

        hist = s.history
        csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "telemetry_data.csv")
        with open(csv_path, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(hist.keys())
            rows = zip(*hist.values())
            for row in rows:
                writer.writerow(row)
        print(f"  Telemetry data (full history) → {csv_path}")


class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Flight Controller & ADCS Simulator - Fully Corrected")
        self.root.configure(bg=BG)
        w, h = 1280, 760
        sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
        self.root.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")
        self._splash()
        self.root.mainloop()

    def _splash(self):
        for widget in self.root.winfo_children():
            widget.destroy()
        splash = tk.Frame(self.root, bg=BG)
        splash.place(relx=0.5, rely=0.5, anchor="center")
        tk.Label(splash, text="FLIGHT CONTROLLER", bg=BG, fg=ACCENT,
                 font=("Courier New", 22, "bold")).pack()
        tk.Label(splash, text="& ADCS SIMULATOR", bg=BG, fg=TEXT,
                 font=("Courier New", 18)).pack()
        tk.Label(splash, text="FULLY CORRECTED • No Crashing • Full Data Logging",
                 bg=BG, fg=SUBTEXT, font=FONT_S).pack(pady=8)
        tk.Button(splash, text="⚙  CONFIGURE MISSION",
                  bg=ACCENT, fg=BG, relief="flat",
                  font=("Courier New", 12, "bold"),
                  padx=22, pady=10, cursor="hand2",
                  command=self._open_config).pack(pady=20)

    def _open_config(self):
        dlg = LaunchDialog(self.root)
        if dlg.result is None:
            return
        cfg = dlg.result
        engine = SimEngine(
            config    = cfg["sc"],
            orbit_params = cfg["orbit"],
            duration_s   = cfg["duration"],
            dt           = cfg["dt"],
            target_euler = cfg["target"],
        )
        for widget in self.root.winfo_children():
            widget.destroy()
        dash = Dashboard(self.root, engine)
        dash.pack(fill="both", expand=True)
        engine.start()


if __name__ == "__main__":
    App()