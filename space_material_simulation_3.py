"""
Space Environment Material Degradation Simulation
===================================================
Simulates material behavior under harsh outer-space conditions including:
  - Extreme thermal cycling (sunlight vs shadow)
  - Vacuum UV / solar radiation damage
  - Atomic oxygen erosion (LEO)
  - High-energy particle / radiation bombardment
  - Micrometeorite impact flux
  - Outgassing under vacuum
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from dataclasses import dataclass, field
from typing import List, Dict


# ──────────────────────────────────────────────
# 1.  MATERIAL DEFINITION
# ──────────────────────────────────────────────

@dataclass
class Material:
    name: str

    # Thermal
    thermal_conductivity: float   # W/(m·K)
    specific_heat: float          # J/(kg·K)
    density: float                # kg/m³
    melting_point: float          # K
    emissivity: float             # 0–1
    absorptivity: float           # 0–1 (solar)

    # Radiation & erosion resistance (0 = no resistance, 1 = perfect)
    uv_resistance: float
    radiation_resistance: float
    atomic_oxygen_resistance: float  # relevant for LEO

    # Mechanical (for micrometeorite impact)
    yield_strength: float         # MPa
    young_modulus: float          # GPa

    # Degradation bookkeeping (tracked over time)
    thickness: float = 1e-3       # m  (initial sample thickness)
    mass: float = field(init=False)

    # State variables (updated by simulation)
    temperature: float = 293.0    # K
    uv_damage: float = 0.0        # cumulative 0–1
    radiation_damage: float = 0.0
    ao_erosion: float = 0.0       # m eroded
    impact_damage: float = 0.0
    outgassing_loss: float = 0.0  # kg/m²

    def __post_init__(self):
        self.mass = self.density * self.thickness  # per m²

    @property
    def total_degradation(self) -> float:
        """Weighted combined degradation index (0 = pristine, 1 = failed)."""
        return np.clip(
            0.30 * self.uv_damage +
            0.25 * self.radiation_damage +
            0.20 * (self.ao_erosion / self.thickness) +
            0.15 * self.impact_damage +
            0.10 * min(self.outgassing_loss / (0.01 * self.mass), 1.0),
            0.0, 1.0
        )

    @property
    def structural_integrity(self) -> float:
        return max(0.0, 1.0 - self.total_degradation)


# ──────────────────────────────────────────────
# 2.  ENVIRONMENT DEFINITION
# ──────────────────────────────────────────────

@dataclass
class SpaceEnvironment:
    name: str
    orbit_altitude_km: float       # km above Earth
    solar_flux: float              # W/m²  (1361 at 1 AU)
    uv_flux: float                 # relative units (1.0 = 1 AU)
    radiation_dose_rate: float     # Gy/day
    atomic_oxygen_flux: float      # atoms/(cm²·s)  0 for deep space
    micrometeorite_flux: float     # impacts/(m²·day)
    temperature_sunlit: float      # K
    temperature_shadow: float      # K
    vacuum_pressure: float         # Pa


# Preset environments
LEO = SpaceEnvironment(
    name="Low Earth Orbit (LEO)",
    orbit_altitude_km=400,
    solar_flux=1361.0,
    uv_flux=1.0,
    radiation_dose_rate=5.0,
    atomic_oxygen_flux=1e15,
    micrometeorite_flux=0.1,
    temperature_sunlit=393.0,
    temperature_shadow=123.0,
    vacuum_pressure=1e-6,
)

GEO = SpaceEnvironment(
    name="Geostationary Orbit (GEO)",
    orbit_altitude_km=35_786,
    solar_flux=1361.0,
    uv_flux=1.0,
    radiation_dose_rate=50.0,
    atomic_oxygen_flux=0.0,
    micrometeorite_flux=0.05,
    temperature_sunlit=423.0,
    temperature_shadow=93.0,
    vacuum_pressure=1e-9,
)

DEEP_SPACE = SpaceEnvironment(
    name="Interplanetary / Deep Space",
    orbit_altitude_km=1e6,
    solar_flux=500.0,
    uv_flux=0.5,
    radiation_dose_rate=200.0,
    atomic_oxygen_flux=0.0,
    micrometeorite_flux=0.005,
    temperature_sunlit=350.0,
    temperature_shadow=40.0,
    vacuum_pressure=1e-12,
)


# ──────────────────────────────────────────────
# 3.  PRESET MATERIALS
# ──────────────────────────────────────────────

def make_materials() -> List[Material]:
    return [
        Material(
            name="Aluminum 6061-T6",
            thermal_conductivity=167, specific_heat=896, density=2700,
            melting_point=925, emissivity=0.05, absorptivity=0.09,
            uv_resistance=0.90, radiation_resistance=0.85,
            atomic_oxygen_resistance=0.60,
            yield_strength=276, young_modulus=68.9,
        ),
        Material(
            name="Carbon Fiber / Epoxy",
            thermal_conductivity=5, specific_heat=900, density=1600,
            melting_point=800, emissivity=0.85, absorptivity=0.92,
            uv_resistance=0.55, radiation_resistance=0.70,
            atomic_oxygen_resistance=0.20,
            yield_strength=600, young_modulus=70,
        ),
        Material(
            name="Titanium Ti-6Al-4V",
            thermal_conductivity=6.7, specific_heat=526, density=4430,
            melting_point=1933, emissivity=0.10, absorptivity=0.40,
            uv_resistance=0.95, radiation_resistance=0.92,
            atomic_oxygen_resistance=0.80,
            yield_strength=880, young_modulus=113.8,
        ),
        Material(
            name="Kapton Polyimide Film",
            thermal_conductivity=0.12, specific_heat=1090, density=1420,
            melting_point=673, emissivity=0.86, absorptivity=0.30,
            uv_resistance=0.60, radiation_resistance=0.75,
            atomic_oxygen_resistance=0.10,
            yield_strength=69, young_modulus=2.5,
        ),
        Material(
            name="Fused Silica (Quartz)",
            thermal_conductivity=1.38, specific_heat=703, density=2203,
            melting_point=1983, emissivity=0.93, absorptivity=0.07,
            uv_resistance=0.99, radiation_resistance=0.88,
            atomic_oxygen_resistance=0.95,
            yield_strength=48, young_modulus=73,
        ),
    ]


# ──────────────────────────────────────────────
# 4.  SIMULATION ENGINE
# ──────────────────────────────────────────────

STEFAN_BOLTZMANN = 5.670374419e-8   # W/(m²·K⁴)
ORBITAL_PERIOD_RATIO = 0.5          # fraction of orbit in sunlight


class SpaceSimulation:
    def __init__(self,
                 materials: List[Material],
                 environment: SpaceEnvironment,
                 duration_days: float = 365,
                 dt_hours: float = 1.0):
        self.materials = materials
        self.env = environment
        self.duration_days = duration_days
        self.dt = dt_hours / 24.0          # days
        self.time_steps = int(duration_days / self.dt)
        self.time_axis = np.linspace(0, duration_days, self.time_steps)

        # History arrays  {mat_name: array}
        self.history: Dict[str, Dict[str, np.ndarray]] = {
            m.name: {k: np.zeros(self.time_steps)
                     for k in ("temperature", "uv_damage", "radiation_damage",
                               "ao_erosion", "impact_damage",
                               "total_degradation", "structural_integrity")}
            for m in materials
        }

    # ── thermal equilibrium (simplified radiative balance) ──────────────────
    def _equilibrium_temp(self, mat: Material, sunlit: bool) -> float:
        if sunlit:
            q_in = mat.absorptivity * self.env.solar_flux   # W/m²
        else:
            q_in = 0.0
        # Radiative cooling: ε·σ·T⁴ = q_in  →  T = (q_in / ε·σ)^0.25
        if q_in > 0:
            t_eq = (q_in / (mat.emissivity * STEFAN_BOLTZMANN)) ** 0.25
        else:
            # cooling toward deep-space background (2.7 K effective sink)
            t_eq = self.env.temperature_shadow
        return t_eq

    # ── UV damage ────────────────────────────────────────────────────────────
    def _uv_step(self, mat: Material, sunlit: bool) -> float:
        if not sunlit:
            return 0.0
        base_rate = 1e-4 * self.env.uv_flux * self.dt   # per day
        return base_rate * (1.0 - mat.uv_resistance) * (1.0 - mat.uv_damage)

    # ── radiation damage ─────────────────────────────────────────────────────
    def _radiation_step(self, mat: Material) -> float:
        dose = self.env.radiation_dose_rate * self.dt   # Gy
        sensitivity = 1.0 - mat.radiation_resistance
        return min(sensitivity * dose * 5e-4, 0.01) * (1.0 - mat.radiation_damage)

    # ── atomic oxygen erosion (LEO) ──────────────────────────────────────────
    def _ao_step(self, mat: Material) -> float:
        if self.env.atomic_oxygen_flux <= 0:
            return 0.0
        flux_cm2s = self.env.atomic_oxygen_flux
        seconds = self.dt * 86400
        # Erosion yield: atoms → depth using simple empirical constant
        ao_sensitivity = 1.0 - mat.atomic_oxygen_resistance
        erosion_m = ao_sensitivity * flux_cm2s * seconds * 1e4 * 1e-28
        return min(erosion_m, mat.thickness * 0.001)

    # ── micrometeorite impacts ────────────────────────────────────────────────
    def _impact_step(self, mat: Material) -> float:
        expected_impacts = self.env.micrometeorite_flux * self.dt
        n_impacts = np.random.poisson(expected_impacts)
        if n_impacts == 0:
            return 0.0
        # Damage inversely proportional to yield strength & thickness
        damage = n_impacts * 1e-3 / (mat.yield_strength / 100.0)
        return min(damage, 0.05) * (1.0 - mat.impact_damage)

    # ── outgassing (very simplified) ─────────────────────────────────────────
    def _outgassing_step(self, mat: Material) -> float:
        # Proportional to vacuum level and temperature
        rate = 1e-8 * mat.temperature / 300.0 * (-np.log10(self.env.vacuum_pressure + 1e-20))
        return rate * self.dt * mat.mass

    # ── main loop ────────────────────────────────────────────────────────────
    def run(self):
        print(f"\n{'='*60}")
        print(f"  Simulation: {self.env.name}")
        print(f"  Duration  : {self.duration_days} days  |  dt = {self.dt*24:.1f} h")
        print(f"{'='*60}")

        for mat in self.materials:
            hist = self.history[mat.name]

            for i, t in enumerate(self.time_axis):
                # Determine whether sunlit (simple sinusoidal proxy for LEO)
                # For simplicity use a periodic toggle; for GEO/deep space always sunlit
                if self.env.orbit_altitude_km < 2000:
                    sunlit = (np.sin(2 * np.pi * t * 15.75) > 0)  # ~16 orbits/day LEO
                else:
                    sunlit = True

                # Temperature drift toward equilibrium
                t_target = self._equilibrium_temp(mat, sunlit)
                tau = mat.specific_heat * mat.density * mat.thickness / (
                    mat.emissivity * STEFAN_BOLTZMANN * mat.temperature ** 3 + 1e-9)
                mat.temperature += (t_target - mat.temperature) * min(self.dt * 86400 / max(tau, 1), 1)

                # Degradation contributions
                mat.uv_damage = min(mat.uv_damage + self._uv_step(mat, sunlit), 1.0)
                mat.radiation_damage = min(mat.radiation_damage + self._radiation_step(mat), 1.0)
                mat.ao_erosion += self._ao_step(mat)
                mat.impact_damage = min(mat.impact_damage + self._impact_step(mat), 1.0)
                mat.outgassing_loss += self._outgassing_step(mat)

                # Record
                hist["temperature"][i] = mat.temperature
                hist["uv_damage"][i] = mat.uv_damage
                hist["radiation_damage"][i] = mat.radiation_damage
                hist["ao_erosion"][i] = mat.ao_erosion * 1e6       # → µm
                hist["impact_damage"][i] = mat.impact_damage
                hist["total_degradation"][i] = mat.total_degradation
                hist["structural_integrity"][i] = mat.structural_integrity

            print(f"  ✓ {mat.name:<35}  integrity @ end: {mat.structural_integrity*100:5.1f}%")

        print()


# ──────────────────────────────────────────────
# 5.  VISUALISATION
# ──────────────────────────────────────────────

COLORS = ["#4C72B0", "#DD8452", "#55A868", "#C44E52", "#8172B2"]

def plot_results(sim: SpaceSimulation, save_path: str = None):
    fig = plt.figure(figsize=(20, 14))
    fig.patch.set_facecolor("#0D1117")
    gs = gridspec.GridSpec(3, 3, figure=fig, hspace=0.45, wspace=0.35)

    panel_cfg = [
        ("temperature",        "Temperature (K)",            "°K",   False),
        ("uv_damage",          "UV / Solar Damage",          "0–1",  True),
        ("radiation_damage",   "Radiation Damage",           "0–1",  True),
        ("ao_erosion",         "Atomic O Erosion",           "µm",   False),
        ("impact_damage",      "Micrometeorite Damage",      "0–1",  True),
        ("structural_integrity","Structural Integrity",      "0–1",  True),
    ]

    axes = [fig.add_subplot(gs[r, c]) for r in range(2) for c in range(3)]

    for ax, (key, title, ylabel, pct) in zip(axes, panel_cfg):
        ax.set_facecolor("#161B22")
        ax.tick_params(colors="white")
        ax.xaxis.label.set_color("white")
        ax.yaxis.label.set_color("white")
        ax.title.set_color("white")
        for spine in ax.spines.values():
            spine.set_edgecolor("#30363D")
        ax.grid(color="#21262D", linestyle="--", linewidth=0.6)

        for mat, color in zip(sim.materials, COLORS):
            y = sim.history[mat.name][key]
            ax.plot(sim.time_axis, y, label=mat.name, color=color, linewidth=1.6)

        ax.set_title(title, fontsize=10, pad=6, color="white", fontweight="bold")
        ax.set_xlabel("Time (days)", fontsize=8)
        ax.set_ylabel(ylabel, fontsize=8)
        if pct:
            ax.set_ylim(-0.02, 1.05)

    # ── Bar chart: final degradation ─────────────────────────────────────────
    ax_bar = fig.add_subplot(gs[2, :2])
    ax_bar.set_facecolor("#161B22")
    ax_bar.tick_params(colors="white")
    ax_bar.title.set_color("white")
    ax_bar.xaxis.label.set_color("white")
    ax_bar.yaxis.label.set_color("white")
    for spine in ax_bar.spines.values():
        spine.set_edgecolor("#30363D")
    ax_bar.grid(color="#21262D", axis="y", linestyle="--", linewidth=0.6)

    names = [m.name for m in sim.materials]
    integrities = [sim.history[m.name]["structural_integrity"][-1] * 100
                   for m in sim.materials]

    bars = ax_bar.barh(names, integrities, color=COLORS, height=0.55)
    for bar, val in zip(bars, integrities):
        ax_bar.text(bar.get_width() + 0.5, bar.get_y() + bar.get_height() / 2,
                    f"{val:.1f}%", va="center", ha="left", color="white", fontsize=8)
    ax_bar.set_xlim(0, 115)
    ax_bar.set_xlabel("Remaining Structural Integrity (%)", fontsize=9)
    ax_bar.set_title(f"Final Integrity After {sim.duration_days} days — {sim.env.name}",
                     fontsize=10, color="white", fontweight="bold")
    ax_bar.tick_params(axis="y", labelsize=8, colors="white")

    # ── Radar / spider: degradation contributors ──────────────────────────────
    ax_radar = fig.add_subplot(gs[2, 2], polar=True)
    ax_radar.set_facecolor("#161B22")
    ax_radar.title.set_color("white")
    ax_radar.tick_params(colors="white")

    labels = ["UV", "Radiation", "Atomic O", "Impacts", "Outgassing"]
    N = len(labels)
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False).tolist()
    angles += angles[:1]

    ax_radar.set_xticks(angles[:-1])
    ax_radar.set_xticklabels(labels, color="white", fontsize=8)
    ax_radar.set_yticklabels([])
    ax_radar.set_ylim(0, 1)
    ax_radar.grid(color="#30363D")

    for mat, color in zip(sim.materials, COLORS):
        h = sim.history[mat.name]
        ao_norm = min(h["ao_erosion"][-1] / (mat.thickness * 1e6 * 0.3 + 1e-9), 1.0)
        vals = [
            h["uv_damage"][-1],
            h["radiation_damage"][-1],
            ao_norm,
            h["impact_damage"][-1],
            min(mat.outgassing_loss / (0.01 * mat.mass + 1e-30), 1.0),
        ]
        vals += vals[:1]
        ax_radar.plot(angles, vals, color=color, linewidth=1.5, label=mat.name)
        ax_radar.fill(angles, vals, alpha=0.07, color=color)

    ax_radar.set_title("Degradation Breakdown\n(final)", fontsize=9,
                        color="white", fontweight="bold", pad=14)

    # ── Legend ────────────────────────────────────────────────────────────────
    handles = [plt.Line2D([0], [0], color=c, linewidth=2) for c in COLORS]
    fig.legend(handles, [m.name for m in sim.materials],
               loc="upper center", ncol=5,
               facecolor="#161B22", edgecolor="#30363D",
               labelcolor="white", fontsize=8,
               bbox_to_anchor=(0.5, 0.97))

    fig.suptitle(
        f"Space Material Degradation Simulation — {sim.env.name}",
        fontsize=14, color="white", fontweight="bold", y=1.00
    )

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches="tight",
                    facecolor=fig.get_facecolor())
        print(f"  Plot saved → {save_path}")
    plt.show()


# ──────────────────────────────────────────────
# 6.  SUMMARY REPORT
# ──────────────────────────────────────────────

def print_report(sim: SpaceSimulation, save_path: str = "simulation_report.txt"):
    from datetime import datetime
    import os

    env = sim.env
    lines_out = []

    lines_out.append("=" * 65)
    lines_out.append("  SPACE MATERIAL DEGRADATION — SIMULATION REPORT")
    lines_out.append(f"  Generated        : {datetime.now().strftime('%Y-%m-%d  %H:%M:%S')}")
    lines_out.append("=" * 65)
    lines_out.append(f"  Environment      : {env.name}")
    lines_out.append(f"  Orbit altitude   : {env.orbit_altitude_km:,.0f} km")
    lines_out.append(f"  Mission duration : {sim.duration_days} days")
    lines_out.append(f"  Time-step        : {sim.dt * 24:.2f} hours")
    lines_out.append(f"  Total steps      : {sim.time_steps}")
    lines_out.append("-" * 65)
    lines_out.append("  ENVIRONMENT CONDITIONS")
    lines_out.append("-" * 65)
    lines_out.append(f"  Solar flux         : {env.solar_flux} W/m2")
    lines_out.append(f"  UV flux            : {env.uv_flux} (rel. to 1 AU)")
    lines_out.append(f"  Radiation dose     : {env.radiation_dose_rate} Gy/day")
    lines_out.append(f"  Atomic O flux      : {env.atomic_oxygen_flux:.2e} atoms/(cm2/s)")
    lines_out.append(f"  Micrometeorite flux: {env.micrometeorite_flux} impacts/(m2/day)")
    lines_out.append(f"  Temp (sunlit)      : {env.temperature_sunlit} K")
    lines_out.append(f"  Temp (shadow)      : {env.temperature_shadow} K")
    lines_out.append(f"  Vacuum pressure    : {env.vacuum_pressure:.2e} Pa")
    lines_out.append("-" * 65)
    lines_out.append("  MATERIAL RESULTS  (end of mission)")
    lines_out.append("-" * 65)
    header = f"  {'Material':<35} {'Integrity':>9} {'UV':>6} {'Rad':>6} {'AO um':>7} {'Temp K':>7}"
    lines_out.append(header)
    lines_out.append("  " + "-" * 63)
    for mat in sim.materials:
        h = sim.history[mat.name]
        lines_out.append(
            f"  {mat.name:<35}"
            f" {h['structural_integrity'][-1]*100:>8.1f}%"
            f" {h['uv_damage'][-1]:>6.3f}"
            f" {h['radiation_damage'][-1]:>6.3f}"
            f" {h['ao_erosion'][-1]:>7.2f}"
            f" {h['temperature'][-1]:>7.1f}"
        )
    lines_out.append("-" * 65)
    lines_out.append("  DEGRADATION BREAKDOWN  (end of mission)")
    lines_out.append("-" * 65)
    sub_header = f"  {'Material':<35} {'UV dmg':>7} {'Rad dmg':>8} {'Impact':>7} {'Outgas kg/m2':>13}"
    lines_out.append(sub_header)
    lines_out.append("  " + "-" * 63)
    for mat in sim.materials:
        h = sim.history[mat.name]
        lines_out.append(
            f"  {mat.name:<35}"
            f" {h['uv_damage'][-1]:>7.4f}"
            f" {h['radiation_damage'][-1]:>8.4f}"
            f" {h['impact_damage'][-1]:>7.4f}"
            f" {mat.outgassing_loss:>13.4e}"
        )
    lines_out.append("=" * 65)
    lines_out.append("")

    report_text = "\n".join(lines_out)

    # Print to console
    print("\n" + report_text)

    # Save to file
    report_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), save_path)
    with open(report_file, "w", encoding="utf-8") as f:
        f.write(report_text)
    print(f"  Report saved -> {report_file}\n")



# ──────────────────────────────────────────────
# 7.  TKINTER GUI LAUNCHER
# ──────────────────────────────────────────────

import tkinter as tk
from tkinter import ttk, messagebox


ENVIRONMENT_MAP = {
    "Low Earth Orbit (LEO)":          LEO,
    "Geostationary Orbit (GEO)":      GEO,
    "Interplanetary / Deep Space":     DEEP_SPACE,
}

BG        = "#0D1117"
PANEL     = "#161B22"
BORDER    = "#30363D"
ACCENT    = "#4C72B0"
TEXT      = "#E6EDF3"
SUBTEXT   = "#8B949E"
SUCCESS   = "#3FB950"
ERROR     = "#F85149"
FONT_H    = ("Segoe UI", 13, "bold")
FONT_B    = ("Segoe UI", 10)
FONT_S    = ("Segoe UI", 9)


def _style_entry(e: tk.Entry):
    e.configure(bg=PANEL, fg=TEXT, insertbackground=TEXT,
                relief="flat", highlightthickness=1,
                highlightbackground=BORDER, highlightcolor=ACCENT,
                font=FONT_B, bd=4)


class SimulatorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Space Material Degradation Simulator")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)

        # Center window
        w, h = 520, 480
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.root.geometry(f"{w}x{h}+{(sw-w)//2}+{(sh-h)//2}")

        self._build_ui()
        self.root.mainloop()

    # ── UI Construction ────────────────────────────────────────────────────
    def _build_ui(self):
        # ── Header ──
        header = tk.Frame(self.root, bg=ACCENT, height=6)
        header.pack(fill="x")

        title_frame = tk.Frame(self.root, bg=BG, pady=18)
        title_frame.pack(fill="x", padx=28)

        tk.Label(title_frame, text="🛸  Space Material Simulator",
                 bg=BG, fg=TEXT, font=("Segoe UI", 16, "bold")).pack(anchor="w")
        tk.Label(title_frame,
                 text="Configure your mission parameters and launch the simulation.",
                 bg=BG, fg=SUBTEXT, font=FONT_S).pack(anchor="w", pady=(2, 0))

        # ── Divider ──
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill="x", padx=28)

        # ── Form ──
        form = tk.Frame(self.root, bg=BG, padx=28, pady=20)
        form.pack(fill="both", expand=True)

        # Environment
        self._label(form, "Space Environment", row=0)
        self.env_var = tk.StringVar(value=list(ENVIRONMENT_MAP.keys())[0])
        env_menu = ttk.Combobox(form, textvariable=self.env_var,
                                values=list(ENVIRONMENT_MAP.keys()),
                                state="readonly", font=FONT_B, width=36)
        env_menu.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(4, 14))
        self._style_combo(env_menu)

        # Duration
        self._label(form, "Mission Duration (days)", row=2)
        self._sublabel(form, "Range: 1 – 3650  |  Default: 365", row=3)
        self.dur_var = tk.StringVar(value="365")
        dur_entry = tk.Entry(form, textvariable=self.dur_var, width=20)
        _style_entry(dur_entry)
        dur_entry.grid(row=4, column=0, sticky="w", pady=(4, 14))
        self.dur_err = self._err_label(form, row=4)

        # Time-step
        self._label(form, "Simulation Time-Step (hours)", row=5)
        self._sublabel(form, "Range: 0.1 – 24  |  Default: 2.0  |  Smaller = more accurate", row=6)
        self.dt_var = tk.StringVar(value="2.0")
        dt_entry = tk.Entry(form, textvariable=self.dt_var, width=20)
        _style_entry(dt_entry)
        dt_entry.grid(row=7, column=0, sticky="w", pady=(4, 14))
        self.dt_err = self._err_label(form, row=7)

        form.columnconfigure(0, weight=1)

        # ── Footer ──
        tk.Frame(self.root, bg=BORDER, height=1).pack(fill="x", padx=28)

        btn_frame = tk.Frame(self.root, bg=BG, pady=16, padx=28)
        btn_frame.pack(fill="x")

        self.status_lbl = tk.Label(btn_frame, text="", bg=BG, fg=SUBTEXT, font=FONT_S)
        self.status_lbl.pack(side="left")

        run_btn = tk.Button(
            btn_frame, text="▶  Run Simulation",
            bg=ACCENT, fg="white", activebackground="#3A5A9A",
            activeforeground="white", relief="flat",
            font=("Segoe UI", 11, "bold"), padx=20, pady=8,
            cursor="hand2", command=self._on_run
        )
        run_btn.pack(side="right")

    # ── Helpers ───────────────────────────────────────────────────────────
    def _label(self, parent, text, row):
        tk.Label(parent, text=text, bg=BG, fg=TEXT,
                 font=("Segoe UI", 10, "bold")).grid(
                     row=row, column=0, sticky="w", pady=(0, 0))

    def _sublabel(self, parent, text, row):
        tk.Label(parent, text=text, bg=BG, fg=SUBTEXT,
                 font=("Segoe UI", 8)).grid(
                     row=row, column=0, sticky="w")

    def _err_label(self, parent, row):
        lbl = tk.Label(parent, text="", bg=BG, fg=ERROR, font=FONT_S)
        lbl.grid(row=row, column=1, sticky="w", padx=10)
        return lbl

    def _style_combo(self, combo):
        style = ttk.Style()
        style.theme_use("default")
        style.configure("TCombobox",
                        fieldbackground=PANEL,
                        background=PANEL,
                        foreground=TEXT,
                        arrowcolor=TEXT,
                        bordercolor=BORDER,
                        lightcolor=BORDER,
                        darkcolor=BORDER)
        style.map("TCombobox",
                  fieldbackground=[("readonly", PANEL)],
                  foreground=[("readonly", TEXT)])

    # ── Validation & Launch ───────────────────────────────────────────────
    def _validate(self):
        ok = True

        # Duration
        try:
            dur = float(self.dur_var.get())
            if not (1 <= dur <= 3650):
                raise ValueError
            self.dur_err.config(text="")
            self._duration = dur
        except ValueError:
            self.dur_err.config(text="✘ Must be 1 – 3650")
            ok = False

        # Time-step
        try:
            dt = float(self.dt_var.get())
            if not (0.1 <= dt <= 24):
                raise ValueError
            self.dt_err.config(text="")
            self._dt = dt
        except ValueError:
            self.dt_err.config(text="✘ Must be 0.1 – 24")
            ok = False

        return ok

    def _on_run(self):
        if not self._validate():
            return

        env_name   = self.env_var.get()
        environment = ENVIRONMENT_MAP[env_name]
        duration   = self._duration
        dt_hours   = self._dt

        self.status_lbl.config(text="⏳  Running simulation…", fg=SUBTEXT)
        self.root.update()

        try:
            np.random.seed(42)
            materials = make_materials()
            sim = SpaceSimulation(
                materials=materials,
                environment=environment,
                duration_days=duration,
                dt_hours=dt_hours,
            )
            sim.run()
            print_report(sim)

            self.status_lbl.config(text="✔  Done! Report saved.", fg=SUCCESS)
            self.root.update()

            plot_results(sim, save_path="space_material_sim_output.png")

        except Exception as exc:
            self.status_lbl.config(text=f"✘ Error: {exc}", fg=ERROR)
            messagebox.showerror("Simulation Error", str(exc))


# ──────────────────────────────────────────────
# 8.  ENTRY POINT
# ──────────────────────────────────────────────

if __name__ == "__main__":
    SimulatorGUI()
