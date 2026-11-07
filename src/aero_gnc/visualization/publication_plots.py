"""
Módulo de Visualización Gráfica de Calidad Técnica Publicable (Matplotlib).

Genera figuras técnicas de alta resolución (300 DPI) para reportes de ingeniería:
1. Perfil de aproximación ILS 2D (Glideslope y Localizer) con compuertas de estabilización.
2. Telemetría de actitudes, actuadores y perturbaciones de viento.
3. Márgenes de seguridad y envolvente de vuelo (V_stall, n_z, alpha_crit).
4. Trayectoria espacial 3D de aproximación a pista con conos de haz ILS.
"""

from typing import Optional
import numpy as np
import matplotlib
matplotlib.use("Agg")  # Backend no interactivo seguro para generación de archivos
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from aero_gnc.core.numerical_integrator import TrajectoryHistory
from aero_gnc.config.airport_ils import ILSSpecs, DEFAULT_ILS


def plot_ils_tracking(
    history: TrajectoryHistory,
    ils: ILSSpecs = DEFAULT_ILS,
    save_path: Optional[str] = "docs/fig1_ils_tracking.png",
    dpi: int = 300,
) -> plt.Figure:
    """Genera la Figura 1: Seguimiento de Senda ILS y Rechazo de Perturbaciones."""
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=False)
    fig.suptitle(
        "Aproximación por Instrumentos ILS bajo Cizalladura y Ráfagas (FAA AC 120-71B)\n"
        "Dinámica 6-DOF y Control Multivariable PID",
        fontsize=14, fontweight="bold", y=0.98
    )

    t = history.time
    x_n = history.state[:, 9]
    y_e = history.state[:, 10]
    alt = history.telemetry["altitude_m"]
    h_nom = history.telemetry["h_nominal"]
    gs_dots = history.telemetry["gs_dots"]
    loc_dots = history.telemetry["loc_dots"]
    w_n = history.wind[:, 0]
    w_d = history.wind[:, 2]

    # Subplot 1: Perfil Vertical Altitud vs Posición X
    ax1 = axes[0]
    ax1.plot(x_n, alt, "b-", linewidth=2.0, label="Trayectoria Real de la Aeronave")
    ax1.plot(x_n, h_nom, "k--", linewidth=1.5, label="Senda Nominal ILS 3.0°")
    ax1.axvline(0.0, color="gray", linestyle=":", label="Umbral de Pista (Threshold)")
    ax1.axvline(ils.glideslope_antenna_x, color="green", linestyle="--", label="Zona de Contacto (TDZ)")
    ax1.set_ylabel("Altitud AGL [m]", fontsize=11, fontweight="bold")
    ax1.set_xlabel("Distancia al Umbral de Pista x [m]", fontsize=11, fontweight="bold")
    ax1.set_title("1. Perfil Vertical de Aproximación (Senda de Planeo Glideslope)", fontsize=11, fontweight="bold")
    ax1.legend(loc="upper left", frameon=True)
    ax1.grid(True, linestyle="--", alpha=0.6)

    # Subplot 2: Desviación de Agujas ILS (Dots)
    ax2 = axes[1]
    ax2.plot(t, gs_dots, "r-", linewidth=1.8, label="Desviación Glideslope (GS Dots)")
    ax2.plot(t, loc_dots, "m-", linewidth=1.8, label="Desviación Localizador (LOC Dots)")
    ax2.axhspan(-1.0, 1.0, color="green", alpha=0.15, label="Ventana Estabilizada FAA (±1 dot)")
    ax2.axhline(0.0, color="black", linestyle="-", linewidth=0.8)
    ax2.set_ylabel("Desviación [Dots]", fontsize=11, fontweight="bold")
    ax2.set_title("2. Indicación de Instrumentos ILS (Criterio de Aproximación Estabilizada)", fontsize=11, fontweight="bold")
    ax2.set_ylim(-2.5, 2.5)
    ax2.legend(loc="upper right", frameon=True)
    ax2.grid(True, linestyle="--", alpha=0.6)

    # Subplot 3: Perturbaciones de Viento (Shear y Microrráfaga)
    ax3 = axes[2]
    ax3.plot(t, -w_n, "teal", linewidth=1.8, label="Viento de Frente / Cola (+ Frente, - Cola) [m/s]")
    ax3.plot(t, w_d, "crimson", linewidth=1.8, label="Viento Vertical Descendente (+ Downdraft) [m/s]")
    ax3.set_ylabel("Velocidad de Viento [m/s]", fontsize=11, fontweight="bold")
    ax3.set_title("3. Perfil de Perturbación Atmosférica (Cizalladura y Downdraft)", fontsize=11, fontweight="bold")
    ax3.legend(loc="upper left", frameon=True)
    ax3.grid(True, linestyle="--", alpha=0.6)

    # Subplot 4: Actuadores y Superficies de Control
    ax4 = axes[3]
    ax4.plot(t, np.degrees(history.controls[:, 0]), "b-", linewidth=1.5, label=r"Elevador $\delta_e$ [deg]")
    ax4.plot(t, np.degrees(history.controls[:, 1]), "g-", linewidth=1.5, label=r"Alerones $\delta_a$ [deg]")
    ax4.plot(t, np.degrees(history.controls[:, 2]), "orange", linewidth=1.5, label=r"Timón $\delta_r$ [deg]")
    ax4.plot(t, history.controls[:, 3] * 100.0, "red", linestyle="--", linewidth=1.5, label="Acelerador Throttle [%]")
    ax4.set_ylabel("Comando Actuador", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Tiempo transcurrido t [s]", fontsize=11, fontweight="bold")
    ax4.set_title("4. Actuación de Mandos de Vuelo y Sistema de Autogases", fontsize=11, fontweight="bold")
    ax4.legend(loc="upper right", frameon=True, ncol=4)
    ax4.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"Figura guardada exitosamente en: {save_path}")
    return fig


def plot_safety_envelope(
    history: TrajectoryHistory,
    save_path: Optional[str] = "docs/fig2_safety_envelope.png",
    dpi: int = 300,
) -> plt.Figure:
    """Genera la Figura 2: Envolvente Operacional, Actitudes y Factores de Carga."""
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    fig.suptitle(
        "Supervisión de Seguridad Operacional y Envolvente de Vuelo Dinámica",
        fontsize=14, fontweight="bold", y=0.98
    )

    t = history.time
    phi_deg = np.degrees(history.state[:, 6])
    theta_deg = np.degrees(history.state[:, 7])
    psi_deg = np.degrees(history.state[:, 8])
    v_tas_kts = history.telemetry["v_tas"] * 1.94384
    alpha_deg = history.telemetry["alpha_deg"]
    n_z = history.telemetry["n_z"]
    vz_fpm = history.telemetry["vertical_speed_fpm"]

    # Subplot 1: Ángulos de Actitud de Euler
    ax1 = axes[0]
    ax1.plot(t, theta_deg, "b-", linewidth=1.8, label=r"Cabeceo $\theta$ [deg]")
    ax1.plot(t, phi_deg, "g-", linewidth=1.8, label=r"Alabeo $\phi$ [deg]")
    ax1.plot(t, psi_deg - 90.0, "m--", linewidth=1.5, label=r"Desviación de Rumbo $\Delta\psi$ [deg]")
    ax1.axhline(15.0, color="gray", linestyle=":", alpha=0.7)
    ax1.axhline(-10.0, color="gray", linestyle=":", alpha=0.7)
    ax1.set_ylabel("Ángulos [deg]", fontsize=11, fontweight="bold")
    ax1.set_title("1. Actitud de la Aeronave en Ejes Inerciales", fontsize=11, fontweight="bold")
    ax1.legend(loc="upper right", frameon=True)
    ax1.grid(True, linestyle="--", alpha=0.6)

    # Subplot 2: Velocidad y Ángulo de Ataque con Límites de Pérdida
    ax2 = axes[1]
    ax2_twin = ax2.twinx()
    p1 = ax2.plot(t, v_tas_kts, "darkblue", linewidth=2.0, label="Velocidad Verdadera TAS [kts]")
    ax2.axhline(135.0, color="black", linestyle="--", label="V_ref (135 kts)")
    ax2.axhline(104.0, color="red", linestyle=":", linewidth=1.8, label="V_stall (104 kts)")
    ax2.set_ylabel("Velocidad Aérea [kts]", color="darkblue", fontsize=11, fontweight="bold")

    p2 = ax2_twin.plot(t, alpha_deg, "darkorange", linewidth=1.8, label=r"Ángulo de Ataque $\alpha$ [deg]")
    ax2_twin.axhline(14.5, color="crimson", linestyle="--", label=r"$\alpha_{stall}$ Crítico (14.5°)")
    ax2_twin.set_ylabel(r"$\alpha$ [deg]", color="darkorange", fontsize=11, fontweight="bold")
    ax2.set_title("2. Velocidad de Referencia vs Pérdida y Margen de Ángulo de Ataque", fontsize=11, fontweight="bold")
    ax2.grid(True, linestyle="--", alpha=0.6)

    # Subplot 3: Factores de Carga Estructural y Tasa de Descenso
    ax3 = axes[2]
    ax3_twin = ax3.twinx()
    ax3.plot(t, n_z, "purple", linewidth=1.8, label="Factor de Carga n_z [g]")
    ax3.axhline(2.5, color="red", linestyle=":", label="Límite Estructural n_z_max (2.5 g)")
    ax3.axhline(1.0, color="gray", linestyle="--", alpha=0.5)
    ax3.set_ylabel("Factor de Carga [g]", color="purple", fontsize=11, fontweight="bold")
    ax3.set_ylim(0.0, 3.0)

    ax3_twin.plot(t, -vz_fpm, "chocolate", linewidth=1.5, label="Tasa de Descenso [ft/min]")
    ax3_twin.axhline(1000.0, color="crimson", linestyle="--", label="Límite GPWS Sink Rate (1000 ft/min)")
    ax3_twin.set_ylabel("Descenso [ft/min]", color="chocolate", fontsize=11, fontweight="bold")
    ax3.set_xlabel("Tiempo transcurrido t [s]", fontsize=11, fontweight="bold")
    ax3.set_title("3. Factor de Carga Estructural y Tasa de Descenso Vertical", fontsize=11, fontweight="bold")
    ax3.grid(True, linestyle="--", alpha=0.6)

    plt.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"Figura guardada exitosamente en: {save_path}")
    return fig


def plot_3d_trajectory(
    history: TrajectoryHistory,
    ils: ILSSpecs = DEFAULT_ILS,
    save_path: Optional[str] = "docs/fig3_trajectory_3d.png",
    dpi: int = 300,
) -> plt.Figure:
    """Genera la Figura 3: Trayectoria Espacial Tridimensional con Pista y Cono ILS."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")
    fig.suptitle(
        "Trayectoria 3D de Aproximación Final y Cono de Radiación ILS",
        fontsize=14, fontweight="bold", y=0.95
    )

    x = history.state[:, 9]
    y = history.state[:, 10]
    z = history.telemetry["altitude_m"]

    # Trayectoria de vuelo real
    ax.plot(x, y, z, "b-", linewidth=2.5, label="Trayectoria de Vuelo Real")

    # Senda nominal ILS
    x_nom = np.linspace(x[0], ils.glideslope_antenna_x, 100)
    y_nom = np.zeros_like(x_nom)
    z_nom = np.maximum(0.0, (ils.glideslope_antenna_x - x_nom) * np.tan(ils.glideslope_rad))
    ax.plot(x_nom, y_nom, z_nom, "r--", linewidth=1.5, label="Eje Nominal Senda ILS 3°")

    # Representación de la Pista
    rwy_x = [0.0, ils.runway_length, ils.runway_length, 0.0, 0.0]
    rwy_y = [-ils.runway_width / 2.0, -ils.runway_width / 2.0, ils.runway_width / 2.0, ils.runway_width / 2.0, -ils.runway_width / 2.0]
    rwy_z = [0.0, 0.0, 0.0, 0.0, 0.0]
    ax.plot(rwy_x, rwy_y, rwy_z, "k-", linewidth=3.0, label="Pista de Aterrizaje (RWY 09L)")

    # Punto de toque
    if len(x) > 0 and z[-1] <= 1.0:
        ax.scatter([x[-1]], [y[-1]], [z[-1]], color="red", s=100, marker="X", label="Punto de Contacto (Touchdown)")

    ax.set_xlabel("Distancia Longitudinal X [m]", fontsize=10, fontweight="bold")
    ax.set_ylabel("Desviación Lateral Y [m]", fontsize=10, fontweight="bold")
    ax.set_zlabel("Altitud Z [m]", fontsize=10, fontweight="bold")
    ax.legend(loc="upper right", frameon=True)
    ax.view_init(elev=20, azim=-65)

    if save_path:
        fig.savefig(save_path, dpi=dpi, bbox_inches="tight")
        print(f"Figura guardada exitosamente en: {save_path}")
    return fig
