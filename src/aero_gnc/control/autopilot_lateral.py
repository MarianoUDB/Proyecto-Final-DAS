"""
Autopiloto Lateral-Direccional y Alineación con Eje de Pista (Localizer).

Implementa:
1. Lazo exterior de seguimiento del localizador ILS (Localizer Guidance) -> genera comando de alabeo phi_cmd.
2. Lazo interior de control de alabeo (Roll Attitude Hold) -> actúa sobre los alerones delta_a.
3. Amortiguador de guiñada (*Yaw Damper*) y coordinación de viraje -> actúa sobre el timón de dirección delta_r.
4. Maniobra de desguiñada (*Decrab*) a baja cota para alinear el fuselaje con el eje de pista antes del toque.
"""

from dataclasses import dataclass
import numpy as np

from aero_gnc.config.aircraft_database import AircraftData
from aero_gnc.config.airport_ils import ILSSpecs
from aero_gnc.control.pid_controller import RobustPID, PIDGains


@dataclass
class LateralGains:
    """Ganancias de sintonización lateral-direccional."""
    # Lazo exterior Localizer
    k_loc: float = 0.0025         # rad / m
    k_ydot: float = 0.008        # rad / (m/s)
    
    # Lazo interior Roll (PID)
    kp_roll: float = 1.8
    ki_roll: float = 0.35
    kd_roll: float = 0.6
    kp_damping: float = 0.4      # Amortiguador de velocidad de alabeo p
    
    # Guiñada / Yaw Damper / Decrab
    kr_damping: float = 1.2      # Amortiguador de velocidad de guiñada r (Dutch Roll)
    k_beta: float = 0.8          # Supresión de resbalamiento lateral beta
    k_psi_align: float = 1.2     # Alineación de rumbo en el flare (Decrab)


class LateralAutopilot:
    """Sistema de control de vuelo lateral-direccional."""

    def __init__(self, aircraft: AircraftData, ils: ILSSpecs, gains: LateralGains = LateralGains()):
        self.ac = aircraft
        self.ils = ils
        self.gains = gains

        # Controlador PID de alabeo
        self.roll_pid = RobustPID(PIDGains(
            kp=gains.kp_roll,
            ki=gains.ki_roll,
            kd=gains.kd_roll,
            tau_filter=0.04,
            u_min=np.radians(-20.0),
            u_max=np.radians(20.0)
        ))

    def reset(self) -> None:
        self.roll_pid.reset()

    def update(
        self,
        y_pos: float,        # Desviación lateral respecto al eje de pista [m]
        v_lateral: float,    # Velocidad transversal y_dot [m/s]
        phi: float,          # Ángulo de alabeo actual [rad]
        psi: float,          # Ángulo de rumbo actual [rad]
        p: float,            # Velocidad angular de alabeo [rad/s]
        r: float,            # Velocidad angular de guiñada [rad/s]
        beta: float,         # Ángulo de resbalamiento lateral [rad]
        altitude: float,     # Altitud sobre el terreno [m]
        x_pos: float,        # Posición longitudinal [m]
        dt: float
    ) -> tuple[float, float, dict]:
        """
        Calcula los comandos de control lateral-direccional: (delta_a, delta_r).
        
        Retorna:
        - delta_a: comando de alerones [rad]
        - delta_r: comando de timón de dirección [rad]
        - debug_info: métricas de desviación de eje
        """
        # 1. Error de seguimiento del localizador (y_pos = 0 es el centro de pista)
        # Si la aeronave está desviada a la derecha (+Y), se necesita alabeo a la izquierda (-phi)
        phi_cmd_raw = -self.gains.k_loc * y_pos - self.gains.k_ydot * v_lateral
        
        # Límite operacional de seguridad en alabeo durante aproximación (máx 20 deg)
        max_bank_approach = np.radians(min(20.0, self.ac.max_bank_deg))
        phi_cmd = float(np.clip(phi_cmd_raw, -max_bank_approach, max_bank_approach))

        # 2. Lazo interior Roll Attitude Hold (PID) + Amortiguador de alabeo p
        delta_a_pid = self.roll_pid.update(
            setpoint=phi_cmd,
            measurement=phi,
            dt=dt
        )
        delta_a = delta_a_pid - self.gains.kp_damping * p
        delta_a = float(np.clip(delta_a, np.radians(-20.0), np.radians(20.0)))

        # 3. Control Direccional (Timón de Dirección delta_r)
        # Amortiguador de guiñada (Yaw Damper): r > 0 requiere delta_r > 0 (Cn_dr = -0.088)
        delta_r_damping = self.gains.kr_damping * r
        delta_r_coord = -self.gains.k_beta * beta
        
        # 4. Decrab a baja cota (h < 15 m / 50 ft): alinear proa con pista antes del toque
        if altitude < 15.0:
            psi_error = self.ils.runway_heading_rad - psi
            delta_r_decrab = -self.gains.k_psi_align * psi_error
        else:
            delta_r_decrab = 0.0

        delta_r = delta_r_damping + delta_r_coord + delta_r_decrab
        delta_r = float(np.clip(delta_r, np.radians(-30.0), np.radians(30.0)))

        # Desviación de aguja ILS Localizer en dots (2 dots escala completa = 2.5 deg)
        dist_loc_antenna = max(100.0, self.ils.localizer_antenna_x - x_pos)
        loc_angle_deg = np.degrees(np.arctan2(y_pos, dist_loc_antenna))
        loc_dots = float(np.clip(loc_angle_deg / (self.ils.localizer_full_scale_deg / 2.0), -2.5, 2.5))

        debug_info = {
            "y_error_m": float(y_pos),
            "loc_dots": loc_dots,
            "phi_cmd_deg": float(np.degrees(phi_cmd)),
            "psi_deg": float(np.degrees(psi)),
        }

        return delta_a, delta_r, debug_info
