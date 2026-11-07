"""
Autopiloto Longitudinal y Sistema de Autogases (Autothrottle).

Implementa la arquitectura clásica de guiado y control en cascada:
1. Lazo exterior de senda de planeo (Glideslope ILS Guidance) -> genera comando de cabeceo theta_cmd.
2. Lazo interior de control de actitud (Pitch Hold & Pitch Rate Damper) -> actúa sobre el timón de profundidad delta_e.
3. Modo de recogida automática (*Flare*) a baja cota (h < 15 m / 50 ft) para toma suave.
4. Autogases (*Autothrottle*) de mantenimiento de velocidad de referencia v_ref con compensación por cabeceo.
"""

from dataclasses import dataclass
import numpy as np

from aero_gnc.config.aircraft_database import AircraftData
from aero_gnc.config.airport_ils import ILSSpecs
from aero_gnc.control.pid_controller import RobustPID, PIDGains


@dataclass
class LongitudinalGains:
    """Ganancias de sintonización longitudinal."""
    # Lazo exterior Glideslope
    k_gs: float = 0.0025          # rad / m
    k_hdot: float = 0.012         # rad / (m/s)
    
    # Lazo interior Pitch (PID)
    kp_pitch: float = 1.4
    ki_pitch: float = 0.35
    kd_pitch: float = 0.7
    kq_damping: float = 0.6       # Amortiguador de velocidad de cabeceo q
    
    # Autogases (Speed hold)
    kp_spd: float = 0.05
    ki_spd: float = 0.015
    kd_spd: float = 0.02
    k_pitch_comp: float = 0.2     # Compensación anticipatoria por cambio de actitud


class LongitudinalAutopilot:
    """Sistema de control de vuelo longitudinal y autogases."""

    def __init__(self, aircraft: AircraftData, ils: ILSSpecs, gains: LongitudinalGains = LongitudinalGains()):
        self.ac = aircraft
        self.ils = ils
        self.gains = gains

        # Controladores PID robustos
        self.pitch_pid = RobustPID(PIDGains(
            kp=gains.kp_pitch,
            ki=gains.ki_pitch,
            kd=gains.kd_pitch,
            tau_filter=0.04,
            u_min=np.radians(-25.0),
            u_max=np.radians(15.0)
        ))

        self.throttle_pid = RobustPID(PIDGains(
            kp=gains.kp_spd,
            ki=gains.ki_spd,
            kd=gains.kd_spd,
            tau_filter=0.1,
            u_min=-0.4,
            u_max=0.6
        ))

        # Valores nominales de trimado longitudinal
        self.theta_trim = np.radians(1.5)  # Cabeceo de trimado estándar en senda de 3 deg
        self.throttle_trim = 0.18          # Empuje de compensación de resistencia a v_ref en senda de 3 deg

    def reset(self) -> None:
        self.pitch_pid.reset()
        self.throttle_pid.reset()

    def compute_glideslope_target(self, x: float) -> float:
        """Calcula la altitud nominal de la senda de 3° a la distancia x del umbral."""
        # La antena / punto de contacto está en x = ils.glideslope_antenna_x
        dist_to_touchdown = max(0.0, self.ils.glideslope_antenna_x - x)
        return self.ils.elevation_msl + dist_to_touchdown * np.tan(self.ils.glideslope_rad)

    def update(
        self,
        altitude: float,
        x_pos: float,
        theta: float,
        q: float,
        v_tas: float,
        v_z: float,  # Derivada vz positiva hacia arriba (vertical speed)
        dt: float
    ) -> tuple[float, float, dict]:
        """
        Calcula las salidas de control longitudinal: (delta_e, throttle).
        
        Retorna:
        - delta_e: comando de timón de profundidad [rad]
        - throttle: comando de acelerador [0.0, 1.0]
        - debug_info: métricas de seguimiento de senda
        """
        # 1. Altitud nominal de senda y error de seguimiento
        h_nominal = self.compute_glideslope_target(x_pos)
        h_error = altitude - h_nominal  # Positivo si estamos por encima de la senda
        
        # Velocidad vertical nominal en senda de 3°: Vz_nom = -V_tas * sin(3 deg)
        vz_nominal = -v_tas * np.sin(self.ils.glideslope_rad)
        vz_error = v_z - vz_nominal

        # 2. Lógica de recogida (Flare) a muy baja cota (h < 15 m / 50 ft AGL)
        is_flaring = altitude < 15.0
        if is_flaring:
            # Durante el flare, reducimos la tasa de descenso suavemente a -0.5 m/s (~100 fpm)
            target_flare_vz = -0.5
            theta_cmd = np.radians(2.5) + 0.015 * (target_flare_vz - v_z)
            # Retracción gradual de gases
            flare_factor = max(0.0, altitude / 15.0)
            throttle_bias = self.throttle_trim * flare_factor
        else:
            # Lazo exterior ILS Glideslope: comando de cabeceo deseado
            # Si estamos por encima (h_error > 0), bajar nariz (theta_cmd menor)
            delta_theta = -self.gains.k_gs * h_error - self.gains.k_hdot * vz_error
            theta_cmd = self.theta_trim + delta_theta
            throttle_bias = self.throttle_trim

        # Límites de seguridad física en el cabeceo ordenado
        theta_cmd = float(np.clip(theta_cmd, np.radians(self.ac.min_pitch_deg), np.radians(self.ac.max_pitch_deg)))

        # 3. Lazo interior Pitch Hold (PID) + Amortiguador de cabeceo q
        delta_e_pid = self.pitch_pid.update(
            setpoint=theta_cmd,
            measurement=theta,
            dt=dt
        )
        # Amortiguamiento: si q > 0 (cabeceo hacia arriba), elevador deflecta hacia abajo (+delta_e)
        delta_e = -delta_e_pid + self.gains.kq_damping * q
        delta_e = float(np.clip(delta_e, np.radians(-25.0), np.radians(15.0)))

        # 4. Sistema de Autogases (Autothrottle)
        speed_error = self.ac.v_ref - v_tas
        throttle_corr = self.throttle_pid.update(
            setpoint=self.ac.v_ref,
            measurement=v_tas,
            dt=dt
        )
        # Compensación por cabeceo: al elevar la nariz, se requiere empuje adicional anticipativo
        pitch_comp = self.gains.k_pitch_comp * (theta - self.theta_trim)
        
        throttle_total = throttle_bias + throttle_corr + pitch_comp
        throttle_cmd = float(np.clip(throttle_total, 0.0, 1.0))

        # Desviación de aguja ILS Glideslope en dots (1 dot = 0.35 deg, 2 dots escala completa = 0.7 deg)
        dist_antenna = max(50.0, self.ils.glideslope_antenna_x - x_pos)
        angle_dev_deg = np.degrees(np.arctan2(h_error, dist_antenna))
        gs_dots = float(np.clip(angle_dev_deg / (self.ils.glideslope_full_scale_deg / 2.0), -2.5, 2.5))

        debug_info = {
            "h_nominal": float(h_nominal),
            "h_error_m": float(h_error),
            "gs_dots": gs_dots,
            "theta_cmd_deg": float(np.degrees(theta_cmd)),
            "is_flaring": bool(is_flaring),
            "vz_error_mps": float(vz_error),
        }

        return delta_e, throttle_cmd, debug_info
