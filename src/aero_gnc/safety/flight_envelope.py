"""
Módulo de Supervisión de la Envolvente de Vuelo y Protección de Seguridad.

Monitorea en tiempo real los límites operativos y estructurales:
- Margen sobre la velocidad de pérdida (V_ref / V_stall).
- Margen de ángulo de ataque y activación de alarma de vibrador de palanca (*Stick Shaker*).
- Factores de carga estructural n_z y n_y.
- Límites de actitud (alabeo y cabeceo).
- Alerta de tasa de descenso excesiva (Sistema GPWS Modo 1: "Sink Rate").
"""

from dataclasses import dataclass
from typing import List, Dict, Any
import numpy as np

from aero_gnc.config.aircraft_database import AircraftData


@dataclass
class SafetyAlert:
    """Registro de una alerta o advertencia de seguridad operacional."""
    time: float
    level: str             # "INFO", "WARNING", "CRITICAL"
    code: str              # "STALL_WARNING", "OVERSPEED", "LOAD_FACTOR", "SINK_RATE", "BANK_ANGLE"
    message: str


class FlightEnvelopeMonitor:
    """Supervisión activa de la envolvente de seguridad de vuelo."""

    def __init__(self, aircraft: AircraftData):
        self.ac = aircraft
        self.alerts_history: List[SafetyAlert] = []

    def check(
        self,
        t: float,
        v_tas: float,
        alpha_deg: float,
        phi_deg: float,
        theta_deg: float,
        n_z: float,
        vertical_speed_mps: float,  # Positiva hacia arriba
        altitude_m: float
    ) -> Dict[str, Any]:
        """
        Evalúa el estado actual frente a los márgenes de seguridad.
        
        Retorna un diccionario de diagnóstico instantáneo.
        """
        active_warnings = []
        is_safe = True

        # 1. Margen sobre velocidad de pérdida
        stall_ratio = v_tas / max(1.0, self.ac.v_stall)
        if stall_ratio < 1.05:
            msg = f"¡PELIGRO DE PÉRDIDA! Velocidad {v_tas:.1f} m/s muy próxima a Vs ({self.ac.v_stall:.1f} m/s)."
            active_warnings.append(("CRITICAL", "STALL_CRITICAL", msg))
            is_safe = False
        elif stall_ratio < 1.15:
            msg = f"Advertencia: Margen bajo sobre pérdida (V/Vs = {stall_ratio:.2f})."
            active_warnings.append(("WARNING", "LOW_SPEED_MARGIN", msg))

        # 2. Margen de ángulo de ataque (Stick Shaker)
        alpha_margin = self.ac.alpha_stall_deg - alpha_deg
        if alpha_margin < 1.5:
            msg = f"¡STICK SHAKER ACTIVADO! Alpha = {alpha_deg:.1f}° (Crítico: {self.ac.alpha_stall_deg:.1f}°)."
            active_warnings.append(("CRITICAL", "STICK_SHAKER", msg))
            is_safe = False
        elif alpha_margin < 3.0:
            msg = f"Advertencia: Ángulo de ataque elevado (Alpha = {alpha_deg:.1f}°)."
            active_warnings.append(("WARNING", "HIGH_ALPHA", msg))

        # 3. Factores de carga estructural n_z
        if n_z > self.ac.n_z_max:
            msg = f"¡SOBRECARGA ESTRUCTURAL! n_z = {n_z:.2f} g supera n_z_max ({self.ac.n_z_max:.1f} g)."
            active_warnings.append(("CRITICAL", "OVER_G", msg))
            is_safe = False
        elif n_z < self.ac.n_z_min:
            msg = f"¡FACTOR DE CARGA NEGATIVO EXCESIVO! n_z = {n_z:.2f} g menor a {self.ac.n_z_min:.1f} g."
            active_warnings.append(("CRITICAL", "UNDER_G", msg))
            is_safe = False

        # 4. Límites de alabeo (Bank Angle)
        if abs(phi_deg) > self.ac.max_bank_deg:
            msg = f"¡ALABEO EXCESIVO! Bank = {phi_deg:.1f}° supera el límite operacional ({self.ac.max_bank_deg:.0f}°)."
            active_warnings.append(("WARNING", "BANK_ANGLE", msg))

        # 5. Tasa de descenso excesiva a baja altitud (GPWS Mode 1: Sink Rate)
        # 1000 ft/min = 5.08 m/s
        descent_rate_mps = -vertical_speed_mps  # Positivo si desciende
        if altitude_m < 305.0 and descent_rate_mps > 5.08:
            msg = f"¡ALERTA GPWS: SINK RATE! Tasa de descenso = {descent_rate_mps * 196.85:.0f} ft/min a {altitude_m:.0f} m AGL."
            active_warnings.append(("WARNING", "SINK_RATE", msg))

        # Registrar alertas en el historial
        for level, code, text in active_warnings:
            self.alerts_history.append(SafetyAlert(time=t, level=level, code=code, message=text))

        return {
            "is_within_envelope": is_safe,
            "stall_ratio": float(stall_ratio),
            "alpha_margin_deg": float(alpha_margin),
            "n_z": float(n_z),
            "active_warnings": active_warnings,
        }
