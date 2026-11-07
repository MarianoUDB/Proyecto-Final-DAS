"""
Monitor de Aproximación Estabilizada (Estándares FAA AC 120-71B y OACI).

Verifica el cumplimiento de las condiciones de aproximación estabilizada en las compuertas críticas:
- Compuerta IMC: 1000 ft AGL (304.8 m)
- Compuerta VMC: 500 ft AGL (152.4 m)

Criterios de Estabilización Evaluados:
1. Senda de planeo vertical (Glideslope) dentro de +-1 dot.
2. Eje de pista horizontal (Localizer) dentro de +-1 dot.
3. Velocidad indicada dentro de la ventana de referencia [-5 kts, +10 kts].
4. Tasa de descenso menor o igual a 1000 ft/min (5.08 m/s).
5. Ajuste de potencia de motores estabilizado.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional
import numpy as np

from aero_gnc.config.airport_ils import ILSSpecs
from aero_gnc.config.aircraft_database import AircraftData


@dataclass
class GateEvaluationResult:
    """Resultado de evaluación en una compuerta de estabilización."""
    gate_name: str
    altitude_target_m: float
    evaluated_at_time: float
    actual_altitude_m: float
    
    # Parámetros evaluados
    gs_dots: float
    loc_dots: float
    speed_error_mps: float       # V_tas - V_ref
    speed_error_kts: float
    descent_rate_fpm: float
    throttle_cmd: float
    
    # Criterios booleanos
    is_gs_stable: bool
    is_loc_stable: bool
    is_speed_stable: bool
    is_sink_rate_stable: bool
    is_throttle_stable: bool
    
    # Veredicto global
    is_stabilized: bool
    discrepancies: List[str]


class StabilizedApproachMonitor:
    """Supervisa el perfil de aproximación y evalúa las compuertas operacionales."""

    def __init__(self, aircraft: AircraftData, ils: ILSSpecs):
        self.ac = aircraft
        self.ils = ils
        self.imc_gate_evaluated = False
        self.vmc_gate_evaluated = False
        self.imc_result: Optional[GateEvaluationResult] = None
        self.vmc_result: Optional[GateEvaluationResult] = None

    def reset(self) -> None:
        self.imc_gate_evaluated = False
        self.vmc_gate_evaluated = False
        self.imc_result = None
        self.vmc_result = None

    def evaluate_step(
        self,
        t: float,
        altitude_m: float,
        gs_dots: float,
        loc_dots: float,
        v_tas: float,
        vertical_speed_mps: float,  # Positiva hacia arriba
        throttle: float
    ) -> Optional[GateEvaluationResult]:
        """
        Evalúa en tiempo real si se cruza una de las compuertas de aproximación.
        """
        # Evaluación compuerta IMC (1000 ft AGL = ~304.8 m)
        if not self.imc_gate_evaluated and altitude_m <= self.ils.gate_imc_altitude_m:
            self.imc_gate_evaluated = True
            self.imc_result = self._check_gate(
                "COMPUERTA IMC (1000 ft AGL)",
                self.ils.gate_imc_altitude_m,
                t, altitude_m, gs_dots, loc_dots, v_tas, vertical_speed_mps, throttle
            )
            return self.imc_result

        # Evaluación compuerta VMC (500 ft AGL = ~152.4 m)
        if not self.vmc_gate_evaluated and altitude_m <= self.ils.gate_vmc_altitude_m:
            self.vmc_gate_evaluated = True
            self.vmc_result = self._check_gate(
                "COMPUERTA VMC (500 ft AGL)",
                self.ils.gate_vmc_altitude_m,
                t, altitude_m, gs_dots, loc_dots, v_tas, vertical_speed_mps, throttle
            )
            return self.vmc_result

        return None

    def _check_gate(
        self,
        gate_name: str,
        target_alt: float,
        t: float,
        alt: float,
        gs_dots: float,
        loc_dots: float,
        v_tas: float,
        vz_mps: float,
        throttle: float
    ) -> GateEvaluationResult:
        descent_rate_fpm = -vz_mps * 196.85
        speed_err_mps = v_tas - self.ac.v_ref
        speed_err_kts = speed_err_mps * 1.94384

        discrepancies = []

        # 1. Glideslope dentro de +-1 dot
        is_gs_stable = abs(gs_dots) <= 1.0
        if not is_gs_stable:
            discrepancies.append(f"Desviación de Senda Glideslope excesiva ({gs_dots:+.2f} dots, límite +-1.0)")

        # 2. Localizer dentro de +-1 dot
        is_loc_stable = abs(loc_dots) <= 1.0
        if not is_loc_stable:
            discrepancies.append(f"Desviación de Localizador excesiva ({loc_dots:+.2f} dots, límite +-1.0)")

        # 3. Velocidad dentro de [-5 kts, +10 kts]
        is_speed_stable = -5.0 <= speed_err_kts <= 10.0
        if not is_speed_stable:
            discrepancies.append(f"Velocidad fuera de ventana ({speed_err_kts:+.1f} kts respecto a Vref)")

        # 4. Tasa de descenso <= 1000 ft/min
        is_sink_stable = descent_rate_fpm <= 1000.0
        if not is_sink_stable:
            discrepancies.append(f"Tasa de descenso excesiva ({descent_rate_fpm:.0f} ft/min > 1000 ft/min)")

        # 5. Ajuste de empuje por encima de ralentí
        is_throt_stable = throttle > 0.15
        if not is_throt_stable:
            discrepancies.append(f"Empuje en ralentí o insuficiente ({throttle * 100:.1f}%)")

        is_stabilized = len(discrepancies) == 0

        return GateEvaluationResult(
            gate_name=gate_name,
            altitude_target_m=target_alt,
            evaluated_at_time=t,
            actual_altitude_m=alt,
            gs_dots=gs_dots,
            loc_dots=loc_dots,
            speed_error_mps=speed_err_mps,
            speed_error_kts=speed_err_kts,
            descent_rate_fpm=descent_rate_fpm,
            throttle_cmd=throttle,
            is_gs_stable=is_gs_stable,
            is_loc_stable=is_loc_stable,
            is_speed_stable=is_speed_stable,
            is_sink_rate_stable=is_sink_stable,
            is_throttle_stable=is_throt_stable,
            is_stabilized=is_stabilized,
            discrepancies=discrepancies
        )
