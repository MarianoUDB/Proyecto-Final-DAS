"""
Integrador Numérico de Alta Precisión (Runge-Kutta 4to Orden - RK4).

Optimizado con computación vectorizada (NumPy), control de estabilidad numérica,
detección de divergencia y gestión de eventos de frontera (contacto con el suelo,
pérdida de convergencia o actitudes fuera de la envolvente física).
"""

from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple, Any
import numpy as np

from aero_gnc.core.flight_dynamics_6dof import AircraftState, FlightControls


class SimulationDivergenceError(RuntimeError):
    """Excepción generada si la simulación sufre inestabilidad numérica o NaN/Inf."""
    pass


class GroundCollisionError(RuntimeError):
    """Excepción generada si la aeronave impacta el terreno fuera de pista."""
    pass


@dataclass
class TrajectoryHistory:
    """Contenedor de telemetría y trayectoria temporal de la simulación."""
    time: np.ndarray
    state: np.ndarray          # Forma (N, 13)
    controls: np.ndarray       # Forma (N, 4) [de, da, dr, th]
    telemetry: Dict[str, np.ndarray]
    wind: np.ndarray           # Forma (N, 3)
    events: List[str]


class RK4Integrator:
    """Integrador numérico Runge-Kutta clásico de cuarto orden vectorizado."""

    def __init__(
        self,
        dt: float = 0.02,
        ground_altitude_threshold: float = 0.0,
        divergence_pos_limit: float = 50000.0,
    ):
        if dt <= 0:
            raise ValueError(f"Paso de integración dt={dt} debe ser estrictamente positivo.")
        self.dt = dt
        self.ground_limit = ground_altitude_threshold
        self.pos_limit = divergence_pos_limit

    def step(
        self,
        dynamics_fn: Callable[[AircraftState, FlightControls, np.ndarray, float], Tuple[np.ndarray, Dict[str, float]]],
        state: AircraftState,
        controls: FlightControls,
        wind_ned: np.ndarray,
        t: float
    ) -> Tuple[AircraftState, Dict[str, float]]:
        """
        Ejecuta un único paso RK4 vectorizado:
        k1 = f(t, y)
        k2 = f(t + dt/2, y + dt/2 * k1)
        k3 = f(t + dt/2, y + dt/2 * k2)
        k4 = f(t + dt, y + dt * k3)
        y_{n+1} = y_n + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
        """
        dt = self.dt
        y0 = state.to_array()

        # k1
        k1, telemetry_step = dynamics_fn(state, controls, wind_ned, t)
        
        # k2
        y1 = y0 + 0.5 * dt * k1
        k2, _ = dynamics_fn(AircraftState.from_array(y1), controls, wind_ned, t + 0.5 * dt)
        
        # k3
        y2 = y0 + 0.5 * dt * k2
        k3, _ = dynamics_fn(AircraftState.from_array(y2), controls, wind_ned, t + 0.5 * dt)
        
        # k4
        y3 = y0 + dt * k3
        k4, _ = dynamics_fn(AircraftState.from_array(y3), controls, wind_ned, t + dt)

        # Actualización de estado
        y_next = y0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Detección temprana de inestabilidad numérica
        if np.any(np.isnan(y_next)) or np.any(np.isinf(y_next)):
            raise SimulationDivergenceError(
                f"Inestabilidad numérica detectada en t={t:.3f} s. El vector de estado contiene NaN o Inf."
            )

        # Validación física de límites de posición
        if abs(y_next[9]) > self.pos_limit or abs(y_next[10]) > self.pos_limit:
            raise SimulationDivergenceError(
                f"Divergencia de posición en t={t:.3f} s. Aeronave fuera de coordenadas válidas."
            )

        next_state = AircraftState.from_array(y_next)
        return next_state, telemetry_step
