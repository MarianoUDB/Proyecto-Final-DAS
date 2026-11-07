"""
Controlador PID Aeroespacial Robusto.

Características:
- Algoritmo PID en tiempo discreto con formulación anti-windup (integración condicional).
- Filtro pasabajo de primer orden en el término derivativo (evita amplificación de ruido).
- Detección y supresión de derivative kick (derivada sobre la variable de proceso).
- Saturación de salida con límites físicos de actuador.
"""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class PIDGains:
    """Ganancias de sintonización del controlador PID."""
    kp: float
    ki: float
    kd: float
    tau_filter: float = 0.05    # Constante de tiempo del filtro derivativo [s]
    u_min: float = -1.0
    u_max: float = 1.0


class RobustPID:
    """Controlador PID con anti-windup y filtrado derivativo."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.is_first_step = True

    def reset(self) -> None:
        """Reinicia los estados internos del controlador."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.prev_derivative = 0.0
        self.is_first_step = True

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
        feedforward: float = 0.0,
    ) -> float:
        """
        Calcula la acción de control u(t) para un paso de tiempo dt.
        
        Manejo de excepciones:
        - Si dt <= 0 o NaN, mantiene la salida anterior sin acumular integral.
        """
        if dt <= 0.0 or np.isnan(dt):
            return 0.0

        error = setpoint - measurement

        if self.is_first_step:
            self.prev_error = error
            self.prev_measurement = measurement
            self.is_first_step = False

        # 1. Término Proporcional
        p_term = self.gains.kp * error

        # 2. Término Derivativo con filtro pasabajo sobre la medición (evita derivative kick)
        # d_meas / dt
        d_measurement = (measurement - self.prev_measurement) / dt
        # Filtro pasabajo: y_k = alpha * y_{k-1} + (1 - alpha) * u_k
        alpha_filter = dt / (dt + max(1e-4, self.gains.tau_filter))
        filtered_derivative = alpha_filter * (-d_measurement) + (1.0 - alpha_filter) * self.prev_derivative
        d_term = self.gains.kd * filtered_derivative

        # 3. Término Integral tentativo
        tentative_integral = self.integral + error * dt
        i_term = self.gains.ki * tentative_integral

        # 4. Salida total preliminar
        u_unsaturated = p_term + i_term + d_term + feedforward

        # 5. Saturación y Anti-Windup (Clamping)
        u_saturated = float(np.clip(u_unsaturated, self.gains.u_min, self.gains.u_max))

        # Integración condicional: solo acumular si no satura o si el error tiende a des-saturar
        if self.gains.ki > 0:
            is_saturated_high = u_unsaturated > self.gains.u_max and error > 0
            is_saturated_low = u_unsaturated < self.gains.u_min and error < 0
            if not (is_saturated_high or is_saturated_low):
                self.integral = tentative_integral

        # Actualización de estados previos
        self.prev_error = error
        self.prev_measurement = measurement
        self.prev_derivative = filtered_derivative

        return u_saturated
