"""
Módulo de Viento, Cizalladura (*Windshear*) y Turbulencia Atmosférica.

Implementa modelos aeroespaciales de perturbación atmosférica:
1. Perfil de viento medio con gradiente de cizalladura vertical (ley de potencias / logarítmica).
2. Modelo de microrráfaga convectiva (*Microburst / Downburst*) basado en la circular
   consultiva FAA AC 00-54 y Woodfield & Woods (con divergencia horizontal y núcleo descendente).
3. Ráfaga discreta cosenoidal ("1 - Cosine Gust", estándar FAA/MIL-F-8785C).
4. Turbulencia estocástica continua tipo Dryden aproximada para baja altitud.
"""

from dataclasses import dataclass
from typing import Tuple
import numpy as np


@dataclass
class WindScenarioConfig:
    """Configuración del entorno meteorológico y perturbaciones."""
    # Viento medio constante a altitud de referencia
    headwind_mean: float = 5.0        # Viento de frente medio a 10 m [m/s] (~10 kts)
    crosswind_mean: float = 2.0       # Viento cruzado medio a 10 m [m/s] (~4 kts)
    ref_height_m: float = 10.0        # Altura de referencia del anemómetro [m]
    shear_exponent: float = 0.143     # Exponente de la ley de potencia (terreno aeroportuario)
    
    # Parámetros de Microrráfaga / Cizalladura Severa (Downburst)
    enable_microburst: bool = True
    microburst_center_x: float = 1500.0   # Distancia al umbral donde se sitúa el núcleo [m]
    microburst_radius: float = 1200.0     # Radio de influencia horizontal [m]
    microburst_delta_u: float = 15.0      # Variación de velocidad horizontal (head-to-tail) [m/s] (~30 kts)
    microburst_w_down: float = 6.0        # Velocidad descendente máxima en el núcleo [m/s] (~1200 ft/min)
    
    # Ráfaga discreta FAA "1 - cosine"
    enable_discrete_gust: bool = True
    gust_amplitude_w: float = 3.0         # Amplitud vertical de la ráfaga [m/s]
    gust_start_time: float = 15.0         # Instante de inicio [s]
    gust_duration: float = 4.0            # Duración [s]
    
    # Turbulencia continua estocástica
    enable_turbulence: bool = True
    turbulence_intensity: float = 0.8     # Desviación estándar sigma_w [m/s] (ligera a moderada)
    seed: int = 42                        # Semilla para repetibilidad numérica


class AtmosphericDisturbance:
    """Calculador del vector de viento perturbador en coordenadas NED (North-East-Down)."""

    def __init__(self, config: WindScenarioConfig):
        self.cfg = config
        self.rng = np.random.default_rng(self.cfg.seed)
        # Filtro pasabajo de primer orden para sintetizar espectro Dryden de baja cota
        self._turb_state = np.zeros(3)

    def get_wind_ned(self, x: float, y: float, z: float, t: float) -> np.ndarray:
        """
        Retorna el vector de viento [W_N, W_E, W_D] en m/s en el sistema de referencia terrestre (NED).
        
        Convención:
        - W_N positivo: Viento sopla hacia el Norte (x).
        - W_E positivo: Viento sopla hacia el Este (y).
        - W_D positivo: Viento descendente (hacia abajo, +z / downdraft).
        Nota: la altitud sobre el terreno es h = -z.
        """
        h = max(0.5, -z)  # Altitud AGL mínima física 0.5 m para evitar divisiones o singularidades
        
        # 1. Gradiente de viento medio con la altura (Shear vertical)
        # Aproximación de ley de potencias estándar: U(h) = U_ref * (h / h_ref)^alpha
        speed_factor = (h / self.cfg.ref_height_m) ** self.cfg.shear_exponent
        # Limitación física razonable a cotas altas
        speed_factor = min(speed_factor, 2.5)
        
        # Orientación estándar para aproximación a pista 09 (eje X hacia pista)
        # Un viento de frente (headwind) sopla en sentido contrario al avance (+X del avión) -> -X
        w_mean_n = -self.cfg.headwind_mean * speed_factor
        w_mean_e = self.cfg.crosswind_mean * speed_factor
        w_mean_d = 0.0
        
        # 2. Microrráfaga / Cizalladura severa (FAA AC 00-54 / Woodfield & Woods)
        w_micro_n = 0.0
        w_micro_d = 0.0
        if self.cfg.enable_microburst:
            dx = x - self.cfg.microburst_center_x
            r = abs(dx)
            R_m = self.cfg.microburst_radius
            if r <= R_m:
                # Perfil sinusoidal de divergencia horizontal (pasa de viento de frente a cola)
                # Al aproximarse (dx < 0): viento de frente adicional (+ sustentación aparente)
                # Al alejarse (dx > 0): viento de cola brusco (- sustentación aparente)
                w_micro_n = -self.cfg.microburst_delta_u * np.sin(np.pi * dx / R_m)
                
                # Núcleo descendente (downdraft) dependiente de la cota (z/h_ref) por continuidad de masa
                # A nivel de suelo (h=0) el flujo vertical es cero y se expande radialmente
                shape_w = 0.5 * (1.0 + np.cos(np.pi * dx / R_m))
                height_scale = min(h / 300.0, 1.0)
                # En NED, +W_D es hacia abajo (downdraft)
                w_micro_d = self.cfg.microburst_w_down * shape_w * height_scale

        # 3. Ráfaga discreta "1 - cosine" (FAA/MIL-F-8785C)
        w_gust_d = 0.0
        if self.cfg.enable_discrete_gust:
            t_g = t - self.cfg.gust_start_time
            if 0.0 <= t_g <= self.cfg.gust_duration:
                omega_g = 2.0 * np.pi / self.cfg.gust_duration
                # Ráfaga vertical descendente o ascendente
                w_gust_d = (self.cfg.gust_amplitude_w / 2.0) * (1.0 - np.cos(omega_g * t_g))

        # 4. Turbulencia estocástica de baja cota (Dryden)
        w_turb = np.zeros(3)
        if self.cfg.enable_turbulence:
            # Filtro Gauss-Markov discreto: x_{k+1} = a*x_k + b*w_k
            dt_approx = 0.02
            tau_turb = max(0.5, h / 30.0)  # Escala de tiempo de correlación espacial
            a_filter = np.exp(-dt_approx / tau_turb)
            b_filter = self.cfg.turbulence_intensity * np.sqrt(1.0 - a_filter ** 2)
            noise = self.rng.standard_normal(3)
            self._turb_state = a_filter * self._turb_state + b_filter * noise
            w_turb = self._turb_state

        w_total_n = w_mean_n + w_micro_n + w_turb[0]
        w_total_e = w_mean_e + w_turb[1]
        w_total_d = w_mean_d + w_micro_d + w_gust_d + w_turb[2]

        return np.array([w_total_n, w_total_e, w_total_d], dtype=np.float64)
