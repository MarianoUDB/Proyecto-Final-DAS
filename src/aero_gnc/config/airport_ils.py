"""
Especificaciones de Pista y Sistema de Aterrizaje por Instrumentos (ILS).

Define las coordenadas geométricas de la pista, el transmisor de senda de planeo
(Glideslope), el localizador (Localizer) y las compuertas de aproximación estabilizada
de acuerdo con los estándares de la OACI (Anexo 10) y la FAA (AC 120-71B).
"""

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class ILSSpecs:
    """Parámetros operacionales del sistema ILS y la pista."""
    runway_id: str
    runway_heading_deg: float      # Rumbo magnético/verdadero de la pista [deg]
    runway_length: float          # Longitud de pista [m]
    runway_width: float           # Ancho de pista [m]
    elevation_msl: float          # Elevación del umbral sobre nivel del mar [m]
    
    # Parámetros de Senda de Planeo (Glideslope)
    glideslope_angle_deg: float   # Ángulo nominal estándar (típicamente 3.0 deg)
    glideslope_antenna_x: float   # Posición longitudinal de la antena tras el umbral [m] (típicamente ~300 m)
    glideslope_full_scale_deg: float # Desviación angular para 2 dots (escala completa, ~0.7 deg)
    
    # Parámetros de Localizador (Localizer)
    localizer_antenna_x: float    # Posición longitudinal al extremo opuesto de pista [m]
    localizer_full_scale_deg: float # Desviación angular para 2 dots (escala completa, ~2.5 deg)
    
    # Parámetros operacionales de aproximación
    decision_height_m: float      # Altura de decisión DH Cat I [m] (típicamente 60 m / 200 ft)
    gate_imc_altitude_m: float    # Compuerta IMC de aproximación estabilizada [m] (1000 ft AGL = 304.8 m)
    gate_vmc_altitude_m: float    # Compuerta VMC de aproximación estabilizada [m] (500 ft AGL = 152.4 m)

    @property
    def glideslope_rad(self) -> float:
        return np.radians(self.glideslope_angle_deg)

    @property
    def runway_heading_rad(self) -> float:
        return np.radians(self.runway_heading_deg)


# Configuración estándar representativa de pista instrumental Categoría I
DEFAULT_ILS = ILSSpecs(
    runway_id="RWY 09L",
    runway_heading_deg=0.0,      # Eje longitudinal de pista alineado con el eje X de simulación
    runway_length=3500.0,
    runway_width=45.0,
    elevation_msl=0.0,           # A nivel del mar para simplificar referencias ISA
    glideslope_angle_deg=3.0,
    glideslope_antenna_x=300.0,  # Punto de toma de contacto ideal (touchdown zone)
    glideslope_full_scale_deg=0.7,
    localizer_antenna_x=3800.0,
    localizer_full_scale_deg=2.5,
    decision_height_m=60.96,     # 200 ft
    gate_imc_altitude_m=304.8,   # 1000 ft AGL
    gate_vmc_altitude_m=152.4,   # 500 ft AGL
)
