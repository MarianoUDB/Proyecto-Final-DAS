"""
Modelo de Atmósfera Estándar Internacional (ISA - International Standard Atmosphere).

Implementa las ecuaciones de la OACI (Doc 7488/3) para el cálculo de:
- Temperatura estática T [K]
- Presión estática P [Pa]
- Densidad del aire rho [kg/m^3]
- Velocidad del sonido a [m/s]
- Viscosidad dinámica mu [Pa*s] mediante la ley de Sutherland
- Presión dinámica q_inf [Pa]
"""

from dataclasses import dataclass
import numpy as np


# Constantes fundamentales ISA
G0 = 9.80665          # Aceleración gravitacional estándar [m/s^2]
R_GAS = 287.05287     # Constante específica del aire seco [J/(kg*K)]
GAMMA = 1.4           # Coeficiente de dilatación adiabática (cp/cv)

# Parámetros a nivel medio del mar (MSL)
T0_ISA = 288.15       # Temperatura estándar MSL [K] (15 °C)
P0_ISA = 101325.0     # Presión estándar MSL [Pa] (1013.25 hPa)
RHO0_ISA = 1.2250     # Densidad estándar MSL [kg/m^3]
LAPSE_RATE = 0.0065   # Gradiente térmico vertical troposférico [K/m] (6.5 K/km)
H_TROPOPAUSE = 11000.0 # Altitud de la tropopausa [m]
T_TROPOPAUSE = 216.65 # Temperatura en la tropopausa [K] (-56.5 °C)

# Constantes de Sutherland para viscosidad del aire
MU0 = 1.7894e-5       # Viscosidad de referencia a T0 [Pa*s]
S_SUTHERLAND = 110.4  # Constante de Sutherland [K]


@dataclass(frozen=True)
class AtmosphericState:
    """Propiedades termodinámicas y aerodinámicas del aire a una altitud dada."""
    altitude: float       # Altitud geopotencial [m]
    temperature: float    # Temperatura [K]
    temperature_c: float  # Temperatura [°C]
    pressure: float       # Presión [Pa]
    density: float        # Densidad [kg/m^3]
    speed_of_sound: float # Velocidad del sonido [m/s]
    dynamic_viscosity: float # Viscosidad [Pa*s]


class StandardAtmosphere:
    """Calculador analítico de la Atmósfera Estándar Internacional (ISA)."""
    
    @staticmethod
    def compute(altitude_m: float) -> AtmosphericState:
        """
        Calcula las variables de estado atmosféricas a una altitud geopotencial dada.
        
        Manejo de excepciones:
        - Si h < -500 m se limita a -500 m y emite alerta si es necesario.
        - Si h > 20,000 m genera un ValueError por salir del rango de operación subsónico.
        """
        h = float(altitude_m)
        if h < -500.0:
            h = -500.0
        elif h > 20000.0:
            raise ValueError(f"Altitud {h:.1f} m fuera del rango del modelo troposférico/estratosférico bajo (<20 km).")

        if h <= H_TROPOPAUSE:
            # Troposfera: Gradiente lineal de temperatura
            T = T0_ISA - LAPSE_RATE * h
            # Relación barométrica hidrostática con gradiente térmico
            exponent = G0 / (R_GAS * LAPSE_RATE)
            P = P0_ISA * (T / T0_ISA) ** exponent
        else:
            # Estratosfera baja (11 km a 20 km): Capa isotérmica
            T = T_TROPOPAUSE
            # Presión en la tropopausa
            p_tropo = P0_ISA * (T_TROPOPAUSE / T0_ISA) ** (G0 / (R_GAS * LAPSE_RATE))
            # Decaimiento exponencial isotermo
            dh = h - H_TROPOPAUSE
            P = p_tropo * np.exp(-G0 * dh / (R_GAS * T_TROPOPAUSE))

        # Ecuación de estado del gas ideal para la densidad
        rho = P / (R_GAS * T)
        
        # Velocidad acústica local
        a = np.sqrt(GAMMA * R_GAS * T)
        
        # Ley de Sutherland para viscosidad dinámica
        mu = MU0 * ((T / T0_ISA) ** 1.5) * ((T0_ISA + S_SUTHERLAND) / (T + S_SUTHERLAND))
        
        return AtmosphericState(
            altitude=h,
            temperature=T,
            temperature_c=T - 273.15,
            pressure=P,
            density=rho,
            speed_of_sound=a,
            dynamic_viscosity=mu,
        )

    @staticmethod
    def dynamic_pressure(density: float, true_airspeed: float) -> float:
        """Calcula la presión dinámica q_inf = 0.5 * rho * V^2 [Pa]."""
        if density <= 0.0:
            raise ValueError(f"Densidad física no válida: {density} kg/m^3")
        v = max(0.0, float(true_airspeed))
        return 0.5 * density * (v ** 2)
