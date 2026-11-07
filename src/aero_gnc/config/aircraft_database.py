"""
Base de datos de parámetros y derivadas de estabilidad de aeronaves de referencia.

Fuentes de referencia:
- Nelson, R. C. (1998). Flight Stability and Automatic Control (2nd ed.). McGraw-Hill.
- Roskam, J. (2001). Airplane Flight Dynamics and Automatic Flight Controls. DARcorporation.
- Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft Control and Simulation. Wiley.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np


@dataclass(frozen=True)
class AircraftData:
    """Parámetros geométricos, inerciales y aerodinámicos de una aeronave."""
    name: str
    description: str
    mass: float               # Masa [kg]
    weight: float             # Peso [N]
    wing_area: float          # Superficie alar S [m^2]
    wing_span: float          # Envergadura b [m]
    mean_chord: float         # Cuerda media aerodinámica c_bar [m]
    
    # Momentos y productos de inercia en ejes cuerpo [kg*m^2]
    ixx: float
    iyy: float
    izz: float
    ixz: float
    
    # Parámetros de referencia en aproximación
    v_ref: float              # Velocidad de aproximación de referencia [m/s]
    v_stall: float            # Velocidad de pérdida en conf. aterrizaje [m/s]
    alpha_stall_deg: float    # Ángulo de ataque crítico de pérdida [deg]
    
    # Derivadas longitudinales (adimensionales, por radián)
    cl_0: float
    cl_alpha: float
    cl_alpha_dot: float
    cl_q: float
    cl_delta_e: float
    
    cd_0: float
    cd_alpha: float
    cd_delta_e: float
    
    cm_0: float
    cm_alpha: float
    cm_alpha_dot: float
    cm_q: float
    cm_delta_e: float
    
    # Derivadas lateral-direccionales (adimensionales, por radián)
    cy_beta: float
    cy_p: float
    cy_r: float
    cy_delta_a: float
    cy_delta_r: float
    
    cl_beta: float
    cl_p: float
    cl_r: float
    cl_delta_a: float
    cl_delta_r: float
    
    cn_beta: float
    cn_p: float
    cn_r: float
    cn_delta_a: float
    cn_delta_r: float
    
    # Sistema propulsivo
    max_thrust: float         # Empuje máximo total disponible [N]
    idle_thrust: float        # Empuje de ralentí [N]
    engine_tau: float         # Constante de tiempo de respuesta del motor [s]
    
    # Límites estructurales y operacionales
    n_z_max: float            # Factor de carga máximo admisible [g]
    n_z_min: float            # Factor de carga mínimo admisible [g]
    max_bank_deg: float       # Alabeo máximo operativo [deg]
    max_pitch_deg: float      # Cabeceo máximo operativo [deg]
    min_pitch_deg: float      # Cabeceo mínimo operativo [deg]
    
    # Autovalores teóricos de referencia (Benchmark de literatura) para validación analítica
    eigenvalues_ref: Dict[str, complex] = field(default_factory=dict)

    def validate(self) -> None:
        """Valida la consistencia física de los parámetros de la aeronave."""
        if self.mass <= 0:
            raise ValueError(f"Masa inválida: {self.mass} kg. Debe ser > 0.")
        if self.wing_area <= 0 or self.wing_span <= 0 or self.mean_chord <= 0:
            raise ValueError("Las dimensiones geométricas de la aeronave deben ser positivas.")
        if self.ixx <= 0 or self.iyy <= 0 or self.izz <= 0:
            raise ValueError("Los momentos principales de inercia deben ser positivos.")
        if self.v_stall <= 0 or self.v_ref <= self.v_stall:
            raise ValueError(f"v_ref ({self.v_ref} m/s) debe ser estrictamente mayor que v_stall ({self.v_stall} m/s).")
        if self.cl_alpha <= 0:
            raise ValueError("cl_alpha debe ser positivo en régimen lineal.")
        if self.cm_alpha >= 0:
            raise ValueError(f"cm_alpha={self.cm_alpha}: Inestabilidad longitudinal estática estricta (debe ser < 0).")


# Definición de la aeronave principal: Boeing 737-100/200 en aproximación
# Referencia: Roskam (2001) Part I, Table 3.2 / Nelson (1998)
B737_APPROACH = AircraftData(
    name="Boeing 737",
    description="Transporte comercial bimotor - Configuración de aproximación final (Flaps 30/40, Gear Down)",
    mass=45359.2,           # 100,000 lb
    weight=444822.0,        # N (m * g0)
    wing_area=91.04,        # 980 ft^2
    wing_span=28.35,        # 93 ft
    mean_chord=3.41,        # 11.19 ft
    
    ixx=1033000.0,          # kg*m^2
    iyy=1952000.0,
    izz=2996000.0,
    ixz=128800.0,
    
    v_ref=69.45,            # 135 kts
    v_stall=53.50,          # 104 kts (~1.3 factor de seguridad en v_ref)
    alpha_stall_deg=14.5,
    
    # Longitudinales
    cl_0=0.85,
    cl_alpha=5.09,
    cl_alpha_dot=0.0,
    cl_q=5.40,
    cl_delta_e=0.40,
    
    cd_0=0.115,             # Configuración aterrizaje con flaps y tren desplegado
    cd_alpha=0.60,
    cd_delta_e=0.00,
    
    cm_0=0.00,
    cm_alpha=-1.02,
    cm_alpha_dot=-1.40,
    cm_q=-14.40,
    cm_delta_e=-1.44,
    
    # Lateral-direccionales
    cy_beta=-0.73,
    cy_p=0.00,
    cy_r=0.40,
    cy_delta_a=0.00,
    cy_delta_r=0.175,
    
    cl_beta=-0.11,
    cl_p=-0.45,
    cl_r=0.160,
    cl_delta_a=0.178,
    cl_delta_r=0.014,
    
    cn_beta=0.124,
    cn_p=-0.028,
    cn_r=-0.180,
    cn_delta_a=-0.002,
    cn_delta_r=-0.088,
    
    max_thrust=140000.0,    # 2 x CFM56/JT8D
    idle_thrust=7000.0,
    engine_tau=1.8,
    
    n_z_max=2.5,
    n_z_min=0.0,
    max_bank_deg=30.0,
    max_pitch_deg=15.0,
    min_pitch_deg=-10.0,
    
    # Autovalores benchmark exactos de la condición de aproximación
    eigenvalues_ref={
        "short_period_1": complex(-0.3235, 0.6650),
        "short_period_2": complex(-0.3235, -0.6650),
        "phugoid_1": complex(0.0039, -0.1338),
        "phugoid_2": complex(0.0039, 0.1338),
        "dutch_roll_1": complex(0.0039, 0.6356),
        "dutch_roll_2": complex(0.0039, -0.6356),
        "roll_subsidence": complex(-0.8397, 0.0),
        "spiral": complex(0.0025, 0.0),
    }
)

# Definición alternativa: Aviación general - Cessna 172 (Nelson 1998, Table B.1)
C172_APPROACH = AircraftData(
    name="Cessna 172",
    description="Aeronave utilitaria monomotor de aviación general",
    mass=1043.3,
    weight=10231.0,
    wing_area=16.16,
    wing_span=10.91,
    mean_chord=1.49,
    
    ixx=1285.0,
    iyy=1825.0,
    izz=2667.0,
    ixz=0.0,
    
    v_ref=33.44,            # 65 kts
    v_stall=24.69,          # 48 kts
    alpha_stall_deg=15.0,
    
    cl_0=0.41,
    cl_alpha=4.41,
    cl_alpha_dot=1.70,
    cl_q=3.90,
    cl_delta_e=0.43,
    
    cd_0=0.050,
    cd_alpha=0.38,
    cd_delta_e=0.00,
    
    cm_0=0.00,
    cm_alpha=-0.613,
    cm_alpha_dot=-3.35,
    cm_q=-12.40,
    cm_delta_e=-1.12,
    
    cy_beta=-0.31,
    cy_p=-0.037,
    cy_r=0.21,
    cy_delta_a=0.00,
    cy_delta_r=0.187,
    
    cl_beta=-0.089,
    cl_p=-0.47,
    cl_r=0.096,
    cl_delta_a=0.229,
    cl_delta_r=0.0147,
    
    cn_beta=0.065,
    cn_p=-0.030,
    cn_r=-0.099,
    cn_delta_a=-0.0216,
    cn_delta_r=-0.0645,
    
    max_thrust=2200.0,
    idle_thrust=150.0,
    engine_tau=0.8,
    
    n_z_max=3.8,
    n_z_min=-1.52,
    max_bank_deg=30.0,
    max_pitch_deg=20.0,
    min_pitch_deg=-10.0,
    
    eigenvalues_ref={
        "short_period_1": complex(-1.42, 2.38),
        "short_period_2": complex(-1.42, -2.38),
        "phugoid_1": complex(-0.015, 0.178),
        "phugoid_2": complex(-0.015, -0.178),
        "dutch_roll_1": complex(-0.165, 1.48),
        "dutch_roll_2": complex(-0.165, -1.48),
        "roll_subsidence": complex(-4.62, 0.0),
        "spiral": complex(-0.004, 0.0),
    }
)

AVAILABLE_AIRCRAFT: Dict[str, AircraftData] = {
    "b737": B737_APPROACH,
    "c172": C172_APPROACH,
}


def get_aircraft(aircraft_id: str = "b737") -> AircraftData:
    """Obtiene y valida la configuración de la aeronave seleccionada."""
    clean_id = aircraft_id.lower().strip()
    if clean_id not in AVAILABLE_AIRCRAFT:
        raise KeyError(
            f"Aeronave '{aircraft_id}' no encontrada. Opciones disponibles: {list(AVAILABLE_AIRCRAFT.keys())}"
        )
    ac = AVAILABLE_AIRCRAFT[clean_id]
    ac.validate()
    return ac
