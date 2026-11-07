"""
Pruebas Unitarias del Modelo de Atmósfera Estándar Internacional (ISA).

Verifica la concordancia analítica frente a las tablas oficiales de la OACI (Doc 7488/3)
a diferentes niveles de vuelo y altitudes de referencia.
"""

import pytest
import numpy as np
from aero_gnc.core.atmosphere import (
    StandardAtmosphere,
    T0_ISA,
    P0_ISA,
    RHO0_ISA,
    G0,
    R_GAS,
)


def test_sea_level_conditions():
    """Verifica que las propiedades a nivel del mar (0 m) coincidan con los estándares OACI."""
    atm = StandardAtmosphere.compute(0.0)
    assert np.isclose(atm.temperature, 288.15, atol=1e-2)
    assert np.isclose(atm.temperature_c, 15.0, atol=1e-2)
    assert np.isclose(atm.pressure, 101325.0, atol=1.0)
    assert np.isclose(atm.density, 1.225, atol=1e-3)
    assert np.isclose(atm.speed_of_sound, 340.29, atol=0.1)


def test_troposphere_gradient():
    """Verifica el gradiente térmico de 6.5 K/km en la troposfera (ej. 3000 m)."""
    h = 3000.0
    atm = StandardAtmosphere.compute(h)
    expected_t = 288.15 - 0.0065 * h
    assert np.isclose(atm.temperature, expected_t, atol=1e-2)
    
    # Presión teórica barométrica
    expected_p = P0_ISA * (expected_t / T0_ISA) ** (G0 / (R_GAS * 0.0065))
    assert np.isclose(atm.pressure, expected_p, rtol=1e-4)


def test_tropopause_isothermal():
    """Verifica la capa isotérmica de la tropopausa (11,000 m)."""
    atm_11k = StandardAtmosphere.compute(11000.0)
    atm_12k = StandardAtmosphere.compute(12000.0)
    assert np.isclose(atm_11k.temperature, 216.65, atol=1e-2)
    assert np.isclose(atm_12k.temperature, 216.65, atol=1e-2)
    assert atm_12k.pressure < atm_11k.pressure


def test_dynamic_pressure_and_guards():
    """Verifica el cálculo de presión dinámica y la gestión de excepciones físicas."""
    # Presión dinámica a 70 m/s
    q = StandardAtmosphere.dynamic_pressure(1.225, 70.0)
    assert np.isclose(q, 0.5 * 1.225 * 70.0 ** 2, atol=1e-3)

    # Excepción ante altitud fuera de rango operacional (>20 km)
    with pytest.raises(ValueError):
        StandardAtmosphere.compute(25000.0)

    # Excepción ante densidad física negativa o nula
    with pytest.raises(ValueError):
        StandardAtmosphere.dynamic_pressure(-1.0, 50.0)
