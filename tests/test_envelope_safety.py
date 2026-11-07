"""
Pruebas Unitarias de Monitoreo de Seguridad de Envolvente y Compuertas FAA.
"""

import pytest
import numpy as np
from aero_gnc.config.aircraft_database import get_aircraft
from aero_gnc.config.airport_ils import DEFAULT_ILS
from aero_gnc.safety.flight_envelope import FlightEnvelopeMonitor
from aero_gnc.safety.approach_monitor import StabilizedApproachMonitor


def test_stall_warning_detection():
    """Verifica que el monitor active alertas críticas de pérdida cuando V < 1.05 Vs."""
    ac = get_aircraft("b737")
    mon = FlightEnvelopeMonitor(ac)

    # Velocidad próxima a pérdida (Vs = 53.5 m/s)
    diag = mon.check(
        t=10.0,
        v_tas=54.0,           # Muy cerca de Vs
        alpha_deg=10.0,
        phi_deg=0.0,
        theta_deg=2.0,
        n_z=1.0,
        vertical_speed_mps=-3.0,
        altitude_m=200.0
    )
    assert not diag["is_within_envelope"]
    assert any(code == "STALL_CRITICAL" for _, code, _ in diag["active_warnings"])


def test_stick_shaker_detection():
    """Verifica la activación del vibrador de palanca ante ángulo de ataque crítico."""
    ac = get_aircraft("b737")
    mon = FlightEnvelopeMonitor(ac)

    # Alpha = 14.0 deg (alpha_stall = 14.5 deg)
    diag = mon.check(
        t=12.0,
        v_tas=70.0,
        alpha_deg=14.0,       # Margen 0.5 deg < 1.5 deg
        phi_deg=0.0,
        theta_deg=12.0,
        n_z=1.0,
        vertical_speed_mps=-3.0,
        altitude_m=150.0
    )
    assert any(code == "STICK_SHAKER" for _, code, _ in diag["active_warnings"])


def test_faa_gate_evaluation():
    """Verifica la evaluación precisa de la compuerta IMC de 1000 ft AGL."""
    ac = get_aircraft("b737")
    app_mon = StabilizedApproachMonitor(ac, DEFAULT_ILS)

    # Caso estabilizado perfecto al cruzar 1000 ft (~304.8 m)
    res_stable = app_mon.evaluate_step(
        t=15.0,
        altitude_m=300.0,     # Cruza la compuerta IMC
        gs_dots=0.2,          # Dentro de +-1.0 dot
        loc_dots=-0.1,        # Dentro de +-1.0 dot
        v_tas=ac.v_ref,       # Exactamente Vref
        vertical_speed_mps=-3.6, # ~700 ft/min < 1000 ft/min
        throttle=0.25
    )
    assert res_stable is not None
    assert res_stable.is_stabilized
    assert len(res_stable.discrepancies) == 0
