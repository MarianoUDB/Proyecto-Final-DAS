"""
Pruebas Unitarias del Controlador PID Robusto y Lazos de Autopiloto.
"""

import pytest
import numpy as np
from aero_gnc.control.pid_controller import RobustPID, PIDGains
from aero_gnc.control.autopilot_longitudinal import LongitudinalAutopilot, LongitudinalGains
from aero_gnc.control.autopilot_lateral import LateralAutopilot, LateralGains
from aero_gnc.config.aircraft_database import get_aircraft
from aero_gnc.config.airport_ils import DEFAULT_ILS


def test_pid_saturation_and_antiwindup():
    """Verifica que el controlador PID sature limpiamente y detenga la acumulación del error (anti-windup)."""
    gains = PIDGains(kp=2.0, ki=1.0, kd=0.1, u_min=-5.0, u_max=5.0)
    pid = RobustPID(gains)

    # Escalón grande continuo de error positivo (setpoint 100, meas 0)
    dt = 0.1
    for _ in range(50):
        out = pid.update(setpoint=100.0, measurement=0.0, dt=dt)
        assert out <= 5.0  # Límite superior estricto

    # El término integral debe haber dejado de acumular al saturar
    assert pid.integral <= 50.0

    # Ahora error negativo: debe responder inmediatamente sin retraso por des-saturación
    out_rev = pid.update(setpoint=-100.0, measurement=0.0, dt=dt)
    assert out_rev == -5.0


def test_longitudinal_glideslope_correction():
    """Verifica que por encima de senda se ordene bajar la nariz (pitch down)."""
    ac = get_aircraft("b737")
    ap = LongitudinalAutopilot(ac, DEFAULT_ILS)

    # Caso: Altitud 30 m por encima de la senda nominal
    x_pos = -3000.0
    h_nom = ap.compute_glideslope_target(x_pos)
    alt_above = h_nom + 30.0

    de, th, dbg = ap.update(
        altitude=alt_above,
        x_pos=x_pos,
        theta=np.radians(1.5),
        q=0.0,
        v_tas=ac.v_ref,
        v_z=-3.6,
        dt=0.02
    )
    # Debe ordenar un ángulo de cabeceo menor que el de trimado (nariz abajo)
    assert dbg["theta_cmd_deg"] < np.degrees(ap.theta_trim)


def test_lateral_localizer_correction():
    """Verifica que si la aeronave está desplazada a la derecha se ordene alabeo a la izquierda."""
    ac = get_aircraft("b737")
    ap = LateralAutopilot(ac, DEFAULT_ILS)

    # Avión 20 m a la derecha (+Y)
    da, dr, dbg = ap.update(
        y_pos=20.0,
        v_lateral=0.0,
        phi=0.0,
        psi=0.0,
        p=0.0,
        r=0.0,
        beta=0.0,
        altitude=200.0,
        x_pos=-2000.0,
        dt=0.02
    )
    # Alabeo ordenado debe ser negativo (a la izquierda)
    assert dbg["phi_cmd_deg"] < 0.0
    # Alerones deben deflectar para iniciar el viraje a la izquierda
    assert da < 0.0
