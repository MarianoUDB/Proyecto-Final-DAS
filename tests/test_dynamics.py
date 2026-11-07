"""
Pruebas Unitarias del Motor de Dinámica de Vuelo 6-DOF e Integrador RK4.
"""

import pytest
import numpy as np
from aero_gnc.config.aircraft_database import get_aircraft
from aero_gnc.core.flight_dynamics_6dof import (
    FlightDynamics6DOF,
    AircraftState,
    FlightControls,
)
from aero_gnc.core.numerical_integrator import (
    RK4Integrator,
    SimulationDivergenceError,
)


def test_6dof_trim_derivatives():
    """Verifica que las aceleraciones sean de orden mínimo en condición casi equilibrada."""
    ac = get_aircraft("b737")
    dyn = FlightDynamics6DOF(ac)

    # Con alpha_trim para equilibrar 1g: w = u * tan(alpha_trim)
    alpha_trim = (ac.weight - 0.5 * 1.225 * (ac.v_ref ** 2) * ac.wing_area * ac.cl_0) / (
        0.5 * 1.225 * (ac.v_ref ** 2) * ac.wing_area * ac.cl_alpha
    )
    w_trim = ac.v_ref * np.tan(alpha_trim)

    state = AircraftState(
        u=ac.v_ref, v=0.0, w=w_trim,
        p=0.0, q=0.0, r=0.0,
        phi=0.0, theta=np.radians(1.5), psi=0.0,
        x_n=-3000.0, y_e=0.0, z_d=-200.0,
        thrust=25000.0
    )
    ctrl = FlightControls(delta_e=0.0, delta_a=0.0, delta_r=0.0, throttle=0.15)
    wind_zero = np.zeros(3)

    state_dot, telem = dyn.compute_derivatives(state, ctrl, wind_zero, t=0.0)

    # Las derivadas angulares deben ser moderadas
    assert abs(state_dot[3]) < 0.1  # p_dot
    assert abs(state_dot[4]) < 0.8  # q_dot
    assert abs(state_dot[5]) < 0.1  # r_dot
    assert telem["v_tas"] > 50.0
    assert 0.8 < telem["n_z"] < 1.2  # Factor de carga cercano a 1 g equilibrado


def test_rk4_step_finite():
    """Verifica que un paso de integración RK4 produzca valores finitos estables."""
    ac = get_aircraft("b737")
    dyn = FlightDynamics6DOF(ac)
    integrator = RK4Integrator(dt=0.02)

    state0 = AircraftState(
        u=ac.v_ref, v=0.0, w=0.0,
        p=0.0, q=0.0, r=0.0,
        phi=0.0, theta=np.radians(1.5), psi=0.0,
        x_n=-3000.0, y_e=0.0, z_d=-200.0,
        thrust=25000.0
    )
    ctrl = FlightControls(delta_e=0.0, delta_a=0.0, delta_r=0.0, throttle=0.15)
    wind = np.array([2.0, 1.0, 0.0])

    next_state, telem = integrator.step(dyn.compute_derivatives, state0, ctrl, wind, t=0.0)

    assert np.isfinite(next_state.u)
    assert np.isfinite(next_state.altitude)
    assert next_state.x_n > state0.x_n  # Debe avanzar en +X


def test_divergence_detection():
    """Verifica que el integrador capture la inestabilidad si se inyecta NaN."""
    integrator = RK4Integrator(dt=0.02)
    state_nan = AircraftState(
        u=np.nan, v=0.0, w=0.0,
        p=0.0, q=0.0, r=0.0,
        phi=0.0, theta=0.0, psi=0.0,
        x_n=0.0, y_e=0.0, z_d=0.0,
        thrust=0.0
    )

    def dummy_dynamics(s, c, w, t):
        return np.full(13, np.nan), {}

    with pytest.raises(SimulationDivergenceError):
        integrator.step(dummy_dynamics, state_nan, FlightControls(), np.zeros(3), t=0.0)
