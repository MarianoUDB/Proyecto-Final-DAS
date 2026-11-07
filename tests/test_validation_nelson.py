"""
Pruebas de Validación Cuantitativa contra la Literatura (Nelson 1998, Roskam 2001).

Cuantifica los errores relativos porcentuales exactos entre los autovalores y frecuencias
calculados por el software frente a los datos publicados en la literatura de referencia.
"""

import pytest
import numpy as np
from aero_gnc.config.aircraft_database import get_aircraft
from aero_gnc.validation.modal_validation import FlightDynamicsLinearizer


def test_modal_benchmark_relative_errors():
    """
    Verifica que los errores relativos frente a los benchmarks de literatura
    sean estrictamente inferiores a las tolerancias analíticas de ingeniería (< 5% en frecuencia).
    """
    ac = get_aircraft("b737")
    lin = FlightDynamicsLinearizer(ac)
    rows = lin.compare_against_reference()

    assert len(rows) >= 4

    for r in rows:
        # Frecuencia natural con error < 5% respecto a la referencia calibrada
        assert r.rel_error_wn_percent < 5.0, (
            f"Error excesivo en frecuencia natural de {r.mode_name}: {r.rel_error_wn_percent:.2f}%"
        )


def test_analytical_2dof_comparison():
    """
    Verifica que las fórmulas analíticas clásicas de 2-DOF (Nelson Secc. 4.5 y 4.6)
    aproximen el modelo completo 4-DOF con discrepancias físicas coherentes (< 20%).
    """
    ac = get_aircraft("b737")
    lin = FlightDynamicsLinearizer(ac)
    approx_rows = lin.compare_analytical_approximations()

    for a in approx_rows:
        assert a.rel_error_wn_percent < 20.0, (
            f"Discrepancia excesiva en aproximación 2-DOF de {a.mode_name}: {a.rel_error_wn_percent:.2f}%"
        )


def test_rk4_order_of_convergence():
    """
    Verifica que el integrador RK4 demuestre un orden empírico de convergencia p >= 3.85
    y un error numérico global inferior al 0.001% en el oscilador de referencia.
    """
    ac = get_aircraft("b737")
    lin = FlightDynamicsLinearizer(ac)
    rk = lin.verify_integrator_order()

    assert rk["is_fourth_order"], f"Orden de convergencia {rk['estimated_order']:.2f} no cumple O(dt^4)"
    assert rk["final_error_percent"] < 1e-3
