"""
Generador de datos estáticos para despliegue en GitHub Pages (sin backend Python requerido).
Genera los escenarios precalculados de simulación y las validaciones analíticas en formato JSON.
"""

import json
import os
import numpy as np

from aero_gnc.config.aircraft_database import get_aircraft, AVAILABLE_AIRCRAFT
from aero_gnc.core.wind_turbulence import WindScenarioConfig
from aero_gnc.control.autopilot_longitudinal import LongitudinalGains
from aero_gnc.control.autopilot_lateral import LateralGains
from aero_gnc.simulator import AeroGNCSimulator, SimulationConfig
from aero_gnc.validation.modal_validation import FlightDynamicsLinearizer

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), "web", "data")
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("Generando escenarios precalculados...")

scenarios = [
    # Boeing 737
    {
        "id": "b737_nominal",
        "aircraft_id": "b737",
        "headwind": 5.0,
        "crosswind": 0.0,
        "microburst": False,
        "microburst_delta_u": 0.0,
        "microburst_w_down": 0.0,
        "turbulence": 0.1,
        "lateral_offset": 5.0,
    },
    {
        "id": "b737_crosswind",
        "aircraft_id": "b737",
        "headwind": 5.0,
        "crosswind": 8.0,
        "microburst": False,
        "microburst_delta_u": 0.0,
        "microburst_w_down": 0.0,
        "turbulence": 0.5,
        "lateral_offset": 22.0,
    },
    {
        "id": "b737_microburst",
        "aircraft_id": "b737",
        "headwind": 5.0,
        "crosswind": 2.0,
        "microburst": True,
        "microburst_delta_u": 14.0,
        "microburst_w_down": 5.5,
        "turbulence": 0.6,
        "lateral_offset": 18.0,
    },
    # Cessna 172
    {
        "id": "c172_nominal",
        "aircraft_id": "c172",
        "headwind": 3.0,
        "crosswind": 0.0,
        "microburst": False,
        "microburst_delta_u": 0.0,
        "microburst_w_down": 0.0,
        "turbulence": 0.1,
        "lateral_offset": 5.0,
    },
    {
        "id": "c172_microburst",
        "aircraft_id": "c172",
        "headwind": 4.0,
        "crosswind": 2.0,
        "microburst": True,
        "microburst_delta_u": 8.0,
        "microburst_w_down": 3.5,
        "turbulence": 0.5,
        "lateral_offset": 12.0,
    }
]

for sc in scenarios:
    print(f"  -> Simulando {sc['id']}...")
    w_cfg = WindScenarioConfig(
        headwind_mean=sc["headwind"],
        crosswind_mean=sc["crosswind"],
        enable_microburst=sc["microburst"],
        microburst_center_x=-1500.0,
        microburst_delta_u=sc["microburst_delta_u"],
        microburst_w_down=sc["microburst_w_down"],
        turbulence_intensity=sc["turbulence"],
    )

    sim_cfg = SimulationConfig(
        aircraft_id=sc["aircraft_id"],
        duration_s=65.0,
        dt=0.02,
        initial_dist_m=-3800.0,
        initial_lateral_offset_m=sc["lateral_offset"],
        wind_config=w_cfg,
    )

    lon_gains = LongitudinalGains(kp_pitch=1.6, kd_pitch=0.8)
    lat_gains = LateralGains(kp_roll=1.8)

    sim = AeroGNCSimulator(config=sim_cfg, lon_gains=lon_gains, lat_gains=lat_gains)
    history, summary = sim.run()

    stride = max(1, int(0.05 / sim_cfg.dt))
    sampled_indices = np.arange(0, len(history.time), stride)
    if len(sampled_indices) == 0 or sampled_indices[-1] != len(history.time) - 1:
        sampled_indices = np.append(sampled_indices, len(history.time) - 1)

    times_s = history.time[sampled_indices].tolist()
    states_s = history.state[sampled_indices].tolist()
    ctrls_s = history.controls[sampled_indices].tolist()
    winds_s = history.wind[sampled_indices].tolist()

    telemetry_s = {
        k: [round(float(v), 3) for v in arr[sampled_indices]]
        for k, arr in history.telemetry.items()
    }

    imc_dict = None
    if summary["imc_gate"]:
        g = summary["imc_gate"]
        imc_dict = {
            "gate_name": g.gate_name,
            "is_stabilized": g.is_stabilized,
            "gs_dots": round(g.gs_dots, 2),
            "loc_dots": round(g.loc_dots, 2),
            "speed_error_kts": round(g.speed_error_kts, 1),
            "descent_rate_fpm": round(g.descent_rate_fpm, 0),
            "discrepancies": g.discrepancies,
        }

    vmc_dict = None
    if summary["vmc_gate"]:
        g = summary["vmc_gate"]
        vmc_dict = {
            "gate_name": g.gate_name,
            "is_stabilized": g.is_stabilized,
            "gs_dots": round(g.gs_dots, 2),
            "loc_dots": round(g.loc_dots, 2),
            "speed_error_kts": round(g.speed_error_kts, 1),
            "descent_rate_fpm": round(g.descent_rate_fpm, 0),
            "discrepancies": g.discrepancies,
        }

    response_data = {
        "status": "success",
        "aircraft": summary["aircraft"],
        "duration_s": summary["duration_simulated_s"],
        "touchdown": summary["touchdown_occurred"],
        "touchdown_info": summary["touchdown_info"],
        "imc_gate": imc_dict,
        "vmc_gate": vmc_dict,
        "total_alerts": summary["total_safety_alerts"],
        "events": summary["events"],
        "data": {
            "time": times_s,
            "state": states_s,
            "controls": ctrls_s,
            "wind": winds_s,
            "telemetry": telemetry_s,
        }
    }

    with open(os.path.join(OUTPUT_DIR, f"sim_{sc['id']}.json"), "w", encoding="utf-8") as f:
        json.dump(response_data, f, ensure_ascii=False)

print("Generando datos de validación...")
for ac_id in ["b737", "c172"]:
    ac = get_aircraft(ac_id)
    lin = FlightDynamicsLinearizer(ac)

    rows_bench = lin.compare_against_reference()
    bench_data = [
        {
            "mode_name": r.mode_name,
            "computed_eigenvalue": f"{r.computed_eigenvalue.real:.4f} + {r.computed_eigenvalue.imag:.4f}j",
            "reference_eigenvalue": f"{r.reference_eigenvalue.real:.4f} + {r.reference_eigenvalue.imag:.4f}j",
            "computed_wn": round(r.computed_wn, 4),
            "reference_wn": round(r.reference_wn, 4),
            "computed_zeta": round(r.computed_zeta, 4),
            "reference_zeta": round(r.reference_zeta, 4),
            "rel_error_wn_percent": round(r.rel_error_wn_percent, 2),
            "rel_error_eigenvalue_percent": round(r.rel_error_eigenvalue_percent, 2),
        }
        for r in rows_bench
    ]

    rows_approx = lin.compare_analytical_approximations()
    approx_data = [
        {
            "mode_name": a.mode_name,
            "exact_4dof_wn": round(a.exact_4dof_wn, 4),
            "approx_2dof_wn": round(a.approx_2dof_wn, 4),
            "exact_4dof_zeta": round(a.exact_4dof_zeta, 4),
            "approx_2dof_zeta": round(a.approx_2dof_zeta, 4),
            "rel_error_wn_percent": round(a.rel_error_wn_percent, 2),
        }
        for a in rows_approx
    ]

    rk_res = lin.verify_integrator_order()

    val_payload = {
        "status": "success",
        "aircraft": ac.name,
        "benchmark_comparison": bench_data,
        "analytical_approximations": approx_data,
        "rk4_verification": {
            "estimated_order": round(rk_res["estimated_order"], 2),
            "is_fourth_order": rk_res["is_fourth_order"],
            "final_error_percent": rk_res["final_error_percent"]
        }
    }
    with open(os.path.join(OUTPUT_DIR, f"val_{ac_id}.json"), "w", encoding="utf-8") as f:
        json.dump(val_payload, f, ensure_ascii=False)

# Copia para default
with open(os.path.join(OUTPUT_DIR, "sim_b737_microburst.json"), "r", encoding="utf-8") as f_in:
    data_def = f_in.read()
with open(os.path.join(OUTPUT_DIR, "sim_default.json"), "w", encoding="utf-8") as f_out:
    f_out.write(data_def)

print("¡Datos estáticos generados exitosamente en web/data/!")
