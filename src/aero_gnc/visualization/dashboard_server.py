"""
Servidor Web de Telemetría e Interfaz Interactiva de Aproximación ILS.

Implementa un servidor HTTP asíncrono y ligero (basado en la biblioteca estándar de Python)
que provee la API REST para simulación dinámica en tiempo real y sirve el panel de control
interactivo web para visualización PFD (Primary Flight Display), instrumentación 3D y telemetría.
"""

import json
import os
import sys
import threading
import urllib.parse
from http.server import HTTPServer, SimpleHTTPRequestHandler
from typing import Dict, Any
import numpy as np

from aero_gnc.config.aircraft_database import get_aircraft, AVAILABLE_AIRCRAFT
from aero_gnc.config.airport_ils import DEFAULT_ILS
from aero_gnc.core.wind_turbulence import WindScenarioConfig
from aero_gnc.control.autopilot_longitudinal import LongitudinalGains
from aero_gnc.control.autopilot_lateral import LateralGains
from aero_gnc.simulator import AeroGNCSimulator, SimulationConfig
from aero_gnc.validation.modal_validation import FlightDynamicsLinearizer


class FlightDashboardHandler(SimpleHTTPRequestHandler):
    """Manejador HTTP para la API de simulación y recursos estáticos del cliente."""

    def __init__(self, *args, **kwargs):
        # Directorio base para archivos estáticos (HTML/CSS/JS)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))
        web_dir = os.path.join(project_root, "web")
        super().__init__(*args, directory=web_dir, **kwargs)

    def do_GET(self):
        """Maneja solicitudes GET (archivos web y endpoints de diagnóstico)."""
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path

        if path == "/api/aircraft":
            self._send_json({
                "aircraft": [
                    {"id": k, "name": v.name, "description": v.description, "v_ref": v.v_ref, "v_stall": v.v_stall}
                    for k, v in AVAILABLE_AIRCRAFT.items()
                ]
            })
        elif path == "/api/validation":
            self._handle_validation(parsed_url.query)
        elif path == "/api/simulate":
            # Permite ejecutar vía GET con parámetros en query string
            params = urllib.parse.parse_qs(parsed_url.query)
            clean_params = {k: v[0] for k, v in params.items()}
            self._execute_simulation(clean_params)
        else:
            # Servir archivo estático (index.html, styles.css, app.js)
            super().do_GET()

    def do_POST(self):
        """Maneja solicitudes POST (ejecución de simulación con parámetros de entrada)."""
        if self.path == "/api/simulate":
            content_length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(content_length).decode("utf-8")
            params = json.loads(body) if body else {}
            self._execute_simulation(params)
        else:
            self.send_error(404, "Endpoint no encontrado")

    def _execute_simulation(self, params: Dict[str, Any]):
        """Ejecuta la corrida de simulación y envía la telemetría en JSON."""
        try:
            ac_id = str(params.get("aircraft_id", "b737"))
            headwind = float(params.get("headwind", 5.0))
            crosswind = float(params.get("crosswind", 2.0))
            
            mb_val = params.get("microburst", True)
            if isinstance(mb_val, str):
                microburst = mb_val.lower() in ("true", "1", "yes")
            else:
                microburst = bool(mb_val)

            micro_u = float(params.get("microburst_delta_u", 14.0))
            micro_w = float(params.get("microburst_w_down", 5.5))
            turbulence = float(params.get("turbulence", 0.6))
            
            # Ganancias opcionales del autopiloto
            kp_pitch = float(params.get("kp_pitch", 1.6))
            kd_pitch = float(params.get("kd_pitch", 0.8))
            kp_roll = float(params.get("kp_roll", 1.8))
            
            w_cfg = WindScenarioConfig(
                headwind_mean=headwind,
                crosswind_mean=crosswind,
                enable_microburst=microburst,
                microburst_center_x=-1500.0,
                microburst_delta_u=micro_u,
                microburst_w_down=micro_w,
                turbulence_intensity=turbulence,
            )

            sim_cfg = SimulationConfig(
                aircraft_id=ac_id,
                duration_s=65.0,
                dt=0.02,
                initial_dist_m=-3800.0,
                initial_lateral_offset_m=float(params.get("lateral_offset", 18.0)),
                wind_config=w_cfg,
            )

            lon_gains = LongitudinalGains(kp_pitch=kp_pitch, kd_pitch=kd_pitch)
            lat_gains = LateralGains(kp_roll=kp_roll)

            sim = AeroGNCSimulator(config=sim_cfg, lon_gains=lon_gains, lat_gains=lat_gains)
            history, summary = sim.run()

            # Muestreo de datos para transmisión web fluida (paso de 50 ms / 20 Hz es ideal)
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

            # Serialización de resultados de compuertas
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
            self._send_json(response_data)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            self._send_json({"status": "error", "message": str(exc)}, status_code=500)

    def _handle_validation(self, query: str):
        """Calcula y retorna la validación cuantitativa de autovalores frente a literatura."""
        try:
            params = urllib.parse.parse_qs(query)
            ac_id = params.get("aircraft_id", ["b737"])[0]
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

            self._send_json({
                "status": "success",
                "aircraft": ac.name,
                "benchmark_comparison": bench_data,
                "analytical_approximations": approx_data,
                "rk4_verification": {
                    "estimated_order": round(rk_res["estimated_order"], 2),
                    "is_fourth_order": rk_res["is_fourth_order"],
                    "final_error_percent": rk_res["final_error_percent"]
                }
            })
        except Exception as exc:
            self._send_json({"status": "error", "message": str(exc)}, status_code=500)

    def _send_json(self, data: Dict[str, Any], status_code: int = 200):
        body = json.dumps(data).encode("utf-8")
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        """Silencia los logs de cada solicitud para no saturar la salida de terminal."""
        pass


def start_dashboard_server(port: int = 8080, blocking: bool = True) -> HTTPServer:
    """Inicia el servidor web interactivo local."""
    server_address = ("", port)
    httpd = HTTPServer(server_address, FlightDashboardHandler)
    url = f"http://localhost:{port}"
    print(f"\n=======================================================")
    print(f"  PANEL INTERACTIVO AERONÁUTICO ACTIVO (AERO-GNC)")
    print(f"  URL: {url}")
    print(f"  Presione Ctrl+C en la terminal para detener el servidor.")
    print(f"=======================================================\n")

    if blocking:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nDeteniendo servidor web...")
            httpd.server_close()
    else:
        t = threading.Thread(target=httpd.serve_forever, daemon=True)
        t.start()

    return httpd
