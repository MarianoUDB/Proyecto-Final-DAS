"""
Interfaz de Línea de Comandos (CLI) de AERO-GNC.

Permite:
- Ejecutar escenarios de simulación y generar reportes técnicos de texto.
- Generar figuras de calidad publicable (Matplotlib 300 DPI).
- Ejecutar la suite de validación analítica modal.
- Iniciar el servidor del panel interactivo web (Dashboard EFIS).
"""

import argparse
import sys
import webbrowser

if hasattr(sys.stdout, "reconfigure"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

from aero_gnc.config.aircraft_database import get_aircraft, AVAILABLE_AIRCRAFT
from aero_gnc.core.wind_turbulence import WindScenarioConfig
from aero_gnc.simulator import AeroGNCSimulator, SimulationConfig
from aero_gnc.validation.modal_validation import FlightDynamicsLinearizer
from aero_gnc.visualization.publication_plots import (
    plot_ils_tracking,
    plot_safety_envelope,
    plot_3d_trajectory,
)
from aero_gnc.visualization.dashboard_server import start_dashboard_server


def cmd_simulate(args):
    """Ejecuta una corrida de simulación desde terminal."""
    print(f"\n=======================================================")
    print(f"  AERO-GNC // EJECUCIÓN DE SIMULACIÓN 6-DOF")
    print(f"  Aeronave: {args.aircraft.upper()} | Duración: {args.duration} s")
    print(f"=======================================================\n")

    w_cfg = WindScenarioConfig(
        headwind_mean=args.headwind,
        crosswind_mean=args.crosswind,
        enable_microburst=not args.no_microburst,
        microburst_delta_u=args.micro_u,
        microburst_w_down=args.micro_w,
        turbulence_intensity=args.turbulence
    )

    sim_cfg = SimulationConfig(
        aircraft_id=args.aircraft,
        duration_s=args.duration,
        dt=args.dt,
        initial_dist_m=args.initial_dist,
        initial_lateral_offset_m=args.initial_lateral,
        wind_config=w_cfg
    )

    sim = AeroGNCSimulator(config=sim_cfg)
    history, summary = sim.run()

    print(f"--- RESUMEN DE LA APROXIMACIÓN ---")
    print(f"Tiempo simulado: {summary['duration_simulated_s']:.2f} s ({len(history.time)} pasos)")
    print(f"Contacto con suelo (Touchdown): {'SÍ' if summary['touchdown_occurred'] else 'NO'}")
    
    if summary["touchdown_occurred"]:
        td = summary["touchdown_info"]
        print(f"  > Distancia de toque: x = {td['x_touchdown_m']:.1f} m tras umbral")
        print(f"  > Desviación de eje:   y = {td['y_touchdown_m']:+.2f} m")
        print(f"  > Tasa de descenso:    Vz = {td['vertical_speed_fpm']:.0f} ft/min")
        print(f"  > Actitud de cabeceo:  theta = {td['pitch_deg']:.1f}°")
        print(f"  > Actitud de alabeo:   phi = {td['bank_deg']:+.1f}°")
        print(f"  > Velocidad al toque:  TAS = {td['v_tas_kts']:.1f} kts")

    print(f"\n--- EVALUACIÓN COMPUERTAS FAA AC 120-71B ---")
    if summary["imc_gate"]:
        g = summary["imc_gate"]
        status = "ESTABILIZADA [PASÓ]" if g.is_stabilized else "DESESTABILIZADA [FALLÓ]"
        print(f"Compuerta IMC (1000 ft AGL): {status}")
        print(f"  Desviación GS: {g.gs_dots:+.2f} dots | LOC: {g.loc_dots:+.2f} dots | dV: {g.speed_error_kts:+.1f} kts | Vz: {g.descent_rate_fpm:.0f} fpm")
        if g.discrepancies:
            for d in g.discrepancies:
                print(f"   [!] {d}")

    if summary["vmc_gate"]:
        g = summary["vmc_gate"]
        status = "ESTABILIZADA [PASÓ]" if g.is_stabilized else "DESESTABILIZADA [FALLÓ]"
        print(f"Compuerta VMC (500 ft AGL):  {status}")
        print(f"  Desviación GS: {g.gs_dots:+.2f} dots | LOC: {g.loc_dots:+.2f} dots | dV: {g.speed_error_kts:+.1f} kts | Vz: {g.descent_rate_fpm:.0f} fpm")
        if g.discrepancies:
            for d in g.discrepancies:
                print(f"   [!] {d}")

    print(f"\nTotal de alertas de seguridad registradas: {summary['total_safety_alerts']}")

    if args.plot:
        print("\nGenerando figuras técnicas para publicación (Matplotlib 300 DPI)...")
        plot_ils_tracking(history, save_path="docs/fig1_ils_tracking.png")
        plot_safety_envelope(history, save_path="docs/fig2_safety_envelope.png")
        plot_3d_trajectory(history, save_path="docs/fig3_trajectory_3d.png")
        print("Gráficos generados correctamente en la carpeta docs/.")


def cmd_validate(args):
    """Ejecuta la suite de validación analítica modal y verificación de convergencia."""
    ac = get_aircraft(args.aircraft)
    lin = FlightDynamicsLinearizer(ac)

    print(f"\n=========================================================================================")
    print(f"  AERO-GNC // VALIDACIÓN CUANTITATIVA DEL MODELO AERONÁUTICO ({ac.name.upper()})")
    print(f"=========================================================================================\n")

    print(f"1. VALIDACIÓN CONTRA BENCHMARKS DE LITERATURA (NELSON 1998, ROSKAM 2001)")
    print(f"{'-'*90}")
    print(f"{'Modo Dinámico':32} | {'Autovalor Calc.':20} | {'Autovalor Ref.':20} | {'Err wn %':9} | {'Err Eig %':9}")
    print(f"{'-'*90}")
    for r in lin.compare_against_reference():
        print(
            f"{r.mode_name:32} | {r.computed_eigenvalue.real:+.4f}{r.computed_eigenvalue.imag:+.4f}j   | "
            f"{r.reference_eigenvalue.real:+.4f}{r.reference_eigenvalue.imag:+.4f}j   | "
            f"{r.rel_error_wn_percent:7.2f}% | {r.rel_error_eigenvalue_percent:7.2f}%"
        )
    print(f"{'-'*90}")

    print(f"\n2. CONTRASTE MODELO 4-DOF VS APROXIMACIONES ANALÍTICAS CLÁSICAS 2-DOF (NELSON)")
    print(f"{'-'*90}")
    print(f"{'Modo Dinámico':32} | {'wn 4-DOF [rad/s]':18} | {'wn 2-DOF [rad/s]':18} | {'Discrepancia %':14}")
    print(f"{'-'*90}")
    for a in lin.compare_analytical_approximations():
        print(
            f"{a.mode_name:32} | {a.exact_4dof_wn:18.4f} | {a.approx_2dof_wn:18.4f} | {a.rel_error_wn_percent:12.2f}%"
        )
    print(f"{'-'*90}")

    print(f"\n3. VERIFICACIÓN DEL ORDEN DE CONVERGENCIA NUMÉRICA DEL INTEGRADOR RK4")
    rk = lin.verify_integrator_order()
    print(f"  Orden de convergencia experimental: p = {rk['estimated_order']:.2f} (Teórico: 4.00)")
    print(f"  Convergencia de cuarto orden confirmada: {'SÍ (O(dt^4))' if rk['is_fourth_order'] else 'NO'}")
    print(f"  Error global relativo al final del intervalo: {rk['final_error_percent']:.6e} %")
    print(f"=========================================================================================\n")


def cmd_dashboard(args):
    """Inicia el servidor web interactivo y abre el navegador."""
    url = f"http://localhost:{args.port}"
    if not args.no_browser:
        webbrowser.open(url)
    start_dashboard_server(port=args.port, blocking=True)


def main():
    parser = argparse.ArgumentParser(
        prog="aero-gnc",
        description="AERO-GNC: Simulación y Control Automático de Aproximación ILS bajo Cizalladura y Turbulencia"
    )
    subparsers = parser.add_subparsers(dest="command", help="Comandos disponibles")

    # Comando simulate
    p_sim = subparsers.add_parser("simulate", help="Ejecutar simulación dinámica")
    p_sim.add_argument("--aircraft", choices=list(AVAILABLE_AIRCRAFT.keys()), default="b737", help="Aeronave")
    p_sim.add_argument("--duration", type=float, default=65.0, help="Duración en segundos")
    p_sim.add_argument("--dt", type=float, default=0.02, help="Paso de integración en segundos")
    p_sim.add_argument("--headwind", type=float, default=5.0, help="Viento de frente medio [m/s]")
    p_sim.add_argument("--crosswind", type=float, default=2.0, help="Viento cruzado medio [m/s]")
    p_sim.add_argument("--no-microburst", action="store_true", help="Desactivar microrráfaga")
    p_sim.add_argument("--micro-u", type=float, default=14.0, help="Cizalladura horizontal microrráfaga [m/s]")
    p_sim.add_argument("--micro-w", type=float, default=5.5, help="Núcleo descendente microrráfaga [m/s]")
    p_sim.add_argument("--turbulence", type=float, default=0.6, help="Intensidad de turbulencia [m/s]")
    p_sim.add_argument("--initial-dist", type=float, default=-3800.0, help="Distancia inicial al umbral [m]")
    p_sim.add_argument("--initial-lateral", type=float, default=18.0, help="Desviación lateral inicial [m]")
    p_sim.add_argument("--plot", action="store_true", help="Generar figuras técnicas en docs/")

    # Comando validate
    p_val = subparsers.add_parser("validate", help="Ejecutar validación analítica modal frente a literatura")
    p_val.add_argument("--aircraft", choices=list(AVAILABLE_AIRCRAFT.keys()), default="b737", help="Aeronave")

    # Comando dashboard
    p_dash = subparsers.add_parser("dashboard", help="Iniciar panel de control web interactivo (EFIS / PFD)")
    p_dash.add_argument("--port", type=int, default=8080, help="Puerto HTTP local")
    p_dash.add_argument("--no-browser", action="store_true", help="No abrir automáticamente el navegador")

    args = parser.parse_args()

    if args.command == "simulate":
        cmd_simulate(args)
    elif args.command == "validate":
        cmd_validate(args)
    elif args.command == "dashboard":
        cmd_dashboard(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
