"""
Orquestador Principal de Simulación de Vuelo y Guiado (AeroGNCSimulator).

Integra el modelo físico 6-DOF, atmósfera ISA, generador de perturbaciones meteorológicas,
autopiloto longitudinal/lateral, monitores de seguridad operacional e integrador RK4.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Any
import numpy as np

from aero_gnc.config.aircraft_database import AircraftData, get_aircraft
from aero_gnc.config.airport_ils import ILSSpecs, DEFAULT_ILS
from aero_gnc.core.atmosphere import StandardAtmosphere
from aero_gnc.core.wind_turbulence import AtmosphericDisturbance, WindScenarioConfig
from aero_gnc.core.flight_dynamics_6dof import (
    FlightDynamics6DOF,
    AircraftState,
    FlightControls,
)
from aero_gnc.core.numerical_integrator import (
    RK4Integrator,
    TrajectoryHistory,
    SimulationDivergenceError,
)
from aero_gnc.control.autopilot_longitudinal import (
    LongitudinalAutopilot,
    LongitudinalGains,
)
from aero_gnc.control.autopilot_lateral import (
    LateralAutopilot,
    LateralGains,
)
from aero_gnc.safety.flight_envelope import FlightEnvelopeMonitor
from aero_gnc.safety.approach_monitor import (
    StabilizedApproachMonitor,
    GateEvaluationResult,
)


@dataclass
class SimulationConfig:
    """Configuración de una corrida de simulación."""
    aircraft_id: str = "b737"
    duration_s: float = 65.0
    dt: float = 0.02
    initial_dist_m: float = -4000.0   # Distancia al umbral [m]
    initial_lateral_offset_m: float = 20.0  # Desviación inicial para probar captura ILS [m]
    wind_config: WindScenarioConfig = None

    def __post_init__(self):
        if self.wind_config is None:
            self.wind_config = WindScenarioConfig()


class AeroGNCSimulator:
    """Simulador integral acoplado de guiado, navegación y control (GNC)."""

    def __init__(
        self,
        config: SimulationConfig = None,
        ils: ILSSpecs = DEFAULT_ILS,
        lon_gains: LongitudinalGains = LongitudinalGains(),
        lat_gains: LateralGains = LateralGains(),
    ):
        self.cfg = config if config is not None else SimulationConfig()
        self.ils = ils
        self.aircraft = get_aircraft(self.cfg.aircraft_id)
        
        # Módulos del sistema
        self.disturbance = AtmosphericDisturbance(self.cfg.wind_config)
        self.dynamics = FlightDynamics6DOF(self.aircraft)
        self.autopilot_lon = LongitudinalAutopilot(self.aircraft, self.ils, lon_gains)
        self.autopilot_lat = LateralAutopilot(self.aircraft, self.ils, lat_gains)
        self.envelope_monitor = FlightEnvelopeMonitor(self.aircraft)
        self.approach_monitor = StabilizedApproachMonitor(self.aircraft, self.ils)
        self.integrator = RK4Integrator(dt=self.cfg.dt)

    def get_initial_state(self) -> AircraftState:
        """Calcula el estado inicial trimado sobre la senda de aproximación."""
        x0 = float(self.cfg.initial_dist_m)
        y0 = float(self.cfg.initial_lateral_offset_m)
        
        # Altitud nominal de senda de 3 deg a la distancia inicial
        h0 = self.autopilot_lon.compute_glideslope_target(x0)
        z0 = -h0  # En marco NED, +Z es hacia abajo
        
        # Determinar viento inicial para trimar TAS exactamente a v_ref
        w0 = self.disturbance.get_wind_ned(x0, y0, z0, 0.0)
        u0 = max(self.aircraft.v_stall + 4.0, self.aircraft.v_ref + w0[0])
        theta0 = self.autopilot_lon.theta_trim
        psi0 = self.ils.runway_heading_rad
        thrust0 = self.aircraft.idle_thrust + self.autopilot_lon.throttle_trim * (
            self.aircraft.max_thrust - self.aircraft.idle_thrust
        )

        return AircraftState(
            u=u0, v=0.0, w=0.0,
            p=0.0, q=0.0, r=0.0,
            phi=0.0, theta=theta0, psi=psi0,
            x_n=x0, y_e=y0, z_d=z0,
            thrust=thrust0
        )

    def run(self) -> Tuple[TrajectoryHistory, Dict[str, Any]]:
        """
        Ejecuta la simulación temporal completa.
        
        Retorna:
        - history: Contenedor TrajectoryHistory con arrays NumPy vectorizados.
        - summary: Diccionario con estadísticas de desempeño, compuertas y seguridad.
        """
        state = self.get_initial_state()
        dt = self.cfg.dt
        n_steps = int(self.cfg.duration_s / dt)

        # Buffers pre-asignados para máxima eficiencia numérica (sin append en bucle)
        times = np.zeros(n_steps, dtype=np.float64)
        states = np.zeros((n_steps, 13), dtype=np.float64)
        controls = np.zeros((n_steps, 4), dtype=np.float64)
        winds = np.zeros((n_steps, 3), dtype=np.float64)
        
        # Diccionario de telemetría vectorizada
        telemetry_keys = [
            "v_tas", "mach", "alpha_deg", "beta_deg", "dynamic_pressure_pa",
            "lift_n", "drag_n", "thrust_n", "n_z", "n_y", "altitude_m",
            "vertical_speed_mps", "vertical_speed_fpm", "flight_path_angle_deg",
            "h_nominal", "h_error_m", "gs_dots", "loc_dots", "phi_cmd_deg", "theta_cmd_deg"
        ]
        telemetry_buffers = {k: np.zeros(n_steps, dtype=np.float64) for k in telemetry_keys}

        events: List[str] = []
        actual_steps = 0
        touchdown_occurred = False
        touchdown_info = {}

        for step_idx in range(n_steps):
            t = step_idx * dt
            times[step_idx] = t
            states[step_idx] = state.to_array()

            # 1. Viento perturbador en la posición actual
            w_ned = self.disturbance.get_wind_ned(state.x_n, state.y_e, state.z_d, t)
            winds[step_idx] = w_ned

            # 2. Dinámica instantánea y derivadas previas para telemetría
            _, step_telem = self.dynamics.compute_derivatives(state, FlightControls(), w_ned, t)

            # 3. Lazos de Autopiloto
            # Longitudinal
            delta_e, throttle_cmd, lon_debug = self.autopilot_lon.update(
                altitude=state.altitude,
                x_pos=state.x_n,
                theta=state.theta,
                q=state.q,
                v_tas=step_telem["v_tas"],
                v_z=step_telem["vertical_speed_mps"],
                dt=dt
            )

            # Lateral-Direccional
            # Velocidad transversal inercial y_dot
            R_nb = self.dynamics.rotation_matrix_body_to_ned(state.phi, state.theta, state.psi)
            vel_ned = R_nb @ np.array([state.u, state.v, state.w])
            v_lateral = vel_ned[1]

            delta_a, delta_r, lat_debug = self.autopilot_lat.update(
                y_pos=state.y_e,
                v_lateral=v_lateral,
                phi=state.phi,
                psi=state.psi,
                p=state.p,
                r=state.r,
                beta=np.radians(step_telem["beta_deg"]),
                altitude=state.altitude,
                x_pos=state.x_n,
                dt=dt
            )

            ctrl = FlightControls(
                delta_e=delta_e,
                delta_a=delta_a,
                delta_r=delta_r,
                throttle=throttle_cmd
            )
            controls[step_idx] = [ctrl.delta_e, ctrl.delta_a, ctrl.delta_r, ctrl.throttle]

            # Registro de telemetría
            telemetry_buffers["v_tas"][step_idx] = step_telem["v_tas"]
            telemetry_buffers["mach"][step_idx] = step_telem["mach"]
            telemetry_buffers["alpha_deg"][step_idx] = step_telem["alpha_deg"]
            telemetry_buffers["beta_deg"][step_idx] = step_telem["beta_deg"]
            telemetry_buffers["dynamic_pressure_pa"][step_idx] = step_telem["dynamic_pressure_pa"]
            telemetry_buffers["lift_n"][step_idx] = step_telem["lift_n"]
            telemetry_buffers["drag_n"][step_idx] = step_telem["drag_n"]
            telemetry_buffers["thrust_n"][step_idx] = step_telem["thrust_n"]
            telemetry_buffers["n_z"][step_idx] = step_telem["n_z"]
            telemetry_buffers["n_y"][step_idx] = step_telem["n_y"]
            telemetry_buffers["altitude_m"][step_idx] = step_telem["altitude_m"]
            telemetry_buffers["vertical_speed_mps"][step_idx] = step_telem["vertical_speed_mps"]
            telemetry_buffers["vertical_speed_fpm"][step_idx] = step_telem["vertical_speed_fpm"]
            telemetry_buffers["flight_path_angle_deg"][step_idx] = step_telem["flight_path_angle_deg"]
            
            telemetry_buffers["h_nominal"][step_idx] = lon_debug["h_nominal"]
            telemetry_buffers["h_error_m"][step_idx] = lon_debug["h_error_m"]
            telemetry_buffers["gs_dots"][step_idx] = lon_debug["gs_dots"]
            telemetry_buffers["theta_cmd_deg"][step_idx] = lon_debug["theta_cmd_deg"]
            
            telemetry_buffers["loc_dots"][step_idx] = lat_debug["loc_dots"]
            telemetry_buffers["phi_cmd_deg"][step_idx] = lat_debug["phi_cmd_deg"]

            # 4. Monitoreo de Seguridad y Compuertas
            self.envelope_monitor.check(
                t=t,
                v_tas=step_telem["v_tas"],
                alpha_deg=step_telem["alpha_deg"],
                phi_deg=np.degrees(state.phi),
                theta_deg=np.degrees(state.theta),
                n_z=step_telem["n_z"],
                vertical_speed_mps=step_telem["vertical_speed_mps"],
                altitude_m=state.altitude
            )

            gate_event = self.approach_monitor.evaluate_step(
                t=t,
                altitude_m=state.altitude,
                gs_dots=lon_debug["gs_dots"],
                loc_dots=lat_debug["loc_dots"],
                v_tas=step_telem["v_tas"],
                vertical_speed_mps=step_telem["vertical_speed_mps"],
                throttle=ctrl.throttle
            )
            if gate_event is not None:
                status_str = "ESTABILIZADA" if gate_event.is_stabilized else "NO ESTABILIZADA"
                events.append(f"t={t:.2f} s: Cruce de {gate_event.gate_name} -> {status_str}")

            # 5. Detección de contacto con el suelo (Touchdown)
            if state.altitude <= 0.05 and state.x_n >= -100.0:
                touchdown_occurred = True
                touchdown_info = {
                    "time_s": t,
                    "x_touchdown_m": float(state.x_n),
                    "y_touchdown_m": float(state.y_e),
                    "vertical_speed_fpm": float(step_telem["vertical_speed_fpm"]),
                    "pitch_deg": float(np.degrees(state.theta)),
                    "bank_deg": float(np.degrees(state.phi)),
                    "v_tas_kts": float(step_telem["v_tas"] * 1.94384),
                }
                events.append(
                    f"t={t:.2f} s: ¡TOMA DE CONTACTO (TOUCHDOWN)! x={state.x_n:.1f} m, y={state.y_e:.1f} m, Vz={step_telem['vertical_speed_fpm']:.0f} ft/min"
                )
                actual_steps = step_idx + 1
                break

            # 6. Integración RK4
            next_state, _ = self.integrator.step(
                self.dynamics.compute_derivatives,
                state,
                ctrl,
                w_ned,
                t
            )
            state = next_state
            actual_steps = step_idx + 1

        # Truncar los arrays al número real de pasos simulados
        times = times[:actual_steps]
        states = states[:actual_steps]
        controls = controls[:actual_steps]
        winds = winds[:actual_steps]
        telemetry = {k: v[:actual_steps] for k, v in telemetry_buffers.items()}

        history = TrajectoryHistory(
            time=times,
            state=states,
            controls=controls,
            telemetry=telemetry,
            wind=winds,
            events=events
        )

        summary = {
            "aircraft": self.aircraft.name,
            "duration_simulated_s": float(times[-1] if len(times) > 0 else 0.0),
            "touchdown_occurred": touchdown_occurred,
            "touchdown_info": touchdown_info,
            "imc_gate": self.approach_monitor.imc_result,
            "vmc_gate": self.approach_monitor.vmc_result,
            "total_safety_alerts": len(self.envelope_monitor.alerts_history),
            "events": events
        }

        return history, summary
