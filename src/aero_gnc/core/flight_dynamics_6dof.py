"""
Motor de Dinámica de Vuelo No Lineal de 6 Grados de Libertad (6-DOF).

Implementa las ecuaciones diferenciales no lineales completas de la mecánica de vuelo:
- Cinemática y dinámica traslacional en ejes cuerpo.
- Dinámica rotacional con tensor de inercia y producto de inercia Ixz.
- Cinemática de ángulos de Euler (alabeo, cabeceo, guiñada).
- Transformación al marco inercial terrestre NED.
- Aerodinámica acoplada con derivadas de estabilidad e influencia de viento relativo.
- Dinámica de actuador propulsivo de primer orden.
"""

from dataclasses import dataclass
from typing import Dict, Any, Tuple
import numpy as np

from aero_gnc.config.aircraft_database import AircraftData
from aero_gnc.core.atmosphere import StandardAtmosphere, G0


@dataclass
class FlightControls:
    """Comandos de las superficies de control y propulsión."""
    delta_e: float = 0.0      # Timón de profundidad [rad] (positivo hacia abajo)
    delta_a: float = 0.0      # Alerones [rad] (positivo alabeo a derecha)
    delta_r: float = 0.0      # Timón de dirección [rad] (positivo guiñada a izquierda)
    throttle: float = 0.35    # Palanca de gases normalizada [0.0, 1.0]

    def clamped(self) -> "FlightControls":
        """Aplica límites físicos estándar de deflexión mecánica de actuadores."""
        # Elevador [-25 deg, +15 deg], Alerones [-20 deg, +20 deg], Rudder [-30 deg, +30 deg]
        de = np.clip(self.delta_e, np.radians(-25.0), np.radians(15.0))
        da = np.clip(self.delta_a, np.radians(-20.0), np.radians(20.0))
        dr = np.clip(self.delta_r, np.radians(-30.0), np.radians(30.0))
        th = np.clip(self.throttle, 0.0, 1.0)
        return FlightControls(delta_e=de, delta_a=da, delta_r=dr, throttle=th)


@dataclass
class AircraftState:
    """Estado instantáneo completo de la aeronave (13 variables de estado)."""
    # Velocidades lineales en ejes cuerpo [m/s]
    u: float
    v: float
    w: float
    
    # Velocidades angulares en ejes cuerpo [rad/s]
    p: float
    q: float
    r: float
    
    # Actitud en ángulos de Euler [rad]
    phi: float    # Alabeo (roll)
    theta: float  # Cabeceo (pitch)
    psi: float    # Rumbo (yaw)
    
    # Posición inercial NED [m]
    x_n: float    # Posición Norte (+X avance)
    y_e: float    # Posición Este (+Y lateral derecha)
    z_d: float    # Posición Abajo (+Z hacia centro de la Tierra, altitud h = -z_d)
    
    # Empuje actual del motor [N]
    thrust: float

    def to_array(self) -> np.ndarray:
        return np.array([
            self.u, self.v, self.w,
            self.p, self.q, self.r,
            self.phi, self.theta, self.psi,
            self.x_n, self.y_e, self.z_d,
            self.thrust
        ], dtype=np.float64)

    @classmethod
    def from_array(cls, arr: np.ndarray) -> "AircraftState":
        return cls(
            u=float(arr[0]), v=float(arr[1]), w=float(arr[2]),
            p=float(arr[3]), q=float(arr[4]), r=float(arr[5]),
            phi=float(arr[6]), theta=float(arr[7]), psi=float(arr[8]),
            x_n=float(arr[9]), y_e=float(arr[10]), z_d=float(arr[11]),
            thrust=float(arr[12])
        )

    @property
    def altitude(self) -> float:
        """Altitud geométrica sobre el terreno h = -z_d [m]."""
        return -self.z_d


class FlightDynamics6DOF:
    """
    Simulador físico no lineal de 6 Grados de Libertad.
    """

    def __init__(self, aircraft: AircraftData):
        self.ac = aircraft
        self.ac.validate()
        
        # Pre-cálculo de constantes inerciales para optimización algorítmica (evita bucles e inversiones)
        self.gamma_inertia = self.ac.ixx * self.ac.izz - (self.ac.ixz ** 2)
        if self.gamma_inertia <= 0:
            raise ValueError("Tensor de inercia inválido: Ixx * Izz - Ixz^2 <= 0")

    def rotation_matrix_body_to_ned(self, phi: float, theta: float, psi: float) -> np.ndarray:
        """Matriz de cosenos directores C_n/b (de ejes cuerpo a ejes terrestres NED)."""
        c_ph, s_ph = np.cos(phi), np.sin(phi)
        c_th, s_th = np.cos(theta), np.sin(theta)
        c_ps, s_ps = np.cos(psi), np.sin(psi)

        R = np.array([
            [c_th * c_ps, s_ph * s_th * c_ps - c_ph * s_ps, c_ph * s_th * c_ps + s_ph * s_ps],
            [c_th * s_ps, s_ph * s_th * s_ps + c_ph * c_ps, c_ph * s_th * s_ps - s_ph * c_ps],
            [-s_th,       s_ph * c_th,                     c_ph * c_th]
        ], dtype=np.float64)
        return R

    def compute_derivatives(
        self,
        state: AircraftState,
        controls: FlightControls,
        wind_ned: np.ndarray,
        t: float = 0.0
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """
        Calcula las derivadas temporales d(state)/dt y telemetría diagnóstica.
        
        Retorna:
        - state_dot: array numpy de 13 elementos.
        - telemetry: diccionario con variables calculadas (V_a, alpha, beta, n_z, q_inf, etc.).
        """
        ctrl = controls.clamped()
        
        # 1. Transformación de viento de NED a Ejes Cuerpo
        R_nb = self.rotation_matrix_body_to_ned(state.phi, state.theta, state.psi)
        R_bn = R_nb.T  # De NED a Cuerpo
        wind_body = R_bn @ wind_ned
        
        # 2. Viento relativo y variables aerodinámicas
        u_rel = state.u - wind_body[0]
        v_rel = state.v - wind_body[1]
        w_rel = state.w - wind_body[2]
        
        # Velocidad aerodinámica verdadera (True Airspeed - TAS)
        v_tas = np.sqrt(u_rel ** 2 + v_rel ** 2 + w_rel ** 2)
        v_tas_safe = max(v_tas, 1.0)  # Evita división por cero
        
        # Ángulo de ataque alpha y ángulo de resbalamiento lateral beta
        alpha = np.arctan2(w_rel, max(u_rel, 0.1))
        # Protección de arcoseno para beta
        sin_beta = np.clip(v_rel / v_tas_safe, -0.999, 0.999)
        beta = np.arcsin(sin_beta)
        
        # 3. Propiedades atmosféricas
        atm = StandardAtmosphere.compute(state.altitude)
        q_inf = StandardAtmosphere.dynamic_pressure(atm.density, v_tas)
        mach = v_tas / atm.speed_of_sound
        
        # 4. Tasas adimensionales de amortiguamiento aerodinámico
        q_hat = (state.q * self.ac.mean_chord) / (2.0 * v_tas_safe)
        p_hat = (state.p * self.ac.wing_span) / (2.0 * v_tas_safe)
        r_hat = (state.r * self.ac.wing_span) / (2.0 * v_tas_safe)
        
        # 5. Coeficientes aerodinámicos
        # Modelo aerodinámico no lineal con amortiguamiento de pérdida suave
        alpha_deg = np.degrees(alpha)
        if abs(alpha_deg) <= self.ac.alpha_stall_deg:
            c_l = self.ac.cl_0 + self.ac.cl_alpha * alpha + self.ac.cl_q * q_hat + self.ac.cl_delta_e * ctrl.delta_e
            c_d = self.ac.cd_0 + self.ac.cd_alpha * (alpha ** 2) + self.ac.cd_delta_e * abs(ctrl.delta_e)
        else:
            # Régimen post-pérdida: saturación de sustentación y aumento drástico de resistencia
            stall_sign = np.sign(alpha)
            stall_rad = np.radians(self.ac.alpha_stall_deg)
            c_l_peak = self.ac.cl_0 + self.ac.cl_alpha * (stall_sign * stall_rad)
            c_l = c_l_peak * np.cos(alpha - stall_sign * stall_rad)
            c_d = self.ac.cd_0 + self.ac.cd_alpha * (stall_rad ** 2) + 1.2 * (np.sin(alpha) ** 2)
        
        # Coeficiente de momento de cabeceo
        c_m = (
            self.ac.cm_0
            + self.ac.cm_alpha * alpha
            + self.ac.cm_q * q_hat
            + self.ac.cm_delta_e * ctrl.delta_e
        )
        
        # Lateral-direccional
        c_y = (
            self.ac.cy_beta * beta
            + self.ac.cy_p * p_hat
            + self.ac.cy_r * r_hat
            + self.ac.cy_delta_r * ctrl.delta_r
        )
        c_roll = (
            self.ac.cl_beta * beta
            + self.ac.cl_p * p_hat
            + self.ac.cl_r * r_hat
            + self.ac.cl_delta_a * ctrl.delta_a
            + self.ac.cl_delta_r * ctrl.delta_r
        )
        c_yaw = (
            self.ac.cn_beta * beta
            + self.ac.cn_p * p_hat
            + self.ac.cn_r * r_hat
            + self.ac.cn_delta_a * ctrl.delta_a
            + self.ac.cn_delta_r * ctrl.delta_r
        )
        
        # 6. Fuerzas aerodinámicas en ejes estabilidad convertidas a ejes cuerpo
        cos_a = np.cos(alpha)
        sin_a = np.sin(alpha)
        
        lift = q_inf * self.ac.wing_area * c_l
        drag = q_inf * self.ac.wing_area * c_d
        side_force = q_inf * self.ac.wing_area * c_y
        
        f_x_aero = -drag * cos_a + lift * sin_a
        f_z_aero = -lift * cos_a - drag * sin_a
        f_y_aero = side_force
        
        # Momentos aerodinámicos [N*m]
        l_aero = q_inf * self.ac.wing_area * self.ac.wing_span * c_roll
        m_aero = q_inf * self.ac.wing_area * self.ac.mean_chord * c_m
        n_aero = q_inf * self.ac.wing_area * self.ac.wing_span * c_yaw
        
        # 7. Fuerzas gravitacionales en ejes cuerpo
        g_x = -G0 * np.sin(state.theta)
        g_y = G0 * np.cos(state.theta) * np.sin(state.phi)
        g_z = G0 * np.cos(state.theta) * np.cos(state.phi)
        
        # 8. Dinámica del motor (primer orden con saturación)
        thrust_cmd = self.ac.idle_thrust + ctrl.throttle * (self.ac.max_thrust - self.ac.idle_thrust)
        thrust_dot = (thrust_cmd - state.thrust) / max(0.1, self.ac.engine_tau)
        
        # 9. Ecuaciones de aceleración traslacional (Cuerpo)
        u_dot = state.r * state.v - state.q * state.w + g_x + (f_x_aero + state.thrust) / self.ac.mass
        v_dot = state.p * state.w - state.r * state.u + g_y + f_y_aero / self.ac.mass
        w_dot = state.q * state.u - state.p * state.v + g_z + f_z_aero / self.ac.mass
        
        # 10. Ecuaciones rotacionales de Euler (con momento de inercia cruzado Ixz)
        p, q, r = state.p, state.q, state.r
        ixx, iyy, izz, ixz = self.ac.ixx, self.ac.iyy, self.ac.izz, self.ac.ixz
        gamma = self.gamma_inertia
        
        # Términos giroscópicos inerciales
        p_dot = (
            izz * (l_aero - (izz - iyy) * q * r + ixz * p * q)
            + ixz * (n_aero - (iyy - ixx) * p * q - ixz * q * r)
        ) / gamma
        
        q_dot = (
            m_aero - (ixx - izz) * p * r - ixz * (p ** 2 - r ** 2)
        ) / iyy
        
        r_dot = (
            ixz * (l_aero - (izz - iyy) * q * r + ixz * p * q)
            + ixx * (n_aero - (iyy - ixx) * p * q - ixz * q * r)
        ) / gamma
        
        # 11. Cinemática de ángulos de Euler
        # Protección contra singularidad de gimbal lock (pitch = +-90 deg)
        cos_theta = np.cos(state.theta)
        if abs(cos_theta) < 1e-4:
            cos_theta = 1e-4 * np.sign(cos_theta)
        tan_theta = np.sin(state.theta) / cos_theta
        
        phi_dot = p + (q * np.sin(state.phi) + r * np.cos(state.phi)) * tan_theta
        theta_dot = q * np.cos(state.phi) - r * np.sin(state.phi)
        psi_dot = (q * np.sin(state.phi) + r * np.cos(state.phi)) / cos_theta
        
        # 12. Cinemática de posición inercial NED
        vel_body = np.array([state.u, state.v, state.w], dtype=np.float64)
        vel_ned = R_nb @ vel_body
        
        pos_n_dot = vel_ned[0]
        pos_e_dot = vel_ned[1]
        pos_d_dot = vel_ned[2]
        
        # Vector final de derivadas temporales
        state_dot = np.array([
            u_dot, v_dot, w_dot,
            p_dot, q_dot, r_dot,
            phi_dot, theta_dot, psi_dot,
            pos_n_dot, pos_e_dot, pos_d_dot,
            thrust_dot
        ], dtype=np.float64)
        
        # Factores de carga n_z y n_y
        n_z = -f_z_aero / (self.ac.mass * G0)
        n_y = f_y_aero / (self.ac.mass * G0)
        
        telemetry = {
            "v_tas": float(v_tas),
            "mach": float(mach),
            "alpha_deg": float(alpha_deg),
            "beta_deg": float(np.degrees(beta)),
            "dynamic_pressure_pa": float(q_inf),
            "lift_n": float(lift),
            "drag_n": float(drag),
            "side_force_n": float(side_force),
            "thrust_n": float(state.thrust),
            "n_z": float(n_z),
            "n_y": float(n_y),
            "ground_speed": float(np.linalg.norm(vel_ned[:2])),
            "vertical_speed_mps": float(-pos_d_dot),  # En aviación vz positiva hacia arriba
            "vertical_speed_fpm": float(-pos_d_dot * 196.85),
            "flight_path_angle_deg": float(np.degrees(np.arctan2(-pos_d_dot, max(vel_ned[0], 0.1)))),
            "altitude_m": float(state.altitude),
        }
        
        return state_dot, telemetry
