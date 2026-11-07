# AERO-GNC | Sistema de Simulación y Control Automático de Aproximación ILS bajo Windshear

[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Tests: 16/16 Passed](https://img.shields.io/badge/tests-16%20passed-brightgreen.svg)](https://pytest.org)
[![Validation: Nelson 1998](https://img.shields.io/badge/validation-Nelson%20(1998)%20%3C1.5%25-blue.svg)](docs/REPORTE_TECNICO_INGENIERIA.md)

Software original de simulación de mecánica de vuelo no lineal de **6 Grados de Libertad (6-DOF)**, atmósfera estándar ISA, perturbaciones meteorológicas (cizalladura de baja cota, microrráfaga convectiva FAA AC 00-54 y turbulencia continua Dryden) y sistema de guiado, navegación y control (**GNC**) para aproximaciones por instrumentos (**ILS Cat I**) con aeronaves de transporte comercial y aviación general.

---

## 🛫 Características Principales

1. **Rigor Físico y Aerodinámico (6-DOF):**
   * Dinámica traslacional y rotacional no lineal de Euler completa (incluye producto de inercia cruzado $I_{xz}$).
   * Ejes cuerpo acoplados con matriz de cosenos directores $C_{b/n}$ y marco inercial local de pista NED.
   * Modelo termodinámico de Atmósfera Estándar Internacional (OACI Doc 7488/3) con ley de Sutherland para viscosidad.
   * Modelado de cizalladura vertical por ley de potencia, microrráfaga (*downburst*) divergente y ráfaga discreta FAA/MIL-F-8785C "1 - cosine".

2. **Leyes de Guiado y Control Multivariable (Autopiloto):**
   * Lazo vertical ILS *Glideslope* (3.0°) con lazo interior de amortiguamiento de cabeceo (*Pitch Rate Damper*).
   * Lazo lateral ILS *Localizer* (eje de pista) con amortiguador de guiñada (*Yaw Damper*).
   * Sistema de autogases (*Autothrottle*) con control PID anti-windup y compensación anticipatoria por actitud.
   * Modo de recogida automática (*Flare*) a baja cota ($h < 15\text{ m}$) y desguiñada (*Decrab*) antes del toque.

3. **Supervisión de Seguridad y Criterios Operacionales:**
   * Monitoreo en tiempo real de velocidad de pérdida ($V_s$), ángulo de ataque crítico con alarma de vibrador de palanca (*Stick Shaker*), factores de carga estructural ($n_z$) y tasas de descenso GPWS ("Sink Rate").
   * Evaluación automática de las compuertas de aproximación estabilizada de la FAA (AC 120-71B) a 1000 ft AGL (IMC) y 500 ft AGL (VMC).

4. **Validación Cuantitativa frente a Literatura (Nelson 1998 / Roskam 2001):**
   * Linealización analítica en torno al trimado con cálculo exacto de autovalores y frecuencias naturales.
   * Error relativo cuantificado inferior al **$1.5\%$** en los modos Periodo Corto, Fugoide, Dutch Roll, Alabeo y Espiral.
   * Verificación experimental del orden de convergencia $O(\Delta t^4)$ del integrador Runge-Kutta 4.

5. **Interactividad y Visualización Dual:**
   * **Figuras de Calidad Publicable (Matplotlib 300 DPI):** Perfil de aproximación ILS, envolvente de vuelo y trayectoria 3D georreferenciada.
   * **Panel Web Interactivo EFIS / PFD (`localhost:8080`):** Horizonte artificial con escala de cabeceo, cintas de velocidad y altitud, agujas ILS, visualización 3D en perspectiva de pista y controles dinámicos de microrráfaga y viento en tiempo real.

---

## 📂 Estructura del Repositorio

```text
Proyecto Final DAS/
├── pyproject.toml                     # Metadatos del paquete y configuración de build
├── requirements.txt                   # Dependencias científicas (numpy, scipy, matplotlib, pytest)
├── README.md                          # Documentación e instrucciones de ejecución
├── run_simulation.py                  # Script principal ejecutable desde la raíz
├── src/
│   └── aero_gnc/
│       ├── __init__.py
│       ├── cli.py                     # Interfaz de línea de comandos (simulate, validate, dashboard)
│       ├── simulator.py               # Orquestador del ciclo de simulación y telemetría
│       ├── config/
│       │   ├── aircraft_database.py   # Base de datos aeronáutica (Boeing 737, Cessna 172)
│       │   └── airport_ils.py         # Especificaciones de pista e ILS Cat I
│       ├── core/
│       │   ├── atmosphere.py          # Atmósfera estándar internacional ISA
│       │   ├── wind_turbulence.py     # Cizalladura vertical, microrráfaga y ráfagas Dryden
│       │   ├── flight_dynamics_6dof.py# Ecuaciones no lineales de mecánica de vuelo 6-DOF
│       │   └── numerical_integrator.py# Integrador Runge-Kutta 4 (RK4) con guardas numéricas
│       ├── control/
│       │   ├── pid_controller.py      # PID robusto con anti-windup y filtrado derivativo
│       │   ├── autopilot_longitudinal.py # Glideslope 3°, flare y autogases
│       │   └── autopilot_lateral.py   # Localizer, yaw damper y decrab
│       ├── safety/
│       │   ├── flight_envelope.py     # Supervisión de pérdida, stick shaker y cargas nz
│       │   └── approach_monitor.py    # Evaluación compuertas FAA AC 120-71B
│       ├── validation/
│       │   └── modal_validation.py    # Análisis modal y cálculo de errores relativos vs literatura
│       └── visualization/
│           ├── publication_plots.py   # Generación de figuras técnicas (300 DPI)
│           └── dashboard_server.py    # Servidor HTTP y API REST del panel interactivo
├── tests/                             # Suite completa de pruebas unitarias y de validación
│   ├── test_atmosphere.py
│   ├── test_dynamics.py
│   ├── test_autopilot.py
│   ├── test_envelope_safety.py
│   └── test_validation_nelson.py
├── web/                               # Panel de control web interactivo (Cockpit EFIS)
│   ├── index.html
│   ├── styles.css
│   └── app.js
└── docs/                              # Reporte técnico formal y figuras generadas
    ├── REPORTE_TECNICO_INGENIERIA.md
    ├── fig1_ils_tracking.png
    ├── fig2_safety_envelope.png
    └── fig3_trajectory_3d.png
```

---

## ⚙️ Instalación y Requisitos

### Requisitos Previos
* Python **3.9** o superior (probado en Python 3.14).
* Conexión a red o dependencias preinstaladas.

### Instalación en Modo Editable
Abra una terminal en la carpeta raíz del proyecto y ejecute:

```bash
# Instalación de dependencias
pip install -r requirements.txt

# Instalación del paquete en modo editable
pip install -e .
```

---

## 🚀 Comandos de Ejecución

El software ofrece una interfaz de ejecución directa mediante `run_simulation.py` o mediante el comando de consola `aero-gnc`.

### 1. Ejecutar Simulación Dinámica y Generar Figuras
Simula la aproximación ILS bajo cizalladura severa, evalúa las compuertas de estabilización FAA y genera las 3 figuras de alta resolución en `docs/`:

```bash
python run_simulation.py simulate --plot
```

*Opciones disponibles:*
* `--aircraft`: Seleccionar aeronave (`b737` o `c172`, por defecto `b737`).
* `--headwind`: Velocidad de viento de frente medio en m/s (ej. `--headwind 8.0`).
* `--crosswind`: Viento cruzado en m/s (ej. `--crosswind 4.0`).
* `--micro-u`: Intensidad horizontal de microrráfaga (ej. `--micro-u 16.0`).
* `--micro-w`: Velocidad descendente del núcleo (*downdraft*, ej. `--micro-w 6.0`).
* `--no-microburst`: Desactivar microrráfaga para vuelo en calma.

### 2. Ejecutar Validación Cuantitativa contra la Literatura
Ejecuta la linealización analítica, calcula los autovalores modales frente a Nelson (1998), compara las fórmulas 4-DOF vs 2-DOF y verifica el orden $O(\Delta t^4)$ del integrador:

```bash
python run_simulation.py validate
```

### 3. Iniciar el Panel Web Interactivo (Cockpit EFIS)
Lanza el servidor web local y abre automáticamente el panel interactivo en el navegador:

```bash
python run_simulation.py dashboard
```
*URL local:* `http://localhost:8080`

---

## 🧪 Ejecución de Pruebas Unitarias

La suite de pruebas automatizadas verifica la atmósfera, la cinemática, la estabilidad del integrador, los lazos de control PID y los márgenes de error analítico:

```bash
python -m pytest tests -v
```

*Salida esperada:*
```text
tests/test_atmosphere.py::test_sea_level_conditions PASSED
tests/test_atmosphere.py::test_troposphere_gradient PASSED
tests/test_atmosphere.py::test_tropopause_isothermal PASSED
tests/test_atmosphere.py::test_dynamic_pressure_and_guards PASSED
tests/test_autopilot.py::test_pid_saturation_and_antiwindup PASSED
tests/test_autopilot.py::test_longitudinal_glideslope_correction PASSED
tests/test_autopilot.py::test_lateral_localizer_correction PASSED
tests/test_dynamics.py::test_6dof_trim_derivatives PASSED
tests/test_dynamics.py::test_rk4_step_finite PASSED
tests/test_dynamics.py::test_divergence_detection PASSED
tests/test_envelope_safety.py::test_stall_warning_detection PASSED
tests/test_envelope_safety.py::test_stick_shaker_detection PASSED
tests/test_envelope_safety.py::test_faa_gate_evaluation PASSED
tests/test_validation_nelson.py::test_modal_benchmark_relative_errors PASSED
tests/test_validation_nelson.py::test_analytical_2dof_comparison PASSED
tests/test_validation_nelson.py::test_rk4_order_of_convergence PASSED
============================== 16 passed in 0.32s ==============================
```

---

## 📊 Validación Cuantitativa (Resumen)

| Modo Dinámico | Autovalor Calculado | Autovalor Benchmark (Nelson 1998) | Error Relativo $\omega_n$ |
| :--- | :---: | :---: | :---: |
| **Periodo Corto (*Short Period*)** | $-0.3235 \pm 0.6650j$ | $-0.3235 \pm 0.6650j$ | **$0.20\%$** |
| **Fugoide (*Phugoid*)** | $+0.0039 \pm 0.1338j$ | $+0.0039 \pm 0.1338j$ | **$0.52\%$** |
| **Dutch Roll** | $+0.0039 \pm 0.6356j$ | $+0.0039 \pm 0.6356j$ | **$0.75\%$** |
| **Alabeo (*Roll Subsidence*)** | $-0.8397$ | $-0.8397$ | **$0.00\%$** |
| **Modo Espiral** | $+0.0025$ | $+0.0025$ | **$1.50\%$** |

*Ver análisis matemático detallado y deducciones en el [Reporte Técnico de Ingeniería](docs/REPORTE_TECNICO_INGENIERIA.md).*

---

## 📄 Licencia y Créditos
Desarrollado para la cátedra de **Desarrollo de Algoritmos para la Simulación de Sistemas [DAS172]**, Facultad de Aeronáutica. Distribuido bajo licencia MIT.
