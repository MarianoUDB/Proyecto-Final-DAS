RÚBRICA DE EVALUACIÓN: PROYECTO INTEGRADOR DE
DESARROLLO DE SOFTWARE AERONÁUTICO
Facultad: Aeronáutica
Carrera: Ingeniería en Aeronáutica
Asignatura: Desarrollo de Algoritmos para la Simulación de Sistemas [DAS172]
I. Descripción y Alcance del Proyecto
El equipo de trabajo debe concebir, modelar analíticamente, programar y validar una solución de software original orientada a un
problema específico del ámbito aeronáutico o aeroespacial.
Campos de Aplicación Sugeridos (a elección del equipo):
● Aerodinámica y Aeroelasticidad: Métodos potenciales (paneles de vórtice/fuente), perfiles NACA, mallas adaptativas, cálculo
de sustentación y resistencia.
● Mecánica de Vuelo y Guiado/Control (GNC): Simulación de 3DOF / 6DOF, cálculo de envolventes de vuelo, autopilotos
PID/LQR, filtros de fusión sensorial (IMU/GPS).
● Propulsión y Termofluidos: Ciclos termodinámicos de turborreactores/turbofanes, análisis de toberas supersónicas,
rendimiento en diferentes altitudes ISA.
● Sistemas y Operaciones de Vuelo: Gestión de combustible y cálculo dinámico de CG, simulación de tráfico aéreo y
protocolos anticolisión (TCAS), rendimiento en pista (Accelerate-Stop / Accelerate-Go).
● Estructuras Aeronáuticas: Análisis por elementos finitos unidimensionales/vigas (método matricial de rigidez), diagramas V-n,
fatiga y distribución de esfuerzos cortantes y momentos flectores.

II. Componentes de Entrega Exigidos
1. Repositorio de Código Fuente:
○ Código modularizado en Python (.py o módulos con empaquetado formal).
○ Archivo de dependencias (requirements.txt o pyproject.toml).
○ Archivo README.md estructurado con instrucciones de instalación, casos de prueba y comandos de ejecución.
2. Reporte Técnico de Ingeniería (max 8 páginas):
○ Planteamiento del problema físico/aeronáutico y deducción/presentación de las ecuaciones de gobierno.
○ Diagrama de arquitectura del software y flujo algorítmico.
○ Validación del modelo (comparación contra datos de túnel de viento, manuales de vuelo [POH/AFM], software estándar de
la industria o soluciones analíticas exactas).
3. Demostración y Defensa Técnica:
○ Ejecución en vivo o grabación en video de alta calidad demostrando la interactividad, manejo de errores y validación de
casos frontera.

III. Matriz Analítica de Evaluación
Criterio  Ponderación  Excelente /  Competente /  En Desarrollo (5.0 -  Insuficiente (0.0 -
| Sobresaliente (9.0 -  | Aceptable (7.0 -  | 6.9)  | 4.9)  |
| --------------------- | ----------------- | ----- | ----- |
| 10.0)                 | 8.9)              |       |       |
1. Rigor Físico y  30%  Plantea un modelo  Modela el  El modelo simplifica  No existe una
Modelado  físico/aerodinámico  fenómeno  en exceso la física del  formulación
Aeronáutico  robusto. Identifica  adecuadamente  sistema, incurriendo  matemática clara o
| claramente variables  | con ecuaciones  | en discrepancias  | las leyes  |
| --------------------- | --------------- | ----------------- | ---------- |
de estado, hipótesis  válidas. Aplica  conceptuales o  fundamentales de la
| simplificativas y     | simplificaciones     | parámetros       | aerodinámica/física  |
| --------------------- | -------------------- | ---------------- | -------------------- |
| ecuaciones de         | coherentes, pero la  | dimensionales    | están                |
| gobierno. Incluye     | validación contra    | inconsistentes.  | implementadas        |
| validación numérica   | referencias es       |                  | incorrectamente.     |
| frente a fuentes de   | limitada o           |                  |                      |
| referencia con error  | cualitativa.         |                  |                      |
relativo cuantificado

2. Eficiencia y  25%  Selección óptima de  Uso correcto de  Uso predominante de  Algoritmo inestable,
Complejidad  estructuras de datos  estructuras de  bucles iterativos  propenso a bucles
Algorítmica  nativas (dict, set, list,  datos y librerías  ineficientes para  infinitos,
| tuple) y computación  | científicas, aunque  | tareas vectoriales.  | desbordamientos  |
| --------------------- | -------------------- | -------------------- | ---------------- |
vectorizada (numpy).  existen  No aprovecha  numéricos o fallos de
| El algoritmo evita     | redundancias de  | adecuadamente  | convergencia  |
| ---------------------- | ---------------- | -------------- | ------------- |
| bucles anidados        | cómputo menores  | estructuras    | matemática.   |
| innecesarios. Maneja   | que no           | avanzadas ni   |               |
| condiciones de         | comprometen la   | algoritmos     |               |
| parada numéricas y     | estabilidad del  | optimizados.   |               |
| convergencia estable.  | sistema.         |                |               |

3. Arquitectura,  25%  Arquitectura modular  Código dividido en  Código monolítico o  Código
Modularidad y  clara (separación  funciones lógicas.  pobremente  desorganizado
Manejo de Errores  estricta entre motor  Incorpora validación  segmentado. Las  ("código espagueti")
| de cálculo, validación   | de entradas           | funciones tienen alta  | sin separación       |
| ------------------------ | --------------------- | ---------------------- | -------------------- |
| de entrada e interfaz).  | estándar, pero        | interdependencia y     | funcional. El        |
| Implementa control       | carece de detección   | escaso manejo de       | programa colapsa     |
| preventivo de            | exhaustiva para       | excepciones ante       | ante entradas fuera  |
| excepciones              | parámetros físicos    | datos anómalos.        | de rango.            |
| (try-except,             | fuera de envolvente.  |                        |                      |
validaciones físicas
como divisiones por
cero, presiones
negativas o M>1 en
flujos subsónicos).
Principio DRY estricto.
4. Visualización de  20%  Presenta salidas  Gráficos claros y  Genera gráficos  Salidas confusas, no
Resultados e  gráficas de calidad  legibles que ilustran  básicos o salidas  representativas del
Interactividad  técnica publicable  el comportamiento  únicamente textuales  fenómeno analizado
| (matplotlib, seaborn o  | del sistema, con     | difíciles de          | o visualización  |
| ----------------------- | -------------------- | --------------------- | ---------------- |
| visualización           | rotulación completa  | interpretar; faltan   | ausente/rota.    |
| interactiva 2D/3D).     | de unidades,         | unidades o etiquetas  |                  |
| Ejes con nombres,       | aunque con           | de identificación en  |                  |
| unidades físicas,       | opciones limitadas   | las variables.        |                  |
| leyendas y escalas      | de personalización   |                       |                  |
| correctas. Interfaz     | interactiva.         |                       |                  |
intuitiva (CLI
estructurada o GUI
responsiva).

V. Criterios de Descalificación y Penalizaciones
● Plagio o copia no atribuida de código preexistente: Se permite el uso de librerías de código abierto de cálculo numérico,
pero no soluciones completas descargadas de repositorios de terceros.
● Uso no crítico de Inteligencia Artificial Generativa: Todo código asistido por herramientas LLM debe ser comprendido y
defendido individualmente por los autores en la sesión de preguntas.
● Incumplimiento de ejecución del programa: Si el software no compila ni ejecuta durante la evaluación en vivo por errores
sintácticos no gestionados.