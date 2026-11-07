#!/usr/bin/env python3
"""
Punto de Entrada Rápido del Proyecto AERO-GNC.

Permite ejecutar directamente la simulación, validación o panel interactivo:
  python run_simulation.py --help
  python run_simulation.py simulate --plot
  python run_simulation.py validate
  python run_simulation.py dashboard
"""

import sys
import os

# Asegurar que el directorio src está en el path de Python
current_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(current_dir, "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from aero_gnc.cli import main

if __name__ == "__main__":
    if len(sys.argv) == 1:
        # Si se ejecuta sin argumentos, por defecto corre la simulación con generación de gráficos
        sys.argv.extend(["simulate", "--plot"])
    main()
