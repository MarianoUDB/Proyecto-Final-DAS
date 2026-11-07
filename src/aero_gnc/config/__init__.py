"""Módulo de configuración y bases de datos de referencia aeronáutica."""

from aero_gnc.config.aircraft_database import (
    AircraftData,
    get_aircraft,
    AVAILABLE_AIRCRAFT,
)
from aero_gnc.config.airport_ils import ILSSpecs, DEFAULT_ILS

__all__ = ["AircraftData", "get_aircraft", "AVAILABLE_AIRCRAFT", "ILSSpecs", "DEFAULT_ILS"]
