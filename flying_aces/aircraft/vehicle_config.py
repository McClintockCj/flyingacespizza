from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from flying_aces.aircraft.battery import BatteryDischargeModel, BatteryRechargeModel


@dataclass(frozen=True)
class DroneConfig:
    """
    Configuration describing one type of drone.

    A DroneConfig is data/configuration, not a live vehicle.
    """

    name: str

    # Flight characteristics
    cruise_speed: float = 15.0
    payload_capacity: float = 5.0
    rotor_speed: float = 1788.53

    # RotorPy vehicle configuration
    rotorpy_params: Any = None

    # Battery configuration
    battery_capacity_wh: float = 100.0
    discharge_model_factory: Callable[[], BatteryDischargeModel] | None = None
    recharge_model_factory: Callable[[], BatteryRechargeModel] | None = None