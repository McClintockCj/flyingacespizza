from __future__ import annotations

import numpy as np

from flying_aces.aircraft.battery import Battery, ConstantRecharge, ConstantPowerDischarge, PayloadWindDischarge
from flying_aces.core.contracts import Vector3, Wind
from flying_aces.aircraft.vehicle_config import DroneConfig
from flying_aces.aircraft.rotorpy_vehicle import RotorPyVehicle

from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.environments import Environment
from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.wind.default_winds import ConstantWind


DRONE_CONFIGS = [
    DroneConfig(
        name="Light",
        cruise_speed=12.0,
        payload_capacity=2.0,
        rotor_speed=1650.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=100.0,
        discharge_model_factory=lambda: ConstantPowerDischarge(power_w=150.0),
        recharge_model_factory=lambda: ConstantRecharge(power_w=400.0),
    ),
    DroneConfig(
        name="Standard",
        cruise_speed=15.0,
        payload_capacity=5.0,
        rotor_speed=1788.53,
        rotorpy_params=quad_params,
        battery_capacity_wh=100.0,
        discharge_model_factory=lambda: ConstantPowerDischarge(power_w=180.0),
        recharge_model_factory=lambda: ConstantRecharge(power_w=500.0),
    ),
    DroneConfig(
        name="Heavy",
        cruise_speed=15.0,
        payload_capacity=8.0,
        rotor_speed=1850.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=150.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=220.0,
            payload_power_per_kg=30.0,
            wind_power_per_mps=20.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(power_w=600.0),
    ),
    DroneConfig(
        name="Fast",
        cruise_speed=22.0,
        payload_capacity=3.0,
        rotor_speed=1950.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=140.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=250.0,
            payload_power_per_kg=25.0,
            wind_power_per_mps=18.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(power_w=700.0),
    ),
    DroneConfig(
        name="Endurance",
        cruise_speed=12.0,
        payload_capacity=4.0,
        rotor_speed=1600.0,
        rotorpy_params=quad_params,
        battery_capacity_wh=200.0,
        discharge_model_factory=lambda: PayloadWindDischarge(
            base_power_w=160.0,
            payload_power_per_kg=18.0,
            wind_power_per_mps=12.0,
        ),
        recharge_model_factory=lambda: ConstantRecharge(power_w=450.0),
    ),
]


def get_drone_config(name: str) -> DroneConfig:
    for config in DRONE_CONFIGS:
        if config.name.lower() == name.lower():
            return config
    raise ValueError(f"Unknown drone configuration: {name}")


def build_vehicle(
    home_position: Vector3,
    wind: Wind,
    config: DroneConfig,
    vehicle_id: str,
) -> RotorPyVehicle:
    initial_state = {
        "x": np.array([home_position.x, home_position.y, home_position.z]),
        "v": np.zeros(3),
        "q": np.array([0.0, 0.0, 0.0, 1.0]),
        "w": np.zeros(3),
        "wind": np.zeros(3),
        "rotor_speeds": np.array([
            config.rotor_speed,
            config.rotor_speed,
            config.rotor_speed,
            config.rotor_speed,
        ]),
    }

    rotorpy_vehicle = Multirotor(
        config.rotorpy_params,
        initial_state=initial_state,
    )

    rotorpy_controller = SE3Control(config.rotorpy_params)

    rotorpy_environment = Environment(
        vehicle=rotorpy_vehicle,
        controller=rotorpy_controller,
        trajectory=HoverTraj(
            x0=np.array([home_position.x, home_position.y, home_position.z])
        ),
        wind_profile=ConstantWind(
            wx=wind.velocity.x,
            wy=wind.velocity.y,
            wz=wind.velocity.z,
        ),
        sim_rate=100,
    )

    battery = Battery(
        capacity_wh=config.battery_capacity_wh,
        discharge_model=config.discharge_model_factory(),
        recharge_model=config.recharge_model_factory(),
    )

    return RotorPyVehicle(
        rotorpy_vehicle=rotorpy_vehicle,
        rotorpy_environment=rotorpy_environment,
        rotorpy_controller=rotorpy_controller,
        config=config,
        vehicle_id=vehicle_id,
        home_position=home_position,
        battery=battery,
        wind=wind,
    )


def _build_rotorpy_fleet(
    base: Vector3,
    wind: Wind,
    configs: list[DroneConfig],
    spacing: float = 1.5,
) -> list[RotorPyVehicle]:
    vehicles = []

    for i, config in enumerate(configs):
        home_position = Vector3(
            base.x + i * spacing,
            base.y,
            base.z,
        )
        vehicles.append(
            build_vehicle(
                home_position=home_position,
                wind=wind,
                config=config,
                vehicle_id=f"{config.name}-{i + 1}",
            )
        )

    return vehicles


def build_diverse_fleet(
    base: Vector3,
    wind: Wind,
    fleet_size: int,
    spacing: float = 1.5,
) -> list[RotorPyVehicle]:
    configs = [
        DRONE_CONFIGS[i % len(DRONE_CONFIGS)]
        for i in range(fleet_size)
    ]
    return _build_rotorpy_fleet(
        base=base,
        wind=wind,
        configs=configs,
        spacing=spacing,
    )

def build_fleet(base: Vector3, wind: Wind, configs: list[DroneConfig], spacing: float = 1.5):
    """Reuse the tested RotorPy factory, wrapping each vehicle with the V2 adapter."""
    from flying_aces.aircraft.rotorpy_vehicle import RotorPyVehicle

    legacy = _build_rotorpy_fleet(base, wind, configs, spacing=spacing)
    return [RotorPyVehicle(
        rotorpy_vehicle=vehicle.vehicle,
        rotorpy_environment=vehicle.environment,
        rotorpy_controller=vehicle.controller,
        config=config, vehicle_id=vehicle.id,
        home_position=vehicle.home_position,
        battery=vehicle.state.battery, wind=wind,
    ) for vehicle, config in zip(legacy, configs)]
