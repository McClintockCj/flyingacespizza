from __future__ import annotations

import json
import random
from pathlib import Path
from dataclasses import asdict, is_dataclass
from typing import Any

from flying_aces.core.contracts import (
    DeliveryOrder,
    Scenario,
    SimulationInterface,
    SimulationSnapshot,
    Vector3,
    Wind,
)
from flying_aces.aircraft.fleet_factory import build_fleet, get_drone_config
from flying_aces.world.maps import MapManager
from flying_aces.missions.mission_manager import MissionManager
from flying_aces.missions.mission_planner import MissionPlanner
from flying_aces.world.order_generator import OrderGenerator
from flying_aces.world.scenario import ScenarioManager
from flying_aces.telemetry import Telemetry, TelemetryConfig, create_telemetry


class Simulation(SimulationInterface):

    def __init__(
        self,
        vehicles,
        environment: ScenarioManager,
        mission_manager: MissionManager,
        map: MapManager,
        order_spawner: OrderGenerator | None = None,
        rng: random.Random | None = None,
        telemetry: Telemetry | None = None,
    ):
        self.vehicles = list(vehicles)
        self.environment = environment
        self.map = map
        self.mission_manager = mission_manager
        self.order_spawner = order_spawner
        self.rng = rng or random.Random()
        self.telemetry = telemetry

        self.auto_dispatch = True

        self.time = 0.0
        self.running = False

        if self.telemetry is not None:
            for vehicle in self.vehicles:
                self.telemetry.register_vehicle(vehicle)

    @classmethod
    def from_json(cls, path: str | Path) -> "Simulation":
        path = Path(path)
        raw = json.loads(path.read_text(encoding="utf-8"))

        seed = raw.get("seed")
        rng = random.Random(seed)

        map_path = raw.get("map", "maps/richmond_small_3.json")
        map_path = Path(map_path)

        if not map_path.is_absolute():
            map_path = path.parent / map_path

        map = MapManager(map_path)
        bounds = map.get_bounds()

        base_data = raw.get("base")
        if base_data is None:
            x0, y0, x1, y1 = bounds
            base = Vector3(
                (x0 + x1) / 2,
                (y0 + y1) / 2,
                2.0,
            )
        else:
            base = Vector3(
                base_data["x"],
                base_data["y"],
                base_data.get("z", 2.0),
            )

        wind_data = raw.get("wind", {})
        if "velocity" in wind_data:
            wind_data = wind_data["velocity"]

        wind = Wind(
            Vector3(
                wind_data.get("x", 0.0),
                wind_data.get("y", 0.0),
                wind_data.get("z", 0.0),
            )
        )

        orders = [
            DeliveryOrder(
                id=order["id"],
                destination=Vector3(
                    order["destination"]["x"],
                    order["destination"]["y"],
                    order["destination"].get("z", 2.0),
                ),
                priority=order.get("priority", 0),
                payload_weight=order.get("payload_weight", 1.0),
                status=order.get("status", "pending"),
                created_at=order.get("created_at", 0.0),
            )
            for order in raw.get("orders", [])
        ]

        scenario = Scenario(
            bounds=bounds,
            obstacles=map.get_obstacles(),
            wind=wind,
            orders=orders,
            base=base,
        )

        environment = ScenarioManager(scenario)

        fleet_config = raw.get("fleet", {})
        fleet_size = fleet_config.get("size", 5)
        config_names = fleet_config.get("configs")
        if config_names:
            configs = [get_drone_config(name) for name in config_names]
        else:
            from flying_aces.aircraft.fleet_factory import DRONE_CONFIGS
            configs = [DRONE_CONFIGS[i % len(DRONE_CONFIGS)] for i in range(fleet_size)]
        vehicles = build_fleet(base=base, wind=wind, configs=configs)
        from flying_aces.missions.mission_manager import GroundOperationsConfig
        ground_config = GroundOperationsConfig(**raw.get("ground_operations", {}))
        planner = MissionPlanner(environment)
        mission_manager = MissionManager(
            vehicles=vehicles, environment=environment, planner=planner,
            ground_config=ground_config,
        )

        spawning = raw.get("spawning")

        if spawning is None:
            order_spawner = None
        else:
            strategy_type = spawning.get("type", "interval")

            if strategy_type != "interval":
                raise ValueError(
                    f"Unsupported spawning strategy: {strategy_type}"
                )

            order_spawner = OrderGenerator(
                interval_seconds=spawning.get("interval", 15.0),
                max_active_orders=spawning.get("max_active_orders", 5),
                rng=rng,
                max_delivery_radius=spawning.get("max_delivery_radius"),
            )

        telemetry_data = raw.get("telemetry")

        if telemetry_data is None:
            telemetry = None
        else:
            telemetry_config = TelemetryConfig(
                enabled=telemetry_data.get("enabled", True),
                output=telemetry_data.get("output", "telemetry.json"),
                state_interval=telemetry_data.get("state_interval", 1.0),
                vehicles=telemetry_data.get("vehicles", True),
                orders=telemetry_data.get("orders", True),
                missions=telemetry_data.get("missions", True),
                battery=telemetry_data.get("battery", True),
            )

            telemetry = create_telemetry(telemetry_config)

        simulation = cls(
            vehicles=vehicles,
            environment=environment,
            mission_manager=mission_manager,
            map=map,
            order_spawner=order_spawner,
            rng=rng,
            telemetry=telemetry,
        )

        return simulation

    @property
    def scenario(self) -> Scenario:
        return self.environment.get_scenario()

    @property
    def time_seconds(self) -> float:
        return self.time

    def add_order(self, order: DeliveryOrder) -> None:
        self.mission_manager.add_order(order)

    def advance(self, dt: float) -> None:
        if not self.running or dt <= 0:
            return
        next_time = self.time + dt
        if self.order_spawner is not None:
            self.order_spawner.update(dt, self.scenario, next_time)
        # MissionManager is the ONLY owner of vehicle physics advancement.
        self.mission_manager.update(dt, next_time, auto_dispatch=self.auto_dispatch)
        self.time = next_time
        if self.telemetry is not None:
            self.telemetry.sample(self.time, self.vehicles)
            for order in self.scenario.orders:
                self.telemetry.record_order(order)
            self._record_mission_telemetry()

    def _record_mission_telemetry(self) -> None:
        if self.telemetry is None:
            return
        for flight in self.mission_manager.flights:
            self.telemetry.record_v2_flight(flight, self.time)
        for mission in self.mission_manager.completed_missions:
            self.telemetry.record_mission_snapshot(mission)
        self.telemetry.record_v2_events(self.mission_manager.events)

    def run(self, dt: float = 0.1) -> None:
        self.start()

        while self.running:
            self.advance(dt)

    def get_snapshot(self) -> SimulationSnapshot:
        return SimulationSnapshot(
            time=self.time,
            vehicles=[vehicle.get_state() for vehicle in self.vehicles],
            scenario=self.scenario,
        )

    def reset(self) -> None:
        self.time = 0.0
        self.mission_manager.reset()
        if self.order_spawner is not None:
            self.order_spawner.timer = 0.0
        if self.telemetry is not None:
            self.telemetry.reset()
        self.running = False

    def start(self) -> None:
        self.running = True
        if self.telemetry is not None:
            self.telemetry.sample(self.time, self.vehicles)
            for order in self.scenario.orders:
                self.telemetry.record_order(order)

    def stop(self) -> None:
        self.running = False

        if self.telemetry is not None:
            self._record_mission_telemetry()
            self.telemetry.write()