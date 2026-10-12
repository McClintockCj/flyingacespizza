from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any


@dataclass
class TelemetryConfig:
    enabled: bool = True
    output: str = "telemetry.json"
    state_interval: float = 1.0
    vehicles: bool = True
    orders: bool = True
    missions: bool = True
    battery: bool = True


class Telemetry(ABC):

    def __init__(self, config: TelemetryConfig):
        self.config = config
        self.next_state_time = 0.0

    @abstractmethod
    def register_vehicle(self, vehicle) -> None:
        pass

    @abstractmethod
    def record_vehicle_sample(self, time: float, vehicle) -> None:
        pass

    @abstractmethod
    def record_order(self, order) -> None:
        pass

    @abstractmethod
    def record_mission(self, mission) -> None:
        pass

    @abstractmethod
    def record_mission_snapshot(self, snapshot: dict[str, Any]) -> None:
        pass

    @abstractmethod
    def write(self, path: str | Path | None = None) -> None:
        pass

    @abstractmethod
    def reset(self) -> None:
        pass

    def sample(self, time: float, vehicles) -> None:
        if not self.config.enabled:
            return

        if not self.config.vehicles:
            return

        if self.config.state_interval <= 0:
            return

        if time < self.next_state_time:
            return

        for vehicle in vehicles:
            self.record_vehicle_sample(time, vehicle)

        while self.next_state_time <= time:
            self.next_state_time += self.config.state_interval


class JsonTelemetry(Telemetry):

    def __init__(self, config: TelemetryConfig):
        super().__init__(config)

        self.vehicles: dict[str, dict[str, Any]] = {}
        self.orders: dict[str, dict[str, Any]] = {}
        self.missions: dict[str, dict[str, Any]] = {}
        self.completed_missions: list[dict[str, Any]] = []
        self.archived_mission_ids: set[tuple[str, int | None]] = set()
        self.events: list[dict[str, Any]] = []
        self.recorded_event_count = 0

    def register_vehicle(self, vehicle) -> None:
        if not self.config.enabled or not self.config.vehicles:
            return

        self.vehicles[vehicle.id] = {
            **vehicle.to_dict(),
            "samples": [],
        }

    def record_vehicle_sample(self, time: float, vehicle) -> None:
        if not self.config.enabled or not self.config.vehicles:
            return

        if vehicle.id not in self.vehicles:
            self.register_vehicle(vehicle)

        self.vehicles[vehicle.id]["samples"].append(
            vehicle.sample(time)
        )

    def record_order(self, order) -> None:
        if not self.config.enabled or not self.config.orders:
            return

        self.orders[order.id] = order.to_dict()

    def record_mission(self, mission) -> None:
        if not self.config.enabled or not self.config.missions:
            return

        self.missions[mission.id] = mission.to_dict()

    def record_mission_snapshot(self, snapshot: dict[str, Any]) -> None:
        if not self.config.enabled or not self.config.missions:
            return

        key = (
            snapshot["id"],
            snapshot.get("mission_number"),
        )

        if key in self.archived_mission_ids:
            return

        self.archived_mission_ids.add(key)
        self.missions.pop(snapshot["id"], None)
        self.completed_missions.append(dict(snapshot))

    def record_v2_flight(self, flight, time: float) -> None:
        """Record the active V2 mission, including ground loading."""
        if not self.config.enabled or not self.config.missions:
            return
        executor = flight.executor
        if executor is None:
            return
        mission = executor.mission
        self.missions[mission.id] = {
            "id": mission.id,
            "vehicle_id": flight.vehicle.id,
            "mission_number": flight.mission_number,
            "status": mission.status.value,
            "manager_state": flight.state.value,
            "ground_phase": flight.ground_phase.value,
            "order_ids": [o.id for o in flight.orders or []],
            "delivered_orders": list(mission.delivered_orders),
            "last_observed_at": time,
        }

    def record_v2_events(self, events: list[dict[str, Any]]) -> None:
        """Append only newly observed events; source list is append-only."""
        if not self.config.enabled:
            return
        if len(events) < self.recorded_event_count:
            self.recorded_event_count = 0
        if len(events) > self.recorded_event_count:
            self.events.extend(dict(e) for e in events[self.recorded_event_count:])
            self.recorded_event_count = len(events)

    def write(self, path: str | Path | None = None) -> None:
        if not self.config.enabled:
            return

        output_path = Path(path or self.config.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        data = {
            "vehicles": list(self.vehicles.values()),
            "orders": list(self.orders.values()),
            "missions": (
                self.completed_missions
                + list(self.missions.values())
            ),
            "events": self.events,
        }

        with output_path.open("w", encoding="utf-8") as file:
            json.dump(data, file, indent=2)

    def reset(self) -> None:
        self.vehicles.clear()
        self.orders.clear()
        self.missions.clear()
        self.completed_missions.clear()
        self.archived_mission_ids.clear()
        self.events.clear()
        self.recorded_event_count = 0
        self.next_state_time = 0.0


def create_telemetry(config: TelemetryConfig) -> Telemetry:
    output = Path(config.output)

    if output.suffix.lower() == ".json":
        return JsonTelemetry(config)

    raise ValueError(
        f"Unsupported telemetry format: {output.suffix}"
    )