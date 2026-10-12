"""Multi-leg mission orchestration."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from flying_aces.core.contracts import DroneState, Route
from flying_aces.aircraft.rotorpy_vehicle import RouteStatus, RotorPyVehicle


class LegAction(str, Enum):
    TRANSIT = "transit"
    DELIVER = "deliver"
    RETURN = "return"


class MissionStatus(str, Enum):
    READY = "ready"
    RUNNING = "running"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass
class MissionLeg:
    route: Route
    action: LegAction = LegAction.TRANSIT
    order_id: str | None = None

    def __post_init__(self):
        if not self.route.waypoints:
            raise ValueError("Mission leg requires at least one waypoint")
        if self.action == LegAction.DELIVER and not self.order_id:
            raise ValueError("Delivery leg requires order_id")


@dataclass
class FlightMission:
    id: str
    legs: list[MissionLeg]
    status: MissionStatus = MissionStatus.READY
    current_leg_index: int = 0
    delivered_orders: list[str] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    fault: str | None = None

    def __post_init__(self):
        if not self.legs:
            raise ValueError("Mission requires at least one leg")


class MissionExecutor:
    def __init__(self, vehicle: RotorPyVehicle, mission: FlightMission, landing_height: float = 0.15, landing_tolerance: float = 0.12, unloading_time: float = 0.0):
        self.vehicle = vehicle
        self.mission = mission
        self.landing_height = landing_height
        self.landing_tolerance = landing_tolerance
        if unloading_time < 0:
            raise ValueError("unloading_time must be nonnegative")
        self.unloading_time = unloading_time
        self.unloading_remaining = 0.0
        self.elapsed_time = 0.0
        self.simulation_time = 0.0
        self.started_at = 0.0
        if landing_height < 0 or landing_tolerance <= 0:
            raise ValueError("Invalid landing configuration")
        for leg in mission.legs:
            if leg.action in (LegAction.DELIVER, LegAction.RETURN):
                z = leg.route.waypoints[-1].position.z
                if abs(z - landing_height) > landing_tolerance:
                    raise ValueError(f"{leg.action.value} route must end at landing height {landing_height}m; got {z}m")

    def _event(self, kind: str, **details):
        self.mission.events.append({"time": round(self.simulation_time, 3),
                                    "mission_elapsed_time": round(self.elapsed_time, 3),
                                    "type": kind, **details})

    def _launch_current_leg(self):
        leg = self.mission.legs[self.mission.current_leg_index]
        self.vehicle.upload_route(leg.route)
        self.vehicle.start_route()
        self._event("leg_started", index=self.mission.current_leg_index, action=leg.action.value)

    def start(self, time: float | None = None):
        if self.mission.status != MissionStatus.READY:
            raise RuntimeError(f"Mission cannot start in status {self.mission.status}")
        self.started_at = self.simulation_time = 0.0 if time is None else float(time)
        self.mission.status = MissionStatus.RUNNING
        self._event("mission_started", mission_id=self.mission.id)
        self._launch_current_leg()

    def _finish_leg(self):
        leg = self.mission.legs[self.mission.current_leg_index]
        if leg.action == LegAction.DELIVER:
            self.mission.delivered_orders.append(leg.order_id)
            self._event("order_delivered", order_id=leg.order_id)
        elif leg.action == LegAction.RETURN:
            self._event("returned_to_base")
        self.mission.current_leg_index += 1
        if self.mission.current_leg_index == len(self.mission.legs):
            self.mission.status = MissionStatus.COMPLETE
            self._event("mission_completed", mission_id=self.mission.id)
        else:
            self._launch_current_leg()

    def advance(self, dt: float, time: float | None = None):
        if dt <= 0 or self.mission.status != MissionStatus.RUNNING:
            return
        self.elapsed_time += dt
        self.simulation_time = (self.simulation_time + dt if time is None else float(time))
        if self.unloading_remaining > 0:
            self.unloading_remaining = max(0.0, self.unloading_remaining - dt)
            if self.unloading_remaining == 0:
                self._event("unloading_completed", order_id=self.mission.legs[self.mission.current_leg_index].order_id)
                self._finish_leg()
            return
        self.vehicle.advance(dt)
        if self.vehicle.route_status == RouteStatus.FAILED:
            self.mission.status = MissionStatus.FAILED
            self.mission.fault = self.vehicle.get_state().fault or "route_failed"
            self._event("mission_failed", fault=self.mission.fault)
            return
        if self.vehicle.route_status != RouteStatus.COMPLETE:
            return
        leg = self.mission.legs[self.mission.current_leg_index]
        self._event("leg_completed", index=self.mission.current_leg_index)
        if leg.action in (LegAction.DELIVER, LegAction.RETURN):
            self.vehicle.land()
            self._event("landed", index=self.mission.current_leg_index,
                        position=vars(self.vehicle.get_state().position).copy())
        if leg.action == LegAction.DELIVER and self.unloading_time > 0:
            self.unloading_remaining = self.unloading_time
            self._event("unloading_started", order_id=leg.order_id)
        else:
            self._finish_leg()
