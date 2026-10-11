"""Parallel obstacle-aware planner for RotorPy V2 missions.

Does not modify the legacy MissionPlanner or MissionManager.
"""
from __future__ import annotations

from contracts import DeliveryOrder, Route, Vector3, Waypoint
from mission_planner import MissionPlanner
from mission_v2 import FlightMissionV2, LegAction, MissionLeg


class MissionPlannerV2:
    def __init__(self, planner: MissionPlanner, landing_height: float = 0.15,
                 cruise_height: float = 3.0, speed: float = 1.0):
        if landing_height < 0 or cruise_height <= landing_height or speed <= 0:
            raise ValueError("Invalid V2 flight parameters")
        self.planner = planner
        self.landing_height = landing_height
        self.cruise_height = cruise_height
        self.speed = speed

    def _clear(self, a: Vector3, b: Vector3) -> bool:
        return self.planner.segment_is_clear(a, b)

    def _append(self, points: list[Vector3], point: Vector3) -> None:
        if points[-1].distance_to(point) > 1e-7:
            points.append(point)

    def plan_landing_leg(self, start: Vector3, destination: Vector3, speed: float = 10.0) -> Route:
        """Plan takeoff, obstacle-aware transit, and a checked vertical descent.

        destination.x/y specify the landing location; destination.z is intentionally
        replaced with landing_height. Unsafe landing corridors are rejected.
        """
        touchdown = Vector3(destination.x, destination.y, self.landing_height)
        approach = Vector3(destination.x, destination.y, self.cruise_height)
        departure = Vector3(start.x, start.y, max(start.z, self.cruise_height))

        # Check the entire vertical descent, including the low-altitude endpoint.
        if not self._clear(approach, touchdown):
            raise RuntimeError(f"Unsafe landing corridor at ({destination.x}, {destination.y})")
        if not self._clear(start, departure):
            raise RuntimeError(f"Unsafe takeoff corridor at ({start.x}, {start.y})")

        transit = self.planner.plan_route(departure, approach)
        points = [start]
        self._append(points, departure)
        for waypoint in transit.waypoints[1:]:
            self._append(points, waypoint.position)
        self._append(points, touchdown)
        route = Route([Waypoint(p, speed=speed) for p in points])
        if not self.planner.route_is_clear(route):
            raise RuntimeError("V2 route failed obstacle clearance validation")
        return route

    def plan_mission(self, mission_id: str, start: Vector3,
                     orders: list[DeliveryOrder], base: Vector3, speed: float = 10.0) -> FlightMissionV2:
        if not orders:
            raise ValueError("At least one delivery order is required")
        legs: list[MissionLeg] = []
        current = start
        for order in orders:
            route = self.plan_landing_leg(current, order.destination, speed=speed)
            legs.append(MissionLeg(route, LegAction.DELIVER, order.id))
            current = route.waypoints[-1].position
        route = self.plan_landing_leg(current, base, speed=speed)
        legs.append(MissionLeg(route, LegAction.RETURN))
        return FlightMissionV2(mission_id, legs)
