# contracts.py
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional
import math

from flying_aces.aircraft.battery import Battery


class MissionState(Enum):
    IDLE = "idle"
    PLANNING = "planning"
    READY = "ready"
    RECHARGING = "recharging"
    EXECUTING = "executing"
    DELIVERING = "delivering"
    RETURNING = "returning"
    COMPLETE = "complete"
    ABORTED = "aborted"
    FAILED = "failed"


class DroneState(Enum):
    LANDED = "landed"
    FLYING = "flying"
    RETURNING = "returning"
    EMERGENCY = "emergency"
    HOVERING = "hovering"


@dataclass
class Vector3:
    x: float
    y: float
    z: float = 0.0

    def distance_to(self, other: "Vector3") -> float:
        return ((self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2) ** 0.5

    def __add__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: "Vector3") -> "Vector3":
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float) -> "Vector3":
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)


@dataclass
class Waypoint:
    position: Vector3
    altitude: Optional[float] = None
    speed: Optional[float] = None

    def __post_init__(self):
        if self.altitude is not None:
            self.position.z = self.altitude


@dataclass
class Route:
    waypoints: list[Waypoint] = field(default_factory=list)

    @property
    def distance(self) -> float:
        return sum(
            self.waypoints[i - 1].position.distance_to(self.waypoints[i].position)
            for i in range(1, len(self.waypoints))
        )

    def __len__(self):
        return len(self.waypoints)

@dataclass
class VehicleState:
    position: Vector3
    battery: Battery
    name: str = "drone"
    velocity: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
    state: DroneState = DroneState.LANDED
    payload_loaded: bool = False
    payload_weight: float = 0.0
    connected: bool = True
    fault: Optional[str] = None
    payload_capacity: float = 5.0
    cruise_speed: float = 15.0


@dataclass(frozen=True)
class Obstacle:
    """Static polygonal obstacle in simulator world coordinates."""

    polygons: tuple[tuple[tuple[float, float], ...], ...]
    height: float = 0.0

    @property
    def bounds(self) -> tuple[float, float, float, float]:
        points = [point for polygon in self.polygons for point in polygon]

        if not points:
            return 0.0, 0.0, 0.0, 0.0

        xs = [point[0] for point in points]
        ys = [point[1] for point in points]

        return min(xs), min(ys), max(xs), max(ys)

    def contains_point(self, point: Vector3, margin: float = 0.0) -> bool:
        """Return True if the point lies inside or near the obstacle footprint."""

        xmin, ymin, xmax, ymax = self.bounds

        if (
            point.x < xmin - margin
            or point.x > xmax + margin
            or point.y < ymin - margin
            or point.y > ymax + margin
        ):
            return False

        return any(
            self._point_in_polygon(point.x, point.y, polygon, margin)
            for polygon in self.polygons
        )

    def intersects_segment(
        self,
        a: Vector3,
        b: Vector3,
        margin: float = 0.0,
    ) -> bool:
        """Return True if a 3D segment intersects the obstacle."""

        if min(a.z, b.z) > self.height:
            return False

        xmin, ymin, xmax, ymax = self.bounds

        if (
            max(a.x, b.x) < xmin - margin
            or min(a.x, b.x) > xmax + margin
            or max(a.y, b.y) < ymin - margin
            or min(a.y, b.y) > ymax + margin
        ):
            return False

        for polygon in self.polygons:
            if self._segment_intersects_polygon(
                a,
                b,
                polygon,
                margin,
            ):
                return True

        return False

    def expanded_boundary_points(
        self,
        margin: float = 0.0,
    ) -> list[tuple[float, float]]:
        """Return obstacle boundary points suitable for routing nodes."""

        if margin <= 0.0:
            return [
                point
                for polygon in self.polygons
                for point in polygon
            ]

        xmin, ymin, xmax, ymax = self.bounds

        return [
            (xmin - margin, ymin - margin),
            (xmin - margin, ymax + margin),
            (xmax + margin, ymin - margin),
            (xmax + margin, ymax + margin),
        ]

    def routing_points(self, margin: float = 0.0) -> list[tuple[float, float]]:
        """Return points around the obstacle suitable for routing nodes."""

        points = []

        for polygon in self.polygons:
            for i, vertex in enumerate(polygon):
                previous = polygon[i - 1]
                following = polygon[(i + 1) % len(polygon)]

                offset = self._vertex_offset(
                    previous,
                    vertex,
                    following,
                    margin,
                )

                points.append(
                    (
                        vertex[0] + offset[0],
                        vertex[1] + offset[1],
                    )
                )

        return points

    @staticmethod
    def _vertex_offset(previous, vertex, following, margin):
        if margin <= 0.0:
            return 0.0, 0.0

        incoming = Obstacle._unit_vector(
            previous,
            vertex,
        )
        outgoing = Obstacle._unit_vector(
            vertex,
            following,
        )

        normal_in = (-incoming[1], incoming[0])
        normal_out = (-outgoing[1], outgoing[0])

        normal = (
            normal_in[0] + normal_out[0],
            normal_in[1] + normal_out[1],
        )

        length = math.hypot(normal[0], normal[1])

        if length < 1e-9:
            return normal_in[0] * margin, normal_in[1] * margin

        return (
            normal[0] / length * margin,
            normal[1] / length * margin,
        )


    @staticmethod
    def _unit_vector(a, b):
        dx = b[0] - a[0]
        dy = b[1] - a[1]

        length = math.hypot(dx, dy)

        if length < 1e-9:
            return 0.0, 0.0

        return dx / length, dy / length

    @staticmethod
    def _point_in_polygon(
        x: float,
        y: float,
        polygon: tuple[tuple[float, float], ...],
        margin: float,
    ) -> bool:
        if len(polygon) < 3:
            return False

        if margin > 0.0:
            for i in range(len(polygon)):
                a = polygon[i]
                b = polygon[(i + 1) % len(polygon)]

                if Obstacle._point_to_segment_distance(x, y, a, b) <= margin:
                    return True

        inside = False

        j = len(polygon) - 1

        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]

            if (yi > y) != (yj > y):
                intersection_x = (
                    (xj - xi) * (y - yi) / (yj - yi) + xi
                )

                if x < intersection_x:
                    inside = not inside

            j = i

        return inside

    @staticmethod
    def _segment_intersects_polygon(
        a: Vector3,
        b: Vector3,
        polygon: tuple[tuple[float, float], ...],
        margin: float,
    ) -> bool:
        if Obstacle._point_in_polygon(a.x, a.y, polygon, margin):
            return True

        if Obstacle._point_in_polygon(b.x, b.y, polygon, margin):
            return True

        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]

            if Obstacle._segments_intersect(
                (a.x, a.y),
                (b.x, b.y),
                p1,
                p2,
            ):
                return True

            if (
                Obstacle._segment_to_segment_distance(
                    (a.x, a.y),
                    (b.x, b.y),
                    p1,
                    p2,
                )
                <= margin
            ):
                return True

        return False

    @staticmethod
    def _segment_to_segment_distance(a, b, c, d) -> float:
        if Obstacle._segments_intersect(a, b, c, d):
            return 0.0

        return min(
            Obstacle._point_to_segment_distance(
                a[0],
                a[1],
                c,
                d,
            ),
            Obstacle._point_to_segment_distance(
                b[0],
                b[1],
                c,
                d,
            ),
            Obstacle._point_to_segment_distance(
                c[0],
                c[1],
                a,
                b,
            ),
            Obstacle._point_to_segment_distance(
                d[0],
                d[1],
                a,
                b,
            ),
        )

    @staticmethod
    def _segments_intersect(a, b, c, d) -> bool:
        def orientation(p, q, r):
            value = (
                (q[1] - p[1]) * (r[0] - q[0])
                - (q[0] - p[0]) * (r[1] - q[1])
            )

            if abs(value) < 1e-9:
                return 0

            return 1 if value > 0 else 2

        def on_segment(p, q, r):
            return (
                min(p[0], r[0]) <= q[0] <= max(p[0], r[0])
                and min(p[1], r[1]) <= q[1] <= max(p[1], r[1])
            )

        o1 = orientation(a, b, c)
        o2 = orientation(a, b, d)
        o3 = orientation(c, d, a)
        o4 = orientation(c, d, b)

        if o1 != o2 and o3 != o4:
            return True

        if o1 == 0 and on_segment(a, c, b):
            return True

        if o2 == 0 and on_segment(a, d, b):
            return True

        if o3 == 0 and on_segment(c, a, d):
            return True

        if o4 == 0 and on_segment(c, b, d):
            return True

        return False

    @staticmethod
    def _point_to_segment_distance(x, y, a, b) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]

        if dx == 0.0 and dy == 0.0:
            return math.hypot(x - a[0], y - a[1])

        t = (
            (x - a[0]) * dx + (y - a[1]) * dy
        ) / (dx * dx + dy * dy)

        t = max(0.0, min(1.0, t))

        closest_x = a[0] + t * dx
        closest_y = a[1] + t * dy

        return math.hypot(x - closest_x, y - closest_y)


@dataclass
class Wind:
    velocity: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))

    @property
    def speed(self) -> float:
        return self.velocity.distance_to(Vector3(0, 0, 0))

# @dataclass
# class DeliveryOrder:
#     id: str
#     destination: Vector3
#     priority: int = 0
#     payload_weight: float = 1.0  # default weight for editor-placed orders
#     status: str = "pending"      # "pending", "assigned", "in_progress", "delivered"

@dataclass
class DeliveryOrder:
    id: str
    destination: Vector3
    priority: int = 0
    payload_weight: float = 1.0
    status: str = "pending"
    created_at: float = 0.0
    completed_at: Optional[float] = None
    failed_at: Optional[float] = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "destination": {
                "x": self.destination.x,
                "y": self.destination.y,
                "z": self.destination.z,
            },
            "priority": self.priority,
            "payload_weight": self.payload_weight,
            "created_at": self.created_at,
            "completed_at": self.completed_at,
            "failed_at": self.failed_at,
            "status": self.status,
        }

@dataclass
class Scenario:
    bounds: tuple[float, float, float, float]
    obstacles: list[Obstacle] = field(default_factory=list)
    wind: Wind = field(default_factory=Wind)
    orders: list[DeliveryOrder] = field(default_factory=list)  # Replaces delivery_points
    base: Vector3 = field(default_factory=lambda: Vector3(0, 0, 0))
    drone_start_battery: float = 100.0


@dataclass
class SimulationSnapshot:
    time: float
    vehicles: list[VehicleState]
    scenario: Scenario

    @property
    def vehicle(self) -> Optional[VehicleState]:
        """Convenience accessor for single-drone code/tests: the first vehicle."""
        return self.vehicles[0] if self.vehicles else None


# @dataclass
# class DroneMission:
#     """Tracks one drone's current assignment within a fleet."""
#     id: str
#     vehicle: "VehicleInterface"
#     state: MissionState = MissionState.IDLE
#     active_orders: list[DeliveryOrder] = field(default_factory=list)
#     active_route: Optional[Route] = None

@dataclass
class DroneMission:
    id: str
    vehicle: "VehicleInterface"
    state: MissionState = MissionState.IDLE
    active_orders: list[DeliveryOrder] = field(default_factory=list)
    active_route: Optional[Route] = None
    created_at: float = 0.0
    assigned_at: Optional[float] = None
    completed_at: Optional[float] = None
    failed_at: Optional[float] = None
    orders_fulfilled: int = 0
    current_waypoint: Optional[int] = None
    mission_number: int = 0  # Added to track the mission number for each drone

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "mission_number": self.mission_number,
            "created_at": self.created_at,
            "assigned_at": self.assigned_at,
            "completed_at": self.completed_at,
            "failed_at": self.failed_at,
            "status": self.state.value,
            "vehicle_id": self.vehicle.id,
            "order_ids": [order.id for order in self.active_orders],
            "route": [
                {
                    "x": waypoint.position.x,
                    "y": waypoint.position.y,
                    "z": waypoint.position.z,
                }
                for waypoint in self.active_route.waypoints
            ] if self.active_route else [],
            "orders_fulfilled": self.orders_fulfilled,
            "current_waypoint": self.current_waypoint,
        }


class VehicleInterface(ABC):

    @property
    @abstractmethod
    def id(self) -> str:
        ...
        
    @abstractmethod
    def get_state(self) -> VehicleState:
        ...

    @abstractmethod
    def upload_route(self, route: Route) -> None:
        ...

    @abstractmethod
    def start_mission(self) -> None:
        ...

    @abstractmethod
    def pause_mission(self) -> None:
        ...

    @abstractmethod
    def abort_mission(self) -> None:
        ...

    @abstractmethod
    def return_to_base(self) -> None:
        ...

    @abstractmethod
    def advance(self, dt: float) -> None:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def to_dict(self) -> dict:
        ...

    @abstractmethod
    def sample(self, time: float) -> dict:
        ...

class EnvironmentInterface(ABC):

    @abstractmethod
    def get_scenario(self) -> Scenario:
        ...

    @abstractmethod
    def set_scenario(self, scenario: Scenario) -> None:
        ...

    @abstractmethod
    def get_obstacles(self) -> list[Obstacle]:
        ...

    @abstractmethod
    def get_wind(self) -> Wind:
        ...


class SimulationInterface(ABC):

    @abstractmethod
    def advance(self, dt: float) -> None:
        ...

    @abstractmethod
    def get_snapshot(self) -> SimulationSnapshot:
        ...

    @abstractmethod
    def reset(self) -> None:
        ...

    @abstractmethod
    def start(self) -> None:
        ...

    @abstractmethod
    def stop(self) -> None:
        ...