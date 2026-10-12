from __future__ import annotations

import heapq
import math

from flying_aces.core.contracts import *
from flying_aces.missions.mission import FlightMission, LegAction, MissionLeg


class MissionPlanner:
    def __init__(self, environment: EnvironmentInterface):
        self.environment = environment
        self.obstacle_margin = 2.0
        self.landing_height = 0.15
        self.cruise_height = 3.0
        self.speed = 10.0

        # Simple energy model used for feasibility checks (not physical flight dynamics).
        # self.energy_per_distance = 0.01      # base energy per meter
        # self.weight_penalty_factor = 0.005    # extra energy per meter per kg carried
        # self.wind_penalty_factor = 0.001      # extra energy per meter per m/s of wind
        self.battery_reserve_margin = 0.85   # only spend this fraction of battery on a plan

    def plan_route(self, start: Vector3, destination: Vector3) -> Route:
        if self.route_is_clear(Route([
            Waypoint(start),
            Waypoint(destination),
        ])):
            return Route([
                Waypoint(start),
                Waypoint(destination),
            ])

        return self.plan_around_obstacles(start, destination)

    def route_is_clear(self, route: Route) -> bool:
        for i in range(1, len(route.waypoints)):
            a = route.waypoints[i - 1].position
            b = route.waypoints[i].position

            if not self.segment_is_clear(a, b):
                return False

        return True

    def segment_is_clear(self, a: Vector3, b: Vector3) -> bool:
        for obstacle in self.environment.get_obstacles():
            if obstacle.intersects_segment(a, b, margin=self.obstacle_margin):
                return False

        return True

    def plan_around_obstacles(
        self,
        start: Vector3,
        destination: Vector3,
    ) -> Route:
        obstacles = self.environment.get_obstacles()

        # Only consider obstacles that could affect the route.
        relevant_obstacles = [
            obstacle
            for obstacle in obstacles
            if min(start.z, destination.z) <= obstacle.height
        ]

        # Try flying over the relevant obstacles.
        max_height = max(
            (obstacle.height for obstacle in relevant_obstacles),
            default=max(start.z, destination.z),
        )

        clearance_altitude = (
            max_height
            + self.obstacle_margin
        )

        altitude_route = Route([
            Waypoint(Vector3(start.x, start.y, start.z)),
            Waypoint(Vector3(start.x, start.y, clearance_altitude)),
            Waypoint(Vector3(destination.x, destination.y, clearance_altitude)),
            Waypoint(Vector3(
                destination.x,
                destination.y,
                destination.z,
            )),
        ])

        if self.route_is_clear(altitude_route):
            return altitude_route

        # Fall back to horizontal routing.
        nodes = [
            Vector3(start.x, start.y, start.z),
            Vector3(destination.x, destination.y, destination.z),
        ]

        z = max(start.z, destination.z)

        for obstacle in relevant_obstacles:
            for x, y in obstacle.routing_points(self.obstacle_margin):
                nodes.append(Vector3(x, y, z))

        path = self._shortest_visible_path(nodes)

        if path is None:
            raise RuntimeError(
                "No collision-free route could be found."
            )

        return Route([
            Waypoint(nodes[index])
            for index in path
        ])
    
    def _shortest_visible_path(
        self,
        nodes: list[Vector3],
    ) -> list[int] | None:
        graph: dict[int, list[tuple[int, float]]] = {
            i: []
            for i in range(len(nodes))
        }

        for i in range(len(nodes)):
            for j in range(i + 1, len(nodes)):
                if not self.segment_is_clear(nodes[i], nodes[j]):
                    continue

                distance = nodes[i].distance_to(nodes[j])

                graph[i].append((j, distance))
                graph[j].append((i, distance))

        distances = {
            i: math.inf
            for i in range(len(nodes))
        }

        previous = {
            i: None
            for i in range(len(nodes))
        }

        distances[0] = 0.0

        queue = [(0.0, 0)]

        while queue:
            distance, current = heapq.heappop(queue)

            if distance > distances[current]:
                continue

            if current == 1:
                break

            for neighbor, edge_distance in graph[current]:
                new_distance = distance + edge_distance

                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current
                    heapq.heappush(
                        queue,
                        (new_distance, neighbor),
                    )

        if math.isinf(distances[1]):
            return None

        path = []
        current = 1

        while current is not None:
            path.append(current)
            current = previous[current]

        path.reverse()

        return self._remove_redundant_waypoints(path, nodes)

    def _remove_redundant_waypoints(
        self,
        path: list[int],
        nodes: list[Vector3],
    ) -> list[int]:
        if len(path) <= 2:
            return path

        result = [path[0]]
        i = 0

        while i < len(path) - 1:
            furthest = i + 1

            for j in range(i + 2, len(path)):
                if self.segment_is_clear(
                    nodes[path[i]],
                    nodes[path[j]],
                ):
                    furthest = j

            result.append(path[furthest])
            i = furthest

        return result

    # def leg_energy(self, distance: float, weight: float, wind: Wind) -> float:
    #     base = distance * self.energy_per_distance
    #     weight_penalty = distance * weight * self.weight_penalty_factor
    #     wind_penalty = wind.speed * distance * self.wind_penalty_factor
    #     return base + weight_penalty + wind_penalty

    # def estimate_energy(
    #     self,
    #     route: Route,
    #     vehicle: VehicleState,
    # ) -> float:
    #     """Single-route estimate assuming no payload (kept for backward compatibility)."""
    #     wind = self.environment.get_wind()
    #     return self.leg_energy(route.distance, 0.0, wind)

    def estimate_energy(self, route: Route, vehicle: VehicleState) -> float:
        """Estimate energy required for a route assuming no payload."""

        wind = self.environment.get_wind()

        return vehicle.battery.estimate_energy(
            distance=route.distance,
            payload_weight=0.0,
            wind=wind,
            cruise_speed=vehicle.cruise_speed,
        )

    def route_feasible(self, route: Route, vehicle: VehicleState) -> bool:
        energy_required = self.estimate_energy(route, vehicle,)

        # return energy_required < vehicle.battery_percent * self.battery_reserve_margin
        available_energy = (
            vehicle.battery.remaining_energy()
            * self.battery_reserve_margin
        )

        return energy_required < available_energy

    # ------------------------------------------------------------------
    # Multi-delivery support
    # ------------------------------------------------------------------

    def plan_multi_stop_route(self, start: Vector3, stops: list[Vector3]) -> Route:
        """Chains obstacle-avoiding legs through an ordered list of stops."""
        waypoints: list[Waypoint] = [Waypoint(start)]
        position = start

        for stop in stops:
            leg = self.plan_route(position, stop)
            waypoints.extend(leg.waypoints[1:])  # skip duplicate junction point
            position = stop

        return Route(waypoints)


    # def estimate_multi_stop_energy(
    #     self,
    #     start: Vector3,
    #     stops_with_weights: list[tuple[Vector3, float]],
    # ) -> float:
    #     """
    #     Energy for start -> stop1 -> stop2 -> ... Each entry's weight is the payload
    #     still on board *while flying that leg* (i.e. already excluding anything
    #     dropped off on an earlier leg).
    #     """
    #     wind = self.environment.get_wind()
    #     position = start
    #     total = 0.0

    #     for stop, weight_on_leg in stops_with_weights:
    #         leg = self.plan_route(position, stop)
    #         total += self._estimate_leg_energy(
    #             battery=vehicle_battery,
    #             distance=leg.distance,
    #             payload_weight=weight_on_leg,
    #             wind=wind,
    #             cruise_speed=cruise_speed,
    #         )
            
    #         position = stop

    #     return total

    def estimate_multi_stop_energy(
        self,
        start: Vector3,
        stops_with_weights: list[tuple[Vector3, float]],
        vehicle: VehicleState,
    ) -> float:
        wind = self.environment.get_wind()
        position = start
        total = 0.0

        for stop, weight_on_leg in stops_with_weights:
            leg = self.plan_route(position, stop)

            total += vehicle.battery.estimate_energy(
                distance=leg.distance,
                payload_weight=weight_on_leg,
                wind=wind,
                cruise_speed=vehicle.cruise_speed,
            )

            position = stop

        return total

    def plan_delivery_batch(
        self,
        vehicle_state: VehicleState,
        candidate_orders: list[DeliveryOrder],
        base: Vector3,
    ) -> tuple[list[DeliveryOrder], Optional[Route]]:
        capacity = vehicle_state.payload_capacity
        
        # Sort candidates so highest priority number comes first
        remaining = sorted(candidate_orders, key=lambda o: o.priority, reverse=True)

        batch: list[DeliveryOrder] = []
        total_weight = 0.0

        for candidate in remaining:
            if total_weight + candidate.payload_weight > capacity:
                continue

            trial_batch = batch + [candidate]
            stops_with_weights = self._stops_with_remaining_weight(trial_batch, base)
            
            try:
                energy = self.estimate_multi_stop_energy(
                    vehicle_state.position, stops_with_weights, vehicle_state
                )
            except RuntimeError:
                continue

            if energy <= vehicle_state.battery.remaining_energy() * self.battery_reserve_margin:
                batch.append(candidate)
                total_weight += candidate.payload_weight

        if not batch:
            return [], None

        try:
            stops = [order.destination for order in batch] + [base]
            route = self.plan_multi_stop_route(vehicle_state.position, stops)
            return batch, route
        except RuntimeError:
            return [], None

    @staticmethod
    def _stops_with_remaining_weight(
        batch: list[DeliveryOrder],
        base: Vector3,
    ) -> list[tuple[Vector3, float]]:
        carried = sum(order.payload_weight for order in batch)
        stops = []

        for order in batch:
            stops.append((order.destination, carried))
            carried -= order.payload_weight

        stops.append((base, 0.0))
        return stops
    # Integrated multi-leg landing and delivery mission planning.
    def _clear(self, a: Vector3, b: Vector3) -> bool:
        return self.segment_is_clear(a, b)

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

        transit = self.plan_route(departure, approach)
        points = [start]
        self._append(points, departure)
        for waypoint in transit.waypoints[1:]:
            self._append(points, waypoint.position)
        self._append(points, touchdown)
        route = Route([Waypoint(p, speed=speed) for p in points])
        if not self.route_is_clear(route):
            raise RuntimeError("V2 route failed obstacle clearance validation")
        return route

    def plan_mission(self, mission_id: str, start: Vector3,
                     orders: list[DeliveryOrder], base: Vector3, speed: float = 10.0) -> FlightMission:
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
        return FlightMission(mission_id, legs)
