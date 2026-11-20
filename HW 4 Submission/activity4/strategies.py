"""Stateless strategy implementations and registry. Replaceable without engine changes."""
from __future__ import annotations
import math
from typing import Callable
from itertools import permutations
from .model import FlightEstimate, Order, Segment, Vehicle, distance


def fifo_kitchen(orders: list[Order]) -> list[Order]:
    return sorted(orders, key=lambda o: (o.spec.created, o.spec.id))


def shortest_prep_kitchen(orders: list[Order]) -> list[Order]:
    return sorted(orders, key=lambda o: (o.spec.prep_duration, o.spec.created, o.spec.id))


def immediate_batch(ready: list[Order], max_batch: int, elapsed: float, wait_min: float) -> list[Order]:
    return ready[:max_batch]


def timed_batch(ready: list[Order], max_batch: int, elapsed: float, wait_min: float) -> list[Order]:
    return ready[:max_batch] if len(ready) >= max_batch or elapsed >= wait_min - 1e-8 else []


def ordered_route(orders: list[Order]) -> list[Order]:
    return list(orders)


def nearest_route(orders: list[Order]) -> list[Order]:
    remaining = list(orders)
    result = []
    x = y = 0.0
    while remaining:
        selected = min(remaining, key=lambda o: (distance(x, y, o.spec.x, o.spec.y), o.spec.id))
        remaining.remove(selected)
        result.append(selected)
        x, y = selected.spec.x, selected.spec.y
    return result


def farthest_route(orders: list[Order]) -> list[Order]:
    """Greedily visit the farthest remaining stop from the current location.

    Intended as an intentionally different, not necessarily better, routing policy.
    Relies on Order and distance already imported in pizza_mc/strategies.py.
    """
    remaining = list(orders)
    route = []
    x = y = 0.0
    while remaining:
        next_order = min(remaining, key=lambda o: (-distance(x, y, o.spec.x, o.spec.y), o.spec.id))
        remaining.remove(next_order)
        route.append(next_order)
        x, y = next_order.spec.x, next_order.spec.y
    return route

def tour_length (route: list[Order]) -> float:
    x = y = total = 0.0
    # todo: accumulate hub -> each order, update x/y at each stop
    # todo: include return from last stop to (0,0)
    for order in route: 
        total += distance(x, y, order.spec.x, order.spec.y)
        x, y = order.spec.x, order.spec.y
    total += distance(x,y,0.0, 0.0)
    
    return total

def shortest_total_route(orders: list[Order]) -> list[Order]:
    if not orders:
        return []
    # todo: choose minimum candiate from permutations (orders)
    # use (tour_length(candidate), tuple(o.spec.id for o in candidate))
    best = min (permutations (orders), 
                key = lambda candidate: (tour_length(candidate), 
                tuple(o.spec.id for o in candidate))
                )
    
    return list (best)
    

def first_feasible(candidates: list[tuple[Vehicle, FlightEstimate]]) -> tuple[Vehicle, FlightEstimate] | None:
    return candidates[0] if candidates else None


def shortest_flight(candidates: list[tuple[Vehicle, FlightEstimate]]) -> tuple[Vehicle, FlightEstimate] | None:
    return min(candidates, key=lambda v: (v[1].duration_min, v[0].id)) if candidates else None


def ground_speed(vehicle: Vehicle, segment: Segment, wind_x: float, wind_y: float) -> float:
    d = max(segment.distance_m, 1e-9)
    ux, uy = segment.dx / d, segment.dy / d
    parallel = wind_x * ux + wind_y * uy
    cross = wind_x * (-uy) + wind_y * ux
    speed = vehicle.spec["cruise_mps"]
    if abs(cross) >= speed:
        return 0.0
    return max(0.0, math.sqrt(speed * speed - cross * cross) + parallel)


def calm_speed(vehicle: Vehicle, segment: Segment, wind_x: float, wind_y: float) -> float:
    return vehicle.spec["cruise_mps"]


def phase_power(vehicle: Vehicle, segment: Segment, minutes: float, ops: dict) -> float:
    cruise = vehicle.spec["cruise_w"] * (1.0 + 0.035 * segment.payload_kg)
    # A fixed overhead approximates takeoff/climb/landing, once per mission (handled by engine).
    return cruise * minutes / 60.0


def constant_power(vehicle: Vehicle, segment: Segment, minutes: float, ops: dict) -> float:
    return vehicle.spec["cruise_w"] * minutes / 60.0


def constant_charge(vehicle: Vehicle, target_fraction: float) -> float:
    missing = max(0.0, target_fraction * vehicle.battery_wh - vehicle.charge_wh)
    return 60.0 * missing / vehicle.spec["charge_w"]


def taper_charge(vehicle: Vehicle, target_fraction: float) -> float:
    # Lower rate for charging above 80% SOC; deliberately simple two-stage model.
    start = vehicle.charge_wh / vehicle.battery_wh
    fast_end = min(target_fraction, 0.8)
    fast = max(0.0, fast_end - start) * vehicle.battery_wh / vehicle.spec["charge_w"]
    slow = max(0.0, target_fraction - max(start, 0.8)) * vehicle.battery_wh / (0.4 * vehicle.spec["charge_w"])
    return 60.0 * (fast + slow)


STRATEGIES: dict[str, dict[str, Callable]] = {
    "kitchen": {"fifo": fifo_kitchen, "shortest_prep": shortest_prep_kitchen},
    "batching": {"immediate": immediate_batch, "timed": timed_batch},
    "routing": {"nearest_neighbor": nearest_route, "arrival_order": ordered_route, "farthest_first": farthest_route, "shortest_total": shortest_total_route},
    "dispatch": {"first_feasible": first_feasible, "shortest_flight": shortest_flight},
    "flight": {"vector_wind": ground_speed, "calm": calm_speed},
    "energy": {"phase_power": phase_power, "constant_power": constant_power},
    "charging": {"constant_power": constant_charge, "taper": taper_charge}
}


def get_strategy(group: str, name: str) -> Callable:
    try:
        return STRATEGIES[group][name]
    except KeyError:
        raise ValueError(f"Unknown strategy {group}={name}") from None


def estimate_flight(orders: list[Order], vehicle: Vehicle, inputs, cfg: dict) -> FlightEstimate:
    """Sequence: hub -> each delivery -> hub. Includes hover service for each order."""
    ops = cfg["operations"]
    flight = get_strategy("flight", cfg["strategies"]["flight"])
    energy = get_strategy("energy", cfg["strategies"]["energy"])
    payload = sum(o.spec.kg for o in orders)
    if payload > vehicle.spec["payload_kg"] + 1e-9:
        return FlightEstimate(0, 0, False, ())
    x = y = 0.0
    segments = []
    sequence = [(o.spec.x, o.spec.y, o.spec.kg, True) for o in orders] + [(0.0, 0.0, 0.0, False)]
    for dest_x, dest_y, delivered_kg, is_delivery in sequence:
        d = distance(x, y, dest_x, dest_y)
        seg = Segment(d, dest_x - x, dest_y - y, payload, is_delivery)
        velocity = flight(vehicle, seg, inputs.wind_x, inputs.wind_y)
        if velocity < 0.01:
            return FlightEstimate(0, 0, False, ())
        minutes = d / velocity / 60.0
        wh = energy(vehicle, seg, minutes, ops)
        if is_delivery:
            service = ops["delivery_service_min"]
            minutes += service
            wh += vehicle.spec["hover_w"] * service / 60.0
            payload -= delivered_kg
        segments.append((minutes, wh))
        x, y = dest_x, dest_y
    overhead = ops["flight_overhead_min"]
    duration = sum(t for t, _ in segments) + overhead
    consumed = sum(wh for _, wh in segments) + vehicle.spec["climb_w"] * overhead / 60.0
    feasible = vehicle.charge_wh - consumed >= vehicle.battery_wh * ops["reserve_fraction"]
    return FlightEstimate(duration, consumed, feasible, tuple(segments))

