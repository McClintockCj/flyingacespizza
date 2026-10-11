"""V2 energy budget using actual planned legs and smooth trajectory durations.

The estimator is deliberately conservative and empirical, not a calibrated
motor/electrical model. Ground handling consumes no propulsion energy.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from contracts import Vector3
from rotorpy_adapter_v2 import SmoothRouteTrajectory


@dataclass(frozen=True)
class EnergyPolicyV2:
    reserve_percent: float = 15.0
    uncertainty_factor: float = 1.25
    headwind_power_per_mps: float = 15.0
    crosswind_power_per_mps: float = 5.0
    tailwind_credit_per_mps: float = 4.0
    max_tailwind_credit_fraction: float = 0.15

    def __post_init__(self):
        if not 0 <= self.reserve_percent < 100:
            raise ValueError('reserve_percent must be in [0,100)')
        if self.uncertainty_factor < 1:
            raise ValueError('uncertainty_factor must be >= 1')


@dataclass(frozen=True)
class EnergyBudgetV2:
    estimated_wh: float
    budgeted_wh: float
    available_wh: float
    reserve_wh: float
    feasible: bool


def _wind_vector(wind):
    if wind is None:
        return (0.0, 0.0, 0.0)
    v = wind.velocity
    return (float(v.x), float(v.y), float(v.z))


def directional_power(battery, payload_kg: float, wind, displacement, speed: float,
                      policy: EnergyPolicyV2) -> float:
    """Directional correction to baseline power. Assumes baseline's wind penalty
    is isotropic; removes it when identifiable to avoid double counting.
    """
    model = battery.discharge_model
    w = _wind_vector(wind)
    wind_speed = math.sqrt(sum(a*a for a in w))
    base = float(model.power_draw(payload_weight=payload_kg, wind=None, cruise_speed=speed))
    norm = math.sqrt(sum(float(x)*float(x) for x in displacement))
    if norm < 1e-9:
        return max(0.0, base + policy.crosswind_power_per_mps * wind_speed)
    along = sum(float(displacement[i]) * w[i] for i in range(3)) / norm
    cross = math.sqrt(max(0.0, wind_speed**2 - along**2))
    extra = (policy.headwind_power_per_mps * max(0.0, -along)
             + policy.crosswind_power_per_mps * cross
             - policy.tailwind_credit_per_mps * max(0.0, along))
    return max(base * (1.0 - policy.max_tailwind_credit_fraction), base + extra)


def estimate_mission_energy(vehicle, mission, orders, wind,
                            policy: EnergyPolicyV2 = EnergyPolicyV2()) -> EnergyBudgetV2:
    battery = vehicle.get_state().battery
    payload = sum(o.payload_weight for o in orders)
    by_id = {o.id: o for o in orders}
    estimated = 0.0
    for leg in mission.legs:
        traj = SmoothRouteTrajectory(leg.route, vehicle.config.cruise_speed)
        for i, duration in enumerate(traj.durations):
            p, q = traj.points[i], traj.points[i+1]
            delta = tuple(float(q[k]-p[k]) for k in range(3))
            power = directional_power(battery, payload, wind, delta,
                                      vehicle.config.cruise_speed, policy)
            estimated += power * duration / 3600.0
        if leg.order_id is not None:
            payload = max(0.0, payload - by_id[leg.order_id].payload_weight)
    budgeted = estimated * policy.uncertainty_factor
    reserve = battery.capacity_wh * policy.reserve_percent / 100.0
    available = battery.remaining_energy()
    return EnergyBudgetV2(estimated, budgeted, available, reserve,
                          budgeted + reserve <= available + 1e-9)
