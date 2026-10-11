"""Incremental RotorPy 3.x adapter for one simulator Route at a time.

Parallel implementation: does not replace rotorpy_adapter.py or change fleet.py.
RotorPy is authoritative for position, velocity, orientation, and wind dynamics.
"""
from __future__ import annotations

from copy import deepcopy
from enum import Enum
import math

import numpy as np

from contracts import DroneState, Route, Vector3, VehicleInterface, VehicleState, Wind
from battery import Battery
from drone_config import DroneConfig
from trajectory import SmoothRouteTrajectory


class RouteStatus(str, Enum):
    IDLE = "idle"
    READY = "ready"
    FLYING = "flying"
    COMPLETE = "complete"
    FAILED = "failed"



class RotorPyVehicleAdapterV2(VehicleInterface):
    def __init__(self, rotorpy_vehicle, rotorpy_environment, rotorpy_controller,
                 config: DroneConfig, vehicle_id: str, home_position: Vector3,
                 battery: Battery, wind: Wind | None = None, sim_rate: int = 100,
                 arrival_tolerance: float = 0.35, velocity_tolerance: float = 0.35,
                 settle_time: float = 0.3, arrival_timeout: float = 15.0):
        if sim_rate <= 0:
            raise ValueError("sim_rate must be positive")
        self.vehicle = rotorpy_vehicle
        self.environment = rotorpy_environment
        self.controller = rotorpy_controller
        self.config = config
        self._vehicle_id = vehicle_id
        self.home_position = deepcopy(home_position)
        self.wind = wind or Wind(velocity=Vector3(0, 0, 0))
        self.sim_rate = sim_rate
        self.physics_dt = 1.0 / sim_rate
        self.arrival_tolerance = arrival_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.settle_time = settle_time
        self.arrival_timeout = arrival_timeout
        self.battery = battery
        self.initial_rotorpy_state = deepcopy(rotorpy_vehicle.initial_state)
        self.rotorpy_state = deepcopy(self.initial_rotorpy_state)
        self.state = VehicleState(position=deepcopy(home_position), battery=battery,
                                  name=config.name, state=DroneState.LANDED,
                                  payload_capacity=config.payload_capacity,
                                  cruise_speed=config.cruise_speed)
        self.current_route: Route | None = None
        self.trajectory: SmoothRouteTrajectory | None = None
        self.route_status = RouteStatus.IDLE
        self.current_waypoint_idx = 0
        self.sim_time = 0.0
        self.route_time = 0.0
        self._accumulator = 0.0
        self._settled_for = 0.0
        self._last_control = None
        self.grounded = True
        self.energy_policy = None  # optional V2 policy; old behavior retained by default
        self._sync_state()

    @property
    def id(self) -> str:
        return self._vehicle_id

    @property
    def route_complete(self) -> bool:
        return self.route_status == RouteStatus.COMPLETE

    def get_state(self) -> VehicleState:
        return self.state

    def upload_route(self, route: Route) -> None:
        if self.route_status == RouteStatus.FLYING:
            raise RuntimeError("Cannot replace a route during flight")
        if not route.waypoints:
            raise ValueError("Cannot upload an empty route")
        # Start the trajectory at the *physical* vehicle position, not a
        # planner's potentially stale starting waypoint.
        current = self.rotorpy_state["x"]
        from contracts import Waypoint
        points = [Waypoint(Vector3(*map(float, current)))] + [deepcopy(w) for w in route.waypoints]
        self.current_route = route
        self.trajectory = SmoothRouteTrajectory(Route(points), self.config.cruise_speed, max_acceleration=2.0)
        self.route_time = 0.0
        self._accumulator = 0.0
        self._settled_for = 0.0
        self.current_waypoint_idx = 0
        self.route_status = RouteStatus.READY

    def start_route(self) -> None:
        print(
            f"[{self.id}] START ROUTE",
            f"position={self.rotorpy_state['x']}",
            f"velocity={self.rotorpy_state['v']}",
            f"orientation={self.rotorpy_state['q']}",
            f"angular_velocity={self.rotorpy_state['w']}",
        )
        
        if self.route_status != RouteStatus.READY:
            raise RuntimeError(f"Cannot start route in status {self.route_status}")
        self.grounded = False
        self.route_status = RouteStatus.FLYING
        self.state.state = DroneState.FLYING

    def land(self) -> None:
        """Operational ground hold; RotorPy does not model physical contact."""
        if self.route_status != RouteStatus.COMPLETE:
            raise RuntimeError("Cannot ground before route completion")
        self.grounded = True
        self.rotorpy_state["v"] = np.zeros(3)
        self.rotorpy_state["w"] = np.zeros(3)
        self._sync_state()
        self.state.state = DroneState.LANDED

    def start_mission(self) -> None:
        self.start_route()

    def pause_mission(self) -> None:
        raise NotImplementedError("Pause requires a new hover trajectory; not implemented")

    def abort_mission(self) -> None:
        self.route_status = RouteStatus.FAILED
        self.state.state = DroneState.EMERGENCY

    def return_to_base(self) -> None:
        raise NotImplementedError("MissionManager must plan and upload a return route")

    def advance(self, dt: float) -> None:
        if dt <= 0 or self.route_status != RouteStatus.FLYING:
            return
        self._accumulator += dt
        while self._accumulator + 1e-12 >= self.physics_dt and self.route_status == RouteStatus.FLYING:
            self._physics_step(self.physics_dt)
            self._accumulator -= self.physics_dt

    def _physics_step(self, dt: float) -> None:
        # Match RotorPy simulate.py: wind -> controller -> vehicle.step.
        self.rotorpy_state["wind"] = np.asarray(
            self.environment.wind_profile.update(self.sim_time, self.rotorpy_state["x"]), dtype=float)
        flat = self.trajectory.update(self.route_time)
        control = self.controller.update(self.sim_time, self.rotorpy_state, flat)
        
        if not np.all(np.isfinite(self.rotorpy_state["x"])):
            raise RuntimeError("Non-finite vehicle position")

        if not np.all(np.isfinite(control["cmd_motor_speeds"])):
            raise RuntimeError("Non-finite motor commands")
        
        self.rotorpy_state = self.vehicle.step(self.rotorpy_state, control, dt)
        
        if self.route_time < 3.0 and int(self.route_time / dt) % 20 == 0:
            print(
                f"[{self.id}]",
                f"t={self.route_time:.2f}",
                f"pos={np.round(self.rotorpy_state['x'], 2)}",
                f"vel={np.round(self.rotorpy_state['v'], 2)}",
                f"target={np.round(flat['x'], 2)}",
                f"motor={np.round(control['cmd_motor_speeds'], 0)}",
            )
        
        self._last_control = control
        self.sim_time += dt
        self.route_time += dt
        self._sync_state()
        if self.energy_policy is None:
            self.battery.discharge(dt=dt, payload_weight=self.state.payload_weight,
                                   wind=self.wind, cruise_speed=self.config.cruise_speed)
        else:
            from energy_feasibility_v2 import directional_power
            # Use the planned trajectory direction rather than noisy instantaneous
            # velocity, especially during takeoff/landing and settling.
            next_flat = self.trajectory.update(min(self.route_time + dt, self.trajectory.total_time))
            displacement = next_flat["x"] - flat["x"]
            power = directional_power(self.battery, self.state.payload_weight,
                                      self.wind, displacement,
                                      self.config.cruise_speed, self.energy_policy)
            self.battery.discharge_power(dt, power)
        if self.battery.is_empty():
            self.abort_mission()
            self.state.fault = "battery_depleted"
            return
        target = self.trajectory.points[-1]
        error = np.linalg.norm(self.rotorpy_state["x"] - target)
        speed = np.linalg.norm(self.rotorpy_state["v"])
        if self.route_time >= self.trajectory.total_time and error <= self.arrival_tolerance and speed <= self.velocity_tolerance:
            self._settled_for += dt
        else:
            self._settled_for = 0.0
        if self._settled_for >= self.settle_time:
            self.route_status = RouteStatus.COMPLETE
            self.state.state = DroneState.HOVERING
            self.current_waypoint_idx = len(self.current_route.waypoints)
        elif self.route_time > self.trajectory.total_time + self.arrival_timeout:
            self.route_status = RouteStatus.FAILED
            self.state.fault = f"route_arrival_timeout: position_error={error:.2f}m"
            self.state.state = DroneState.EMERGENCY

    def _sync_state(self) -> None:
        x = self.rotorpy_state["x"]
        v = self.rotorpy_state["v"]
        self.state.position = Vector3(*map(float, x))
        self.state.velocity = Vector3(*map(float, v))

    def reset(self) -> None:
        self.rotorpy_state = deepcopy(self.initial_rotorpy_state)
        self.battery.reset()
        self.state.payload_weight = 0.0
        self.state.payload_loaded = False
        self.state.fault = None
        self.state.state = DroneState.LANDED
        self.current_route = None
        self.trajectory = None
        self.route_status = RouteStatus.IDLE
        self.grounded = True
        self.current_waypoint_idx = 0
        self.sim_time = self.route_time = self._accumulator = self._settled_for = 0.0
        self._sync_state()

    def to_dict(self) -> dict:
        return {"id": self.id, "name": self.config.name, "type": self.config.name,
                "specs": {"cruise_speed": self.config.cruise_speed,
                          "payload_capacity": self.config.payload_capacity,
                          "rotor_speed": self.config.rotor_speed,
                          "battery_capacity_wh": self.config.battery_capacity_wh}}

    def sample(self, time: float) -> dict:
        return {"time": time,
                "position": vars(self.state.position).copy(),
                "velocity": vars(self.state.velocity).copy(),
                "battery": {"remaining_wh": self.battery.remaining_energy(),
                            "capacity_wh": self.battery.capacity_wh,
                            "remaining_percent": self.battery.remaining_percent()},
                "payload_weight": self.state.payload_weight,
                "state": self.state.state.value,
                "route_status": self.route_status.value,
                "fault": self.state.fault}
