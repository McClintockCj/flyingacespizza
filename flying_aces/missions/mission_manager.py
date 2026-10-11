"""Fleet mission manager.

The manager owns V2 physics advancement: do NOT also call vehicle.advance(dt)
from Simulation for these vehicles.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from flying_aces.core.contracts import DeliveryOrder, MissionState, Vector3
from flying_aces.missions.mission import FlightMission, MissionExecutor, MissionStatus, MissionLeg, LegAction
from flying_aces.missions.mission_planner import MissionPlanner
from flying_aces.aircraft.rotorpy_vehicle import RotorPyVehicle
from flying_aces.missions.energy_estimator import EnergyPolicy, estimate_mission_energy


class GroundPhase(str, Enum):
    IDLE = "idle"
    LOADING = "loading"
    RECHARGING = "recharging"


@dataclass
class GroundOperationsConfig:
    loading_time: float = 5.0
    unloading_time: float = 3.0
    recharge_threshold: float = 30.0
    recharge_release_threshold: float = 80.0
    emergency_threshold: float = 15.0

    def __post_init__(self):
        if min(self.loading_time, self.unloading_time) < 0:
            raise ValueError("Ground delays cannot be negative")
        if not (0 <= self.emergency_threshold < self.recharge_threshold < self.recharge_release_threshold <= 100):
            raise ValueError("Invalid battery thresholds")


@dataclass
class ActiveFlight:
    vehicle: RotorPyVehicle
    executor: MissionExecutor | None = None
    orders: list[DeliveryOrder] | None = None
    mission_number: int = 0
    state: MissionState = MissionState.IDLE
    ground_phase: GroundPhase = GroundPhase.IDLE
    loading_remaining: float = 0.0
    rtb_requested: bool = False
    recoverable_abort: bool = False
    recovery_requested: bool = False


class MissionManager:
    def __init__(self, vehicles: list[RotorPyVehicle], environment, planner,
                 landing_height: float = 0.15, cruise_height: float = 3.0,
                 speed: float = 1.0, min_battery_percent: float = 30.0,
                 ground_config: GroundOperationsConfig | None = None,
                 energy_policy: EnergyPolicy | None = None):
        self.environment = environment
        self.planner = planner
        self.leg_planner = planner
        self.leg_planner.landing_height = landing_height
        self.leg_planner.cruise_height = cruise_height
        self.leg_planner.speed = speed
        self.flights = [ActiveFlight(vehicle=v) for v in vehicles]
        self.completed_missions: list[dict] = []
        self.events: list[dict] = []
        self.time = 0.0
        self.ground_config = ground_config or GroundOperationsConfig(recharge_threshold=min_battery_percent)
        self.min_battery_percent = self.ground_config.recharge_threshold
        self.energy_policy = energy_policy or EnergyPolicy(
            reserve_percent=self.ground_config.emergency_threshold)
        for flight in self.flights:
            flight.vehicle.energy_policy = self.energy_policy

    @property
    def vehicles(self):
        return [flight.vehicle for flight in self.flights]

    @property
    def orders(self) -> list[DeliveryOrder]:
        return self.environment.get_scenario().orders

    @property
    def pending_orders(self) -> list[DeliveryOrder]:
        return sorted((o for o in self.orders if o.status == 'pending'),
                      key=lambda o: o.priority, reverse=True)

    def add_order(self, order: DeliveryOrder):
        self.orders.append(order)

    def _candidate_mission(self, flight: ActiveFlight, assume_full: bool = False):
        """Greedy priority batch, evaluated against *actual* V2 legs and battery.

        assume_full checks whether charging can ever help, without changing
        battery state. A too-heavy or intrinsically infeasible order stays pending.
        """
        state = flight.vehicle.get_state()
        # Each fleet vehicle has its own parking/home location. The shared
        # scenario base is only the fleet's reference point.
        home = flight.vehicle.home_position
        base = Vector3(home.x, home.y, self.leg_planner.landing_height)
        landing_base = base
        if state.position.distance_to(landing_base) > 2.5:
            return None
        battery = state.battery
        capacity = state.payload_capacity
        chosen = []
        chosen_mission = None
        chosen_budget = None
        total_weight = 0.0
        for order in self.pending_orders:
            if total_weight + order.payload_weight > capacity:
                continue
            trial = chosen + [order]
            try:
                mission = self.leg_planner.plan_mission(
                    f'{flight.vehicle.id}-mission-{flight.mission_number + 1}',
                    state.position, trial, base, speed=flight.vehicle.state.cruise_speed)
                budget = estimate_mission_energy(
                    flight.vehicle, mission, trial, self.environment.get_wind(), self.energy_policy)
            except (RuntimeError, ValueError):
                continue
            available = battery.capacity_wh if assume_full else battery.remaining_energy()
            if budget.budgeted_wh + budget.reserve_wh <= available + 1e-9:
                chosen = trial
                chosen_mission = mission
                chosen_budget = budget
                total_weight += order.payload_weight
        if not chosen:
            return None
        return chosen, chosen_mission, chosen_budget

    def _dispatch(self, flight: ActiveFlight):
        state = flight.vehicle.get_state()
        if state.battery.remaining_percent() < self.min_battery_percent:
            return False
        candidate = self._candidate_mission(flight)
        if candidate is None:
            return False
        batch, mission, budget = candidate
        executor = MissionExecutor(flight.vehicle, mission,
                                     landing_height=self.leg_planner.landing_height,
                                     unloading_time=self.ground_config.unloading_time)
        for order in batch:
            order.status = 'dispatched'
        flight.mission_number += 1
        flight.executor = executor
        flight.orders = batch
        flight.state = MissionState.READY
        flight.ground_phase = GroundPhase.LOADING
        flight.loading_remaining = self.ground_config.loading_time
        flight.rtb_requested = False
        flight.recoverable_abort = False
        self._ground_event(flight, 'loading_started')
        self.events[-1].update(estimated_wh=round(budget.estimated_wh, 4),
                               budgeted_wh=round(budget.budgeted_wh, 4),
                               reserve_wh=round(budget.reserve_wh, 4))
        try:
            if flight.loading_remaining == 0:
                self._finish_loading(flight)
        except Exception:
            for order in batch:
                order.status = 'pending'
            flight.executor = None
            flight.orders = None
            flight.state = MissionState.IDLE
            state.payload_weight = 0.0
            state.payload_loaded = False
            flight.ground_phase = GroundPhase.IDLE
            raise
        return True

    def _ground_event(self, flight, kind):
        self.events.append({'vehicle_id': flight.vehicle.id,
                            'mission_id': flight.executor.mission.id if flight.executor else None,
                            'time': round(self.time, 3), 'type': kind})

    def _finish_loading(self, flight):
        state = flight.vehicle.get_state()
        state.payload_weight = sum(o.payload_weight for o in flight.orders or [])
        state.payload_loaded = state.payload_weight > 0
        flight.ground_phase = GroundPhase.IDLE
        self._ground_event(flight, 'loading_completed')
        flight.executor.start(time=self.time)
        flight.state = MissionState.EXECUTING

    def _consume_events(self, flight: ActiveFlight):
        executor = flight.executor
        mission = executor.mission
        for event in mission.events:
            if event.get('_processed'):
                continue
            event['_processed'] = True
            clean = {k: v for k, v in event.items() if k != '_processed'}
            self.events.append({'vehicle_id': flight.vehicle.id, 'mission_id': mission.id, **clean})
            if event['type'] == 'leg_started':
                for order in flight.orders or []:
                    if order.status == 'dispatched':
                        order.status = 'in_progress'
            elif event['type'] == 'order_delivered':
                order = next((o for o in flight.orders or [] if o.id == event['order_id']), None)
                if order is not None and order.status != 'delivered':
                    order.status = 'delivered'
                    order.completed_at = self.time
                    state = flight.vehicle.get_state()
                    state.payload_weight = max(0.0, state.payload_weight - order.payload_weight)
                    state.payload_loaded = state.payload_weight > 0

    def _finish(self, flight: ActiveFlight):
        mission = flight.executor.mission
        failed = mission.status == MissionStatus.FAILED
        # Any undelivered orders (including those skipped by RTB) return to the queue.
        for order in flight.orders or []:
            if order.status in ('dispatched', 'in_progress'):
                order.status = 'pending'
        self.completed_missions.append({
            'id': mission.id, 'vehicle_id': flight.vehicle.id,
            'status': mission.status.value, 'fault': mission.fault,
            'order_ids': [o.id for o in flight.orders or []],
            'delivered_orders': list(mission.delivered_orders),
            'events': list(self.events_for_mission(mission.id)),
            'completed_at': self.time,
        })
        flight.state = MissionState.FAILED if failed else MissionState.COMPLETE
        flight.executor = None
        flight.orders = None
        flight.rtb_requested = False
        flight.ground_phase = GroundPhase.IDLE
        if not failed:
            state = flight.vehicle.get_state()
            state.payload_weight = 0.0
            state.payload_loaded = False

    def events_for_mission(self, mission_id):
        return (event for event in self.events if event.get('mission_id') == mission_id)

    def update(self, dt: float = 0.1, time: float | None = None, auto_dispatch: bool = True):
        if dt <= 0:
            return
        self.time = self.time + dt if time is None else time
        cfg = self.ground_config
        for flight in self.flights:
            battery = flight.vehicle.get_state().battery
            if flight.ground_phase == GroundPhase.LOADING:
                flight.loading_remaining = max(0.0, flight.loading_remaining - dt)
                if flight.loading_remaining == 0:
                    self._finish_loading(flight)
                continue
            if flight.executor is None and flight.recoverable_abort:
                # Aborted aircraft must keep running RotorPy physics while hovering.
                flight.vehicle.advance(dt)
                if flight.recovery_requested and flight.vehicle.hover_ready:
                    self._recover_aborted_flight(flight)
                continue
            if flight.executor is not None:
                flight.executor.advance(dt, time=self.time)
                self._consume_events(flight)
                if flight.executor.mission.status in (MissionStatus.COMPLETE, MissionStatus.FAILED):
                    self._finish(flight)
                continue
            if flight.ground_phase == GroundPhase.RECHARGING:
                battery.recharge(dt)
                if battery.remaining_percent() >= cfg.recharge_release_threshold:
                    # If no feasible work exists, stop at release threshold.
                    # If work exists but still cannot be flown, keep charging.
                    if not self.pending_orders or self._candidate_mission(flight) is not None:
                        flight.ground_phase = GroundPhase.IDLE
                        flight.state = MissionState.IDLE
                        self._ground_event(flight, 'recharging_completed')
                    elif self._candidate_mission(flight, assume_full=True) is None:
                        flight.ground_phase = GroundPhase.IDLE
                        flight.state = MissionState.IDLE
                        self._ground_event(flight, 'recharging_completed')
                continue
            low_charge = battery.remaining_percent() < cfg.recharge_threshold
            insufficient_for_work = (bool(self.pending_orders)
                                     and self._candidate_mission(flight) is None
                                     and self._candidate_mission(flight, assume_full=True) is not None)
            if low_charge or insufficient_for_work:
                flight.ground_phase = GroundPhase.RECHARGING
                flight.state = MissionState.RECHARGING
                self._ground_event(flight, 'recharging_started')
                battery.recharge(dt)
        if auto_dispatch:
            for flight in self.flights:
                if not self.pending_orders:
                    break
                if flight.executor is None and flight.ground_phase == GroundPhase.IDLE and flight.state in (MissionState.IDLE, MissionState.COMPLETE):
                    self._dispatch(flight)

    def _recover_aborted_flight(self, flight: ActiveFlight) -> str | None:
        """Create a new, return-only mission after an explicit manual abort.

        This is a best-effort flight recovery, not a guaranteed emergency landing.
        Preserve the failed mission in history and keep its undelivered orders
        pending. Never automatically recover a physics or battery failure.
        """
        if (not flight.recoverable_abort or flight.executor is not None
                or not flight.vehicle.hover_ready):
            return None
        vehicle = flight.vehicle
        state = vehicle.get_state()
        position = state.position
        velocity = state.velocity
        if (not all(math.isfinite(v) for v in
                    (position.x, position.y, position.z,
                     velocity.x, velocity.y, velocity.z))
                or position.z < 0.0 or state.battery.is_empty()):
            return None
        home = vehicle.home_position
        base = Vector3(home.x, home.y, self.leg_planner.landing_height)
        try:
            route = self.leg_planner.plan_landing_leg(
                position, base, speed=state.cruise_speed)
        except (RuntimeError, ValueError):
            return None
        flight.mission_number += 1
        mission = FlightMission(
            f'{vehicle.id}-recovery-{flight.mission_number}',
            [MissionLeg(route, LegAction.RETURN)],
        )
        executor = MissionExecutor(
            vehicle, mission,
            landing_height=self.leg_planner.landing_height,
            unloading_time=self.ground_config.unloading_time,
        )
        # upload_route accepts FAILED, but the previous abort fault must not
        # contaminate the recovery mission's diagnostics.
        state.fault = None
        try:
            executor.start(time=self.time)
        except Exception:
            state.fault = 'manual_abort'
            flight.mission_number -= 1
            return None
        flight.executor = executor
        flight.orders = []
        flight.state = MissionState.EXECUTING
        flight.ground_phase = GroundPhase.IDLE
        flight.recoverable_abort = False
        flight.recovery_requested = False
        flight.rtb_requested = True
        self._ground_event(flight, 'aborted_mission_rtb_started')
        return mission.id

    def request_return_to_base(self) -> str | None:
        """Defer RTB during a mission; recover manually aborted flights on R."""
        for flight in self.flights:
            if flight.executor is None:
                if flight.recoverable_abort:
                    flight.recovery_requested = True
                    if flight.vehicle.hover_ready:
                        return self._recover_aborted_flight(flight)
                    self._ground_event(flight, 'rtb_waiting_for_stable_hover')
                    return f'{flight.vehicle.id}-rtb-pending'
                continue
            if flight.ground_phase == GroundPhase.LOADING:
                # No takeoff yet; cancel safely, keeping all orders pending.
                return self.abort_active_mission()
            mission = flight.executor.mission
            if mission.status != MissionStatus.RUNNING:
                continue
            if flight.rtb_requested:
                return mission.id
            executor = flight.executor
            current = mission.legs[mission.current_leg_index]
            if current.action == LegAction.RETURN:
                return mission.id
            landing = current.route.waypoints[-1].position
            home = flight.vehicle.home_position
            base = Vector3(home.x, home.y, self.leg_planner.landing_height)
            try:
                route = self.leg_planner.plan_landing_leg(
                    landing, base, speed=flight.vehicle.state.cruise_speed)
            except (RuntimeError, ValueError):
                return None
            # Replace future legs, never the active leg. Executor will upload
            # the return leg when the current delivery/unloading is finished.
            mission.legs = mission.legs[:mission.current_leg_index + 1] + [
                MissionLeg(route, LegAction.RETURN)]
            flight.rtb_requested = True
            self._ground_event(flight, 'return_to_base_requested')
            return mission.id
        return None

    def preview_next_mission(self) -> dict | None:
        """Read-only preview of the first feasible dispatch (does not reserve orders)."""
        for flight in self.flights:
            if (flight.executor is not None or flight.ground_phase != GroundPhase.IDLE
                    or flight.state not in (MissionState.IDLE, MissionState.COMPLETE)):
                continue
            if flight.vehicle.get_state().battery.remaining_percent() < self.min_battery_percent:
                continue
            candidate = self._candidate_mission(flight)
            if candidate is not None:
                orders, mission, budget = candidate
                return {"vehicle_id": flight.vehicle.id, "mission_id": mission.id,
                        "order_ids": [order.id for order in orders],
                        "estimated_wh": budget.estimated_wh,
                        "budgeted_wh": budget.budgeted_wh, "mission": mission}
        return None

    def dispatch_next_mission(self) -> str | None:
        """Start loading one feasible mission; automatic dispatch may remain disabled."""
        for flight in self.flights:
            if (flight.executor is None and flight.ground_phase == GroundPhase.IDLE
                    and flight.state in (MissionState.IDLE, MissionState.COMPLETE)):
                if self._dispatch(flight):
                    return flight.executor.mission.id
        return None

    def abort_active_mission(self) -> str | None:
        """Emergency stop of one active mission; does not simulate safe landing.

        Affected undelivered orders are requeued. The vehicle remains in
        EMERGENCY/FAILED and is not eligible for automatic redispatch.
        """
        for flight in self.flights:
            if flight.executor is None:
                continue
            mission = flight.executor.mission
            if mission.status not in (MissionStatus.READY, MissionStatus.RUNNING):
                continue
            if flight.ground_phase == GroundPhase.LOADING:
                # No flight has started: cancel the pending load safely.
                for order in flight.orders or []:
                    if order.status in ('dispatched', 'in_progress'):
                        order.status = 'pending'
                flight.executor = None
                flight.orders = None
                flight.loading_remaining = 0.0
                flight.ground_phase = GroundPhase.IDLE
                flight.state = MissionState.IDLE
                flight.recoverable_abort = False
                state = flight.vehicle.get_state()
                state.payload_weight = 0.0
                state.payload_loaded = False
                self._ground_event(flight, 'loading_cancelled')
                return mission.id
            # Manual abort is a controlled interruption, not a physics failure.
            # Keep the RotorPy controller active and let it brake into hover.
            try:
                flight.vehicle.begin_hover()
            except RuntimeError:
                flight.vehicle.abort_mission()
                return None
            mission.status = MissionStatus.FAILED
            mission.fault = 'manual_abort'
            flight.executor._event('mission_failed', fault=mission.fault)
            self._consume_events(flight)
            self._finish(flight)
            flight.recoverable_abort = True
            flight.vehicle.get_state().fault = 'manual_abort'
            flight.recovery_requested = False
            self._ground_event(flight, 'hover_started_after_abort')
            return mission.id
        return None

    def reset(self):
        self.time = 0.0
        self.completed_missions.clear()
        self.events.clear()
        for flight in self.flights:
            flight.vehicle.reset()
            flight.executor = None
            flight.orders = None
            flight.mission_number = 0
            flight.state = MissionState.IDLE
            flight.ground_phase = GroundPhase.IDLE
            flight.loading_remaining = 0.0
            flight.rtb_requested = False
            flight.recoverable_abort = False
            flight.recovery_requested = False
        for order in self.orders:
            order.status = 'pending'
            order.completed_at = None
            order.failed_at = None
