"""Regression for ground hold, timed handling, consecutive missions, and conditional charging."""
from contracts import DeliveryOrder, Scenario, Vector3, Wind
from fleet import build_vehicle, get_drone_config
from mission_manager_v2 import GroundOperationsConfig, GroundPhase, MissionManagerV2
from mission_planner import MissionPlanner
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2
from scenario import ScenarioManager

home = Vector3(0, 0, 2)
wind = Wind(Vector3(0, 2, 0))
config = get_drone_config('Light')
old = build_vehicle(home, wind, config, 'test-1')
vehicle = RotorPyVehicleAdapterV2(old.vehicle, old.environment, old.controller,
                                  config, 'test-1', home, old.state.battery, wind)
order_a = DeliveryOrder('order-A', Vector3(5, 0, 2), payload_weight=0.5)
scenario = Scenario(bounds=(-10, -10, 10, 10), obstacles=[], wind=wind,
                    orders=[order_a], base=home)
environment = ScenarioManager(scenario)
manager = MissionManagerV2([vehicle], environment, MissionPlanner(environment),
                           speed=1.0, ground_config=GroundOperationsConfig(
                               loading_time=2.0, unloading_time=3.0))

second_added = False
unload_battery = None
unload_position = None
unload_physics_time = None
for i in range(1600):
    manager.update(0.1)
    flight = manager.flights[0]
    if flight.executor and flight.executor.unloading_remaining > 0:
        if unload_battery is None:
            unload_battery = vehicle.battery.remaining_energy()
            unload_position = vehicle.get_state().position
            unload_physics_time = vehicle.sim_time
        else:
            assert abs(vehicle.battery.remaining_energy() - unload_battery) < 1e-8
            assert vehicle.sim_time == unload_physics_time
            assert vehicle.get_state().position.distance_to(unload_position) < 1e-8
            assert vehicle.grounded
    else:
        unload_battery = unload_position = unload_physics_time = None
    if len(manager.completed_missions) == 1 and not second_added:
        assert order_a.status == 'delivered'
        assert vehicle.grounded
        assert manager.flights[0].ground_phase == GroundPhase.IDLE
        energy = vehicle.battery.remaining_energy()
        for _ in range(10):
            manager.update(0.1)
        assert abs(vehicle.battery.remaining_energy() - energy) < 1e-8, 'Idle base unexpectedly recharged'
        manager.add_order(DeliveryOrder('order-B', Vector3(0, 5, 2), payload_weight=0.5))
        second_added = True
    if len(manager.completed_missions) == 2:
        break
assert len(manager.completed_missions) == 2
assert all(m['status'] == 'complete' for m in manager.completed_missions)
assert [o.status for o in manager.orders] == ['delivered', 'delivered']
types = [e['type'] for e in manager.events]
assert types.count('loading_started') == types.count('loading_completed') == 2
assert types.count('unloading_started') == types.count('unloading_completed') == 2
assert 'recharging_started' not in types

# Recharge only after the threshold is crossed. No order is necessary.
vehicle.battery.set_percent(25.0)
manager.update(0.1)
assert manager.flights[0].ground_phase == GroundPhase.RECHARGING
before = vehicle.battery.remaining_energy()
manager.update(1.0)
assert vehicle.battery.remaining_energy() > before
assert vehicle.grounded
print('PASS: ground holds, timed loading/unloading, consecutive missions, conditional recharge')
