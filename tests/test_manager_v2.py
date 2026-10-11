"""Standalone V2 manager integration test using the existing fleet and scenario interfaces."""
from contracts import DeliveryOrder, Scenario, Vector3, Wind
from fleet import build_vehicle, get_drone_config
from mission_manager_v2 import MissionManagerV2
from mission_planner import MissionPlanner
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2
from scenario import ScenarioManager

home = Vector3(0, 0, 2)
wind = Wind(Vector3(0, 2, 0))
config = get_drone_config('Light')
old = build_vehicle(home, wind, config, 'test-1')
vehicle = RotorPyVehicleAdapterV2(
    rotorpy_vehicle=old.vehicle, rotorpy_environment=old.environment,
    rotorpy_controller=old.controller, config=config, vehicle_id='test-1',
    home_position=home, battery=old.state.battery, wind=wind)
orders = [
    DeliveryOrder('order-A', Vector3(5, 0, 2), payload_weight=0.5),
    DeliveryOrder('order-B', Vector3(5, 5, 2), payload_weight=0.5),
]
scenario = Scenario(bounds=(-10, -10, 10, 10), obstacles=[], wind=wind,
                    orders=orders, base=home)
environment = ScenarioManager(scenario)
manager = MissionManagerV2([vehicle], environment, MissionPlanner(environment), speed=1.0)
seen = 0
for i in range(1200):
    manager.update(0.1)
    for event in manager.events[seen:]:
        print('EVENT', event)
    seen = len(manager.events)
    if i % 50 == 0:
        print(f't={manager.time:.1f}', vehicle.get_state().position,
              [(o.id, o.status) for o in orders])
    if manager.completed_missions:
        break
assert len(manager.completed_missions) == 1, 'Mission did not finish'
assert manager.completed_missions[0]['status'] == 'complete', manager.completed_missions[0]
assert all(order.status == 'delivered' for order in orders)
assert vehicle.get_state().payload_weight == 0.0
print('PASS:', manager.completed_missions[0]['id'])
