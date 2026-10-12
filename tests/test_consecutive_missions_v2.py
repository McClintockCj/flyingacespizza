"""Regression: one RotorPy V2 vehicle executes two missions without resetting."""
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

order_a = DeliveryOrder('order-A', Vector3(5, 0, 2), payload_weight=0.5)
scenario = Scenario(bounds=(-10, -10, 10, 10), obstacles=[], wind=wind,
                    orders=[order_a], base=home)
environment = ScenarioManager(scenario)
manager = MissionManagerV2([vehicle], environment, MissionPlanner(environment), speed=1.0)

seen = 0
second_added = False
order_b = None

for i in range(1800):
    manager.update(0.1)
    for event in manager.events[seen:]:
        if event['type'] in ('mission_started', 'order_delivered', 'mission_completed', 'mission_failed'):
            print('EVENT', event)
    seen = len(manager.events)

    if not second_added and len(manager.completed_missions) == 1:
        first = manager.completed_missions[0]
        assert first['status'] == 'complete', first
        assert first['id'] == 'test-1-mission-1', first
        assert order_a.status == 'delivered', order_a.status
        assert vehicle.get_state().payload_weight == 0.0
        print(f'FIRST MISSION COMPLETE at t={manager.time:.2f}; position={vehicle.get_state().position}')
        # Inject a new order only after the first mission has returned to base.
        order_b = DeliveryOrder('order-B', Vector3(0, 5, 2), payload_weight=0.5)
        manager.add_order(order_b)
        second_added = True

    if i % 100 == 0:
        print(f't={manager.time:.1f} position={vehicle.get_state().position} '
              f'missions={len(manager.completed_missions)} '
              f'orders={[(o.id, o.status) for o in manager.orders]}')

    if len(manager.completed_missions) == 2:
        break

assert second_added, 'Second order was never added'
assert len(manager.completed_missions) == 2, 'Vehicle did not finish two missions'
first, second = manager.completed_missions
assert [first['id'], second['id']] == ['test-1-mission-1', 'test-1-mission-2']
assert first['status'] == second['status'] == 'complete', manager.completed_missions
assert first['delivered_orders'] == ['order-A'], first
assert second['delivered_orders'] == ['order-B'], second
assert order_a.status == order_b.status == 'delivered'
assert vehicle.get_state().payload_weight == 0.0
assert vehicle.get_state().fault is None, vehicle.get_state().fault
assert len([e for e in manager.events if e['type'] == 'mission_completed']) == 2
print('PASS: two consecutive missions with the same RotorPy vehicle')
print('Final:', vehicle.sample(vehicle.sim_time))
