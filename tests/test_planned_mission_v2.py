"""Run from repo root: python test_planned_mission_v2.py"""
from contracts import DeliveryOrder, Obstacle, Vector3, Wind
from fleet import build_vehicle, get_drone_config
from mission_planner import MissionPlanner
from mission_planner_v2 import MissionPlannerV2
from mission_v2 import MissionExecutorV2, MissionStatus
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2


class TestEnvironment:
    def __init__(self, obstacles=()):
        self.obstacles = list(obstacles)

    def get_obstacles(self):
        return self.obstacles

    def get_wind(self):
        return Wind(Vector3(0, 2, 0))


home = Vector3(0, 0, 2)
wind = Wind(Vector3(0, 2, 0))
config = get_drone_config("Light")
old = build_vehicle(home, wind, config, "test-1")
vehicle = RotorPyVehicleAdapterV2(
    rotorpy_vehicle=old.vehicle, rotorpy_environment=old.environment,
    rotorpy_controller=old.controller, config=config, vehicle_id="test-1",
    home_position=home, battery=old.state.battery, wind=wind,
)

# A small building directly beneath the delivery point makes descent unsafe.
blocking = Obstacle(polygons=(((4.5, -0.5), (5.5, -0.5), (5.5, 0.5), (4.5, 0.5)),), height=2.0)
unsafe_planner = MissionPlannerV2(MissionPlanner(TestEnvironment([blocking])))
try:
    unsafe_planner.plan_landing_leg(home, Vector3(5, 0, 0))
except RuntimeError as exc:
    print("EXPECTED UNSAFE LANDING:", exc)
else:
    raise AssertionError("Planner accepted a landing inside an obstacle")

planner = MissionPlannerV2(MissionPlanner(TestEnvironment()))
orders = [
    DeliveryOrder("order-A", Vector3(5, 0, 0)),
    DeliveryOrder("order-B", Vector3(5, 5, 0)),
]
mission = planner.plan_mission("mission-test-1", home, orders, Vector3(0, 0, 0))
for i, leg in enumerate(mission.legs):
    print("PLANNED LEG", i + 1, leg.action.value, [vars(w.position) for w in leg.route.waypoints])
executor = MissionExecutorV2(vehicle, mission)
executor.start()
last_event = 0
for i in range(1200):
    executor.advance(0.1)
    if i % 20 == 0:
        print(f"t={vehicle.sim_time:.1f} leg={mission.current_leg_index + 1} pos={vehicle.get_state().position} status={mission.status.value}")
    for event in mission.events[last_event:]:
        print("EVENT", event)
    last_event = len(mission.events)
    if mission.status in (MissionStatus.COMPLETE, MissionStatus.FAILED):
        break
print("Final:", vehicle.sample(vehicle.sim_time))
print("Mission:", mission.status.value, "delivered:", mission.delivered_orders, "fault:", mission.fault)
assert mission.status == MissionStatus.COMPLETE, mission.fault
assert mission.delivered_orders == ["order-A", "order-B"]
assert vehicle.get_state().position.distance_to(Vector3(0, 0, 0.15)) < 0.35
assert len([e for e in mission.events if e["type"] == "landed"]) == 3
