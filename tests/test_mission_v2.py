"""Run from repository root: python test_mission_v2.py"""
from contracts import Vector3, Wind, Route, Waypoint
from fleet import build_vehicle, get_drone_config
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2
from mission_v2 import FlightMissionV2, LegAction, MissionExecutorV2, MissionLeg, MissionStatus

home = Vector3(0, 0, 2)
wind = Wind(Vector3(0, 2, 0))
config = get_drone_config("Light")
old = build_vehicle(home, wind, config, "test-1")
vehicle = RotorPyVehicleAdapterV2(
    rotorpy_vehicle=old.vehicle, rotorpy_environment=old.environment,
    rotorpy_controller=old.controller, config=config, vehicle_id="test-1",
    home_position=home, battery=old.state.battery, wind=wind,
)

def route(*points):
    return Route([Waypoint(Vector3(*point), speed=1.0) for point in points])

LANDING_Z = 0.15
mission = FlightMissionV2("mission-test-1", [
    MissionLeg(route((0, 0, 2), (5, 0, 3), (5, 0, LANDING_Z)), LegAction.DELIVER, "order-A"),
    MissionLeg(route((5, 0, LANDING_Z), (5, 0, 3), (5, 5, 3), (5, 5, LANDING_Z)), LegAction.DELIVER, "order-B"),
    MissionLeg(route((5, 5, LANDING_Z), (5, 5, 3), (0, 0, 2), (0, 0, LANDING_Z)), LegAction.RETURN),
])
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
assert vehicle.get_state().position.distance_to(Vector3(0, 0, LANDING_Z)) < 0.35
assert len([e for e in mission.events if e["type"] == "landed"]) == 3
