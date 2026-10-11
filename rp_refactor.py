
from contracts import Vector3, Wind, Route, Waypoint
from fleet import build_vehicle, get_drone_config
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2

home = Vector3(0, 0, 2)
wind = Wind(Vector3(0, 2, 0))
config = get_drone_config("Light")

old = build_vehicle(home, wind, config, "test-1")

drone = RotorPyVehicleAdapterV2(
    rotorpy_vehicle=old.vehicle,
    rotorpy_environment=old.environment,
    rotorpy_controller=old.controller,
    config=config,
    vehicle_id="test-1",
    home_position=home,
    battery=old.state.battery,
    wind=wind,
)

print("Configured wind:", drone.wind.velocity)
print("RotorPy wind:", drone.environment.wind_profile.update(
    0.0, drone.rotorpy_state["x"]
))

route = Route([
    Waypoint(Vector3(0, 0, 2)),
    Waypoint(Vector3(5, 0, 3), speed=1.0),
    Waypoint(Vector3(5, 5, 3), speed=1.0),
])

# route = Route([
#     Waypoint(Vector3(0, 0, 2)),
#     Waypoint(Vector3(5, 0, 3)),
#     Waypoint(Vector3(5, 5, 3)),
# ])

drone.upload_route(route)
drone.start_route()

for i in range(300):
    drone.advance(0.1)

    if i % 10 == 0:
        print(i / 10, drone.get_state().position, drone.route_status)

    if drone.route_complete or drone.get_state().fault:
        break

print("Final:", drone.sample(drone.sim_time))
