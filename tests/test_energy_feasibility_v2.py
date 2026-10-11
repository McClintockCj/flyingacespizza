"""Run in project venv: python test_energy_feasibility_v2.py"""
from contracts import DeliveryOrder, Scenario, Vector3, Wind
from fleet import build_vehicle, get_drone_config
from mission_manager_v2 import GroundOperationsConfig, GroundPhase, MissionManagerV2
from mission_planner import MissionPlanner
from rotorpy_adapter_v2 import RotorPyVehicleAdapterV2
from scenario import ScenarioManager
from energy_feasibility_v2 import estimate_mission_energy, EnergyPolicyV2

home = Vector3(0, 0, 2)
wind = Wind(Vector3(2, 0, 0))
config = get_drone_config('Light')
legacy = build_vehicle(home, wind, config, 'energy-1')
vehicle = RotorPyVehicleAdapterV2(legacy.vehicle, legacy.environment,
                                  legacy.controller, config, 'energy-1',
                                  home, legacy.state.battery, wind)
order = DeliveryOrder('energy-order', Vector3(5, 0, 2), payload_weight=0.5)
scenario = Scenario(bounds=(-10,-10,10,10), obstacles=[], wind=wind,
                    orders=[order], base=home)
environment = ScenarioManager(scenario)
manager = MissionManagerV2([vehicle], environment, MissionPlanner(environment),
                           speed=1.0, ground_config=GroundOperationsConfig(
                               loading_time=1, unloading_time=1),
                           energy_policy=EnergyPolicyV2(reserve_percent=35))
mission = manager.leg_planner.plan_mission('preview', home, [order], home)
budget = estimate_mission_energy(vehicle, mission, [order], wind, manager.energy_policy)
assert budget.estimated_wh > 0 and budget.budgeted_wh >= budget.estimated_wh
assert budget.reserve_wh == 35.0

# Below feasibility budget but above the normal recharge threshold: charge.
initial_wh = budget.budgeted_wh + budget.reserve_wh - 0.05
assert initial_wh > 0
vehicle.battery.remaining_wh = initial_wh
assert vehicle.battery.remaining_percent() >= 30, 'Choose a longer route if this assertion fails'
manager.update(0.1)
assert manager.flights[0].ground_phase == GroundPhase.RECHARGING
assert order.status == 'pending'
assert vehicle.grounded

# Battery charges until release threshold AND feasible; then loads.
for i in range(20000):
    manager.update(0.1)
    if order.status != 'pending':
        break
assert order.status == 'dispatched'
assert manager.flights[0].ground_phase == GroundPhase.LOADING

# No recharging during flight or unloading, and order only completes after unloading.
for i in range(1500):
    manager.update(0.1)
    if len(manager.completed_missions) == 1:
        break
assert len(manager.completed_missions) == 1
assert order.status == 'delivered'
assert manager.completed_missions[0]['status'] == 'complete'
assert vehicle.grounded
assert 'recharging_started' in [e['type'] for e in manager.events]
assert 'recharging_completed' in [e['type'] for e in manager.events]
print('PASS: V2 energy budget, feasibility-triggered recharge, and dispatch after recharge')
