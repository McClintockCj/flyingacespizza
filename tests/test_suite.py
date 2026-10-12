import os
import unittest
import tempfile

from contracts import (
    Vector3,
    Waypoint,
    Route,
    Obstacle,
    Wind,
    DeliveryOrder,
    Scenario,
    VehicleState,
    DroneState,
    MissionState,
    VehicleInterface,
)
from scenario import ScenarioManager
from mission_planner import MissionPlanner
from mission_manager import MissionManager


# ----------------------------------------------------------------------
# Mock Classes for Unit Testing
# ----------------------------------------------------------------------

class MockVehicle(VehicleInterface):
    """Predictable vehicle mockup for mission lifecycle testing."""

    def __init__(self, position: Vector3 = None, payload_capacity: float = 5.0):
        self.state = VehicleState(
            position=position or Vector3(0, 0, 0),
            battery_percent=100.0,
            state=DroneState.LANDED,
            payload_capacity=payload_capacity,
        )
        self.uploaded_route = None
        self.mission_started = False
        self.aborted = False

    def get_state(self) -> VehicleState:
        return self.state

    def upload_route(self, route: Route) -> None:
        self.uploaded_route = route

    def start_mission(self) -> None:
        self.mission_started = True
        self.state.state = DroneState.FLYING

    def pause_mission(self) -> None:
        pass

    def abort_mission(self) -> None:
        self.aborted = True
        self.state.state = DroneState.EMERGENCY

    def return_to_base(self) -> None:
        self.state.state = DroneState.RETURNING

    def advance(self, dt: float) -> None:
        pass

    def reset(self) -> None:
        self.state.state = DroneState.LANDED
        self.uploaded_route = None
        self.mission_started = False
        self.aborted = False


# ----------------------------------------------------------------------
# Test Suite
# ----------------------------------------------------------------------

class TestContractsAndMath(unittest.TestCase):
    def test_vector3_distance(self):
        v1 = Vector3(0, 0, 0)
        v2 = Vector3(3, 4, 0)
        self.assertAlmostEqual(v1.distance_to(v2), 5.0)

    def test_route_distance(self):
        route = Route([
            Waypoint(Vector3(0, 0, 0)),
            Waypoint(Vector3(10, 0, 0)),
            Waypoint(Vector3(10, 10, 0)),
        ])
        self.assertAlmostEqual(route.distance, 20.0)

    def test_obstacle_contains(self):
        obs = Obstacle(position=Vector3(10, 10, 0), width=4, depth=4, height=10)
        self.assertTrue(obs.contains(Vector3(10, 10, 5)))
        self.assertFalse(obs.contains(Vector3(20, 20, 5)))


class TestMissionPlanner(unittest.TestCase):
    def setUp(self):
        self.scenario = Scenario(
            bounds=(0, 0, 50, 50),
            base=Vector3(0, 0, 0),
            obstacles=[
                Obstacle(position=Vector3(10, 10, 0), width=4, depth=4, height=10)
            ],
            wind=Wind(Vector3(0, 0, 0)),
        )
        self.env = ScenarioManager(self.scenario)
        self.planner = MissionPlanner(self.env)

    def test_direct_clear_route(self):
        start = Vector3(0, 0, 2)
        dest = Vector3(0, 10, 2)
        route = self.planner.plan_route(start, dest)
        self.assertEqual(len(route.waypoints), 2)
        self.assertTrue(self.planner.route_is_clear(route))

    def test_obstacle_avoidance(self):
        start = Vector3(0, 10, 2)
        dest = Vector3(20, 10, 2)
        route = self.planner.plan_route(start, dest)
        self.assertTrue(self.planner.route_is_clear(route))

    def test_delivery_batching_capacity_limit(self):
        vehicle = VehicleState(position=Vector3(0, 0, 0), payload_capacity=3.0)
        orders = [
            DeliveryOrder(id="o1", destination=Vector3(5, 0, 0), priority=1, payload_weight=2.0),
            DeliveryOrder(id="o2", destination=Vector3(10, 0, 0), priority=2, payload_weight=2.0),
        ]
        batch, route = self.planner.plan_delivery_batch(vehicle, orders, base=Vector3(0, 0, 0))
        self.assertEqual(len(batch), 1)
        self.assertIsNotNone(route)


class TestMissionManager(unittest.TestCase):
    def setUp(self):
        self.scenario = Scenario(bounds=(0, 0, 50, 50), base=Vector3(0, 0, 0))
        self.env = ScenarioManager(self.scenario)
        self.planner = MissionPlanner(self.env)
        self.vehicle = MockVehicle()
        self.manager = MissionManager(self.vehicle, self.env, self.planner)

    def test_plan_and_start_mission(self):
        order = DeliveryOrder(id="order-1", destination=Vector3(10, 10, 2), priority=1, payload_weight=1.0)
        self.manager.add_order(order)

        route = self.manager.plan_next_mission()
        self.assertIsNotNone(route)
        self.assertEqual(self.manager.state, MissionState.READY)

        started = self.manager.start_next_mission()
        self.assertTrue(started)
        self.assertEqual(self.manager.state, MissionState.EXECUTING)

    def test_editor_to_manager_order_integration(self):
        """Verifies orders added to scenario are immediately visible to MissionManager."""
        new_order = DeliveryOrder(id="editor-pizza-1", destination=Vector3(15, 15, 2))
        self.env.get_scenario().orders.append(new_order)

        self.assertEqual(len(self.manager.pending_orders), 1)
        self.assertEqual(self.manager.pending_orders[0].id, "editor-pizza-1")


class TestScenarioSerialization(unittest.TestCase):
    def test_save_and_load_scenario(self):
        scenario = Scenario(
            bounds=(0, 0, 100, 100),
            base=Vector3(2, 2, 0),
            obstacles=[Obstacle(Vector3(10, 10, 0), 2, 2, 5)],
            orders=[DeliveryOrder("p1", Vector3(20, 20, 2), priority=1, payload_weight=1.5)],
        )
        env = ScenarioManager(scenario)

        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as tmp:
            tmp_path = tmp.name

        try:
            env.save(tmp_path)
            loaded_env = ScenarioManager.load(tmp_path)
            loaded_sc = loaded_env.get_scenario()

            self.assertEqual(loaded_sc.base, Vector3(2, 2, 0))
            self.assertEqual(len(loaded_sc.obstacles), 1)
            self.assertEqual(len(loaded_sc.orders), 1)
            self.assertEqual(loaded_sc.orders[0].id, "p1")
            self.assertEqual(loaded_sc.orders[0].payload_weight, 1.5)
        finally:
            if os.path.exists(tmp_path):
                os.remove(tmp_path)


if __name__ == "__main__":
    unittest.main()