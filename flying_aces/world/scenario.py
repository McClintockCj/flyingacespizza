# scenario.py
import json
from dataclasses import asdict

from flying_aces.core.contracts import *


class ScenarioManager(EnvironmentInterface):

    def __init__(self, scenario: Scenario):
        self.scenario = scenario

    def get_scenario(self) -> Scenario:
        return self.scenario

    def set_scenario(self, scenario: Scenario) -> None:
        self.scenario = scenario

    def get_obstacles(self) -> list[Obstacle]:
        return self.scenario.obstacles

    def get_wind(self) -> Wind:
        return self.scenario.wind

    def add_obstacle(self, obstacle: Obstacle):
        self.scenario.obstacles.append(obstacle)

    def remove_obstacle(self, obstacle: Obstacle):
        self.scenario.obstacles.remove(obstacle)

    def set_wind(self, wind: Wind):
        self.scenario.wind = wind

    def add_delivery_point(self, delivery: DeliveryOrder):
        self.scenario.orders.append(delivery)

    def remove_delivery_point(self, delivery: DeliveryOrder):
        self.scenario.orders.remove(delivery)

    # ------------------------------------------------------------------
    # Persistence: lets students save/share hand-built scenarios as JSON.
    # ------------------------------------------------------------------

    def to_json(self) -> str:
        return json.dumps(asdict(self.scenario), indent=2)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            f.write(self.to_json())

    @staticmethod
    def from_json(data: str) -> "ScenarioManager":
        raw = json.loads(data)
        
        # Handle wind if present, or default to zero vector
        wind_data = raw.get("wind", {}).get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
        
        scenario = Scenario(
            bounds=tuple(raw["bounds"]),
            obstacles=[
                Obstacle(Vector3(**o["position"]), o["width"], o["depth"], o["height"])
                for o in raw.get("obstacles", [])
            ],
            wind=Wind(Vector3(**wind_data)),
            orders=[
                DeliveryOrder(
                    id=d["id"],
                    destination=Vector3(**(d.get("destination") or d.get("position"))),
                    priority=d.get("priority", 0),
                    payload_weight=d.get("payload_weight", 1.0),
                    status=d.get("status", "pending")
                )
                for d in raw.get("orders", [])
            ],
            base=Vector3(**raw["base"]),
            drone_start_battery=raw.get("drone_start_battery", 100.0),
        )
        return ScenarioManager(scenario)

    @staticmethod
    def load(path: str) -> "ScenarioManager":
        with open(path) as f:
            return ScenarioManager.from_json(f.read())