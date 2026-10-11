# editor.py
import pygame

from flying_aces.core.contracts import DeliveryOrder, Obstacle, Vector3, Wind
from flying_aces.world.scenario import ScenarioManager


class ScenarioEditor:
    """
    Mouse-driven scenario construction, active while the UI is in EDIT mode.

    Tools:
      obstacle  - left-drag a rectangle to place an obstacle (height via +/-)
      delivery  - left-click to drop a delivery point
      base      - left-click to move the home/base position
      wind      - left-drag an arrow to set wind direction + speed
    Right-click always deletes the nearest obstacle.
    """

    TOOLS = ["obstacle", "delivery", "base", "wind"]

    def __init__(self, scenario_manager: ScenarioManager, camera):
        self.scenario_manager = scenario_manager
        self.camera = camera
        self.tool = "obstacle"
        self.default_obstacle_height = 5.0
        self._drag_start_world = None
        self._drag_start_screen = None
        # Track next order index based on scenario.orders
        self._next_delivery_n = len(scenario_manager.get_scenario().orders) + 1

    def cycle_tool(self):
        self.tool = self.TOOLS[(self.TOOLS.index(self.tool) + 1) % len(self.TOOLS)]

    def set_tool(self, name):
        if name in self.TOOLS:
            self.tool = name

    def adjust_obstacle_height(self, delta):
        self.default_obstacle_height = max(1.0, self.default_obstacle_height + delta)

    @property
    def drag_preview(self):
        """(start_screen, end_screen) for the in-progress drag, or None."""
        if self._drag_start_screen is None:
            return None
        return self._drag_start_screen, pygame.mouse.get_pos()

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self._drag_start_world = self.camera.screen_to_world(event.pos)
            self._drag_start_screen = event.pos

            if self.tool == "delivery":
                self._add_delivery(self._drag_start_world)
                self._drag_start_world = None
                self._drag_start_screen = None
            elif self.tool == "base":
                self._move_base(self._drag_start_world)
                self._drag_start_world = None
                self._drag_start_screen = None

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if self._drag_start_world is None:
                return
            end_world = self.camera.screen_to_world(event.pos)
            if self.tool == "obstacle":
                self._add_obstacle(self._drag_start_world, end_world)
            elif self.tool == "wind":
                self._set_wind(self._drag_start_world, end_world)
            self._drag_start_world = None
            self._drag_start_screen = None

        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
            self._remove_nearest_obstacle(self.camera.screen_to_world(event.pos))

    def _add_obstacle(self, start: Vector3, end: Vector3):
        width = max(abs(end.x - start.x), 1.0)
        depth = max(abs(end.y - start.y), 1.0)
        center = Vector3((start.x + end.x) / 2, (start.y + end.y) / 2, 0.0)
        self.scenario_manager.add_obstacle(
            Obstacle(center, width, depth, self.default_obstacle_height)
        )

    def _add_delivery(self, position: Vector3):
        order = DeliveryOrder(
            id=f"pizza-{self._next_delivery_n}",
            destination=Vector3(position.x, position.y, 2.0),
            priority=1,
            payload_weight=1.0,
        )
        self._next_delivery_n += 1
        # Adds order directly to scenario
        self.scenario_manager.get_scenario().orders.append(order)

    def _move_base(self, position: Vector3):
        scenario = self.scenario_manager.get_scenario()
        scenario.base = Vector3(position.x, position.y, scenario.base.z)

    def _set_wind(self, start: Vector3, end: Vector3):
        self.scenario_manager.set_wind(Wind(Vector3(end.x - start.x, end.y - start.y, 0.0)))

    def _remove_nearest_obstacle(self, point: Vector3):
        obstacles = self.scenario_manager.get_obstacles()
        if not obstacles:
            return
        nearest = min(obstacles, key=lambda o: o.position.distance_to(point))
        reach = max(nearest.width, nearest.depth) / 2 + 2.0
        if nearest.position.distance_to(point) <= reach:
            self.scenario_manager.remove_obstacle(nearest)
