# pygame_ui.py
from __future__ import annotations

import math
from typing import TYPE_CHECKING
import pygame

from camera import Camera
from contracts import DeliveryOrder, DroneMission, MissionState, Vector3
from editor import ScenarioEditor
from map_module import MapData

if TYPE_CHECKING:
    from scenario import ScenarioManager
    from mission_manager import MissionManager
    from simulation import Simulation


BG = (240, 240, 240)
GRID = (215, 215, 215)
TEXT = (20, 20, 20)
PANEL_BG = (255, 255, 255)
PANEL_BORDER = (190, 190, 190)

MODE_COLORS = {"edit": (180, 60, 60), "sim": (40, 120, 60)}
STATUS_COLORS = {
    "pending": (220, 50, 50),      # Red: waiting in queue
    "dispatched": (230, 180, 0),   # Yellow: assigned to a drone
    "in_progress": (50, 150, 250), # Blue: drone currently delivering
    "delivered": (50, 200, 50),    # Green: successfully delivered
    "unfeasible": (140, 50, 140),  # Purple: no path found / obstacle blocked
}

DRONE_COLORS = [
    (0, 100, 200), (200, 60, 60), (60, 150, 60), (160, 100, 200), (200, 140, 0),
]


class PygameUI:

    def __init__(self, scenario_manager: ScenarioManager, mission_manager: MissionManager, simulation: Simulation):
        pygame.init()

        # self.screen_size = (1150, 760)
        # self.screen = pygame.display.set_mode(self.screen_size)
        # pygame.display.set_caption("Pizza Drone Simulation")
        # display_info = pygame.display.Info()
        self.screen = pygame.display.set_mode(
            (1150, 760),
            pygame.RESIZABLE,
        )

        self.screen_size = self.screen.get_size()
        pygame.display.set_caption("Flying Ace's Pizza Delivery")

        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)
        self.small_font = pygame.font.SysFont(None, 18)

        self.scenario_manager = scenario_manager
        self.mission_manager = mission_manager
        self.simulation = simulation
        # self.spawner: OrderSpawner = OrderSpawner(scenario=self.scenario_manager.get_scenario(),
        #                             spawn_interval=10.0,
        #                             max_active_orders=10,
        #                             landing_clearance=2.0,
        #                             search_radius=100.0,
        #                             search_step=10.0,)
        self.spawner = simulation.order_spawner

        self.auto_dispatch = True  # Auto-Dispatch enabled by default

        # self.map = MapData("maps/richmond_small_3.json")
        self.map = simulation.map

        self.camera = Camera()
        # self.camera.fit(scenario_manager.get_scenario().bounds, self.screen_size)
        # self.editor = ScenarioEditor(scenario_manager, self.camera)
        self.camera.fit(self.map.get_bounds(), self.screen_size)

        self.mode = "sim"  # "sim" or "edit"
        self.show_help = False
        self.running = True
        self.messages: list[str] = []

    def run(self):
        try:
            while self.running:
                dt = self.clock.tick(60) / 1000.0
                self.handle_events()
                self.update(dt)
                self.render()
        finally:
            self.simulation.stop()
            pygame.quit()

    def log(self, text: str):
        self.messages.append(text)
        self.messages = self.messages[-5:]

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return False
            
            # Camera gets first opportunity to consume pan/zoom events.
            if self.camera.handle_event(event):
                continue

            # --- Left-click to place delivery on map ---
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mouse_pos = event.pos
                # Don't place delivery if clicking inside the top toolbar
                if mouse_pos[1] > 34 and self.mode == "sim":
                    world_pos = self.camera.screen_to_world(mouse_pos)
                    scenario = self.scenario_manager.get_scenario()
                    bx_min, by_min, bx_max, by_max = scenario.bounds

                    if bx_min <= world_pos.x <= bx_max and by_min <= world_pos.y <= by_max:
                        new_order = DeliveryOrder(
                            id=f"p{len(scenario.orders) + 1}",
                            destination=Vector3(world_pos.x, world_pos.y, 2.0),
                            payload_weight=1.5,
                            priority=1,
                        )
                        self.simulation.add_order(new_order)
                        self.log(f"Placed order at ({world_pos.x:.1f}, {world_pos.y:.1f})")

            # --- Keybinds ---
            elif event.type == pygame.KEYDOWN:
                self.handle_key(event)

        return True

    def handle_key(self, event):
        key = event.key
        mods = event.mod

        if key == pygame.K_TAB:
            # self.mode = "edit" if self.mode == "sim" else "sim"
            # self.log(f"Switched to {self.mode.upper()} mode")
            return

        if key == pygame.K_h:
            self.show_help = not self.show_help
            return

        # Toggle Auto-Dispatch with 'A'
        if key == pygame.K_a:
            self.simulation.auto_dispatch = not self.auto_dispatch
            self.auto_dispatch = self.simulation.auto_dispatch
            self.log(f"Auto-Dispatch: {'ON' if self.auto_dispatch else 'OFF'}")
            return

        # Adjust spawn interval with UP / DOWN arrows
        if key == pygame.K_UP:
            self.spawner.interval = max(0.5, self.spawner.interval - 1.0)
            self.log(f"Spawn Interval: {self.spawner.interval:.1f}s")
            return
        elif key == pygame.K_DOWN:
            self.spawner.interval += 1.0
            self.log(f"Spawn Interval: {self.spawner.interval:.1f}s")
            return

        if key == pygame.K_s and (mods & pygame.KMOD_CTRL):
            self.scenario_manager.save("scenario.json")
            self.log("Scenario saved to scenario.json")
            return

        if key == pygame.K_l and (mods & pygame.KMOD_CTRL):
            try:
                loaded = self.scenario_manager.load("scenario.json")
                self.scenario_manager.set_scenario(loaded.get_scenario())
                self.log("Scenario loaded from scenario.json")
            except (OSError, KeyError, ValueError) as exc:
                self.log(f"Load failed: {exc}")
            return

        if self.mode == "edit":
            self.handle_edit_key(key)
        else:
            self.handle_sim_key(key)

    def handle_edit_key(self, key):
        tool_keys = {
            pygame.K_1: "obstacle",
            pygame.K_2: "delivery",
            pygame.K_3: "base",
            pygame.K_4: "wind",
        }
        if key in tool_keys:
            self.editor.set_tool(tool_keys[key])
        elif key == pygame.K_c:
            self.editor.cycle_tool()
        elif key in (pygame.K_EQUALS, pygame.K_PLUS):
            self.editor.adjust_obstacle_height(1.0)
        elif key == pygame.K_MINUS:
            self.editor.adjust_obstacle_height(-1.0)

    def handle_sim_key(self, key):
        if key == pygame.K_SPACE:
            if self.simulation.running:
                self.simulation.stop()
            else:
                self.simulation.start()
        elif key == pygame.K_p:
            route = self.mission_manager.plan_next_mission()
            self.log("Route planned" if route else "No feasible route")
        elif key == pygame.K_s:
            ok = self.mission_manager.start_next_mission()
            self.log("Mission started" if ok else "Nothing to start")
        elif key == pygame.K_r:
            self.mission_manager.return_to_base()
            self.log("Returning to base")
        elif key == pygame.K_ESCAPE:
            self.mission_manager.abort_mission()
            self.log("Mission aborted")
        elif key == pygame.K_n:
            self.simulation.reset()
            self.mission_manager.reset()
            self.log("Simulation reset")

    def update(self, dt: float):
        self.simulation.advance(dt)
        # self.spawner.update(dt)

        # if self.auto_dispatch:
        #     self.mission_manager.update(dt)

    def render(self):
        # self.screen.fill(BG)
        # self.render_grid()
        # self.render_obstacles()
        self.map.render(self.screen, self.camera)

        self.render_deliveries()
        self.render_base()
        self.render_route()
        self.render_drone()
        self.render_wind()
        
        # if self.mode == "edit":
            # self.render_drag_preview()
        self.render_toolbar()
        self.render_hud()
        if self.show_help:
            self.render_help()
        pygame.display.flip()

    def render_grid(self, spacing=5):
        x0, y0, x1, y1 = self.scenario_manager.get_scenario().bounds
        x = (int(x0) // spacing) * spacing
        while x <= x1:
            a = self.camera.world_to_screen(Vector3(x, y0))
            b = self.camera.world_to_screen(Vector3(x, y1))
            pygame.draw.line(self.screen, GRID, a, b, 1)
            x += spacing
        y = (int(y0) // spacing) * spacing
        while y <= y1:
            a = self.camera.world_to_screen(Vector3(x0, y))
            b = self.camera.world_to_screen(Vector3(x1, y))
            pygame.draw.line(self.screen, GRID, a, b, 1)
            y += spacing

    def render_obstacles(self):
        for obstacle in self.scenario_manager.get_obstacles():
            top_left = self.camera.world_to_screen(
                Vector3(
                    obstacle.position.x - obstacle.width / 2,
                    obstacle.position.y - obstacle.depth / 2,
                )
            )
            w = max(int(obstacle.width * self.camera.scale), 1)
            h = max(int(obstacle.depth * self.camera.scale), 1)

            shade = max(60, 200 - int(obstacle.height * 12))
            color = (shade, shade, shade)
            pygame.draw.rect(self.screen, color, pygame.Rect(*top_left, w, h))
            pygame.draw.rect(self.screen, (60, 60, 60), pygame.Rect(*top_left, w, h), 1)

            label = self.small_font.render(f"h={obstacle.height:.0f}m", True, (255, 255, 255))
            self.screen.blit(label, (top_left[0] + 4, top_left[1] + 4))

    def render_deliveries(self):
        for order in self.scenario_manager.get_scenario().orders:
            x, y = self.camera.world_to_screen(order.destination)
            color = STATUS_COLORS.get(order.status, (150, 150, 150))
            pygame.draw.circle(self.screen, color, (x, y), 7)
            pygame.draw.circle(self.screen, (0, 0, 0), (x, y), 7, 1)
            label = self.small_font.render(f"{order.id} [{order.status}]", True, TEXT)
            self.screen.blit(label, (x + 8, y - 8))

    def render_base(self):
        base = self.scenario_manager.get_scenario().base
        x, y = self.camera.world_to_screen(base)
        pygame.draw.circle(self.screen, (0, 100, 0), (x, y), 9)
        pygame.draw.circle(self.screen, (0, 0, 0), (x, y), 9, 1)

    def render_drone(self):
        for i, state in enumerate(self.simulation.get_snapshot().vehicles):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            x, y = self.camera.world_to_screen(state.position)

            speed = state.velocity.distance_to(Vector3(0, 0, 0))
            if speed > 0.1:
                heading = math.atan2(state.velocity.y, state.velocity.x)
                tip = (x + int(14 * math.cos(heading)), y + int(14 * math.sin(heading)))
                pygame.draw.line(self.screen, color, (x, y), tip, 2)

            pygame.draw.circle(self.screen, color, (x, y), 8)
            pygame.draw.circle(self.screen, (0, 0, 0), (x, y), 8, 1)

    def render_wind(self):
        wind = self.scenario_manager.get_wind()
        cx, cy = self.screen_size[0] - 70, 70
        pygame.draw.circle(self.screen, PANEL_BG, (cx, cy), 45)
        pygame.draw.circle(self.screen, PANEL_BORDER, (cx, cy), 45, 1)

        if wind.speed > 1e-6:
            angle = math.atan2(wind.velocity.y, wind.velocity.x)
            length = min(35, 10 + wind.speed * 4)
            tip = (cx + length * math.cos(angle), cy + length * math.sin(angle))
            pygame.draw.line(self.screen, (30, 100, 200), (cx, cy), tip, 3)

        label = self.small_font.render(f"wind {wind.speed:.1f} m/s", True, TEXT)
        self.screen.blit(label, (cx - label.get_width() // 2, cy + 50))

    def render_drag_preview(self):
        preview = self.editor.drag_preview
        if preview is None:
            return
        start, end = preview
        if self.editor.tool == "obstacle":
            rect = pygame.Rect(
                min(start[0], end[0]), min(start[1], end[1]),
                abs(end[0] - start[0]), abs(end[1] - start[1]),
            )
            pygame.draw.rect(self.screen, (180, 60, 60), rect, 2)
        elif self.editor.tool == "wind":
            pygame.draw.line(self.screen, (30, 100, 200), start, end, 2)

    def render_route(self):
        for i, mission in enumerate(self.mission_manager.missions):
            if (
                mission.active_route 
                and mission.state in (MissionState.EXECUTING, MissionState.READY, MissionState.RETURNING)
            ):
                points = [
                    self.camera.world_to_screen(wp.position)
                    for wp in mission.active_route.waypoints
                ]
                color = DRONE_COLORS[i % len(DRONE_COLORS)]
                if len(points) > 1:
                    pygame.draw.lines(self.screen, color, False, points, 2)

    def render_toolbar(self):
        rect = pygame.Rect(0, 0, self.screen_size[0], 34)
        pygame.draw.rect(self.screen, PANEL_BG, rect)
        pygame.draw.line(self.screen, PANEL_BORDER, (0, 34), (self.screen_size[0], 34))

        mode_label = self.font.render(f"[{self.mode.upper()}]", True, MODE_COLORS[self.mode])
        self.screen.blit(mode_label, (10, 6))

        if self.mode == "edit":
            pass
            # tools_text = "  ".join(
            #     f"({i + 1}){t}" if t != self.editor.tool else f"[{i + 1}:{t.upper()}]"
            #     for i, t in enumerate(self.editor.TOOLS)
            # )
            # text = f"{tools_text}   height={self.editor.default_obstacle_height:.0f}m (+/-)"
        else:
            auto_str = "AUTO" if self.auto_dispatch else "MANUAL"
            text = (
                f"Dispatch: [{auto_str}] (A) | Spawn: {self.spawner.interval:.1f}s (UP/DN) | "
                f"[SPACE] pause  [P] plan  [S] start  [R] RTB  [N] reset"
            )

        self.screen.blit(self.font.render(text, True, TEXT), (110, 8))

        help_hint = self.small_font.render(
            "TAB: mode   H: help   Ctrl+S/L: save/load", True, (120, 120, 120)
        )
        self.screen.blit(help_hint, (self.screen_size[0] - help_hint.get_width() - 10, 10))

    def render_hud(self):
        vehicle_states = self.simulation.get_snapshot().vehicles
        missions: list[DroneMission] = self.mission_manager.missions

        row_h = 20
        height = 34 + row_h * len(missions)
        panel = pygame.Rect(10, self.screen_size[1] - height - 10, 420, height)
        pygame.draw.rect(self.screen, PANEL_BG, panel)
        pygame.draw.rect(self.screen, PANEL_BORDER, panel, 1)

        pending_count = len([o for o in self.scenario_manager.get_scenario().orders if o.status == "pending"])
        header = f"Orders Queue: {pending_count} pending | Click map to add order"
        self.screen.blit(self.small_font.render(header, True, TEXT), (18, panel.y + 8))

        for i, (mission, state) in enumerate(zip(missions, vehicle_states)):
            color = DRONE_COLORS[i % len(DRONE_COLORS)]
            batch = f"{len(mission.active_orders)} deliver(ies)" if mission.active_orders else "-"
            text = (
                f"{mission.vehicle.id}: {mission.state.value:<10} "
                f"batt {state.battery.remaining_percent():5.1f}%   batch {batch}"
            )
            y = panel.y + 30 + i * row_h
            pygame.draw.circle(self.screen, color, (24, y + 8), 5)
            self.screen.blit(self.small_font.render(text, True, TEXT), (36, y))

        for i, msg in enumerate(reversed(self.messages)):
            surface = self.small_font.render(msg, True, (90, 90, 90))
            self.screen.blit(surface, (18, panel.y - 20 - i * 18))

    def render_help(self):
        lines = [
            "-- EDIT mode --",
            "1/2/3/4 or C: select tool (obstacle/delivery/base/wind)",
            "Left-drag: place obstacle / draw wind vector",
            "Left-click: place delivery point / move base",
            "Right-click: delete nearest obstacle",
            "+/-: change new-obstacle height",
            "",
            "-- SIM mode --",
            "Left-click map: drop new delivery order",
            "A: toggle Auto-Dispatch (ON/OFF)",
            "UP / DOWN arrows: speed up / slow down order spawn rate",
            "SPACE: play/pause   P: plan route   S: start mission",
            "R: return to base   ESC: abort   N: reset simulation",
            "",
            "-- Always --",
            "TAB: toggle edit/sim   Middle-drag: pan   Wheel: zoom",
            "Ctrl+S / Ctrl+L: save / load scenario.json   H: toggle help",
        ]
        panel = pygame.Rect(300, 140, 550, 30 + 22 * len(lines))
        pygame.draw.rect(self.screen, PANEL_BG, panel)
        pygame.draw.rect(self.screen, PANEL_BORDER, panel, 1)
        for i, line in enumerate(lines):
            font = self.font if line.startswith("--") else self.small_font
            self.screen.blit(font.render(line, True, TEXT), (panel.x + 15, panel.y + 12 + i * 22))