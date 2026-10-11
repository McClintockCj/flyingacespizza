"""Time-parametrized waypoint trajectory shared by execution and energy planning."""
from __future__ import annotations
import math
import numpy as np
from contracts import Route

class SmoothRouteTrajectory:
    """Cubic, zero-endpoint-velocity trajectory through Route waypoints.

    Simple and robust for the initial integration test; each segment stops at
    its waypoint. Future work can replace this with RotorPy's MinSnap planner.
    """

    def __init__(self, route: Route, default_speed: float, minimum_segment_time: float = 1.0, max_acceleration: float = 2.0) -> None:
        if not route.waypoints:
            raise ValueError("Route must contain at least one waypoint")
        if default_speed <= 0:
            raise ValueError("default_speed must be positive")
        self.points = [np.array([w.position.x, w.position.y, w.position.z], dtype=float) for w in route.waypoints]
        self.durations = []
        for i in range(len(self.points) - 1):
            distance = float(np.linalg.norm(self.points[i + 1] - self.points[i]))
            speed = route.waypoints[i + 1].speed or default_speed
            if speed <= 0:
                raise ValueError("Waypoint speed must be positive")
            # Cubic smoothstep has peak speed 1.5 * distance / duration.
            # self.durations.append(max(minimum_segment_time, 1.5 * distance / speed))
            duration = max(
                minimum_segment_time,
                1.5 * distance / speed,
                math.sqrt(6.0 * distance / max_acceleration),
            )

            self.durations.append(duration)
        self.total_time = sum(self.durations)

    def update(self, t: float) -> dict:
        t = max(0.0, float(t))
        if len(self.points) == 1 or t >= self.total_time:
            p = self.points[-1]
            v = np.zeros(3)
            a = np.zeros(3)
        else:
            i = 0
            while i < len(self.durations) - 1 and t > self.durations[i]:
                t -= self.durations[i]
                i += 1
            duration = self.durations[i]
            u = np.clip(t / duration, 0.0, 1.0)
            delta = self.points[i + 1] - self.points[i]
            p = self.points[i] + (3 * u**2 - 2 * u**3) * delta
            v = (6 * u - 6 * u**2) * delta / duration
            a = (6 - 12 * u) * delta / duration**2
        return {
            "x": p, "x_dot": v, "x_ddot": a,
            "x_dddot": np.zeros(3), "x_ddddot": np.zeros(3),
            "yaw": 0.0, "yaw_dot": 0.0, "yaw_ddot": 0.0,
        }

