from __future__ import annotations

import math, random

from contracts import DeliveryOrder, Scenario, Vector3


class OrderSpawner:
    """
    Temporary interval-based order spawning strategy.

    This is intentionally kept as a concrete strategy for now. The Simulation
    owns the spawner, while the spawner owns only the mechanics of generating
    orders. More sophisticated spawning strategies can replace this later.
    """

    def __init__(
        self,
        interval_seconds: float = 30.0,
        max_active_orders: int = 5,
        rng: random.Random | None = None,
        max_delivery_radius: float | None = None,
    ):
        self.interval = interval_seconds
        self.max_active_orders = max_active_orders
        self.timer = 0.0
        self.counter = 100
        self.rng = rng or random.Random()
        self.max_delivery_radius = max_delivery_radius

    def _is_valid_location(
        self,
        pos: Vector3,
        scenario: Scenario,
        buffer: float = 2.0,
    ) -> bool:
        for obstacle in scenario.obstacles:
            if hasattr(obstacle, "bounds"):
                x0, y0, x1, y1 = obstacle.bounds
                if (
                    x0 - buffer <= pos.x <= x1 + buffer
                    and y0 - buffer <= pos.y <= y1 + buffer
                ):
                    return False
                continue

            if hasattr(obstacle, "position"):
                if (
                    obstacle.position.x - obstacle.width / 2 - buffer <= pos.x <= obstacle.position.x + obstacle.width / 2 + buffer
                    and obstacle.position.y - obstacle.depth / 2 - buffer <= pos.y <= obstacle.position.y + obstacle.depth / 2 + buffer
                ):
                    return False

        return True

    def update(self, dt: float, scenario: Scenario, time: float = 0.0) -> DeliveryOrder | None:
        self.timer += dt

        if self.timer < self.interval:
            return None

        active_orders = [
            order
            for order in scenario.orders
            if order.status in ("pending", "dispatched", "in_progress")
        ]

        if len(active_orders) >= self.max_active_orders:
            return None

        self.timer = 0.0

        x0, y0, x1, y1 = scenario.bounds
        margin = 3.0

        for _ in range(20):
            if self.max_delivery_radius is not None:
                angle = self.rng.uniform(0.0, 2.0 * math.pi)
                radius = self.max_delivery_radius * math.sqrt(self.rng.random())

                rand_x = scenario.base.x + radius * math.cos(angle)
                rand_y = scenario.base.y + radius * math.sin(angle)
            else:
                rand_x = self.rng.uniform(x0 + margin, x1 - margin)
                rand_y = self.rng.uniform(y0 + margin, y1 - margin)

            if not (
                x0 + margin <= rand_x <= x1 - margin
                and y0 + margin <= rand_y <= y1 - margin
            ):
                continue

            candidate_pos = Vector3(rand_x, rand_y, 2.0)

            if not self._is_valid_location(candidate_pos, scenario):
                continue

            # Keep the existing order creation and append logic below.

            self.counter += 1

            new_order = DeliveryOrder(
                id=f"order-{self.counter}",
                destination=candidate_pos,
                priority=self.rng.randint(1, 3),
                payload_weight=round(self.rng.uniform(0.8, 2.0), 1),
                created_at=time,
            )

            scenario.orders.append(new_order)
            return new_order

        return None