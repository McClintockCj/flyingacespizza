import pygame

from flying_aces.core.contracts import Vector3


class Viewport:
    """Screen<->world transform supporting pan (middle-drag) and zoom (wheel)."""

    def __init__(self, offset=(120, 120), scale=15.0):
        self.offset = list(offset)
        self.scale = scale
        self.min_scale = 0.05
        self.max_scale = 60.0
        self._panning = False
        self._pan_anchor = None
        self._offset_anchor = None

    def world_to_screen(self, position: Vector3):
        return (
            int(position.x * self.scale + self.offset[0]),
            int(position.y * self.scale + self.offset[1]),
        )

    def screen_to_world(self, point, z: float = 0.0) -> Vector3:
        x, y = point
        return Vector3(
            (x - self.offset[0]) / self.scale,
            (y - self.offset[1]) / self.scale,
            z,
        )

    # def fit(self, bounds, screen_size, margin=140):
    #     """Center + scale so the given world bounds fit on screen."""
    #     x0, y0, x1, y1 = bounds
    #     width = max(x1 - x0, 1.0)
    #     height = max(y1 - y0, 1.0)

    #     sx = (screen_size[0] - margin) / width
    #     sy = (screen_size[1] - margin) / height

    #     self.scale = max(self.min_scale, min(self.max_scale, min(sx, sy)))

    #     self.offset[0] = margin / 2 - x0 * self.scale
    #     self.offset[1] = margin / 2 - y0 * self.scale

    def fit(self, bounds, screen_size, margin=60):
        x0, y0, x1, y1 = bounds
        width = max(x1 - x0, 1.0)
        height = max(y1 - y0, 1.0)

        sx = (screen_size[0] - margin * 2) / width
        sy = (screen_size[1] - margin * 2) / height

        self.scale = max(self.min_scale, min(self.max_scale, min(sx, sy)))

        self.offset[0] = (screen_size[0] - width * self.scale) / 2 - x0 * self.scale
        self.offset[1] = (screen_size[1] - height * self.scale) / 2 - y0 * self.scale

    def handle_event(self, event) -> bool:
        """Returns True if the event was consumed by camera panning/zooming."""
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 2:
            self._panning = True
            self._pan_anchor = event.pos
            self._offset_anchor = tuple(self.offset)
            return True

        if event.type == pygame.MOUSEBUTTONUP and event.button == 2:
            self._panning = False
            return True

        if event.type == pygame.MOUSEMOTION and self._panning:
            dx = event.pos[0] - self._pan_anchor[0]
            dy = event.pos[1] - self._pan_anchor[1]
            self.offset[0] = self._offset_anchor[0] + dx
            self.offset[1] = self._offset_anchor[1] + dy
            return True

        if event.type == pygame.MOUSEWHEEL:
            old_scale = self.scale
            factor = 1.1 if event.y > 0 else 1 / 1.1
            self.scale = max(self.min_scale, min(self.max_scale, self.scale * factor))

            mx, my = pygame.mouse.get_pos()

            self.offset[0] = mx - (mx - self.offset[0]) * (self.scale / old_scale)
            self.offset[1] = my - (my - self.offset[1]) * (self.scale / old_scale)

            return True

        return False