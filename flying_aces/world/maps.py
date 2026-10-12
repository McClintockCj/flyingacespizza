from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import pygame

from flying_aces.core.contracts import Obstacle


COLORS = {
    "building": (190, 184, 170),
    "road": (215, 215, 215),
    "road_centerline": (150, 150, 150),
    "park": (145, 190, 130),
    "green": (170, 205, 150),
    "water": (115, 175, 215),
    "landuse": (225, 215, 190),
}

BACKGROUND = (242, 240, 232)
BUILDING_OUTLINE = (125, 120, 110)


# @dataclass(frozen=True)
# class Obstacle:
#     """Static obstacle geometry in simulator world coordinates."""

#     polygons: tuple[tuple[tuple[float, float], ...], ...]
#     height: float = 0.0

#     @property
#     def bounds(self):
#         points = [point for polygon in self.polygons for point in polygon]
#         if not points:
#             return 0.0, 0.0, 0.0, 0.0

#         xs = [point[0] for point in points]
#         ys = [point[1] for point in points]
#         return min(xs), min(ys), max(xs), max(ys)


class MapManager:
    """
    Loaded map data and Pygame rendering support.

    Map JSON coordinates are normalized into the simulator's existing
    coordinate convention: +X right and +Y down.
    """

    def __init__(self, map_path: str | Path):
        self.path = Path(map_path)
        self.data = json.loads(self.path.read_text(encoding="utf-8"))

        self.features = self.data.get("features", [])
        self.roads = self.data.get("road_network", [])

        self.metadata = self.data.get("metadata", {})
        self.name = self.metadata.get("name", "Map")

        self.bounds = self._normalize_bounds(self.metadata["bounds"])
        self.obstacles = self._load_obstacles()

        # Cached static map image and the camera/viewport state used to draw it.
        self._render_cache = None
        self._cache_scale = None
        self._cache_size = None
        self._cache_offset = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def get_bounds(self):
        return self.bounds

    def get_obstacles(self) -> list[Obstacle]:
        return list(self.obstacles)

    # def render(self, surface: pygame.Surface, camera):
    #     """Render the complete map background using the simulator camera."""
    #     surface.fill(BACKGROUND)

    #     self._render_land(surface, camera)
    #     self._render_roads(surface, camera)
    #     self._render_buildings(surface, camera)
    
    def render(self, surface: pygame.Surface, camera):
        """Render the cached static map background."""
        size = surface.get_size()
        scale = camera.scale
        offset = tuple(camera.offset)

        cache_invalid = (
            self._render_cache is None
            or self._cache_size != size
            or self._cache_scale != scale
            or self._cache_offset != offset
        )

        if cache_invalid:
            self._render_cache = pygame.Surface(size)
            self._render_cache.fill(BACKGROUND)

            self._render_land(self._render_cache, camera)
            self._render_roads(self._render_cache, camera)
            self._render_buildings(self._render_cache, camera)

            self._cache_size = size
            self._cache_scale = scale
            self._cache_offset = offset

        surface.blit(self._render_cache, (0, 0))

    def render_land(self, surface: pygame.Surface, camera):
        self._render_land(surface, camera)

    def render_roads(self, surface: pygame.Surface, camera):
        self._render_roads(surface, camera)

    def render_buildings(self, surface: pygame.Surface, camera):
        self._render_buildings(surface, camera)

    # ------------------------------------------------------------------
    # Loading / normalization
    # ------------------------------------------------------------------

    @staticmethod
    def _normalize_point(point):
        x, y = point[:2]
        return float(x), -float(y)

    @classmethod
    def _normalize_bounds(cls, bounds):
        x0, y0, x1, y1 = bounds
        return float(x0), -float(y1), float(x1), -float(y0)

    @classmethod
    def _normalize_geometry(cls, geometry):
        if not geometry:
            return None

        kind = geometry.get("type")
        coords = geometry.get("coordinates", [])

        if kind == "Polygon":
            return {
                "type": "Polygon",
                "coordinates": [
                    [cls._normalize_point(point) for point in ring]
                    for ring in coords
                ],
            }

        if kind == "MultiPolygon":
            return {
                "type": "MultiPolygon",
                "coordinates": [
                    [
                        [cls._normalize_point(point) for point in ring]
                        for ring in polygon
                    ]
                    for polygon in coords
                ],
            }

        if kind in ("LineString", "LinearRing"):
            return {
                "type": kind,
                "coordinates": [
                    cls._normalize_point(point)
                    for point in coords
                ],
            }

        if kind == "MultiLineString":
            return {
                "type": "MultiLineString",
                "coordinates": [
                    [cls._normalize_point(point) for point in line]
                    for line in coords
                ],
            }

        return geometry

    def _load_obstacles(self):
        obstacles = []

        for feature in self.features:
            if feature.get("kind") != "building":
                continue

            geometry = self._normalize_geometry(feature.get("geometry"))
            if not geometry:
                continue

            polygons = self._extract_polygons(geometry)
            if not polygons:
                continue

            height = self._get_height(feature)

            obstacles.append(
                Obstacle(
                    polygons=tuple(polygons),
                    height=height,
                )
            )

        return obstacles

    @staticmethod
    def _get_height(feature):
        properties = feature.get("properties", {})

        value = feature.get("height")
        if value is None:
            value = properties.get("height")

        if value is None:
            value = properties.get("building_height")

        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.0

    @staticmethod
    def _extract_polygons(geometry):
        kind = geometry["type"]
        coords = geometry["coordinates"]

        if kind == "Polygon":
            if not coords:
                return []

            return [
                tuple(ring)
                for ring in coords
                if len(ring) >= 3
            ]

        if kind == "MultiPolygon":
            polygons = []

            for polygon in coords:
                if not polygon:
                    continue

                outer_ring = polygon[0]

                if len(outer_ring) >= 3:
                    polygons.append(tuple(outer_ring))

            return polygons

        return []

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def _render_land(self, surface, camera):
        for feature in self.features:
            if feature.get("kind") not in {
                "park",
                "green",
                "water",
                "landuse",
            }:
                continue

            geometry = self._normalize_geometry(feature.get("geometry"))
            if geometry:
                self._draw_geometry(
                    surface,
                    camera,
                    geometry,
                    COLORS.get(feature["kind"], COLORS["landuse"]),
                )

    def _render_roads(self, surface, camera):
        for feature in self.features:
            if feature.get("kind") != "road":
                continue

            geometry = self._normalize_geometry(feature.get("geometry"))
            if geometry:
                self._draw_geometry(
                    surface,
                    camera,
                    geometry,
                    COLORS["road"],
                    width=2,
                )

        for road in self.roads:
            geometry = self._normalize_geometry(road.get("geometry"))
            if geometry:
                self._draw_geometry(
                    surface,
                    camera,
                    geometry,
                    COLORS["road_centerline"],
                    width=1,
                )

    def _render_buildings(self, surface, camera):
        for obstacle in self.obstacles:
            for polygon in obstacle.polygons:
                points = [
                    self._world_to_screen(camera, point)
                    for point in polygon
                ]

                if len(points) < 3:
                    continue

                pygame.draw.polygon(
                    surface,
                    COLORS["building"],
                    points,
                )

                pygame.draw.polygon(
                    surface,
                    BUILDING_OUTLINE,
                    points,
                    1,
                )

    # ------------------------------------------------------------------
    # Geometry rendering
    # ------------------------------------------------------------------

    @staticmethod
    def _world_to_screen(camera, point):
        x, y = point

        # Camera accepts Vector3 in the existing simulator.
        # Avoid importing contracts here so this module stays independent.
        class Point:
            pass

        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0

        return camera.world_to_screen(p)

    def _draw_geometry(self, surface, camera, geometry, color, width=0):
        kind = geometry.get("type")
        coords = geometry.get("coordinates", [])

        if kind == "Polygon":
            self._draw_polygon(surface, camera, coords, color)

        elif kind == "MultiPolygon":
            for polygon in coords:
                self._draw_polygon(surface, camera, polygon, color)

        elif kind in ("LineString", "LinearRing"):
            points = [
                self._world_to_screen(camera, point)
                for point in coords
            ]

            if len(points) >= 2:
                pygame.draw.lines(
                    surface,
                    color,
                    False,
                    points,
                    max(width, 1),
                )

        elif kind == "MultiLineString":
            for line in coords:
                points = [
                    self._world_to_screen(camera, point)
                    for point in line
                ]

                if len(points) >= 2:
                    pygame.draw.lines(
                        surface,
                        color,
                        False,
                        points,
                        max(width, 1),
                    )

    def _draw_polygon(self, surface, camera, rings, color):
        if not rings:
            return

        points = [
            self._world_to_screen(camera, point)
            for point in rings[0]
        ]

        if len(points) >= 3:
            pygame.draw.polygon(surface, color, points)