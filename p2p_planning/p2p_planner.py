from __future__ import annotations
from typing import List, Tuple, Optional
from p2p_planning.utils import latlon_to_utm, utm_to_latlon
import json
import math

# type aliases
Lat = float          # degrees
Lon = float          # degrees
BearingDeg = float   # degrees, 0° = North, clockwise positive
Meters = float

XY = Tuple[Meters, Meters] # UTM meters (x=east, y=north)
LatLonBearing = Tuple[Lat, Lon, BearingDeg]

def _close_ring(ring: List[XY]) -> List[XY]:
    """Ensure the polygon ring is closed (first point equals last)."""
    if not ring:
        return ring
    return ring if ring[0] == ring[-1] else (ring + [ring[0]])


def _shoelace_area(ring: List[XY]) -> float:
    """Signed area via shoelace (positive if CCW)."""
    if len(ring) < 3:
        return 0.0
    a = 0.0
    for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
        a += x1 * y2 - x2 * y1
    return 0.5 * a

def _point_in_polygon(pt: XY, ring: List[XY]) -> bool:
    """Ray-casting point-in-polygon. Assumes ring is closed (but works either way)."""
    x, y = pt
    inside = False
    n = len(ring)
    if n < 3:
        return False
    for i in range(n - 1):
        x1, y1 = ring[i]
        x2, y2 = ring[i + 1]
        if ((y1 > y) != (y2 > y)):
            xints = (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1
            if xints > x:
                inside = not inside
    return inside


def _dist_point_to_segment(p: XY, a: XY, b: XY) -> float:
    """Euclidean distance from point p to line segment ab."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 == 0.0:
        return math.hypot(apx, apy)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    cx, cy = ax + t * abx, ay + t * aby
    return math.hypot(px - cx, py - cy)


def _min_dist_to_edges(pt: XY, ring: List[XY]) -> float:
    """Minimum distance from a point to polygon edges of a ring."""
    ring = _close_ring(ring)
    return min(_dist_point_to_segment(pt, ring[i], ring[i + 1]) for i in range(len(ring) - 1))


def _bearing_deg_to_yaw_rad(bearing_deg: float) -> float:
    """
    CSV bearings: 0° = North, clockwise positive.
    ENU yaw (rad): 0 along +X (East), counterclockwise positive.
    Conversion: yaw = +pi/2 - bearing_rad
    """
    return math.pi / 2.0 - math.radians(bearing_deg)


def _yaw_rad_to_bearing_deg(yaw_rad: float) -> float:
    """Inverse of the above conversion."""
    return (90.0 - math.degrees(yaw_rad)) % 360.0

class P2pPlanner:
    """
    Point-to-Point path planner in a 2D geofenced environment.

    Workflow:
      - call load_geofence(geojson_str) once to cache the map (UTM meters)
      - call plan(start_pose, end_pose, path_resolution, obstacle_clearance) for each request

    Conventions:
      - GeoJSON coordinates are [lon, lat] (degrees).
      - Internally, everything is in UTM meters (x=east, y=north) using one UTM zone.
      - Bearings are 0° at North, clockwise positive (N→E→S→W).
    """

    def __init__(
        self,
        robot_width: Meters,
        robot_length_forward: Meters,
        robot_length_rear: Meters,
        max_curvature: float,   # [1/m] = 1 / min_turning_radius
    ):
        
        # Store robot parameters (used in planning later steps)
        self.w = float(robot_width)
        self.lf = float(robot_length_forward)
        self.lr = float(robot_length_rear)
        self.r = float(max_curvature)

        # Map state (populated by load_geofence)
        self.boundary_xy: List[XY] = []
        self.obstacles_xy: List[List[XY]] = []
        self.utm_zone: Optional[int] = None
        self.band: Optional[str] = None

    # ------------------------------
    # Map loading
    # ------------------------------

    def load_geofence(self, geofence: str) -> bool:
        """
        Load a geofence from a GeoJSON string and cache it in ABSOLUTE UTM meters.

        Rules:
          - One main polygon (outer ring) is the boundary.
          - Its inner rings (if any) are holes/obstacles.
          - If multiple polygons exist, the polygon with the largest |outer area|
            is taken as the boundary; the outer rings of the rest are obstacles.
        """
        # 1) Parse GeoJSON
        try:
            gj = json.loads(geofence)
        except Exception as e:
            raise ValueError(f"Invalid GeoJSON string: {e}")

        # 2) Collect polygons as list-of-rings in lon/lat
        #    type: List[ polygon ], polygon: List[ ring ], ring: List[(lon,lat)]
        polys_lonlat: List[List[List[Tuple[float, float]]]] = []

        def _add_polygon(poly_coords) -> None:
            rings: List[List[Tuple[float, float]]] = []
            for ring in poly_coords:
                # Coordinates may include altitude, take first two (lon, lat)
                rings.append([(float(lon), float(lat)) for lon, lat, *rest in ring])
            polys_lonlat.append(rings)

        t = gj.get("type")
        if t == "FeatureCollection":
            for ft in gj.get("features", []):
                geom = (ft or {}).get("geometry") or {}
                gtype = geom.get("type")
                coords = geom.get("coordinates")
                if not coords:
                    continue
                if gtype == "Polygon":
                    _add_polygon(coords)
                elif gtype == "MultiPolygon":
                    for poly in coords:
                        _add_polygon(poly)
        elif t in ("Polygon", "MultiPolygon"):
            if t == "Polygon":
                _add_polygon(gj.get("coordinates"))
            else:
                for poly in gj.get("coordinates"):
                    _add_polygon(poly)
        else:
            # Some files wrap geometry like {"type":"Feature", "geometry":{...}}
            geom = gj.get("geometry") if isinstance(gj, dict) else None
            if isinstance(geom, dict):
                gtype = geom.get("type")
                coords = geom.get("coordinates")
                if gtype == "Polygon":
                    _add_polygon(coords)
                elif gtype == "MultiPolygon":
                    for poly in coords:
                        _add_polygon(poly)

        if not polys_lonlat:
            raise ValueError("No polygon geometry found in GeoJSON.")

        # 3) Fix UTM zone/band from the very first vertex using the provided utils
        first_lon, first_lat = polys_lonlat[0][0][0]
        conv0 = latlon_to_utm(first_lat, first_lon)  # dict with keys incl. 'utm_zone','band'
        try:
            self.utm_zone = int(conv0["utm_zone"])
            self.band = str(conv0["band"])
        except Exception as e:
            raise KeyError(
                f"latlon_to_utm must return keys 'utm_zone' and 'band'; got {list(conv0.keys())}"
            ) from e

        # 4) Convert all rings to ABSOLUTE UTM meters and validate the zone/band
        def _to_xy(lat: float, lon: float) -> XY:
            c = latlon_to_utm(lat, lon)
            cz = c.get("utm_zone")
            cb = c.get("band")
            if cz != self.utm_zone or cb != self.band:
                raise ValueError(
                    f"Point ({lat:.7f},{lon:.7f}) is in UTM {cz}{cb}, "
                    f"but map zone is {self.utm_zone}{self.band}."
                )
            return float(c["easting"]), float(c["northing"])

        polys_xy: List[List[List[XY]]] = []
        for rings in polys_lonlat:
            xy_rings: List[List[XY]] = []
            for ring in rings:
                xy = [_to_xy(lat, lon) for (lon, lat) in ring]  # GeoJSON is [lon, lat]
                xy_rings.append(_close_ring(xy))
            polys_xy.append(xy_rings)

        # 5) Choose boundary polygon = one with largest |outer-ring area|
        def _outer_abs_area(poly_rings: List[List[XY]]) -> float:
            if not poly_rings or not poly_rings[0]:
                return -1.0
            return abs(_shoelace_area(poly_rings[0]))

        boundary_index = max(range(len(polys_xy)), key=lambda i: _outer_abs_area(polys_xy[i]))
        boundary_poly = polys_xy[boundary_index]

        # 6) Publish boundary and obstacles
        self.boundary_xy = boundary_poly[0]
        obstacles: List[List[XY]] = []
        # Holes of the main polygon
        obstacles.extend(boundary_poly[1:])
        # Outer rings of all other polygons become obstacles
        for i, poly in enumerate(polys_xy):
            if i == boundary_index:
                continue
            if poly:
                obstacles.append(poly[0])
        # Ensure each obstacle ring is closed
        self.obstacles_xy = [_close_ring(r) for r in obstacles if r]

        if len(self.boundary_xy) < 4:
            raise ValueError("Boundary ring must contain at least 3 distinct vertices (closed).")

        return True

    # ------------------------------
    # Planning (baseline)
    # ------------------------------

    def plan(
        self,
        start_pose: Tuple[float, float, float],
        end_pose: Tuple[float, float, float],
        path_resolution: float = 0.1,
        obstacle_clearance: float = 0.2,
    ) -> List[Tuple[float, float, float]]:
        """
        Baseline planner:
      - Convert start/goal to UTM using the cached (utm_zone, band)
      - Generate a straight-line center path sampled every `path_resolution` meters
      - Enforce clearance against boundary/obstacles using a conservative robot radius
      - Convert the path back to (lat, lon, bearing_deg) and return

        Notes:
      - Curvature along a straight line is 0, so it respects max_curvature.
      - Endpoint orientation changes are not enforced here; we align the path heading
        with the line direction. (Lets upgrade to Dubins/RS/Hybrid A* later.)
        """
        if self.utm_zone is None or self.band is None:
            raise RuntimeError("Geofence not loaded. Call load_geofence() first.")

        if path_resolution <= 0:
            raise ValueError("path_resolution must be > 0.")

        # Unpack
        s_lat, s_lon, s_b = map(float, start_pose)
        g_lat, g_lon, g_b = map(float, end_pose)

        # Convert to UTM and validate zone/band
        su = latlon_to_utm(s_lat, s_lon)
        gu = latlon_to_utm(g_lat, g_lon)
        if (su["utm_zone"], su["band"]) != (self.utm_zone, self.band) or \
        (gu["utm_zone"], gu["band"]) != (self.utm_zone, self.band):
            raise ValueError(
                f"Start/goal UTM zones {su['utm_zone']}{su['band']}/{gu['utm_zone']}{gu['band']} "
                f"do not match map zone {self.utm_zone}{self.band}."
            )
        sx, sy = float(su["easting"]), float(su["northing"])
        gx, gy = float(gu["easting"]), float(gu["northing"])

        # Build straight polyline (sampled)
        dx, dy = gx - sx, gy - sy
        dist = math.hypot(dx, dy)
        if dist < 1e-9:
            pts_xy: List[XY] = [(sx, sy)]
        else:
            steps = max(2, int(math.ceil(dist / path_resolution)) + 1)
            pts_xy = [(sx + (i / (steps - 1)) * dx, sy + (i / (steps - 1)) * dy) for i in range(steps)]

        # Conservative safety radius: disk that contains the rectangular footprint + extra clearance
        half_w = self.w * 0.5
        half_len = max(self.lf, self.lr)
        robot_radius = math.hypot(half_w, half_len)
        safety_radius = robot_radius + float(obstacle_clearance)

        boundary = self.boundary_xy
        holes = self.obstacles_xy

        # Sanity: start & goal should be valid
        def _valid_pose(p: XY) -> bool:
            if not _point_in_polygon(p, boundary):
                return False
            if _min_dist_to_edges(p, boundary) < safety_radius:
                return False
            for h in holes:
                if _point_in_polygon(p, h):
                    return False
                if _min_dist_to_edges(p, h) < safety_radius:
                    return False
            return True

        if not _valid_pose((sx, sy)):
            raise RuntimeError("Start pose is invalid (outside or too close to boundary/obstacles).")
        if not _valid_pose((gx, gy)):
            raise RuntimeError("Goal pose is invalid (outside or too close to boundary/obstacles).")

        # Check every sample
        for p in pts_xy:
            if not _valid_pose(p):
                raise RuntimeError(
                    "No collision-free straight-line path with the given clearance/footprint. "
                    "Use a curved planner (Dubins/RS/Hybrid A*) or adjust start/goal/clearance."
                )

        # Heading along the path (constant yaw for a straight line)
        if len(pts_xy) >= 2:
            yaw = math.atan2(pts_xy[1][1] - pts_xy[0][1], pts_xy[1][0] - pts_xy[0][0])
        else:
            yaw = _bearing_deg_to_yaw_rad(s_b)
        bearing_deg = _yaw_rad_to_bearing_deg(yaw)

        # Convert path back to lat/lon
        path_llb: List[Tuple[float, float, float]] = []
        for x, y in pts_xy:
            ll = utm_to_latlon(x, y, self.utm_zone, self.band)
            # utils typically returns dict with 'latitude' and 'longitude'
            lat = float(ll["latitude"]) if isinstance(ll, dict) else ll[0]
            lon = float(ll["longitude"]) if isinstance(ll, dict) else ll[1]
            path_llb.append((lat, lon, bearing_deg))

        return path_llb
