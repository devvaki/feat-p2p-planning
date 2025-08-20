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

# --- Optional Cython accel (safe fallback if not built) ---
CY_FAST = False
try:
    import numpy as np
    from p2p_planning.geo_cyth import (
        dist_point_to_segment as cy_dist_seg,
        min_dist_to_edges as cy_min_dist,
        violates_clearance as cy_violates,
    )
    CY_FAST = True
except Exception:
    pass

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
    if len(ring) < 2:
        return float('inf')
    return min(_dist_point_to_segment(pt, ring[i], ring[i + 1]) for i in range(len(ring) - 1))

def _violates_clearance(pt: XY, ring: List[XY], clearance: float) -> bool:
    """Fast YES/NO: return True the moment any edge is closer than `clearance`."""
    ringc = _close_ring(ring)
    for i in range(len(ringc) - 1):
        if _dist_point_to_segment(pt, ringc[i], ringc[i + 1]) < clearance:
            return True
    return False

def _dist_point_to_segment_fast(p, a, b) -> float:
    """Use cython fast distance if available."""
    x, y = p; x1, y1 = a; x2, y2 = b
    if CY_FAST:
        return float(cy_dist_seg(x, y, x1, y1, x2, y2))
    return _dist_point_to_segment(p, a, b)

def _min_dist_to_edges_fast(pt, ring, *, _np_arr_cache=None) -> float:
    """Fast min distance to a ring. Accepts list[(x,y)]."""
    if CY_FAST:
        arr = np.asarray(ring, dtype=np.float64)
        return float(cy_min_dist(pt[0], pt[1], arr))
    return _min_dist_to_edges(pt, ring)

def _violates_clearance_fast(pt, ring, clearance: float) -> bool:
    """Fast YES/NO clearance test against a ring."""
    if CY_FAST:
        arr = np.asarray(ring, dtype=np.float64)
        return bool(cy_violates(pt[0], pt[1], arr, float(clearance)))
    return _violates_clearance(pt, ring, clearance)

def _seg_intersects(a: XY, b: XY, c: XY, d: XY, eps: float = 1e-9) -> bool:
    """Proper segment intersection test, including collinear overlap."""
    (ax, ay), (bx, by), (cx, cy), (dx, dy) = a, b, c, d

    def orient(p, q, r):
        (px, py), (qx, qy), (rx, ry) = p, q, r
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    def on_seg(p, q, r):
        (px, py), (qx, qy), (rx, ry) = p, q, r
        return (min(px, rx) - eps <= qx <= max(px, rx) + eps) and \
               (min(py, ry) - eps <= qy <= max(py, ry) + eps)

    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if (o1 * o2 < 0) and (o3 * o4 < 0):
        return True
    if abs(o1) <= eps and on_seg(a, c, b): return True
    if abs(o2) <= eps and on_seg(a, d, b): return True
    if abs(o3) <= eps and on_seg(c, a, d): return True
    if abs(o4) <= eps and on_seg(c, b, d): return True
    return False


def _segment_hits_ring(a: XY, b: XY, ring: List[XY]) -> bool:
    """True if segment ab intersects any edge of ring."""
    ring = _close_ring(ring)
    for i in range(len(ring) - 1):
        if _seg_intersects(a, b, ring[i], ring[i + 1]):
            return True
    return False

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
    # Helper methods for pose validation
    # ------------------------------
    
    def _get_robot_radius(self, obstacle_clearance: float = 0.0) -> float:
        """Calculate conservative robot radius including clearance."""
        half_w = self.w * 0.5
        half_len = max(self.lf, self.lr)
        robot_radius = math.hypot(half_w, half_len)
        return robot_radius + float(obstacle_clearance)
    
    def _is_pose_valid_strict(self, p: XY, safety_radius: float) -> bool:
        print(f"[_is_pose_valid_strict] Checking pose {p} with safety_radius={safety_radius}")
        
        if _min_dist_to_edges_fast(p, self.boundary_xy) < safety_radius:
            print(f"[_is_pose_valid_strict] Pose {p} too close to boundary ❌")
            return False

        for obs in self.obstacles_xy:
            if _min_dist_to_edges_fast(p, obs) < safety_radius:
                print(f"[_is_pose_valid_strict] Pose {p} collides with obstacle ❌")
                return False

        print(f"[_is_pose_valid_strict] Pose {p} is valid ✅")
        return True
    
    def _is_pose_valid_loose(self, p: XY, tolerance: float = 0.1) -> bool:
        """Loose validation for start/goal: allow near boundary edge, but not in obstacles."""
        # Must be inside boundary OR very close to boundary edge
        if not (_point_in_polygon(p, self.boundary_xy) or _min_dist_to_edges(p, self.boundary_xy) <= tolerance):
            return False
        # Must not be inside any obstacle or too close to obstacle edges
        for h in self.obstacles_xy:
            if _point_in_polygon(p, h):
                return False
            if _min_dist_to_edges_fast(p, h) <= tolerance:
                return False
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
        mode: str = "straight",
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
        if self.utm_zone is None or self.band is None or not self.boundary_xy:
            raise RuntimeError("Geofence not loaded. Call load_geofence() first.")
        if path_resolution <= 0:
            raise ValueError("path_resolution must be > 0")

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

        # Get safety radius
        safety_radius = self._get_robot_radius(obstacle_clearance)

        # Validate start & goal with loose constraints
        if not self._is_pose_valid_loose((sx, sy), obstacle_clearance):
            raise RuntimeError("Start pose is invalid (outside boundary or inside/near obstacles).")
        if not self._is_pose_valid_loose((gx, gy), obstacle_clearance):
            raise RuntimeError("Goal pose is invalid (outside boundary or inside/near obstacles).")

        # Check path points with strict constraints (except endpoints)
        for i, p in enumerate(pts_xy):
            if i == 0 or i == len(pts_xy) - 1:  # Allow endpoints to be near boundaries
                if not self._is_pose_valid_loose(p, obstacle_clearance):
                    raise RuntimeError(f"Endpoint at {p} is invalid.")
            else:
                if not self._is_pose_valid_strict(p, safety_radius):
                    raise RuntimeError(
                        "No collision-free straight-line path with the given clearance/footprint. "
                        "Try visgraph or hybrid mode, or adjust start/goal/clearance."
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
    
    def plan_visgraph(
        self,
        start_pose: Tuple[float, float, float],
        end_pose: Tuple[float, float, float],
        path_resolution: float = 0.1,
        obstacle_clearance: float = 0.2,
    ) -> List[Tuple[float, float, float]]:

        if self.utm_zone is None or self.band is None or not self.boundary_xy:
            raise RuntimeError("Geofence not loaded. Call load_geofence() first.")

        if path_resolution <= 0:
            raise ValueError("path_resolution must be > 0")

        # Get safety radius
        safety_radius = self._get_robot_radius(obstacle_clearance)

        # Convert start/goal to UTM
        s_lat, s_lon, _ = map(float, start_pose)
        g_lat, g_lon, _ = map(float, end_pose)
        su = latlon_to_utm(s_lat, s_lon)
        gu = latlon_to_utm(g_lat, g_lon)
        if (su["utm_zone"], su["band"]) != (self.utm_zone, self.band) or \
           (gu["utm_zone"], gu["band"]) != (self.utm_zone, self.band):
            raise ValueError("Start/goal UTM zone mismatch.")
        s = (float(su["easting"]), float(su["northing"]))
        g = (float(gu["easting"]), float(gu["northing"]))

        # Validate start/goal with loose constraints
        tolerance = max(0.05, 0.5 * obstacle_clearance)
        if not self._is_pose_valid_loose(s, tolerance):
            raise RuntimeError("Start pose invalid (outside boundary or inside obstacle).")
        if not self._is_pose_valid_loose(g, tolerance):
            raise RuntimeError("Goal pose invalid (outside boundary or inside obstacle).")

        # Line-of-sight check
        def _los(a, b):
            # Quick rejections with segment-edge intersection tests
            if _segment_hits_ring(a, b, self.boundary_xy):
                return False
            for ring in self.obstacles_xy:
                if _segment_hits_ring(a, b, ring):
                    return False

            # Sample clearance inside the map (skip endpoints)
            ax, ay = a
            bx, by = b
            dist = math.hypot(bx - ax, by - ay)
            step = max(path_resolution, safety_radius * 0.5, 0.1)
            steps = max(3, int(math.ceil(dist / step)) + 1)
            for i in range(1, steps - 1):
                t = i / (steps - 1)
                x = ax + t * (bx - ax)
                y = ay + t * (by - ay)
                if not self._is_pose_valid_strict((x, y), safety_radius):
                    return False
            return True

        # Build nodes: start, goal, all boundary + obstacle vertices
        nodes = [s, g]
        nodes += self.boundary_xy[:-1]  # skip closing duplicate
        for h in self.obstacles_xy:
            nodes += h[:-1]
        
        # De-duplicate nodes
        seen = set()
        uniq = []
        for x, y in nodes:
            key = (round(x, 3), round(y, 3))
            if key not in seen:
                seen.add(key)
                uniq.append((x, y))
        nodes = uniq

        # Build edges if LOS is valid
        import heapq
        N = len(nodes)
        adj = [[] for _ in range(N)]
        for i in range(N):
            for j in range(i + 1, N):
                a, b = nodes[i], nodes[j]
                if _los(a, b):
                    w = math.hypot(b[0] - a[0], b[1] - a[1])
                    adj[i].append((j, w))
                    adj[j].append((i, w))

        # A*: start index 0 (s), goal index 1 (g)
        def h(i):  # Euclidean heuristic
            x, y = nodes[i]
            return math.hypot(x - g[0], y - g[1])
        
        start_idx, goal_idx = 0, 1
        gscore = [float("inf")] * N
        gscore[start_idx] = 0.0
        fscore = [float("inf")] * N
        fscore[start_idx] = h(start_idx)
        came = [-1] * N
        pq = [(fscore[start_idx], start_idx)]
        
        while pq:
            _, u = heapq.heappop(pq)
            if u == goal_idx:
                break
            for v, w in adj[u]:
                alt = gscore[u] + w
                if alt < gscore[v]:
                    gscore[v] = alt
                    came[v] = u
                    f = alt + h(v)
                    fscore[v] = f
                    heapq.heappush(pq, (f, v))
        
        if came[goal_idx] == -1:
            raise RuntimeError("No visgraph path found.")

        # Reconstruct path
        path_xy = []
        u = goal_idx
        while u != -1:
            path_xy.append(nodes[u])
            u = came[u]
        path_xy.reverse()

        # Optional shortcutting
        i = 0
        while i + 2 < len(path_xy):
            if _los(path_xy[i], path_xy[i + 2]):
                del path_xy[i + 1]
            else:
                i += 1

        # Densify with resolution and convert back to (lat,lon,bearing)
        dense = []
        for (x0, y0), (x1, y1) in zip(path_xy[:-1], path_xy[1:]):
            dx, dy = x1 - x0, y1 - y0
            seg = math.hypot(dx, dy)
            steps = max(2, int(math.ceil(seg / max(path_resolution, 1e-3))) + 1)
            for k in range(steps - 1):
                t = k / (steps - 1)
                dense.append((x0 + t * dx, y0 + t * dy))
        dense.append(path_xy[-1])

        if len(dense) >= 2:
            yaw = math.atan2(dense[1][1] - dense[0][1], dense[1][0] - dense[0][0])
        else:
            yaw = 0.0
        bearing = _yaw_rad_to_bearing_deg(yaw)

        path_llb = []
        for x, y in dense:
            ll = utm_to_latlon(x, y, self.utm_zone, self.band)
            lat = float(ll["latitude"])
            lon = float(ll["longitude"])
            path_llb.append((lat, lon, bearing))
        return path_llb
        
    def plan_hybrid(
        self,
        start_pose: Tuple[float, float, float],
        end_pose: Tuple[float, float, float],
        path_resolution: float = 0.1,
        obstacle_clearance: float = 0.2,
        heading_bins: int = 32,
        step_m: float = None,
        goal_xy_tol_m: float = 1.0,  # Increased tolerance
        goal_yaw_tol_deg: float = 20.0,  # Increased tolerance
        max_iters: int = 200000,
    ) -> List[Tuple[float, float, float]]:
        """
        Hybrid A* with simple motion primitives (left/straight/right) at curvature k = max_curvature.
        - Respects min turning radius (1/k), checks clearance along each primitive.
        - Heuristic: Euclidean distance (admissible).
        """
        if self.utm_zone is None or self.band is None or not self.boundary_xy:
            raise RuntimeError("Geofence not loaded. Call load_geofence() first.")
        if path_resolution <= 0:
            raise ValueError("path_resolution must be > 0")

        # Get safety radius
        safety_radius = self._get_robot_radius(obstacle_clearance)

        # Start/goal to UTM
        s_lat, s_lon, s_b = map(float, start_pose)
        g_lat, g_lon, g_b = map(float, end_pose)
        su = latlon_to_utm(s_lat, s_lon)
        gu = latlon_to_utm(g_lat, g_lon)  # Fixed the typo here!
        if (su["utm_zone"], su["band"]) != (self.utm_zone, self.band) or \
           (gu["utm_zone"], gu["band"]) != (self.utm_zone, self.band):
            raise ValueError("Start/goal UTM zone mismatch.")
        sx, sy = float(su["easting"]), float(su["northing"])
        gx, gy = float(gu["easting"]), float(gu["northing"])

        yaw_s = _bearing_deg_to_yaw_rad(s_b)
        yaw_g = _bearing_deg_to_yaw_rad(g_b)

        # Validate start/goal with loose constraints
        tolerance = max(0.1, 0.5 * obstacle_clearance)
        if not self._is_pose_valid_loose((sx, sy), tolerance):
            raise RuntimeError("Start pose invalid (outside boundary or inside obstacle).")
        if not self._is_pose_valid_loose((gx, gy), tolerance):
            raise RuntimeError("Goal pose invalid (outside boundary or inside obstacle).")

        # Curvature - fallback to visgraph if no curvature constraint
        k = float(getattr(self, "r", getattr(self, "max_curvature", 0.0)))
        if k <= 1e-6:
            return self.plan_visgraph(start_pose, end_pose, path_resolution, obstacle_clearance)

        # Step length for primitives
        if step_m is None:
            step_m = max(path_resolution, 0.3, safety_radius * 0.5)

        TWO_PI = 2.0 * math.pi
        def wrap(y):
            while y < -math.pi: y += TWO_PI
            while y >  math.pi: y -= TWO_PI
            return y

        # Heading discretization
        HB = int(max(8, heading_bins))
        def yaw_to_bin(y):
            y = wrap(y)
            frac = (y + math.pi) / TWO_PI
            return int(min(max(frac * HB, 0), HB - 1))

        # Integrate primitive of length ds at curvature curv
        def step_pose(x, y, yaw, curv, ds):
            if abs(curv) < 1e-9:
                nx = x + ds * math.cos(yaw)
                ny = y + ds * math.sin(yaw)
                nyaw = yaw
            else:
                R = 1.0 / curv
                nyaw = yaw + curv * ds
                nx = x + R * (math.sin(nyaw) - math.sin(yaw))
                ny = y - R * (math.cos(nyaw) - math.cos(yaw))
            return nx, ny, wrap(nyaw)

        # Sample along primitive (skip endpoints)
        def primitive_ok(x, y, yaw, curv, ds):
            steps = max(2, int(math.ceil(ds / path_resolution)))
            for i in range(1, steps + 1):
                t = i / steps
                xi, yi, _ = step_pose(x, y, yaw, curv, ds * t)
                if not self._is_pose_valid_strict((xi, yi), safety_radius):
                    return False
            return True

        # Heuristic: Euclidean
        def hfun(x, y):
            return math.hypot(gx - x, gy - y)

        import heapq
        start_state = (sx, sy, yaw_s)
        start_key = (round(sx, 1), round(sy, 1), yaw_to_bin(yaw_s))

        openq = [(hfun(sx, sy), 0.0, start_key, start_state)]
        came = {}
        gscore = {start_key: 0.0}

        PRIMS = (-k, 0.0, +k)  # left / straight / right
        found_key = None
        iters = 0

        while openq and iters < max_iters:
            _, gcur, key, (x, y, yaw) = heapq.heappop(openq)
            iters += 1

            # Goal test with more reasonable tolerances
            if math.hypot(x - gx, y - gy) <= goal_xy_tol_m:
                yaw_err = abs((math.degrees(wrap(yaw - yaw_g)) + 180.0) % 360.0 - 180.0)
                if yaw_err <= goal_yaw_tol_deg:
                    found_key = key
                    came.setdefault(key, None)
                    break

            # Skip if we've seen this state with better cost
            if key in came and gcur > gscore.get(key, float("inf")):
                continue

            for curv in PRIMS:
                nx, ny, nyaw = step_pose(x, y, yaw, curv, step_m)
                
                # Check if end pose is valid (use loose for goal region)
                if math.hypot(nx - gx, ny - gy) <= goal_xy_tol_m:
                    if not self._is_pose_valid_loose((nx, ny), tolerance):
                        continue
                else:
                    if not self._is_pose_valid_strict((nx, ny), safety_radius):
                        continue
                
                # Check primitive path
                if not primitive_ok(x, y, yaw, curv, step_m):
                    continue

                nkey = (round(nx, 1), round(ny, 1), yaw_to_bin(nyaw))
                alt = gcur + step_m
                if alt < gscore.get(nkey, float("inf")):
                    gscore[nkey] = alt
                    came[nkey] = (key, (nx, ny, nyaw))
                    f = alt + hfun(nx, ny)
                    heapq.heappush(openq, (f, alt, nkey, (nx, ny, nyaw)))

        if found_key is None:
            # Fallback to visgraph if hybrid A* fails
            try:
                return self.plan_visgraph(start_pose, end_pose, path_resolution, obstacle_clearance)
            except:
                raise RuntimeError("No hybrid-A* path found, and visgraph fallback also failed.")

        # Reconstruct path
        path_xyyaw = []
        cur = found_key
        state_map = {start_key: start_state}
        while cur in came and came[cur] is not None:
            prev, s = came[cur]
            path_xyyaw.append(s)
            state_map[prev] = s
            cur = prev
        path_xyyaw.append(start_state)
        path_xyyaw.reverse()

        # Convert to (lat, lon, bearing) and densify if needed
        if len(path_xyyaw) <= 1:
            # Single point, just return start
            ll = utm_to_latlon(sx, sy, self.utm_zone, self.band)
            return [(float(ll["latitude"]), float(ll["longitude"]), _yaw_rad_to_bearing_deg(yaw_s))]
        
        # Densify path if segments are too long
        dense_path = []
        for i in range(len(path_xyyaw) - 1):
            x0, y0, yaw0 = path_xyyaw[i]
            x1, y1, yaw1 = path_xyyaw[i + 1]
            
            seg_len = math.hypot(x1 - x0, y1 - y0)
            if seg_len > path_resolution:
                steps = int(math.ceil(seg_len / path_resolution)) + 1
                for j in range(steps):
                    t = j / (steps - 1) if steps > 1 else 0
                    x = x0 + t * (x1 - x0)
                    y = y0 + t * (y1 - y0)
                    yaw = wrap(yaw0 + t * wrap(yaw1 - yaw0))
                    if j < steps - 1:  # Don't duplicate the last point
                        dense_path.append((x, y, yaw))
            else:
                dense_path.append((x0, y0, yaw0))
        
        # Add final point
        dense_path.append(path_xyyaw[-1])

        # Convert to lat/lon/bearing
        out = []
        for (x, y, yaw) in dense_path:
            ll = utm_to_latlon(x, y, self.utm_zone, self.band)
            bearing = _yaw_rad_to_bearing_deg(yaw)
            out.append((float(ll["latitude"]), float(ll["longitude"]), bearing))
        
        return out