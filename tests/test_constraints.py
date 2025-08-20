# tests/test_constraints.py
import math, csv, time, pathlib
import pytest

from p2p_planning.p2p_planner import P2pPlanner
from p2p_planning.utils import latlon_to_utm

DATA = pathlib.Path(__file__).parent / "data"

# ---------- helpers ----------
def load_pairs(field, limit=None):
    rows = []
    with open(DATA / f"{field}_point_pairs.csv", "r") as f:
        r = csv.reader(f); next(r)
        for row in r:
            s = tuple(map(float, row[0:3]))
            g = tuple(map(float, row[3:6]))
            rows.append((s, g))
    return rows if limit is None else rows[:limit]

def robot_radius(planner: P2pPlanner):
    half_w = planner.w * 0.5
    half_len = max(planner.lf, planner.lr)
    return math.hypot(half_w, half_len)

def to_xy(planner, lat, lon):
    c = latlon_to_utm(lat, lon)
    assert (c["utm_zone"], c["band"]) == (planner.utm_zone, planner.band)
    return float(c["easting"]), float(c["northing"])

def length_m(planner, path):
    pts = [to_xy(planner, lat, lon) for (lat, lon, _) in path]
    return sum(math.hypot(x2-x1, y2-y1) for (x1,y1),(x2,y2) in zip(pts, pts[1:]))

def max_curvature_estimate(planner, path):
    pts = [to_xy(planner, lat, lon) for (lat, lon, _) in path]
    if len(pts) < 3: return 0.0
    kmax = 0.0
    for i in range(1, len(pts)-1):
        x1,y1 = pts[i-1]; x2,y2 = pts[i]; x3,y3 = pts[i+1]
        a = math.hypot(x2-x1, y2-y1); b = math.hypot(x3-x2, y3-y2); c = math.hypot(x3-x1, y3-y1)
        if a*b*c < 1e-6: continue
        s = 0.5*(a+b+c)
        A = max(0.0, s*(s-a)*(s-b)*(s-c))**0.5
        k = (4.0*A)/(a*b*c)
        kmax = max(kmax, k)
    return kmax

def collision_free(planner, path, clearance):
    """Use the planner's own strict validator on every interior point."""
    safety = planner._get_robot_radius(clearance)
    for idx, (lat, lon, _) in enumerate(path):
        # convert to planner's UTM
        c = latlon_to_utm(lat, lon)
        x, y = float(c["easting"]), float(c["northing"])
        # allow endpoints to be near the boundary; interiors must be strictly valid
        if idx == 0 or idx == len(path) - 1:
            # loose check via strict with a tiny tolerance: endpoints are permitted to sit near boundary
            if not planner._is_pose_valid_loose((x, y), tolerance=clearance):
                return False
        else:
            if not planner._is_pose_valid_strict((x, y), safety):
                return False
    return True

# ---------- fixtures ----------
@pytest.fixture(scope="module")
def planner():
    return P2pPlanner(robot_width=0.5, robot_length_forward=0.8, robot_length_rear=0.2, max_curvature=0.4)

# ---------- tests ----------
@pytest.mark.parametrize("field", ["field_1","field_2","field_3"])
def test_connectivity_and_collision_visgraph(planner, field):
    geojson = (DATA / f"{field}.geojson").read_text()
    assert planner.load_geofence(geojson)
    pairs = load_pairs(field, limit=5)

    t0 = time.perf_counter()
    successes = 0
    for s, g in pairs:
        try:
            path = planner.plan_visgraph(s, g, path_resolution=0.1, obstacle_clearance=0.1)
        except RuntimeError:
            continue  # no corridor at this clearance is acceptable
        assert len(path) >= 2, "empty path"
        assert collision_free(planner, path, clearance=0.1), "visgraph path not collision-free"
        successes += 1
    elapsed_ms = (time.perf_counter() - t0) * 1000
    # time & compute budget (coarse): 5 pairs within 8s total
    assert elapsed_ms < 8000, f"visgraph too slow: {elapsed_ms:.0f} ms"
    # at least one success expected per field (maps have some reachable pairs)
    assert successes >= 1, "no visgraph successes in sample pairs"

@pytest.mark.parametrize("field", ["field_1","field_2"])
def test_kinematics_and_length_hybrid(planner, field):
    geojson = (DATA / f"{field}.geojson").read_text()
    assert planner.load_geofence(geojson)
    pairs = load_pairs(field, limit=5)

    for s, g in pairs:
        # visgraph (upper-bound length) — may fail if no corridor; that's fine
        vg_path = None
        try:
            vg_path = planner.plan_visgraph(s, g, path_resolution=0.1, obstacle_clearance=0.1)
        except RuntimeError:
            pass

        # hybrid (kinematic)
        try:
            hy_path = planner.plan_hybrid(s, g, path_resolution=0.1, obstacle_clearance=0.1)
        except RuntimeError:
            continue  # unreachable with given radius/clearance

        assert len(hy_path) >= 2
        # connectivity (near goal)
        gx, gy = to_xy(planner, g[0], g[1])
        px, py = to_xy(planner, hy_path[-1][0], hy_path[-1][1])
        assert math.hypot(px-gx, py-gy) <= 1.0, "hybrid not close to goal"
        # collision-free
        assert collision_free(planner, hy_path, clearance=0.1), "hybrid path not collision-free"
        # kinematic constraint
        k_seen = max_curvature_estimate(planner, hy_path)
        assert k_seen <= planner.r + 0.05, f"curvature violated: {k_seen:.3f} > {planner.r:.3f}"
        # length competitiveness vs visgraph (when visgraph exists)
        if vg_path is not None:
            assert length_m(planner, hy_path) <= 1.2 * length_m(planner, vg_path), "hybrid much longer than visgraph"
