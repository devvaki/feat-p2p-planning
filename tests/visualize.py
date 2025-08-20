import os
import argparse
import csv
import json
import math
import time
from pathlib import Path
import matplotlib.pyplot as plt
from p2p_planning.p2p_planner import P2pPlanner
from p2p_planning.utils import latlon_to_utm  # for path length in meters

Pose = tuple[float, float, float]

def parse_index(index: int, points_pair_path: str) -> tuple[Pose, Pose]:
    """
    Parse the index-th point pair from CSV (start_lat, start_lon, start_bearing, end_lat, end_lon, end_bearing).
    """
    with open(points_pair_path, "r") as f:
        reader = csv.reader(f)
        next(reader)  # header
        for idx, row in enumerate(reader):
            if idx == index:
                start = tuple(map(float, row[0:3]))
                goal  = tuple(map(float, row[3:6]))
                return start, goal
    raise IndexError(f"Point pair index {index} not found in {points_pair_path}")

# -------------------------------
# Plot helpers
# -------------------------------

def _plot_ring(ax, ring, **kw):
    xs = [p[0] for p in ring]
    ys = [p[1] for p in ring]
    ax.plot(xs, ys, **kw)

def _draw_heading(ax, lon: float, lat: float, bearing_deg: float, length: float = 0.0005):
    """
    Draw a short arrow indicating heading. Bearings: 0°=North, clockwise positive.
    We convert to ENU yaw (rad): yaw = +pi/2 - bearing_rad, then compute a tiny (dx,dy) in lon/lat space.
    """
    yaw = math.pi / 2.0 - math.radians(bearing_deg)
    dx = math.cos(yaw) * length
    dy = math.sin(yaw) * length
    ax.annotate("", xy=(lon + dx, lat + dy), xytext=(lon, lat),
                arrowprops=dict(arrowstyle="->", lw=1.2))

def _format_table(headers, rows):
    """Return a pretty, fixed-width table string (no external deps)."""
    # compute column widths
    widths = []
    for i, h in enumerate(headers):
        w = len(str(h))
        for r in rows:
            w = max(w, len(str(r[i])))
        widths.append(w)
    def fmt(row):
        return "  ".join(str(val).ljust(widths[i]) for i, val in enumerate(row))
    sep = "  ".join("-" * w for w in widths)
    return "\n".join([fmt(headers), sep] + [fmt(r) for r in rows])

def _compute_path_length_m(path_llb: list[Pose]) -> float:
    """
    Approximate path length by converting each (lat,lon) to UTM and summing Euclidean distances.
    Assumes all points lie in the same UTM zone/band (true for these small fields).
    """
    if len(path_llb) < 2:
        return 0.0
    e_prev = n_prev = None
    length = 0.0
    z0 = b0 = None
    for lat, lon, _ in path_llb:
        conv = latlon_to_utm(lat, lon)
        e, n = float(conv["easting"]), float(conv["northing"])
        z, b = int(conv["utm_zone"]), str(conv["band"])
        if z0 is None:
            z0, b0 = z, b
        if (z, b) != (z0, b0):
            # Fallback: if zone changed (very unlikely here), stop accumulating further
            break
        if e_prev is not None:
            length += math.hypot(e - e_prev, n - n_prev)
        e_prev, n_prev = e, n
    return length

def run_once(args, quiet: bool = False):
    data_dir = os.path.join(os.path.dirname(__file__), 'data')
    geojson_path = os.path.join(data_dir, f'{args.field}.geojson')
    points_pair_csv_path = os.path.join(data_dir, f'{args.field}_point_pairs.csv')

    planner = P2pPlanner(
        robot_width=args.robot_width,
        robot_length_forward=args.robot_length_forward,
        robot_length_rear=args.robot_length_rear,
        max_curvature=args.max_curvature
    )

    # load geofence
    with open(geojson_path, "r") as f:
        geofence_data = f.read()
    t0 = time.perf_counter()
    ok = planner.load_geofence(geofence_data)
    t1 = time.perf_counter()
    if not ok:
        raise RuntimeError(f"Failed to load geofence from {geojson_path}")

    # read start/goal
    start_pose, goal_pose = parse_index(args.point_pair_idx, points_pair_csv_path)

    # Plan
    try:
        t2 = time.perf_counter()
        if args.mode == "visgraph":
            path = planner.plan_visgraph(
                start_pose=start_pose,
                end_pose=goal_pose,
                path_resolution=args.path_resolution,
                obstacle_clearance=args.obstacle_clearance,
            )
        elif args.mode == "hybrid":
            path = planner.plan_hybrid(
                start_pose=start_pose,
                end_pose=goal_pose,
                path_resolution=args.path_resolution,
                obstacle_clearance=args.obstacle_clearance,
            )
        else:
            path = planner.plan(
                start_pose=start_pose,
                end_pose=goal_pose,
                path_resolution=args.path_resolution,
                obstacle_clearance=args.obstacle_clearance,
            )


        t3 = time.perf_counter()
    except Exception as e:
        print(f"[planner] failed: {e}")
        path = []
        t3 = t2 = time.perf_counter()

    # --- Plot the geofence (in lon/lat) ---
    fig = plt.figure()
    ax = plt.gca()

    gj = json.loads(geofence_data)
    feats = gj["features"] if gj.get("type") == "FeatureCollection" else [{"geometry": gj.get("geometry", gj)}]
    for ft in feats:
        geom = ft["geometry"]
        if geom["type"] == "Polygon":
            polys = [geom["coordinates"]]
        elif geom["type"] == "MultiPolygon":
            polys = geom["coordinates"]
        else:
            polys = []
        for poly in polys:
            outer = poly[0]; holes = poly[1:]
            _plot_ring(ax, outer, lw=1.0)
            for h in holes:
                _plot_ring(ax, h, lw=1.0, linestyle="-")

    # Start/goal markers + headings
    s_lat, s_lon, s_b = start_pose
    g_lat, g_lon, g_b = goal_pose
    ax.scatter([s_lon], [s_lat], marker="o", label="start")
    ax.scatter([g_lon], [g_lat], marker="*", label="goal")
    _draw_heading(ax, s_lon, s_lat, s_b)
    _draw_heading(ax, g_lon, g_lat, g_b)

    # Path polyline
    if path:
        xs = [lon for (_, lon, _) in path]
        ys = [lat for (lat, _, _) in path]
        ax.plot(xs, ys, linestyle="-", label="path")

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Longitude"); ax.set_ylabel("Latitude")
    title = f"{Path(geojson_path).name} | pair #{args.point_pair_idx}"
    ax.set_title(title)
    ax.legend()

    # --- Metrics ---
    load_ms = (t1 - t0) * 1000.0
    plan_ms = (t3 - t2) * 1000.0
    n_pts = len(path)
    length_m = _compute_path_length_m(path) if path else 0.0
    status = "OK" if n_pts > 0 else "BLOCKED"

    if not quiet:
        print(f"[metrics] geofence load: {load_ms:.2f} ms")
        print(f"[metrics] planning:      {plan_ms:.2f} ms")
        print(f"[metrics] path points:   {n_pts}")
        print(f"[metrics] path length:   {length_m:.2f} m")

    # If user gave --save_dir but not --save, auto-name inside that dir
    if not args.save and args.save_dir:
        args.save_dir.mkdir(parents=True, exist_ok=True)
        args.save = args.save_dir / f"{args.field}_pair{args.point_pair_idx}.png"

    # Save if requested
    out_file = None
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160, bbox_inches="tight")
        out_file = str(args.save)
        if not quiet:
            print(f"Saved figure to {args.save}")

    # Only show interactively if asked
    if args.show:
        plt.show()
    plt.close(fig)
    return {
        "field": args.field,
        "pair": args.point_pair_idx,
        "status": status,
        "load_ms": round(load_ms, 2),
        "plan_ms": round(plan_ms, 2),
        "points": n_pts,
        "length_m": round(length_m, 2),
        "file": Path(out_file).name if out_file else "",
    }

def main():
    parser = argparse.ArgumentParser(description="Visualize robot path planning.")
    parser.add_argument('--robot_width', type=float, default=0.5, help='Width of the robot (meters)')
    parser.add_argument('--robot_length_forward', type=float, default=0.8, help='Length from center to front (meters)')
    parser.add_argument('--robot_length_rear', type=float, default=0.2, help='Length from center to rear (meters)')
    parser.add_argument('--max_curvature', type=float, default=0.4, help='Maximum curvature (1/meters)')
    parser.add_argument('--path_resolution', type=float, default=0.05, help='Path resolution (meters)')
    parser.add_argument('--obstacle_clearance', type=float, default=0.1, help='Minimum obstacle clearance (meters)')
    
    # dataset selection
    parser.add_argument('--field', type=str, default='field_1', help='The field to visualize')
    parser.add_argument('--point_pair_idx', default=0, type=int, help="The index of the point pair in test data corresponding to selected field")
    
    # output control
    parser.add_argument("--save", type=Path, help="Save figure to this path (e.g., .png)")
    parser.add_argument("--save_dir", type=Path, help="Directory to save plots (auto-names field_X_pairY.png)")
    parser.add_argument("--batch_all", action="store_true", help="Render ALL pairs for ALL three fields to --save_dir (non-interactive)")
    parser.add_argument("--show", action="store_true", help="Show a window (omit for batch runs)")
    
    parser.add_argument('--mode', type=str, default='straight', choices=['straight','visgraph','hybrid'], help='Planning mode')

    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    if args.save_dir is None:
        args.save_dir = repo_root / "results" / "plot_v2"

    if args.batch_all:
        out_dir = args.save_dir
        out_dir.mkdir(parents=True, exist_ok=True)

        data_dir = Path(__file__).parent / "data"
        rows = []
        for field in ["field_1", "field_2", "field_3"]:
            csv_path = data_dir / f"{field}_point_pairs.csv"
            # count rows minus header
            with open(csv_path, "r") as f:
                n_pairs = max(0, sum(1 for _ in f) - 1)

            for i in range(n_pairs):
                # clone args cleanly
                import argparse as _argparse
                args_i = _argparse.Namespace(**vars(args))
                args_i.field = field
                args_i.point_pair_idx = i
                args_i.save = out_dir / f"{field}_pair{i}.png"
                args_i.show = False
                rec = run_once(args_i, quiet=True)
                rows.append([
                rec["field"], rec["pair"], rec["status"],
                f'{rec["load_ms"]:.2f}', f'{rec["plan_ms"]:.2f}',
                rec["points"], f'{rec["length_m"]:.2f}', rec["file"]
                ])

        print(f"\nAll plots saved under: {out_dir}\n")
        headers = ["field", "pair", "status", "load_ms", "plan_ms", "points", "length_m", "file"]
        print(_format_table(headers, rows))
        return

    # --- single run ---
    rec = run_once(args, quiet=False)
    # also show a one-row table for quick copy/paste
    headers = ["field", "pair", "status", "load_ms", "plan_ms", "points", "length_m", "file"]
    row = [[rec["field"], rec["pair"], rec["status"],
            f'{rec["load_ms"]:.2f}', f'{rec["plan_ms"]:.2f}',
            rec["points"], f'{rec["length_m"]:.2f}', rec["file"]]]
    print("\n" + _format_table(headers, row))

if __name__ == "__main__":
    main()
