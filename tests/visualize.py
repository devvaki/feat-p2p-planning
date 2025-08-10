import os
import argparse
import csv
from p2p_planning.p2p_planner import P2pPlanner

Pose = tuple[float, float, float]

def parse_index(index_str, points_pair_path) -> tuple[Pose, Pose]:
    """
    Parse The index of the point pair in test data corresponding to selected field.
    """
    with open(points_pair_path, "r") as f:
        reader = csv.reader(f)
        next(reader) # header
        for idx, row in enumerate(reader):
            if idx == index_str:
                return tuple(map(float, row[0:3])), tuple(map(float, row[3:6]))
    raise Exception(f"Point pair index {index_str} not found in {points_pair_path}")

def main():
    parser = argparse.ArgumentParser(description="Visualize robot path planning.")
    parser.add_argument('--robot_width', type=float, default=0.5, help='Width of the robot (meters)')
    parser.add_argument('--robot_length_forward', type=float, default=0.8, help='Length from center to front (meters)')
    parser.add_argument('--robot_length_rear', type=float, default=0.2, help='Length from center to rear (meters)')
    parser.add_argument('--max_curvature', type=float, default=0.4, help='Maximum curvature (1/meters)')
    parser.add_argument('--path_resolution', type=float, default=0.05, help='Path resolution (meters)')
    parser.add_argument('--obstacle_clearance', type=float, default=0.1, help='Minimum obstacle clearance (meters)')
    parser.add_argument('--field', type=str, default='field_1', help='The field to visualize')
    parser.add_argument('--point_pair_idx', default=0, type=int, help="The index of the point pair in test data corresponding to selected field")

    args = parser.parse_args()

    planner = P2pPlanner(
        robot_width=args.robot_width,
        robot_length_forward=args.robot_length_forward,
        robot_length_rear=args.robot_length_rear,
        max_curvature=args.max_curvature
    )

    data_dir = os.path.join(os.path.dirname(__file__), 'data')
    geojson_path = os.path.join(data_dir, f'{args.field}.geojson')
    points_pair_csv_path = os.path.join(data_dir, f'{args.field}_point_pairs.csv')

    with open(geojson_path, "r") as f:
        geofence_data = f.read()
    if not planner.load_geofence(geofence_data):
        raise Exception(f"Failed to load geofence from {geojson_path}")

    start_pose, goal_pose = parse_index(args.point_pair_idx, points_pair_csv_path)

    path = planner.plan(
        start_pose=start_pose,
        end_pose=goal_pose,
        path_resolution=args.path_resolution,
        obstacle_clearance=args.obstacle_clearance,
    )

    # TODO: Visualize the path, robot motion, and geofence

    # TODO: Print performance metrics

if __name__ == "__main__":
    main()
