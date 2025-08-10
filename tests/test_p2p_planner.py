import pathlib

from p2p_planning.p2p_planner import P2pPlanner


def load_planner(geofence_file: str) -> P2pPlanner:
    robot = {
        "robot_width": 0.5,
        "robot_length_forward": 0.8,
        "robot_length_rear": 0.2,
        "max_curvature": 0.4
    }
    planner = P2pPlanner(**robot)
    with open(geofence_file, "r") as f:
        geofence_data = f.read()
    if planner.load_geofence(geofence_data):
        return planner
    else:
        raise Exception(f"Failed to load geofence from {geofence_file}")

def test_geofence_loaded_field_1():
    planner = load_planner(pathlib.Path(__file__).parent / "data" / "field_1.geojson")
    # TODO: Implement test
    assert True

def test_geofence_loaded_field_2():
    planner = load_planner(pathlib.Path(__file__).parent / "data" / "field_2.geojson")
    # TODO: Implement test
    assert True

def test_geofence_loaded_field_3():
    planner = load_planner(pathlib.Path(__file__).parent / "data" / "field_3.geojson")
    # TODO: Implement test
    assert True