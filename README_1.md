# p2p_planning (Point-to-Point Path Planning) of a Robot

- This project implements a **2D point-to-point path planning library** for robots in a geofenced area.   
- It loads **GeoJSON** maps (outer boundary + inner holes/obstacles), converts everything to **UTM meters**, and computes collision-free paths, kinematically feasible that respect **robot footprint** and **clearance** constraints.
- A CLI visualizer lets you render maps, start/goal poses, and planned paths (PNG output).

The planner outputs:
- A **collision-free path** connecting each start to its goal.  
- Paths that respect **robot kinematic constraints** (via Hybrid-A*).  
- Visualization plots (`.png`) showing the paths inside the geofence.

## Methods and Approaches

The path-planning pipeline integrates several methods to ensure collision-free, kinematically feasible, and near-optimal trajectories between start and goal points within geofenced areas.

### 1. Geofence Handling
Input: GeoJSON files defining outer boundaries and internal obstacles.
Processing: Converted lat/lon → UTM for metric calculations.
Purpose: Ensures robot stays strictly inside the allowed region.

### 2. Graph-based Planning (A*)
Used A* search for initial pathfinding.
Guarantees a valid path if one exists.
Provides a baseline for optimality in terms of shortest path length.

### 3. Collision Checking
Implemented using polygonal intersection tests against obstacles.
Each candidate path segment is checked to avoid any overlap with geofence boundaries.
Ensures collision-free trajectories.

### 4. Kinematic Constraints
Integrated heading and turning-radius limits in the planner.
Paths are filtered and adjusted to respect differential-drive/nonholonomic robot motion models.
Prevents infeasible maneuvers (e.g., sharp turns).

### 5. Hybrid A*
Combines grid-based A* with continuous vehicle kinematics.
Produces smooth, drivable paths (not just piecewise linear).
Balances optimality and real-time feasibility.

### 6. Visualization and Evaluation
Automated plotting for each start–goal pair.
Metrics evaluated:
Path length
Collision-freeness
Kinematic feasibility
Computation time

### 7. Testing
Unit tests (pytest) check:
Correct geofence loading
Path validity and safety
Constraint satisfaction
Performance across multiple fields

## Challenges and Solution

## 1. **Hybrid A\* Primitive Validation**
- **Problem:** `primitive_ok` in `plan_hybrid` caused failures due to wrong arguments and overly strict collision checks.  
- **Solution:** Fixed the function signature, clarified **pose validation**, and adjusted **safety radius** margins to prevent false negatives.

### 2. **Collision Checking Overhead**
- **Problem:** Pathfinding slowed down because each pose checked every edge repeatedly.  
- **Solution:** Optimized distance-to-segment checks and used bounding-box pre-filters to speed up calculations.

### 3. **Hybrid A\* Failures**
- **Problem:** Hybrid A\* sometimes failed to find paths in narrow spaces.  
- **Solution:** Added **VisGraph fallback** – ensures at least a geometric path is returned if Hybrid A\* fails.

## ▶️ How to Run

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```
### 2. Run Tests
```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```
Expected output: 8 passed

### 3. Visualize Paths
```bash
# Run Hybrid-A* mode
python tests/visualize.py --batch_all --mode hybrid --save_dir results/hybrid_plots

# Run Visibility Graph mode
python tests/visualize.py --batch_all --mode visgraph --save_dir results/visgraph_plots
```
### 4. Install Cython (optional)
```bash
pip install cython
#Build the extension in place:
python setup.py build_ext --inplace
```

## Current Limitations
- Robot footprint is approximated as a point with safety radius (not full polygon footprint).
- Hybrid A* is slower and may fail in very tight geofences.
- No dynamic obstacle handling (static maps only).
- No explicit optimality guarantee (A* is near-optimal, Hybrid A* is heuristic).

## Future Improvements
- Full polygonal robot footprint collision checking.
- Smarter heuristic tuning for Hybrid A*.
- Multi-agent / dynamic obstacle support.
- Combined Hybrid + VisGraph mode in a single planner.