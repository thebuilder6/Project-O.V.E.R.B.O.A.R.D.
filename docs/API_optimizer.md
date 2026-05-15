# API Documentation: optimizer.py

Core trajectory optimization using direct collocation with CasADi and IPOPT solver.

## Class

### `TrajectoryOptimizer`

Optimizes time-optimal trajectories for differential-drive robots using direct collocation.

#### Constructor

```python
TrajectoryOptimizer(config: RobotConfig)
```

**Parameters:**

- `config` (RobotConfig): Robot configuration object containing physical parameters

**Attributes:**

- `config` (RobotConfig): Robot configuration
- `model` (DifferentialDriveModel): Differential-drive dynamics model

---

#### Methods

##### `solve(waypoints, num_samples_per_segment=10, accuracy_weight=0.0, stop_waypoint_indices=None, waypoint_events=None, apply_headroom=True, verbose=True, capture_iterations=False)`

Solves the trajectory optimization problem using direct collocation.

**Parameters:**

- `waypoints` (list[tuple]): List of waypoints as `(x, y, heading)` tuples
  - `x` (float): X position in meters
  - `y` (float): Y position in meters
  - `heading` (float|None): Heading in radians (None = unconstrained)
- `num_samples_per_segment` (int): Number of collocation points per segment (default: 10)
- `accuracy_weight` (float): Weight for smoothness/jerk penalty (default: 0.0)
  - 0.0 = pure time-optimal
  - Higher values = smoother trajectories at time cost
- `stop_waypoint_indices` (list[int]|None): List of waypoint indices where robot must come to rest (vl=0, vr=0)
  - None means only start and end at rest (default)
  - Example: `[2, 5, 7]` will stop at waypoints 2, 5, and 7
- `waypoint_events` (dict[int, str]|None): Dictionary mapping waypoint indices to event names
  - None means no events (default)
  - Example: `{2: "lower_arm", 5: "release"}` triggers events at waypoints 2 and 5
  - Events are embedded in output samples at corresponding waypoint indices
- `apply_headroom` (bool): If True, applies safety margin for real-world tracking (default: True)
  - Reduces effective motor torque and speed limits during optimization
  - Ensures Ramsete controller has reserve torque/speed for path corrections
- `verbose` (bool): If True, prints progress messages (default: True)
- `capture_iterations` (bool): If True, captures intermediate solver states for convergence visualization (default: False)

**Returns:**

- (list[dict]): List of trajectory samples, each containing:
  - `t` (float): Time in seconds
  - `x` (float): X position in meters
  - `y` (float): Y position in meters
  - `heading` (float): Heading in radians
  - `vl` (float): Left wheel velocity in m/s
  - `vr` (float): Right wheel velocity in m/s
  - `omega` (float): Angular velocity in rad/s
  - `al` (float): Left wheel acceleration in m/s²
  - `ar` (float): Right wheel acceleration in m/s²
  - `fl` (float): Left wheel force in N
  - `fr` (float): Right wheel force in N
  - `event` (str|optional): Event name at waypoint samples (if specified)

**Optimization Problem Formulation:**

**Decision Variables:**

- `dt`: Time step between collocation points
- `X`: State matrix of shape (N, 5) containing `[x, y, theta, vl, vr]` at each point

**Objective:**

```
minimize: time_cost + accuracy_weight * smoothness_cost

where:
  time_cost = dt * (N - 1)
  smoothness_cost = sum((al_{k+1} - al_k)² + (ar_{k+1} - ar_k)²)
```

**Constraints:**

1. **Time step bounds:** `0.001 <= dt <= 1.0`
2. **State bounds:** Position/heading within reasonable limits
3. **Wheel speed bounds:** `|vl|, |vr| <= 0.99 * max_linear_speed`
4. **Kinematic constraints** (trapezoidal collocation):
   ```
   x[k+1] = x[k] + 0.5 * (v1*cos(theta[k]) + v2*cos(theta[k+1])) * dt
   y[k+1] = y[k] + 0.5 * (v1*sin(theta[k]) + v2*sin(theta[k+1])) * dt
   theta[k+1] = theta[k] + 0.5 * (omega1 + omega2) * dt
   ```
5. **Motor force limits:** `|fl| <= max_force_at_velocity(vl)`, `|fr| <= max_force_at_velocity(vr)`
6. **Traction limit:** `|fl| + |fr| <= cof * mass * g`
7. **Waypoint constraints:** Position and optional heading at each waypoint
8. **Start/end at rest:** `vl[0] = vr[0] = vl[N-1] = vr[N-1] = 0`

**Solver Configuration:**

- Solver: IPOPT (interior point optimizer)
- Tolerance: `1e-2` (relaxed for speed)
- Hessian approximation: Limited-memory (L-BFGS)
- Scaling: Gradient-based
- Max iterations: 5000

**Usage Example:**

```python
from robot_model import RobotConfig
from optimizer import TrajectoryOptimizer
import json

# Load configuration
with open('fll_choreo.chor', 'r') as f:
    config_data = json.load(f)

robot_cfg = RobotConfig(config_data)
optimizer = TrajectoryOptimizer(robot_cfg)

# Define waypoints
waypoints = [
    (0.0, 0.0, 0.0),      # Start at origin, facing East
    (1.0, 0.5, 0.5),      # Waypoint 1
    (2.0, 1.0, 1.0),      # Waypoint 2
    (3.0, 1.5, None)      # End at (3, 1.5), heading unconstrained
]

# Solve with default settings (time-optimal)
samples = optimizer.solve(waypoints, num_samples_per_segment=10)

# Solve with accuracy weighting (smoother trajectory)
samples_smooth = optimizer.solve(
    waypoints,
    num_samples_per_segment=10,
    accuracy_weight=1.0
)

print(f"Trajectory has {len(samples)} samples")
print(f"Total time: {samples[-1]['t']:.3f}s")
```

**Performance:**

- 2 waypoints (10 samples): ~67 ms
- 5 waypoints (40 samples): ~383 ms
- Complex paths: < 500 ms

---

##### `_dynamics_symbolic(vl, vr, al, ar) -> tuple`

CasADi symbolic version of dynamics calculation for use in optimization constraints.

**Parameters:**

- `vl` (casadi.SX/MX): Left wheel velocity (symbolic)
- `vr` (casadi.SX/MX): Right wheel velocity (symbolic)
- `al` (casadi.SX/MX): Left wheel acceleration (symbolic)
- `ar` (casadi.SX/MX): Right wheel acceleration (symbolic)

**Returns:**

- (tuple): `(fl, fr)` symbolic wheel forces

---

##### `_max_force_symbolic(v_wheel) -> casadi.SX/MX`

CasADi symbolic version of motor force limit calculation.

**Parameters:**

- `v_wheel` (casadi.SX/MX): Wheel velocity (symbolic)

**Returns:**

- (casadi.SX/MX): Maximum force at given velocity

---

##### `_build_initial_guess(waypoints, num_samples_per_segment, N) -> numpy.ndarray`

Builds an initial guess for the optimizer using linear interpolation between waypoints.

**Parameters:**

- `waypoints` (list): Waypoint tuples
- `num_samples_per_segment` (int): Samples per segment
- `N` (int): Total number of collocation points

**Returns:**

- (numpy.ndarray): Initial guess vector `[dt, x0, y0, theta0, vl0, vr0, ...]`

**Strategy:**

- Linear interpolation of position between waypoints
- Linear interpolation of heading (if constrained)
- Zero wheel velocities throughout

---

##### `format_output(params, N) -> list[dict]`

Formats the optimizer output into a list of sample dictionaries.

**Parameters:**

- `params` (numpy.ndarray): Flattened parameter vector `[dt, x0, y0, theta0, vl0, vr0, ...]`
- `N` (int): Number of collocation points

**Returns:**

- (list[dict]): Formatted trajectory samples with all derived quantities

**Derivatives computed:**

- Wheel accelerations: `al = (vl_next - vl) / dt`, `ar = (vr_next - vr) / dt`
- Angular velocity: `omega = (vr - vl) / track_width`
- Wheel forces: via `model.get_dynamics()`
