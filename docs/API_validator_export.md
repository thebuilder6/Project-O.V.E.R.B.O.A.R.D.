# API Documentation: validator.py and export.py

Trajectory validation and controller export utilities.

---

# validator.py

Functions for validating trajectories via forward integration and constraint auditing.

## Functions

### `forward_integrate(samples, robot_cfg: RobotConfig, fine_dt: float = 0.001) -> tuple[list, dict]`

Forward-integrates differential-drive kinematics from the optimized wheel-velocity profile using RK4 integration.

**Parameters:**

- `samples` (list[dict]): Trajectory samples from optimizer output
- `robot_cfg` (RobotConfig): Robot configuration object
- `fine_dt` (float): Integration timestep in seconds (default: 0.001)

**Returns:**

- (tuple): `(integrated, errors)`
  - `integrated` (list[dict]): List of integrated states with keys `{t, x, y, heading}`
  - `errors` (dict): Error metrics:
    - `max_pos_error_m`: Maximum position error in meters
    - `rms_pos_error_m`: RMS position error in meters
    - `max_heading_error_rad`: Maximum heading error in radians
    - `final_pos_error_m`: Final position error in meters
    - `final_heading_error_rad`: Final heading error in radians

**Method:**

- Interpolates wheel velocities at fine timesteps
- Uses RK4 integration for kinematic equations:
  ```
  dx/dt = v * cos(theta)
  dy/dt = v * sin(theta)
  dtheta/dt = omega
  ```
- Compares integrated positions with planned positions at corresponding timestamps
- Computes position error as Euclidean distance
- Computes heading error with angle wrapping to [-π, π]

**Usage Example:**

```python
from validator import forward_integrate
from robot_model import RobotConfig
import json

# Load trajectory and config
with open('output.traj', 'r') as f:
    traj_data = json.load(f)
with open('fll_choreo.chor', 'r') as f:
    config_data = json.load(f)

samples = traj_data['trajectory']['samples']
robot_cfg = RobotConfig(config_data)

# Forward integrate at 1ms timesteps
integrated, errors = forward_integrate(samples, robot_cfg, fine_dt=0.001)

print(f"Max position error: {errors['max_pos_error_m']:.6f} m")
print(f"Final position error: {errors['final_pos_error_m']:.6f} m")
```

---

### `audit_constraints(samples, robot_cfg: RobotConfig, apply_headroom=True) -> dict`

Re-evaluates motor and traction limits for each sample to detect constraint violations and wheel slip.

**Parameters:**

- `samples` (list[dict]): Trajectory samples from optimizer output
- `robot_cfg` (RobotConfig): Robot configuration object
- `apply_headroom` (bool): If True, applies safety margin for real-world tracking (default: True)

**Returns:**

- (dict): Audit results with keys:
  - `left_motor_force`: Max left motor force violation (N)
  - `right_motor_force`: Max right motor force violation (N)
  - `traction_total`: Max traction limit violation (N)
  - `num_violating_samples`: Number of samples with violations
  - `violating_sample_indices`: List of indices of violating samples
  - `num_slip_points`: Number of samples with wheel slip (NEW)
  - `left_wheel_slip`: Max left wheel slip force (N) (NEW)
  - `right_wheel_slip`: Max right wheel slip force (N) (NEW)
  - `slip_points`: List of slip point details (NEW)
    - Each entry: `time`, `x`, `y`, `left_wheel_slip_N`, `right_wheel_slip_N`, `left_normal_force_N`, `right_normal_force_N`

**Constraints Checked:**

1. Left motor force: `|fl| <= max_force_at_velocity(vl)`
2. Right motor force: `|fr| <= max_force_at_velocity(vr)`
3. Traction limit: `|fl| + |fr| <= cof * mass * g`
4. Wheel slip: Individual wheel force exceeds friction limit (NEW)

**Wheel Slip Detection:**

The audit now includes wheel slip detection that identifies points where the required wheel force exceeds the friction limit for that individual wheel. This provides early warning of potential tracking issues on real robots. The slip detection computes:
- Left wheel slip force (excess over friction limit)
- Right wheel slip force (excess over friction limit)
- Normal forces on each wheel
- Slip point locations and times

**Usage Example:**

```python
from validator import audit_constraints
from robot_model import RobotConfig
import json

# Load trajectory and config
with open('output.traj', 'r') as f:
    traj_data = json.load(f)
with open('fll_choreo.chor', 'r') as f:
    config_data = json.load(f)

samples = traj_data['trajectory']['samples']
robot_cfg = RobotConfig(config_data)

# Audit constraints
audit = audit_constraints(samples, robot_cfg)

if audit['num_violating_samples'] == 0:
    print("No constraint violations detected")
else:
    print(f"Found {audit['num_violating_samples']} violating samples")
    print(f"Max left motor violation: {audit['left_motor_force']:.6f} N")
    print(f"Max right motor violation: {audit['right_motor_force']:.6f} N")
    print(f"Max traction violation: {audit['traction_total']:.6f} N")

# Check for wheel slip
if audit['num_slip_points'] > 0:
    print(f"Found {audit['num_slip_points']} wheel slip points")
    print(f"Max left wheel slip: {audit['left_wheel_slip']:.6f} N")
    print(f"Max right wheel slip: {audit['right_wheel_slip']:.6f} N")
    print("Slip point details:")
    for sp in audit['slip_points'][:5]:  # Show first 5
        print(f"  t={sp['time']:.3f}s at ({sp['x']:.3f}, {sp['y']:.3f})")
```

---

### `compute_metrics(samples) -> dict`

Computes basic trajectory metrics from sample data.

**Parameters:**

- `samples` (list[dict]): Trajectory samples from optimizer output

**Returns:**

- (dict): Metrics with keys:
  - `total_time_s`: Total trajectory duration in seconds
  - `path_length_m`: Total path length in meters
  - `max_linear_speed_m_s`: Maximum linear speed in m/s
  - `max_wheel_speed_m_s`: Maximum individual wheel speed in m/s
  - `max_accel_m_s2`: Maximum wheel acceleration in m/s²
  - `max_jerk_m_s3`: Maximum jerk in m/s³

**Usage Example:**

```python
from validator import compute_metrics
import json

with open('output.traj', 'r') as f:
    traj_data = json.load(f)

samples = traj_data['trajectory']['samples']
metrics = compute_metrics(samples)

print(f"Total time: {metrics['total_time_s']:.3f} s")
print(f"Path length: {metrics['path_length_m']:.3f} m")
print(f"Max speed: {metrics['max_linear_speed_m_s']:.3f} m/s")
```

---

### `validate_trajectory(traj_file: str, config_file: str, apply_headroom=True) -> tuple`

CLI entry point for trajectory validation. Loads files, runs validation, and prints a human-readable report.

**Parameters:**

- `traj_file` (str): Path to `.traj` file
- `config_file` (str): Path to `.chor` config file
- `apply_headroom` (bool): If True, applies safety margin for real-world tracking (default: True)

**Returns:**

- (tuple): `(metrics, audit, errors)` - Results from `compute_metrics`, `audit_constraints`, and `forward_integrate`

**Output:**
Prints a comprehensive validation report including:

- Sample count and config file
- Basic metrics (time, length, speeds, accelerations)
- Constraint audit (violations and magnitudes)
- Forward integration errors (position and heading)
- Pass/fail verdict

**Pass Criteria:**

- `max_pos_error_m < 0.01` meters
- `final_pos_error_m < 0.01` meters
- `num_violating_samples == 0`

**Usage:**

```bash
python validator.py output.traj fll_choreo.chor
```

---

# export.py

Functions for resampling trajectories to fixed timesteps for controller consumption.

## Functions

### `resample_to_fixed_dt(samples, target_dt: float = 0.02, track_width: float = 0.0965) -> list[dict]`

Linearly resamples a variable-timestep trajectory to a fixed controller timestep.

**Parameters:**

- `samples` (list[dict]): Trajectory samples from optimizer output
- `target_dt` (float): Fixed timestep in seconds (default: 0.02)
- `track_width` (float): Robot track width in meters (default: 0.0965)

**Returns:**

- (list[dict]): Resampled samples with keys:
  - `t`: Timestamp in seconds
  - `x`: X position in meters
  - `y`: Y position in meters
  - `heading`: Heading in radians
  - `vl`: Left wheel velocity in m/s
  - `vr`: Right wheel velocity in m/s
  - `v`: Linear velocity in m/s
  - `omega`: Angular velocity in rad/s

**Method:**

- Uses linear interpolation between source samples
- Generates samples at `t = 0, target_dt, 2*target_dt, ...` until trajectory end
- Computes derived quantities (v, omega) from interpolated wheel velocities

**Usage Example:**

```python
from export import resample_to_fixed_dt
import json

with open('output.traj', 'r') as f:
    traj_data = json.load(f)

samples = traj_data['trajectory']['samples']

# Resample to 20ms timesteps for controller
resampled = resample_to_fixed_dt(samples, target_dt=0.02, track_width=0.0965)

print(f"Resampled to {len(resampled)} samples at 20ms")
print(f"First sample: {resampled[0]}")
```

---

### `export_controller_json(samples, target_dt: float = 0.02, track_width: float = 0.0965) -> dict`

Returns a JSON-serializable dict with controller-ready samples.

**Parameters:**

- `samples` (list[dict]): Trajectory samples from optimizer output
- `target_dt` (float): Fixed timestep in seconds (default: 0.02)
- `track_width` (float): Robot track width in meters (default: 0.0965)

**Returns:**

- (dict): Controller profile with keys:
  - `format`: String identifier ("controller_profile")
  - `version`: Format version (1)
  - `dt`: Timestep in seconds
  - `num_samples`: Number of samples
  - `samples`: List of resampled sample dicts

**Output Format:**

```json
{
  "format": "controller_profile",
  "version": 1,
  "dt": 0.02,
  "num_samples": 339,
  "samples": [
    {
      "t": 0.0,
      "x": 0.0,
      "y": 0.0,
      "heading": 0.0,
      "vl": 0.0,
      "vr": 0.0,
      "v": 0.0,
      "omega": 0.0
    },
    ...
  ]
}
```

---

### `write_controller_file(input_traj_file: str, output_file: str, target_dt: float = 0.02, track_width: float = 0.0965)`

Loads a trajectory file, resamples it, and writes a controller-ready JSON file.

**Parameters:**

- `input_traj_file` (str): Path to input `.traj` file
- `output_file` (str): Path to output controller JSON file
- `target_dt` (float): Fixed timestep in seconds (default: 0.02)
- `track_width` (float): Robot track width in meters (default: 0.0965)

**Side Effects:**

- Creates/overwrites `output_file` with controller profile JSON
- Prints confirmation message with sample count

**Usage:**

```bash
python export.py output.traj output_controller.json 0.02
```

**Or from Python:**

```python
from export import write_controller_file

write_controller_file(
    'output.traj',
    'output_controller.json',
    target_dt=0.02,
    track_width=0.0965
)
```

---

### `write_python_file(input_traj_file: str, output_file: str)`

Loads a trajectory file and exports it as a Python file with trajectory samples and robot configuration.

**Parameters:**

- `input_traj_file` (str): Path to input `.traj` file
- `output_file` (str): Path to output Python file

**Side Effects:**

- Creates/overwrites `output_file` with Python code containing:
  - Robot configuration parameters as a `config` dictionary
  - Trajectory samples as a `samples` list
- Prints confirmation message with sample count

**Output Format:**

```python
# Trajectory exported from FLL Trajectory Optimizer
# Source: input_traj_file
# Number of samples: 136

# Robot configuration parameters
config = {
    "mass": 0.723000,
    "inertia": 0.002400,
    "track_width": 0.096500,
    "wheel_radius": 0.028000,
    "v_max_rad_s": 15.700000,
    "t_max_nm": 0.040000,
    "gearing": 1.000000,
    "cof": 0.400000,
}

# Trajectory samples
samples = [
    {"t": 0.000000, "x": 0.000000, "y": 0.000000, "heading": 0.000000, "vl": 0.000000, "vr": 0.000000, "omega": 0.000000},
    {"t": 0.100176, "x": 0.001904, "y": 0.000030, "heading": 0.015507, "vl": 0.023083, "vr": 0.052958, "omega": 0.309586},
    # ... more samples
]
```

**Usage:**

```python
from export import write_python_file

write_python_file('output.traj', 'trajectory.py')
```

**Or from CLI:**

```bash
python main.py -c config.chor -w waypoints.json -o output.traj --export-format python
```

**Use case:** Direct integration with Python-based robot controllers or analysis scripts, avoiding JSON parsing overhead.
