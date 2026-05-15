# Configuration Guide

This guide explains how to configure the robot parameters and Multiverse optimizer settings for the FLL Trajectory Optimizer.

## Configuration File Format

The optimizer uses a JSON configuration file (e.g., `robot_config.json`). The file contains robot physical parameters and optimization settings grouped into nested objects.

### Full Example (`robot_config.json`)

```json
{
  "name": "fll_choreo",
  "version": 3,
  "type": "Differential",
  "robot": {
    "mass": 0.723,
    "inertia": 0.0024,
    "track_width": 0.0965,
    "wheel_radius": 0.028,
    "v_max_rad_s": 15.7,
    "t_max_nm": 0.04,
    "gearing": 1.0,
    "cof": 0.40,
    "gravity": 9.81,
    "torque_headroom": 0.85,
    "speed_headroom": 0.90
  },
  "multiverse": {
    "enable_parallel": true,
    "num_workers": 8,
    "stomp_variants": 5,
    "stomp_noise": {
      "position_std": 0.05,
      "heading_std": 0.1
    },
    "teb_weights": {
      "forward_bias": 10.0,
      "reverse_bias": 10.0,
      "point_turn_bias": 5.0,
      "wide_sweep_bias": 2.0
    },
    "critic_thresholds": {
      "tortuosity": 1.5,
      "yaw_buffer_rad": 0.5,
      "velocity_chattering_threshold": 3
    },
    "bootstrap": {
      "turning_radius": 0.0,
      "resolution": 0.05
    }
  }
}
```

---

## Robot Parameters (`robot` object)

These define the physical limitations and properties of your robot.

### `mass`
Robot mass in kilograms.
- **Typical FLL robot:** 0.5 - 1.5 kg
- **Impact:** Affects acceleration limits (heavier = slower acceleration) and traction limit calculation.

### `inertia`
Rotational moment of inertia in kg·m².
- **Typical FLL robot:** 1e-6 to 1e-3 kg·m²
- **Impact:** Affects how much torque is needed for rotation. Higher inertia = slower turns.

### `track_width`
Distance between the centers of the left and right wheels in meters.
- **Typical FLL robot:** 0.08 - 0.15 m (80 - 150 mm)
- **Impact:** Affects turning kinematics. Wider track width = slower angular velocity for the same wheel speed difference.

### `wheel_radius`
Radius of the drive wheels in meters.
- **Typical LEGO wheel:** 0.028 - 0.056 m (28 - 56 mm)
- **Impact:** Affects speed conversion between wheel RPM and linear velocity.

### `v_max_rad_s`
Motor no-load angular velocity in radians/second.
- **Example:** LEGO EV3 Large Motor is ~150 RPM. (150 * 2π / 60) ≈ 15.7 rad/s.
- **Impact:** Maximum possible wheel speed, directly limiting the maximum robot speed.

### `t_max_nm`
Motor stall torque in Newton-meters.
- **Typical LEGO motors:** ~0.02 to ~0.05 N·m
- **Impact:** Maximum force the motor can apply. Directly affects acceleration capability.

### `gearing`
Gear ratio between motor and wheel.
- **Direct drive:** 1.0
- **Geared down (wheel turns slower):** > 1.0
- **Geared up (wheel turns faster):** < 1.0
- **Impact:** Multiplies torque and divides speed (or vice versa).

### `cof`
Coefficient of friction between wheels and field surface.
- **Typical FLL mat:** ~0.4 - 1.2
- **Impact:** Maximum traction force before wheel slip.

### `gravity`
Acceleration due to gravity in m/s².
- **Default:** 9.81

### `torque_headroom`
Safety margin for motor torque limits (e.g., 0.85 = 15% headroom).
- **Impact:** Ensures the Ramsete controller has reserve torque for path corrections against real-world factors like battery sag.

### `speed_headroom`
Safety margin for wheel speed limits (e.g., 0.90 = 10% headroom).
- **Impact:** Ensures the Ramsete controller has reserve speed for path corrections.

---

## Multiverse Parameters (`multiverse` object)

Settings for the advanced Multi-Verse optimization pipeline.

### General Settings
- `enable_parallel` (bool): Run optimization heuristics in parallel.
- `num_workers` (int): Number of parallel workers (threads/processes) to use.

### `stomp_variants` and `stomp_noise`
Stochastic Trajectory Optimization for Motion Planning parameters.
- `stomp_variants` (int): Number of noisy variants generated per problematic segment.
- `position_std` (float): Standard deviation of Gaussian noise added to positional (x, y) coordinates.
- `heading_std` (float): Standard deviation of Gaussian noise added to angular coordinates.

### `teb_weights`
Timed Elastic Band topology biases.
- `forward_bias`: Weight to encourage forward motion topologies.
- `reverse_bias`: Weight to encourage reverse motion topologies.
- `point_turn_bias`: Weight to encourage in-place turns.
- `wide_sweep_bias`: Weight to encourage gentle, sweeping turns.

### `critic_thresholds`
Thresholds used by the Trajectory Critic to identify problematic path segments.
- `tortuosity`: Ratio of path length to straight-line distance. Higher values (> 1.5) indicate wandering paths.
- `yaw_buffer_rad`: Allowed excess yaw rotation beyond the expected turn.
- `velocity_chattering_threshold`: Maximum number of wheel velocity zero-crossings allowed before a segment is flagged for refinement.
- `jerk_cost_threshold`: Threshold for rate of acceleration change. High jerk leads to tracking errors.
- `curvature_cost_threshold`: Threshold for turn sharpness. High curvature increases slip risk.
- `centripetal_cost_threshold`: Threshold for centripetal acceleration relative to friction limits.

### `bootstrap`
Initial guess generation settings using Reeds-Shepp paths.
- `turning_radius`: Minimum turning radius for Reeds-Shepp paths.
- `resolution`: Step size used when discretizing the paths.

---

## Waypoint File Format

Waypoints are specified in a JSON file. Each waypoint can include position, heading, and an optional stop or event constraint.

### Basic Format

```json
[
  { "x": 0.0, "y": 0.0, "heading": 0.0 },
  { "x": 1.0, "y": 0.5, "heading": 0.5 },
  { "x": 2.0, "y": 1.0 }
]
```

### Fields

- `x` (float, required): X position in meters
- `y` (float, required): Y position in meters
- `heading` (float, optional): Heading in radians. If omitted or `null`, the heading is unconstrained.
- `stop` (boolean, optional): Whether the robot must come to complete rest at this waypoint (default: `false`).
- `event` (string, optional): Event name to trigger at this waypoint (e.g., "lower_arm"). Used for triggering attachments.

### Example with Stops and Events

```json
[
  { "x": 0.0, "y": 0.0, "heading": 0.0, "stop": true },
  { "x": 0.5, "y": 0.3, "heading": 0.5, "event": "lower_arm", "stop": true },
  { "x": 1.0, "y": 0.0, "heading": 0.0 }
]
```

*(Note: The CLI `--events` and `--stop-waypoints` flags can override JSON settings.)*

---

## Command Line Interface (CLI)

The optimizer can be run from the command line using `main.py`.

### Common Options
- `-c, --config`: Path to the robot configuration JSON file.
- `-w, --waypoints`: Path to waypoints JSON file.
- `-o, --output`: Output trajectory file path (default: `output.traj`).
- `-n, --samples`: Samples per segment (default: 10).
- `-a, --accuracy-weight`: Smoothness/accuracy weight (0 = pure time-optimal).

### Optimizer Strategy
- `--simple`: Use simple optimizer instead of Multi-Verse refinement.
- `--no-parallel`: Disable parallel processing for Multi-Verse refinement.
- `--workers`: Number of parallel workers for Multi-Verse refinement (default: 8).

### Analysis and Validation
- `--validate`: Run validation report on the generated trajectory.
- `--benchmark`: Collect comprehensive benchmarking data (timing, heuristics, and quality metrics) and save to `[output]_stats.json`.
- `--show-convergence`: Show convergence visualization of the optimization process.

### Exporting
- `--export-format`: Export format for controller consumption (`none`, `controller`, `python`).
- `--controller-dt`: Fixed timestep for controller export (seconds).
