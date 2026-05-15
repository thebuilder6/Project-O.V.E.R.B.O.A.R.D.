import json
import numpy as np
from robot_model import RobotConfig, DifferentialDriveModel


def forward_integrate(samples, robot_cfg: RobotConfig, fine_dt: float = 0.001):
    """
    Forward-integrate differential-drive kinematics from the optimized
    wheel-velocity profile using a fine timestep.

    Returns a list of dicts with keys {t, x, y, heading} and a dict of
    max / RMS errors versus the planned trajectory.
    """
    if not samples:
        return [], {}

    model = DifferentialDriveModel(robot_cfg)
    t_arr = np.array([s["t"] for s in samples])
    vl_arr = np.array([s["vl"] for s in samples])
    vr_arr = np.array([s["vr"] for s in samples])

    # Interpolators for wheel speeds
    def interp_v(t_query):
        if t_query <= t_arr[0]:
            return vl_arr[0], vr_arr[0]
        if t_query >= t_arr[-1]:
            return vl_arr[-1], vr_arr[-1]
        idx = int(np.searchsorted(t_arr, t_query)) - 1
        idx = max(0, min(idx, len(t_arr) - 2))
        t0, t1 = t_arr[idx], t_arr[idx + 1]
        frac = (t_query - t0) / (t1 - t0)
        vl = vl_arr[idx] + (vl_arr[idx + 1] - vl_arr[idx]) * frac
        vr = vr_arr[idx] + (vr_arr[idx + 1] - vr_arr[idx]) * frac
        return vl, vr

    total_t = t_arr[-1]
    num_steps = int(np.ceil(total_t / fine_dt))
    x, y, theta = samples[0]["x"], samples[0]["y"], samples[0]["heading"]

    integrated = []
    for step in range(num_steps + 1):
        t = min(step * fine_dt, total_t)
        vl, vr = interp_v(t)
        v = (vl + vr) / 2.0
        omega = (vr - vl) / robot_cfg.track_width

        # RK4 integration step
        if step < num_steps:
            dt = min(fine_dt, total_t - t)
            k1x = v * np.cos(theta)
            k1y = v * np.sin(theta)
            k1t = omega

            x2 = x + k1x * dt * 0.5
            y2 = y + k1y * dt * 0.5
            theta2 = theta + k1t * dt * 0.5
            v2 = v  # vl/vr assumed constant over tiny dt
            k2x = v2 * np.cos(theta2)
            k2y = v2 * np.sin(theta2)
            k2t = omega

            x3 = x + k2x * dt * 0.5
            y3 = y + k2y * dt * 0.5
            theta3 = theta + k2t * dt * 0.5
            k3x = v2 * np.cos(theta3)
            k3y = v2 * np.sin(theta3)
            k3t = omega

            x4 = x + k3x * dt
            y4 = y + k3y * dt
            theta4 = theta + k3t * dt
            k4x = v2 * np.cos(theta4)
            k4y = v2 * np.sin(theta4)
            k4t = omega

            x += (k1x + 2 * k2x + 2 * k3x + k4x) * dt / 6.0
            y += (k1y + 2 * k2y + 2 * k3y + k4y) * dt / 6.0
            theta += (k1t + 2 * k2t + 2 * k3t + k4t) * dt / 6.0

        integrated.append({"t": float(t), "x": float(x), "y": float(y), "heading": float(theta)})

    # Error vs planned samples
    planned = np.array([[s["x"], s["y"], s["heading"]] for s in samples])
    integrated_at_t = np.array(
        [
            [
                integrated[int(min(np.ceil(s["t"] / fine_dt), num_steps))]["x"],
                integrated[int(min(np.ceil(s["t"] / fine_dt), num_steps))]["y"],
                integrated[int(min(np.ceil(s["t"] / fine_dt), num_steps))]["heading"],
            ]
            for s in samples
        ]
    )

    dx = planned[:, 0] - integrated_at_t[:, 0]
    dy = planned[:, 1] - integrated_at_t[:, 1]
    dtheta = (planned[:, 2] - integrated_at_t[:, 2] + np.pi) % (2 * np.pi) - np.pi

    pos_err = np.hypot(dx, dy)
    errors = {
        "max_pos_error_m": float(np.max(pos_err)),
        "rms_pos_error_m": float(np.sqrt(np.mean(pos_err ** 2))),
        "max_heading_error_rad": float(np.max(np.abs(dtheta))),
        "final_pos_error_m": float(pos_err[-1]),
        "final_heading_error_rad": float(abs(dtheta[-1])),
    }
    return integrated, errors


def audit_constraints(samples, robot_cfg: RobotConfig, apply_headroom=True):
    """
    Re-evaluate motor and traction limits for each sample.
    Returns max violations and a list of violating sample indices with slip details.
    
    Args:
        samples: Trajectory samples
        robot_cfg: Robot configuration
        apply_headroom: If True, applies safety margin for real-world tracking
    """
    model = DifferentialDriveModel(robot_cfg)
    
    # Track max violations
    max_violations = {
        "left_motor_force": 0.0,
        "right_motor_force": 0.0,
        "left_wheel_slip": 0.0,
        "right_wheel_slip": 0.0,
        "traction_total": 0.0
    }
    
    # Track violating samples with details
    violating_indices = []
    slip_points = []  # Detailed slip information
    
    for i, s in enumerate(samples):
        violations = model.check_constraints(s["vl"], s["vr"], s["al"], s["ar"], apply_headroom)
        
        # Update max violations
        for key in max_violations:
            if violations[key] > max_violations[key]:
                max_violations[key] = violations[key]
        
        # Check for any violation
        has_violation = any(violations[k] > 1e-6 for k in ["left_motor_violation", "right_motor_violation", "traction_violation"])
        has_slip = violations["left_wheel_slip"] > 1e-6 or violations["right_wheel_slip"] > 1e-6
        
        if has_violation:
            violating_indices.append(i)
        
        if has_slip:
            slip_points.append({
                "index": i,
                "time": s["t"],
                "x": s["x"],
                "y": s["y"],
                "heading": s["heading"],
                "left_wheel_slip_N": violations["left_wheel_slip"],
                "right_wheel_slip_N": violations["right_wheel_slip"],
                "left_normal_force_N": violations["left_normal_force"],
                "right_normal_force_N": violations["right_normal_force"],
                "vl": s["vl"],
                "vr": s["vr"]
            })
    
    audit = {
        "left_motor_force": float(max_violations["left_motor_force"]),
        "right_motor_force": float(max_violations["right_motor_force"]),
        "left_wheel_slip": float(max_violations["left_wheel_slip"]),
        "right_wheel_slip": float(max_violations["right_wheel_slip"]),
        "traction_total": float(max_violations["traction_total"]),
        "num_violating_samples": len(violating_indices),
        "violating_sample_indices": violating_indices,
        "num_slip_points": len(slip_points),
        "slip_points": slip_points
    }
    return audit


def compute_metrics(samples):
    """Basic trajectory metrics."""
    if not samples:
        return {}

    # TODO: Collect additional detailed metrics for whitepaper:
    # - Average speed, acceleration, jerk (not just max)
    # - Speed/acceleration distributions (percentiles)
    # - Force utilization statistics (how close to motor/traction limits)
    # - Time spent in different regimes (accelerating, cruising, decelerating)
    # - Heading change statistics
    
    t = [s["t"] for s in samples]
    vl = [s["vl"] for s in samples]
    vr = [s["vr"] for s in samples]
    x = [s["x"] for s in samples]
    y = [s["y"] for s in samples]

    v_lin = [(l + r) / 2.0 for l, r in zip(vl, vr)]
    al = [s["al"] for s in samples]
    ar = [s["ar"] for s in samples]

    # Jerk: second difference of accelerations
    dt_vals = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    jerks = []
    for i in range(len(al) - 1):
        if dt_vals[i] > 1e-9:
            jerks.append((al[i + 1] - al[i]) / dt_vals[i])
            jerks.append((ar[i + 1] - ar[i]) / dt_vals[i])

    path_len = sum(
        np.hypot(x[i + 1] - x[i], y[i + 1] - y[i])
        for i in range(len(x) - 1)
    )

    return {
        "total_time_s": float(t[-1]),
        "path_length_m": float(path_len),
        "max_linear_speed_m_s": float(max(abs(v) for v in v_lin)),
        "max_wheel_speed_m_s": float(max(max(abs(v) for v in vl), max(abs(v) for v in vr))),
        "max_accel_m_s2": float(max(max(abs(a) for a in al), max(abs(a) for a in ar))),
        "max_jerk_m_s3": float(max(abs(j) for j in jerks)) if jerks else 0.0,
    }


def validate_trajectory(traj_file: str, config_file: str, apply_headroom=True):
    """
    CLI entry point: load a .traj file and a .chor config, run validation,
    and print a human-readable report.
    
    Args:
        traj_file: Path to trajectory file
        config_file: Path to config file
        apply_headroom: If True, applies safety margin for real-world tracking
    """
    # TODO: Collect comprehensive validation data for benchmarking:
    # - Store metrics, audit, errors in structured format
    # - Record validation time
    # - Track constraint violation patterns (which constraints, where, when)
    # - Store for whitepaper empirical data collection
    
    with open(traj_file, "r") as f:
        traj_data = json.load(f)
    with open(config_file, "r") as f:
        config_data = json.load(f)

    samples = traj_data["trajectory"]["samples"]
    robot_cfg = RobotConfig(config_data)

    print(f"\n=== Validation Report: {traj_file} ===")
    print(f"Samples: {len(samples)} | Config: {config_file}")
    if apply_headroom:
        print(f"Safety margins: torque_headroom={robot_cfg.torque_headroom:.2f}, speed_headroom={robot_cfg.speed_headroom:.2f}")

    # Metrics
    metrics = compute_metrics(samples)
    print("\n-- Metrics --")
    for k, v in metrics.items():
        print(f"  {k}: {v:.4f}")
    # TODO: Store metrics for benchmarking

    # Constraint audit
    audit = audit_constraints(samples, robot_cfg, apply_headroom)
    print("\n-- Constraint Audit --")
    print(f"  Violating samples: {audit['num_violating_samples']}")
    if audit["num_violating_samples"]:
        print(f"  Indices: {audit['violating_sample_indices']}")
    for label in ["left_motor_force", "right_motor_force", "traction_total"]:
        print(f"  Max {label} violation: {audit[label]:.6f} N")
    
    # Wheel slip detection
    print(f"\n-- Wheel Slip Detection --")
    print(f"  Slip points: {audit['num_slip_points']}")
    print(f"  Max left wheel slip: {audit['left_wheel_slip']:.6f} N")
    print(f"  Max right wheel slip: {audit['right_wheel_slip']:.6f} N")
    
    if audit['num_slip_points'] > 0:
        print(f"\n  Slip point details (first 5):")
        for i, sp in enumerate(audit['slip_points'][:5]):
            print(f"    [{i}] t={sp['time']:.3f}s at ({sp['x']:.3f}, {sp['y']:.3f})")
            if sp['left_wheel_slip_N'] > 1e-6:
                print(f"        Left wheel slip: {sp['left_wheel_slip_N']:.6f} N (normal: {sp['left_normal_force_N']:.2f} N)")
            if sp['right_wheel_slip_N'] > 1e-6:
                print(f"        Right wheel slip: {sp['right_wheel_slip_N']:.6f} N (normal: {sp['right_normal_force_N']:.2f} N)")
        if audit['num_slip_points'] > 5:
            print(f"    ... and {audit['num_slip_points'] - 5} more")
    # TODO: Store audit data for analysis of constraint violation patterns

    # Forward integration
    integrated, errors = forward_integrate(samples, robot_cfg, fine_dt=0.001)
    print("\n-- Forward Integration (1 ms RK4) --")
    for k, v in errors.items():
        print(f"  {k}: {v:.6f}")
    # TODO: Store integrated trajectory and errors for tracking analysis

    # Pass / fail summary
    ok = (
        errors["max_pos_error_m"] < 0.01
        and errors["final_pos_error_m"] < 0.01
        and audit["num_violating_samples"] == 0
        and audit["num_slip_points"] == 0
    )
    print(f"\n{'PASS' if ok else 'WARN'} — trajectory {'is' if ok else 'may not be'} safe to run.")
    # TODO: Store pass/fail result and summary statistics
    
    return metrics, audit, errors


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print("Usage: python validator.py <trajectory.traj> <config.chor>")
        sys.exit(1)
    validate_trajectory(sys.argv[1], sys.argv[2])
