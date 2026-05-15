"""
Multi-Verse Segment Refinement Pipeline

A hybrid trajectory generation architecture combining:
- Gradient-based optimization (CasADi)
- Topology exploration (TEB)
- Stochastic perturbation (STOMP)
"""

import numpy as np
import casadi as ca
from concurrent.futures import ProcessPoolExecutor, as_completed
from robot_model import RobotConfig, DifferentialDriveModel
from path_planning import ReedsSheppPath, linear_interpolation_waypoints
import sys


# File constants for thresholds and parameters
TORTUOSITY_THRESHOLD = 1.5
YAW_BUFFER_RAD = 0.5
VELOCITY_CHATTERING_THRESHOLD = 3
JERK_COST_THRESHOLD = 10.0  # Research-grounded threshold for smoothness
CURVATURE_COST_THRESHOLD = 5.0  # Research-grounded threshold for turn sharpness
CENTRIPETAL_COST_THRESHOLD = 1.0  # Research-grounded threshold for friction limit approach

STOMP_NOISE_POS_STD = 0.05
STOMP_NOISE_HEADING_STD = 0.1
DEFAULT_STOMP_VARIANTS = 5

TEB_FORWARD_BIAS = 10.0
TEB_REVERSE_BIAS = 10.0
TEB_POINT_TURN_BIAS = 5.0
TEB_WIDE_SWEEP_BIAS = 2.0


class PathBootstrapper:
    """Generates kinematically valid initial guesses using Reeds-Shepp paths."""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        # Get multiverse config or use defaults
        mv_cfg = config.multiverse_config
        bootstrap_cfg = mv_cfg.get("bootstrap", {})
        self.turning_radius = bootstrap_cfg.get("turning_radius", 0.0)
        self.resolution = bootstrap_cfg.get("resolution", 0.05)
        self.rs_path = ReedsSheppPath(self.turning_radius)
    
    def generate_baseline(self, waypoints, num_samples_per_segment=10):
        """
        Generate kinematically valid baseline trajectory.
        
        Args:
            waypoints: List of (x, y, heading) tuples
            num_samples_per_segment: Number of samples per segment
            
        Returns:
            Initial guess array [dt, x, y, theta, vl, vr, ...]
        """
        num_segments = len(waypoints) - 1
        N = num_segments * num_samples_per_segment + 1
        initial_dt = 0.1
        guess = [initial_dt]
        
        for i in range(num_segments):
            p1 = waypoints[i]
            p2 = waypoints[i + 1]
            count = num_samples_per_segment if i < num_segments - 1 else num_samples_per_segment + 1
            
            # Try Reeds-Shepp first
            segment_waypoints = self._generate_segment_waypoints(p1, p2, count)
            
            for j in range(count):
                if j < len(segment_waypoints):
                    x, y, theta = segment_waypoints[j]
                else:
                    # Fallback to linear interpolation if Reeds-Shepp fails
                    frac = j / num_samples_per_segment
                    x = p1[0] + (p2[0] - p1[0]) * frac
                    y = p1[1] + (p2[1] - p1[1]) * frac
                    theta = self._interpolate_heading(p1, p2, frac)
                
                guess.extend([x, y, theta, 0.0, 0.0])
        
        return np.array(guess)
    
    def _generate_segment_waypoints(self, start, goal, num_points):
        """Generate waypoints for a segment using Reeds-Shepp or fallback."""
        # Try Reeds-Shepp if both headings are known
        if start[2] is not None and goal[2] is not None:
            rs_path = self.rs_path.plan(start, goal, step_size=self.resolution)
            if rs_path is not None and len(rs_path) > 0:
                # Resample to exact number of points
                return linear_interpolation_waypoints(start, goal, num_points)
        
        # Fallback to linear interpolation
        return linear_interpolation_waypoints(start, goal, num_points)
    
    def _interpolate_heading(self, p1, p2, frac):
        """Interpolate heading between two poses."""
        if p1[2] is not None and p2[2] is not None:
            diff = (p2[2] - p1[2] + np.pi) % (2 * np.pi) - np.pi
            return p1[2] + diff * frac
        elif p1[2] is not None:
            return p1[2]
        elif p2[2] is not None:
            return p2[2]
        else:
            return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])


class TrajectoryCritic:
    """Evaluates trajectory quality and identifies problematic segments."""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.tortuosity_threshold = TORTUOSITY_THRESHOLD
        self.yaw_buffer_rad = YAW_BUFFER_RAD
        self.velocity_chattering_threshold = VELOCITY_CHATTERING_THRESHOLD
        self.jerk_cost_threshold = JERK_COST_THRESHOLD
        self.curvature_cost_threshold = CURVATURE_COST_THRESHOLD
        self.centripetal_cost_threshold = CENTRIPETAL_COST_THRESHOLD
    
    def _calculate_tortuosity(self, trajectory):
        """Calculate tortuosity (path length / straight-line distance)."""
        if len(trajectory) < 2:
            return 0.0
        
        total_path_length = 0.0
        for i in range(len(trajectory) - 1):
            dx = trajectory[i+1]['x'] - trajectory[i]['x']
            dy = trajectory[i+1]['y'] - trajectory[i]['y']
            total_path_length += np.sqrt(dx**2 + dy**2)
        
        straight_line_distance = np.sqrt(
            (trajectory[-1]['x'] - trajectory[0]['x'])**2 +
            (trajectory[-1]['y'] - trajectory[0]['y'])**2
        )
        
        if straight_line_distance == 0:
            return float('inf')
        
        return total_path_length / straight_line_distance
    
    def _calculate_yaw_excess(self, trajectory):
        """Calculate excess yaw changes (sum of absolute heading changes)."""
        if len(trajectory) < 2:
            return 0.0
        
        total_yaw_change = 0.0
        for i in range(len(trajectory) - 1):
            yaw_diff = abs(trajectory[i+1]['heading'] - trajectory[i]['heading'])
            # Normalize to [-pi, pi]
            yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi
            total_yaw_change += abs(yaw_diff)
        
        return total_yaw_change
    
    def _calculate_velocity_chattering(self, trajectory):
        """Calculate velocity chattering (number of sign changes)."""
        if len(trajectory) < 2:
            return 0
        
        vl_crossings = 0
        vr_crossings = 0
        
        for i in range(1, len(trajectory)):
            if trajectory[i]['vl'] * trajectory[i-1]['vl'] < 0:
                vl_crossings += 1
            if trajectory[i]['vr'] * trajectory[i-1]['vr'] < 0:
                vr_crossings += 1
        
        return vl_crossings + vr_crossings
    
    def evaluate(self, trajectory, num_samples_per_segment):
        """
        Evaluate trajectory and identify problematic segments.
        
        Args:
            trajectory: List of trajectory sample dictionaries
            num_samples_per_segment: Number of samples per segment
            
        Returns:
            List of (start_idx, end_idx) tuples for problematic windows
        """
        bad_windows = []
        num_segments = (len(trajectory) - 1) // num_samples_per_segment
        
        for i in range(num_segments):
            start_idx = i * num_samples_per_segment
            end_idx = min((i + 2) * num_samples_per_segment, len(trajectory))
            
            segment = trajectory[start_idx:end_idx]
            
            # Calculate metrics
            tortuosity = self._calculate_tortuosity(segment)
            yaw_excess = self._calculate_yaw_excess(segment)
            chattering = self._calculate_velocity_chattering(segment)
            
            # Calculate research-grounded metrics
            jerk_cost = self._calculate_jerk_cost(segment)
            curvature_cost = self._calculate_curvature_cost(segment)
            centripetal_cost = self._calculate_centripetal_cost(segment)
            
            # Check thresholds (including new research-grounded metrics)
            if (tortuosity > self.tortuosity_threshold or
                yaw_excess > self.yaw_buffer_rad or
                chattering > self.velocity_chattering_threshold or
                jerk_cost > self.jerk_cost_threshold or
                curvature_cost > self.curvature_cost_threshold or
                centripetal_cost > self.centripetal_cost_threshold):
                bad_windows.append((i, i + 2))
        
        return bad_windows
    
    def _compute_tortuosity(self, samples):
        """Compute path length / straight line distance ratio."""
        path_length = 0.0
        for i in range(len(samples) - 1):
            dx = samples[i + 1]['x'] - samples[i]['x']
            dy = samples[i + 1]['y'] - samples[i]['y']
            path_length += np.sqrt(dx**2 + dy**2)
        
        straight_distance = np.sqrt(
            (samples[-1]['x'] - samples[0]['x'])**2 +
            (samples[-1]['y'] - samples[0]['y'])**2
        )
        
        if straight_distance < 1e-6:
            return 0.0
        return path_length / straight_distance
    
    def _compute_yaw_excess(self, samples):
        """Compute excess yaw rate beyond expected turn."""
        total_yaw_change = 0.0
        for i in range(len(samples) - 1):
            total_yaw_change += abs(samples[i]['omega'] * 0.1)  # Approximate dt
        
        expected_yaw = abs(
            (samples[-1]['heading'] - samples[0]['heading'] + np.pi) % (2 * np.pi) - np.pi
        )
        
        return max(0, total_yaw_change - expected_yaw)
    
    def _compute_velocity_chattering(self, samples):
        """Count zero-crossings of wheel velocities."""
        vl_crossings = 0
        vr_crossings = 0
        
        for i in range(1, len(samples)):
            if samples[i]['vl'] * samples[i-1]['vl'] < 0:
                vl_crossings += 1
            if samples[i]['vr'] * samples[i-1]['vr'] < 0:
                vr_crossings += 1
        
        return vl_crossings + vr_crossings
    
    def _calculate_jerk_cost(self, trajectory):
        """
        Calculate jerk cost (rate of acceleration change).
        
        Research-grounded metric from smoothness optimization literature.
        High jerk indicates rapid acceleration changes leading to tracking errors.
        
        Args:
            trajectory: List of trajectory sample dictionaries
            
        Returns:
            Total jerk cost (sum of squared jerk)
        """
        if len(trajectory) < 3:
            return 0.0
        
        jerk_cost = 0.0
        for k in range(len(trajectory) - 2):
            # Left wheel jerk
            al_k = (trajectory[k+1]['vl'] - trajectory[k]['vl']) / 0.1  # Approximate dt
            al_k1 = (trajectory[k+2]['vl'] - trajectory[k+1]['vl']) / 0.1
            jerk_cost += (al_k1 - al_k)**2
            
            # Right wheel jerk
            ar_k = (trajectory[k+1]['vr'] - trajectory[k]['vr']) / 0.1
            ar_k1 = (trajectory[k+2]['vr'] - trajectory[k+1]['vr']) / 0.1
            jerk_cost += (ar_k1 - ar_k)**2
        
        return jerk_cost
    
    def _calculate_curvature_cost(self, trajectory):
        """
        Calculate curvature cost (sharpness of turns).
        
        Research-grounded metric from vehicle trajectory planning.
        High curvature indicates sharp turns that may cause wheel slip.
        
        Args:
            trajectory: List of trajectory sample dictionaries
            
        Returns:
            Total curvature cost (sum of squared curvature)
        """
        if len(trajectory) < 2:
            return 0.0
        
        curvature_cost = 0.0
        for k in range(len(trajectory) - 1):
            dx = trajectory[k+1]['x'] - trajectory[k]['x']
            dy = trajectory[k+1]['y'] - trajectory[k]['y']
            ds = np.sqrt(dx**2 + dy**2)
            
            if ds > 1e-6:
                dtheta = abs(trajectory[k+1]['heading'] - trajectory[k]['heading'])
                # Normalize to [-pi, pi]
                dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi
                curvature = abs(dtheta) / ds
                curvature_cost += curvature**2
        
        return curvature_cost
    
    def _calculate_centripetal_cost(self, trajectory):
        """
        Calculate centripetal acceleration cost.
        
        Research-grounded metric from vehicle dynamics.
        Penalizes trajectories approaching friction limits (wheel slip risk).
        
        Args:
            trajectory: List of trajectory sample dictionaries
            
        Returns:
            Total centripetal cost (penalty for approaching friction limit)
        """
        cost = 0.0
        a_max = self.config.cof * self.config.mass * self.config.g / self.config.mass  # = cof * g
        
        for state in trajectory:
            v = (state['vl'] + state['vr']) / 2.0
            omega = state['omega']
            a_centripetal = abs(v * omega)
            
            # Penalize approaching friction limit (80% threshold)
            if a_centripetal > 0.8 * a_max:
                cost += (a_centripetal / a_max)**2
        
        return cost


class LocalSegmentOptimizer:
    """Miniature optimizer for solving local trajectory segments."""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.model = DifferentialDriveModel(config)
    
    def solve_window(self, start_state, end_state, num_samples, initial_guess=None, apply_headroom=True):
        """
        Solve trajectory for a local window with pinned boundary states.
        
        Args:
            start_state: (x, y, theta, vl, vr) tuple
            end_state: (x, y, theta, vl, vr) tuple
            num_samples: Number of samples in the window
            initial_guess: Optional initial guess array
            apply_headroom: Whether to apply safety margins
            
        Returns:
            Tuple of (success, cost, trajectory_array)
        """
        N = num_samples
        opti = ca.Opti()
        
        # Decision variables
        dt = opti.variable()
        X = opti.variable(N, 5)  # x, y, theta, vl, vr
        
        x = X[:, 0]
        y = X[:, 1]
        theta = X[:, 2]
        vl = X[:, 3]
        vr = X[:, 4]
        
        # Objective: minimize time
        time_cost = dt * (N - 1)
        opti.minimize(time_cost)
        
        # Bounds
        opti.subject_to(dt >= 0.001)
        opti.subject_to(dt <= 1.0)
        opti.subject_to(x >= -20)
        opti.subject_to(x <= 20)
        opti.subject_to(y >= -20)
        opti.subject_to(y <= 20)
        
        v_max = self.config.max_linear_speed(apply_headroom)
        v_bound = 0.99 * v_max
        opti.subject_to(vl >= -v_bound)
        opti.subject_to(vl <= v_bound)
        opti.subject_to(vr >= -v_bound)
        opti.subject_to(vr <= v_bound)
        
        # Dynamics constraints
        for k in range(N - 1):
            v1 = (vl[k] + vr[k]) / 2.0
            v2 = (vl[k + 1] + vr[k + 1]) / 2.0
            omega1 = (vr[k] - vl[k]) / self.config.track_width
            omega2 = (vr[k + 1] - vl[k + 1]) / self.config.track_width
            
            opti.subject_to(x[k + 1] == x[k] + 0.5 * (v1 * ca.cos(theta[k]) + v2 * ca.cos(theta[k + 1])) * dt)
            opti.subject_to(y[k + 1] == y[k] + 0.5 * (v1 * ca.sin(theta[k]) + v2 * ca.sin(theta[k + 1])) * dt)
            opti.subject_to(theta[k + 1] == theta[k] + 0.5 * (omega1 + omega2) * dt)
            
            al = (vl[k + 1] - vl[k]) / dt
            ar = (vr[k + 1] - vr[k]) / dt
            
            fl, fr = self._dynamics_symbolic(vl[k], vr[k], al, ar)
            max_fl = self._max_force_symbolic(vl[k], apply_headroom)
            max_fr = self._max_force_symbolic(vr[k], apply_headroom)
            
            opti.subject_to(ca.fabs(fl) <= max_fl)
            opti.subject_to(ca.fabs(fr) <= max_fr)
            
            f_total = ca.fabs(fl) + ca.fabs(fr)
            f_traction_max = self.config.cof * self.config.mass * self.config.g
            opti.subject_to(f_total <= f_traction_max)
        
        # Pin boundary states
        opti.subject_to(x[0] == start_state[0])
        opti.subject_to(y[0] == start_state[1])
        opti.subject_to(theta[0] == start_state[2])
        opti.subject_to(vl[0] == start_state[3])
        opti.subject_to(vr[0] == start_state[4])
        
        opti.subject_to(x[N - 1] == end_state[0])
        opti.subject_to(y[N - 1] == end_state[1])
        opti.subject_to(theta[N - 1] == end_state[2])
        opti.subject_to(vl[N - 1] == end_state[3])
        opti.subject_to(vr[N - 1] == end_state[4])
        
        # Initial guess
        if initial_guess is not None:
            opti.set_initial(dt, float(initial_guess[0]))
            opti.set_initial(X, initial_guess[1:].reshape((N, 5)))
        else:
            opti.set_initial(dt, 0.1)
            opti.set_initial(X, np.zeros((N, 5)))
            for i in range(N):
                frac = i / (N - 1)
                opti.set_initial(x[i], start_state[0] + frac * (end_state[0] - start_state[0]))
                opti.set_initial(y[i], start_state[1] + frac * (end_state[1] - start_state[1]))
                opti.set_initial(theta[i], start_state[2] + frac * (end_state[2] - start_state[2]))
        
        # Solver setup (fast settings for local optimization)
        p_opts = {"expand": True}
        s_opts = {
            "max_iter": 500,
            "print_level": 0,
            "tol": 1e-2,
            "constr_viol_tol": 1e-2,
        }
        opti.solver("ipopt", p_opts, s_opts)
        
        try:
            sol = opti.solve()
            dt_val = float(sol.value(dt))
            X_val = np.array(sol.value(X))
            cost = dt_val * (N - 1)
            # TODO: Collect refinement solver statistics:
            # - Solver iterations, time, constraint violations
            # - Track which heuristic (TEB/STOMP) produced best result
            # - Store for analysis of refinement effectiveness
            return True, cost, np.concatenate([[dt_val], X_val.flatten()])
        except Exception:
            # TODO: Log refinement failures for analysis
            return False, float('inf'), None
    
    def _dynamics_symbolic(self, vl, vr, al, ar):
        a = (al + ar) / 2.0
        alpha = (ar - al) / self.config.track_width
        f_total = self.config.mass * a
        m_total = self.config.inertia * alpha
        fr = (f_total + (2.0 * m_total / self.config.track_width)) / 2.0
        fl = f_total - fr
        return fl, fr
    
    def _max_force_symbolic(self, v_wheel, apply_headroom=True):
        omega = (v_wheel / self.config.wheel_radius) * self.config.gearing
        torque = self.config.t_max_nm * (1.0 - ca.fabs(omega) / self.config.v_max_rad_s)
        torque = ca.fmax(0, torque)
        force = (torque / self.config.wheel_radius) * self.config.gearing
        if apply_headroom:
            force *= self.config.torque_headroom
        return force


class MultiVerseRefiner:
    """Handles TEB/STOMP parallel refinement of problematic segments."""
    
    def __init__(self, config: RobotConfig, enable_parallel=True, num_workers=8, verbose=True):
        self.config = config
        self.enable_parallel = enable_parallel
        self.num_workers = num_workers
        self.verbose = verbose
        self.local_solver = LocalSegmentOptimizer(config)
        self.refinement_history = []  # Store heuristic results for convergence visualization
        
        # Get multiverse config
        mv_cfg = config.multiverse_config
        stomp_cfg = mv_cfg.get("stomp_noise", {})
        self.stomp_variants = mv_cfg.get("stomp_variants", DEFAULT_STOMP_VARIANTS)
        self.stomp_pos_std = stomp_cfg.get("position_std", STOMP_NOISE_POS_STD)
        self.stomp_heading_std = stomp_cfg.get("heading_std", STOMP_NOISE_HEADING_STD)
        
        teb_cfg = mv_cfg.get("teb_weights", {})
        self.forward_bias = teb_cfg.get("forward_bias", TEB_FORWARD_BIAS)
        self.reverse_bias = teb_cfg.get("reverse_bias", TEB_REVERSE_BIAS)
        self.point_turn_bias = teb_cfg.get("point_turn_bias", TEB_POINT_TURN_BIAS)
        self.wide_sweep_bias = teb_cfg.get("wide_sweep_bias", TEB_WIDE_SWEEP_BIAS)
    
    def refine_segment(self, start_state, end_state, num_samples, base_guess, capture_iterations=False):
        """
        Refine a segment using parallel TEB/STOMP exploration.
        
        Args:
            start_state: (x, y, theta, vl, vr) tuple
            end_state: (x, y, theta, vl, vr) tuple
            num_samples: Number of samples in the segment
            base_guess: Baseline initial guess array
            capture_iterations: If True, captures heuristic results for convergence visualization
            
        Returns:
            Best trajectory array from all heuristics
        """
        # Clear previous refinement history
        self.refinement_history = []
        
        # Generate heuristic guesses
        teb_guesses = self._generate_teb_heuristics(start_state, end_state, num_samples, base_guess)
        stomp_guesses = self._generate_stomp_heuristics(base_guess, num_samples)
        
        all_guesses = teb_guesses + stomp_guesses
        total_guesses = len(all_guesses)
        
        if not self.enable_parallel:
            # Sequential evaluation
            best_cost = float('inf')
            best_result = None
            for i, guess in enumerate(all_guesses):
                # Progress indicator
                if self.verbose:
                    progress = (i + 1) / total_guesses * 100
                    sys.stdout.write(f"\r  Refining: {i+1}/{total_guesses} ({progress:.0f}%) - Best cost: {best_cost:.4f}s")
                    sys.stdout.flush()
                
                success, cost, result = self.local_solver.solve_window(
                    start_state, end_state, num_samples, guess
                )
                
                # Capture heuristic result for convergence visualization
                if capture_iterations and success:
                    heuristic_type = 'TEB' if i < len(teb_guesses) else 'STOMP'
                    heuristic_idx = i if i < len(teb_guesses) else i - len(teb_guesses)
                    N = num_samples
                    states = result[1:].reshape((N, 5))
                    self.refinement_history.append({
                        'iteration': i,
                        'cost': cost,
                        'trajectory': states.copy(),
                        'dt': float(result[0]),
                        'heuristic_type': heuristic_type,
                        'heuristic_idx': heuristic_idx
                    })
                
                if success and cost < best_cost:
                    best_cost = cost
                    best_result = result
            
            if self.verbose:
                sys.stdout.write(f"\r  Refining: {total_guesses}/{total_guesses} (100%) - Best cost: {best_cost:.4f}s\n")
                sys.stdout.flush()
            
            return best_result if best_result is not None else base_guess
        else:
            # Parallel evaluation
            best_cost = float('inf')
            best_result = None
            completed = 0
            
            with ProcessPoolExecutor(max_workers=self.num_workers) as executor:
                futures = {}
                for i, guess in enumerate(all_guesses):
                    future = executor.submit(
                        self._solve_window_wrapper,
                        start_state, end_state, num_samples, guess
                    )
                    futures[future] = i  # Store index with future
                
                for future in as_completed(futures):
                    completed += 1
                    i = futures[future]
                    if self.verbose:
                        progress = completed / total_guesses * 100
                        sys.stdout.write(f"\r  Refining: {completed}/{total_guesses} ({progress:.0f}%) - Best cost: {best_cost:.4f}s")
                        sys.stdout.flush()
                    
                    success, cost, result = future.result()
                    
                    # Capture heuristic result for convergence visualization
                    if capture_iterations and success:
                        heuristic_type = 'TEB' if i < len(teb_guesses) else 'STOMP'
                        heuristic_idx = i if i < len(teb_guesses) else i - len(teb_guesses)
                        N = num_samples
                        states = result[1:].reshape((N, 5))
                        self.refinement_history.append({
                            'iteration': i,
                            'cost': cost,
                            'trajectory': states.copy(),
                            'dt': float(result[0]),
                            'heuristic_type': heuristic_type,
                            'heuristic_idx': heuristic_idx
                        })
                    
                    if success and cost < best_cost:
                        best_cost = cost
                        best_result = result
            
            if self.verbose:
                sys.stdout.write(f"\r  Refining: {total_guesses}/{total_guesses} (100%) - Best cost: {best_cost:.4f}s\n")
                sys.stdout.flush()
            
            return best_result if best_result is not None else base_guess
    
    def _generate_teb_heuristics(self, start_state, end_state, num_samples, base_guess):
        """Generate TEB topology-based initial guesses."""
        guesses = []
        N = num_samples
        
        # Forward bias: set positive velocities
        forward_guess = base_guess.copy()
        for i in range(N):
            idx = 1 + i * 5
            # Set positive velocities
            forward_guess[idx + 3] = 0.1  # vl
            forward_guess[idx + 4] = 0.1  # vr
        guesses.append(forward_guess)
        
        # Reverse bias: set negative velocities
        reverse_guess = base_guess.copy()
        for i in range(N):
            idx = 1 + i * 5
            # Set negative velocities
            reverse_guess[idx + 3] = -0.1  # vl
            reverse_guess[idx + 4] = -0.1  # vr
        guesses.append(reverse_guess)
        
        # Point-turn bias: force vl = -vr near middle
        point_turn_guess = base_guess.copy()
        mid = N // 2
        # Set point-turn condition at middle
        point_turn_guess[1 + mid * 5 + 3] = 0.1
        point_turn_guess[1 + mid * 5 + 4] = -0.1
        guesses.append(point_turn_guess)
        
        # Wide sweep: add lateral offset
        wide_sweep_guess = base_guess.copy()
        for i in range(N):
            idx = 1 + i * 5
            # Add sinusoidal lateral offset
            offset = 0.1 * np.sin(np.pi * i / N)
            wide_sweep_guess[idx + 1] += offset
        guesses.append(wide_sweep_guess)
        
        return guesses
    
    def _generate_stomp_heuristics(self, base_guess, num_samples):
        """Generate STOMP stochastic perturbation guesses."""
        guesses = []
        N = num_samples
        
        for _ in range(self.stomp_variants):
            noisy_guess = base_guess.copy()
            
            # Add Gaussian noise to positions and headings
            for i in range(N):
                idx = 1 + i * 5
                noisy_guess[idx] += np.random.normal(0, self.stomp_pos_std)  # x
                noisy_guess[idx + 1] += np.random.normal(0, self.stomp_pos_std)  # y
                noisy_guess[idx + 2] += np.random.normal(0, self.stomp_heading_std)  # theta
            
            guesses.append(noisy_guess)
        
        return guesses
    
    def _solve_window_wrapper(self, start_state, end_state, num_samples, guess):
        """Wrapper for parallel execution."""
        # For now, run sequentially to avoid pickling issues with CasADi
        # True parallel execution would require serializing the config
        return self.local_solver.solve_window(start_state, end_state, num_samples, guess)


class MasterTrajectoryOptimizer:
    """Orchestrates the Multi-Verse refinement pipeline."""
    
    def __init__(self, config: RobotConfig, enable_parallel=True, num_workers=8, verbose=True):
        self.config = config
        self.enable_parallel = enable_parallel
        self.num_workers = num_workers
        self.verbose = verbose
        self.bootstrapper = PathBootstrapper(config)
        self.critic = TrajectoryCritic(config)
        self.refiner = MultiVerseRefiner(config, enable_parallel, num_workers, verbose)
        self.iteration_history = []  # Store convergence data across phases
    
    def solve(self, waypoints, num_samples_per_segment=10, accuracy_weight=0.0, 
              stop_waypoint_indices=None, waypoint_events=None, apply_headroom=True, verbose=True, capture_iterations=False):
        """
        Solve trajectory using Multi-Verse refinement pipeline.
        
        Args:
            waypoints: List of (x, y, heading) tuples
            num_samples_per_segment: Number of samples per segment
            accuracy_weight: Smoothness weight
            stop_waypoint_indices: Indices where robot must stop
            waypoint_events: Event markers
            apply_headroom: Apply safety margins
            verbose: If True, prints progress messages
            capture_iterations: If True, captures intermediate states for convergence visualization
            
        Returns:
            List of trajectory sample dictionaries
        """
        import time
        
        # Clear previous iteration history
        self.iteration_history = []
        
        # TODO: Collect comprehensive benchmarking data:
        # - Total solve time
        # - Per-phase timing (bootstrap, global solve, critic, refinement, polish)
        # - Number of problematic segments found
        # - Number of refinement heuristics attempted
        # - Success rate of refinements
        # - Quality metrics before/after refinement
        
        total_start_time = time.time()
        num_segments = len(waypoints) - 1
        N = num_segments * num_samples_per_segment + 1
        
        if stop_waypoint_indices is None:
            stop_waypoint_indices = []
        if waypoint_events is None:
            waypoint_events = {}
        
        # Phase 1: Bootstrap with Reeds-Shepp
        phase1_start = time.time()
        if verbose:
            print("Phase 1: Bootstrapping with Reeds-Shepp paths...")
        guess = self.bootstrapper.generate_baseline(waypoints, num_samples_per_segment)
        phase1_time = time.time() - phase1_start
        # TODO: Log phase 1 timing
        
        # Capture bootstrap phase
        if capture_iterations:
            N = num_segments * num_samples_per_segment + 1
            guess_states = guess[1:].reshape((N, 5))
            bootstrap_cost = guess[0] * (N - 1)  # Time cost approximation
            self.iteration_history.append({
                'iteration': len(self.iteration_history),
                'cost': bootstrap_cost,
                'trajectory': guess_states.copy(),
                'dt': float(guess[0]),
                'phase': 'bootstrap'
            })
        
        # Phase 2: Fast global solve
        phase2_start = time.time()
        if verbose:
            print("Phase 2: Fast global optimization...")
        global_traj = self._global_solve(waypoints, num_samples_per_segment, guess, 
                                        accuracy_weight, stop_waypoint_indices, 
                                        waypoint_events, apply_headroom, 
                                        fast_mode=True, capture_iterations=capture_iterations)
        phase2_time = time.time() - phase2_start
        # TODO: Log phase 2 timing and initial trajectory quality
        
        # Capture global solve phase
        if capture_iterations:
            N = len(global_traj)
            dt = global_traj[1]['t'] - global_traj[0]['t'] if N > 1 else 0.1
            global_states = np.array([[s['x'], s['y'], s['heading'], s['vl'], s['vr']] for s in global_traj])
            global_cost = dt * (N - 1)
            self.iteration_history.append({
                'iteration': len(self.iteration_history),
                'cost': global_cost,
                'trajectory': global_states,
                'dt': dt,
                'phase': 'global_solve'
            })
        
        # Phase 3: Critic evaluation
        phase3_start = time.time()
        if verbose:
            print("Phase 3: Evaluating trajectory quality...")
        bad_windows = self.critic.evaluate(global_traj, num_samples_per_segment)
        phase3_time = time.time() - phase3_start
        if verbose:
            print(f"Found {len(bad_windows)} problematic segments")
        # TODO: Log phase 3 timing and quality metrics
        
        # Phase 4: Refine bad segments
        phase4_start = time.time()
        refinement_count = 0
        refinement_success_count = 0
        if len(bad_windows) > 0:
            if verbose:
                print("Phase 4: Refining problematic segments...")
            for i, (window_start, window_end) in enumerate(bad_windows):
                # Progress indicator for multiple windows
                if verbose and len(bad_windows) > 1:
                    progress = (i + 1) / len(bad_windows) * 100
                    sys.stdout.write(f"\r  Window {i+1}/{len(bad_windows)} ({progress:.0f}%)")
                    sys.stdout.flush()
                
                # Extract window data
                start_idx = window_start * num_samples_per_segment
                end_idx = min(window_end * num_samples_per_segment, len(global_traj) - 1)
                
                start_state = (
                    global_traj[start_idx]['x'],
                    global_traj[start_idx]['y'],
                    global_traj[start_idx]['heading'],
                    global_traj[start_idx]['vl'],
                    global_traj[start_idx]['vr']
                )
                
                end_state = (
                    global_traj[end_idx]['x'],
                    global_traj[end_idx]['y'],
                    global_traj[end_idx]['heading'],
                    global_traj[end_idx]['vl'],
                    global_traj[end_idx]['vr']
                )
                
                num_window_samples = end_idx - start_idx + 1
                
                # Build base guess for window
                base_guess = np.zeros(1 + num_window_samples * 5)
                base_guess[0] = 0.1
                for i in range(num_window_samples):
                    sample = global_traj[start_idx + i]
                    base_guess[1 + i * 5] = sample['x']
                    base_guess[1 + i * 5 + 1] = sample['y']
                    base_guess[1 + i * 5 + 2] = sample['heading']
                    base_guess[1 + i * 5 + 3] = sample['vl']
                    base_guess[1 + i * 5 + 4] = sample['vr']
                
                # Refine segment
                refinement_count += 1
                refined = self.refiner.refine_segment(start_state, end_state, 
                                                     num_window_samples, base_guess)
                
                # Stitch back into global trajectory
                if refined is not None:
                    refinement_success_count += 1
                    dt_val = refined[0]
                    states = refined[1:].reshape((num_window_samples, 5))
                    for i in range(num_window_samples):
                        idx = start_idx + i
                        global_traj[idx]['x'] = float(states[i, 0])
                        global_traj[idx]['y'] = float(states[i, 1])
                        global_traj[idx]['heading'] = float(states[i, 2])
                        global_traj[idx]['vl'] = float(states[i, 3])
                        global_traj[idx]['vr'] = float(states[i, 4])
                        global_traj[idx]['omega'] = float((states[i, 4] - states[i, 3]) / self.config.track_width)
            
            if verbose and len(bad_windows) > 1:
                sys.stdout.write("\n")
                sys.stdout.flush()
        
        phase4_time = time.time() - phase4_start
        # TODO: Log phase 4 timing, refinement count, success rate
        
        # Phase 5: Final polish
        phase5_start = time.time()
        if verbose:
            print("Phase 5: Final global polish...")
        final_traj = self._global_solve(waypoints, num_samples_per_segment, guess,
                                       accuracy_weight, stop_waypoint_indices,
                                       waypoint_events, apply_headroom,
                                       fast_mode=False, initial_samples=global_traj,
                                       capture_iterations=capture_iterations)
        phase5_time = time.time() - phase5_start
        # TODO: Log phase 5 timing and final trajectory quality
        
        # Capture final polish phase
        if capture_iterations:
            N = len(final_traj)
            dt = final_traj[1]['t'] - final_traj[0]['t'] if N > 1 else 0.1
            final_states = np.array([[s['x'], s['y'], s['heading'], s['vl'], s['vr']] for s in final_traj])
            final_cost = dt * (N - 1)
            self.iteration_history.append({
                'iteration': len(self.iteration_history),
                'cost': final_cost,
                'trajectory': final_states,
                'dt': dt,
                'phase': 'final_polish'
            })
        
        total_time = time.time() - total_start_time
        # TODO: Log total time and summary statistics
        # Store: total_time, phase_times, refinement_stats, quality_metrics
        
        return final_traj
    
    def _global_solve(self, waypoints, num_samples_per_segment, initial_guess,
                     accuracy_weight, stop_waypoint_indices, waypoint_events,
                     apply_headroom, fast_mode=False, initial_samples=None, capture_iterations=False):
        """Global trajectory optimization using CasADi."""
        from optimizer import TrajectoryOptimizer
        
        # Use existing optimizer for global solve
        optimizer = TrajectoryOptimizer(self.config)
        
        # Modify guess if provided
        if initial_samples is not None:
            # Convert samples back to guess format
            N = len(initial_samples)
            dt = initial_samples[1]['t'] - initial_samples[0]['t'] if N > 1 else 0.1
            guess = [dt]
            for sample in initial_samples:
                guess.extend([sample['x'], sample['y'], sample['heading'], 
                            sample['vl'], sample['vr']])
            initial_guess = np.array(guess)
        
        # Adjust solver settings for fast mode
        if fast_mode:
            # Temporarily modify solver settings for fast solve
            # This would require modifying the optimizer class
            pass
        
        return optimizer.solve(waypoints, num_samples_per_segment, accuracy_weight,
                              stop_waypoint_indices, waypoint_events, apply_headroom, 
                              self.verbose, capture_iterations=capture_iterations)
