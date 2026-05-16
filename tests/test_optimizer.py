"""
Unit tests for optimizer.py
"""

import pytest
import numpy as np
from robot_model import RobotConfig
from optimizer import TrajectoryOptimizer


class TestTrajectoryOptimizer:
    """Test cases for TrajectoryOptimizer class."""

    def test_init(self):
        """Test optimizer initialization."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        assert optimizer.config == config
        assert optimizer.model is not None
        assert optimizer.iteration_history == []

    def test_solve_simple_straight_line(self):
        """Test optimization for simple straight line trajectory."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Simple straight line: (0, 0, 0) to (1, 0, 0)
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5, 
                                  accuracy_weight=0.0, verbose=False)
        
        assert len(samples) > 0
        assert samples[0]["x"] == pytest.approx(0.0, abs=1e-6)
        assert samples[0]["y"] == pytest.approx(0.0, abs=1e-6)
        assert samples[-1]["x"] == pytest.approx(1.0, abs=1e-6)
        assert samples[-1]["y"] == pytest.approx(0.0, abs=1e-6)
        # Start and end at rest
        assert samples[0]["vl"] == pytest.approx(0.0, abs=1e-6)
        assert samples[0]["vr"] == pytest.approx(0.0, abs=1e-6)
        assert samples[-1]["vl"] == pytest.approx(0.0, abs=1e-6)
        assert samples[-1]["vr"] == pytest.approx(0.0, abs=1e-6)

    def test_solve_with_heading_constraint(self):
        """Test optimization with heading constraints."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Path with heading constraints
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, np.pi/4)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, verbose=False)
        
        assert len(samples) > 0
        assert samples[0]["heading"] == pytest.approx(0.0, abs=1e-6)
        # Check that final heading is close to constraint
        final_heading = (samples[-1]["heading"] + np.pi) % (2 * np.pi) - np.pi
        expected_heading = (np.pi/4 + np.pi) % (2 * np.pi) - np.pi
        assert final_heading == pytest.approx(expected_heading, abs=1e-2)

    def test_solve_without_heading_constraint(self):
        """Test optimization without heading constraints."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Path with unconstrained headings
        waypoints = [(0.0, 0.0, None), (1.0, 0.5, None)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, verbose=False)
        
        assert len(samples) > 0
        assert samples[0]["x"] == pytest.approx(0.0, abs=1e-6)
        assert samples[-1]["x"] == pytest.approx(1.0, abs=1e-6)

    def test_solve_with_stop_waypoint(self):
        """Test optimization with stop waypoint constraint."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Path with stop at middle waypoint
        waypoints = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, stop_waypoint_indices=[1],
                                  verbose=False)
        
        assert len(samples) > 0
        # Check that robot stops at waypoint index 1 (sample index 5)
        stop_idx = 5
        assert samples[stop_idx]["vl"] == pytest.approx(0.0, abs=1e-3)
        assert samples[stop_idx]["vr"] == pytest.approx(0.0, abs=1e-3)

    def test_solve_with_event_markers(self):
        """Test optimization with event markers."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Path with event marker
        waypoints = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.0, 0.0)]
        waypoint_events = {1: "lower_arm"}
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, waypoint_events=waypoint_events,
                                  verbose=False)
        
        assert len(samples) > 0
        # Check that event is present at waypoint index 1 (sample index 5)
        event_idx = 5
        assert "event" in samples[event_idx]
        assert samples[event_idx]["event"] == "lower_arm"

    def test_solve_with_accuracy_weight(self):
        """Test optimization with accuracy/smoothness weight."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        # Solve with accuracy weight
        samples_smooth, _ = optimizer.solve(waypoints, num_samples_per_segment=5,
                                         accuracy_weight=1.0, verbose=False)
        
        # Solve without accuracy weight
        samples_fast, _ = optimizer.solve(waypoints, num_samples_per_segment=5,
                                       accuracy_weight=0.0, verbose=False)
        
        assert len(samples_smooth) > 0
        assert len(samples_fast) > 0
        # Smooth trajectory should have different characteristics
        # (we can't easily compare jerk without more analysis)

    def test_compute_cost(self):
        """Test cost computation."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Create simple trajectory parameters
        N = 11
        dt = 0.1
        states = np.zeros((N, 5))
        states[:, 0] = np.linspace(0, 1, N)  # x
        states[:, 3] = 0.5  # vl
        states[:, 4] = 0.5  # vr
        params = np.concatenate([[dt], states.flatten()])
        
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        cost = optimizer._compute_cost(params, N, 10, 0.0, waypoints)
        
        # Cost should be positive (time cost)
        assert cost > 0

    def test_build_initial_guess(self):
        """Test initial guess generation."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        guess = optimizer._build_initial_guess(waypoints, num_samples_per_segment=5, N=6)
        
        assert len(guess) == 1 + 6 * 5  # dt + N * 5 states
        assert guess[0] > 0  # dt should be positive
        # Check that start and end positions match waypoints
        assert guess[1] == pytest.approx(0.0, abs=1e-6)  # x[0]
        assert guess[2] == pytest.approx(0.0, abs=1e-6)  # y[0]
        assert guess[1 + 10 * 5] == pytest.approx(1.0, abs=1e-6)  # x[10]
        assert guess[2 + 10 * 5] == pytest.approx(0.0, abs=1e-6)  # y[10]

    def test_format_output(self):
        """Test output formatting."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Create simple trajectory parameters
        N = 11
        dt = 0.1
        states = np.zeros((N, 5))
        states[:, 0] = np.linspace(0, 1, N)  # x
        states[:, 3] = 0.5  # vl
        states[:, 4] = 0.5  # vr
        params = np.concatenate([[dt], states.flatten()])
        
        samples = optimizer.format_output(params, N, num_samples_per_segment=10)
        
        assert len(samples) == N
        # Check required fields
        assert "t" in samples[0]
        assert "x" in samples[0]
        assert "y" in samples[0]
        assert "heading" in samples[0]
        assert "vl" in samples[0]
        assert "vr" in samples[0]
        assert "omega" in samples[0]
        assert "al" in samples[0]
        assert "ar" in samples[0]
        assert "fl" in samples[0]
        assert "fr" in samples[0]

    def test_iteration_history_capture(self):
        """Test iteration history capture."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, capture_iterations=True,
                                  verbose=False)
        
        # Should have captured initial guess and final solution
        assert len(optimizer.iteration_history) >= 2
        assert optimizer.iteration_history[0]["phase"] == "initial_guess"
        # Note: may be 'failed_solution' if optimization doesn't converge
        assert optimizer.iteration_history[-1]["phase"] in ["final_solution", "failed_solution"]

    def test_solve_multiple_segments(self):
        """Test optimization with multiple segments."""
        config_dict = {
            "robot": {
                "mass": 0.8,
                "inertia": 0.002,
                "track_width": 0.1,
                "wheel_radius": 0.03,
                "v_max_rad_s": 15.0,
                "t_max_nm": 0.05,
                "gearing": 1.0,
                "cof": 0.5
            }
        }
        config = RobotConfig(config_dict)
        optimizer = TrajectoryOptimizer(config)
        
        # Multi-segment path
        waypoints = [(0.0, 0.0, 0.0), (0.5, 0.0, 0.0), (1.0, 0.5, np.pi/4)]
        
        samples, stats = optimizer.solve(waypoints, num_samples_per_segment=5,
                                  accuracy_weight=0.0, verbose=False)
        
        assert len(samples) > 0
        # Check waypoint constraints
        assert samples[0]["x"] == pytest.approx(0.0, abs=1e-6)
        assert samples[5]["x"] == pytest.approx(0.5, abs=1e-6)
        assert samples[10]["x"] == pytest.approx(1.0, abs=1e-6)
        assert samples[10]["y"] == pytest.approx(0.5, abs=1e-6)
