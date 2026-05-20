"""
Performance benchmarking suite for FLL Trajectory Optimizer.

Compares simple optimizer vs Multi-Verse optimizer across various test cases.
"""

import json
import time
import subprocess
import os
from pathlib import Path
from validator import validate_trajectory


class BenchmarkRunner:
    """Runs performance benchmarks comparing optimizers."""
    
    def __init__(self, config_path="robot_config.json"):
        self.config_path = config_path
        self.results = []
    
    def run_benchmark(self, waypoint_file, optimizer_type="simple", samples=10, runs=3, validate=True):
        """
        Run a single benchmark.
        
        Args:
            waypoint_file: Path to waypoint JSON file
            optimizer_type: "simple" or "multiverse"
            samples: Number of samples per segment
            runs: Number of runs to average over
            validate: If True, validate generated trajectories
            
        Returns:
            Dictionary with benchmark results
        """
        times = []
        total_times = []
        validation_results = []
        
        for run in range(runs):
            output_file = f"benchmark_{optimizer_type}_{Path(waypoint_file).stem}_run{run}.traj"
            
            # Build command
            cmd = [
                "py", "main.py",
                "-c", self.config_path,
                "-w", waypoint_file,
                "-o", output_file,
                "-n", str(samples)
            ]
            
            if optimizer_type == "simple":
                cmd.append("--simple")
            else:
                cmd.append("--no-parallel")  # Use sequential for fair comparison
            
            # Run and time
            start_time = time.time()
            result = subprocess.run(cmd, capture_output=True, text=True)
            elapsed = time.time() - start_time
            
            # Extract total time from output
            total_time = self._extract_total_time(result.stdout)
            
            times.append(elapsed)
            if total_time is not None:
                total_times.append(total_time)
            
            # Validate trajectory if requested
            if validate and os.path.exists(output_file):
                try:
                    metrics, audit, errors = validate_trajectory(output_file, self.config_path, apply_headroom=False)
                    validation_results.append({
                        "metrics": metrics,
                        "audit": audit,
                        "errors": errors
                    })
                except Exception as e:
                    print(f"  Warning: Validation failed for run {run}: {e}")
                    validation_results.append(None)
            
            # Clean up output file
            if os.path.exists(output_file):
                os.remove(output_file)
        
        # Calculate statistics
        avg_time = sum(times) / len(times)
        avg_total_time = sum(total_times) / len(total_times) if total_times else None
        
        # Aggregate validation results
        validation_summary = None
        if validation_results and all(v is not None for v in validation_results):
            validation_summary = self._aggregate_validation(validation_results)
        
        return {
            "waypoint_file": waypoint_file,
            "optimizer_type": optimizer_type,
            "samples": samples,
            "runs": runs,
            "avg_wall_time": avg_time,
            "avg_trajectory_time": avg_total_time,
            "min_wall_time": min(times),
            "max_wall_time": max(times),
            "validation": validation_summary
        }
    
    def _extract_total_time(self, output):
        """Extract total trajectory time from optimizer output."""
        for line in output.split('\n'):
            if "Total time:" in line:
                try:
                    # Extract time value (e.g., "Total time: 5.0166s")
                    time_str = line.split("Total time:")[1].strip().replace("s", "")
                    return float(time_str)
                except (IndexError, ValueError):
                    pass
        return None
    
    def _aggregate_validation(self, validation_results):
        """
        Aggregate validation results across multiple runs.
        
        Args:
            validation_results: List of validation result dictionaries
            
        Returns:
            Dictionary with aggregated validation statistics
        """
        aggregated = {
            "avg_max_pos_error_m": 0.0,
            "avg_rms_pos_error_m": 0.0,
            "avg_max_heading_error_rad": 0.0,
            "avg_num_violating_samples": 0,
            "avg_num_slip_points": 0,
            "avg_max_left_wheel_slip": 0.0,
            "avg_max_right_wheel_slip": 0.0,
            "pass_rate": 0.0
        }
        
        n = len(validation_results)
        pass_count = 0
        
        for v in validation_results:
            errors = v["errors"]
            audit = v["audit"]
            
            aggregated["avg_max_pos_error_m"] += errors["max_pos_error_m"]
            aggregated["avg_rms_pos_error_m"] += errors["rms_pos_error_m"]
            aggregated["avg_max_heading_error_rad"] += errors["max_heading_error_rad"]
            aggregated["avg_num_violating_samples"] += audit["num_violating_samples"]
            aggregated["avg_num_slip_points"] += audit["num_slip_points"]
            aggregated["avg_max_left_wheel_slip"] += audit["left_wheel_slip"]
            aggregated["avg_max_right_wheel_slip"] += audit["right_wheel_slip"]
            
            # Check if this run passed
            if (errors["max_pos_error_m"] < 0.01
                and errors["final_pos_error_m"] < 0.01
                and audit["num_violating_samples"] == 0
                and audit["num_slip_points"] == 0):
                pass_count += 1
        
        # Calculate averages
        for key in ["avg_max_pos_error_m", "avg_rms_pos_error_m", "avg_max_heading_error_rad",
                    "avg_max_left_wheel_slip", "avg_max_right_wheel_slip"]:
            aggregated[key] /= n
        
        aggregated["avg_num_violating_samples"] /= n
        aggregated["avg_num_slip_points"] /= n
        aggregated["pass_rate"] = pass_count / n
        
        return aggregated
    
    def run_comparison_suite(self, waypoint_files, samples=10, runs=3, validate=True):
        """
        Run full comparison suite across all waypoint files.
        
        Args:
            waypoint_files: List of waypoint file paths
            samples: Number of samples per segment
            runs: Number of runs per benchmark
            validate: If True, validate generated trajectories
        """
        print("Running benchmark suite...")
        print(f"Config: {self.config_path}")
        print(f"Samples per segment: {samples}")
        print(f"Runs per benchmark: {runs}")
        print(f"Validation: {'enabled' if validate else 'disabled'}")
        print("-" * 60)
        
        for wp_file in waypoint_files:
            print(f"\nBenchmarking: {wp_file}")
            
            # Simple optimizer
            print("  Running simple optimizer...")
            simple_result = self.run_benchmark(wp_file, "simple", samples, runs, validate)
            self.results.append(simple_result)
            
            # Multi-Verse optimizer
            print("  Running Multi-Verse optimizer...")
            multiverse_result = self.run_benchmark(wp_file, "multiverse", samples, runs, validate)
            self.results.append(multiverse_result)
            
            # Print comparison
            speedup = simple_result["avg_wall_time"] / multiverse_result["avg_wall_time"]
            print(f"  Simple: {simple_result['avg_wall_time']:.2f}s")
            print(f"  Multi-Verse: {multiverse_result['avg_wall_time']:.2f}s")
            print(f"  Speedup: {speedup:.2f}x")
    
    def print_summary(self):
        """Print summary of all benchmark results."""
        print("\n" + "=" * 60)
        print("BENCHMARK SUMMARY")
        print("=" * 60)
        
        # Group results by waypoint file
        wp_files = set(r["waypoint_file"] for r in self.results)
        
        for wp_file in sorted(wp_files):
            simple = next((r for r in self.results if r["waypoint_file"] == wp_file and r["optimizer_type"] == "simple"), None)
            multiverse = next((r for r in self.results if r["waypoint_file"] == wp_file and r["optimizer_type"] == "multiverse"), None)
            
            if simple and multiverse:
                print(f"\n{Path(wp_file).name}:")
                print(f"  Simple:      {simple['avg_wall_time']:.2f}s (±{simple['max_wall_time'] - simple['min_wall_time']:.2f}s)")
                print(f"  Multi-Verse: {multiverse['avg_wall_time']:.2f}s (±{multiverse['max_wall_time'] - multiverse['min_wall_time']:.2f}s)")
                if simple['avg_trajectory_time'] and multiverse['avg_trajectory_time']:
                    print(f"  Traj time:   Simple={simple['avg_trajectory_time']:.4f}s, MV={multiverse['avg_trajectory_time']:.4f}s")
                speedup = simple['avg_wall_time'] / multiverse['avg_wall_time']
                print(f"  Speedup:     {speedup:.2f}x")
                
                # Print validation results if available
                if simple.get("validation") and multiverse.get("validation"):
                    print(f"\n  Validation:")
                    v_simple = simple["validation"]
                    v_multiverse = multiverse["validation"]
                    print(f"    Simple:      pos_err={v_simple['avg_max_pos_error_m']:.6f}m, slip={v_simple['avg_num_slip_points']:.1f}, pass={v_simple['pass_rate']:.0%}")
                    print(f"    Multi-Verse: pos_err={v_multiverse['avg_max_pos_error_m']:.6f}m, slip={v_multiverse['avg_num_slip_points']:.1f}, pass={v_multiverse['pass_rate']:.0%}")
    
    def save_results(self, output_file="benchmark_results.json"):
        """Save benchmark results to JSON file."""
        with open(output_file, 'w') as f:
            json.dump(self.results, f, indent=2)
        print(f"\nResults saved to {output_file}")


def main():
    """Run benchmark suite on example files."""
    # Example waypoint files to test
    waypoint_files = [
        "examples/example_straight.json",
        "examples/example_s_curve.json",
        "examples/example_complex_mission.json"
    ]
    
    # Check if files exist
    existing_files = [f for f in waypoint_files if os.path.exists(f)]
    
    if not existing_files:
        print("No example waypoint files found!")
        return
    
    # Run benchmarks with validation enabled
    runner = BenchmarkRunner(config_path="robot_config.json")
    runner.run_comparison_suite(existing_files, samples=10, runs=3, validate=True)
    runner.print_summary()
    runner.save_results()


if __name__ == "__main__":
    main()
