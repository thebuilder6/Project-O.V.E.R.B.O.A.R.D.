import os
import json
import time
import random
import numpy as np
from pathlib import Path
from robot_model import RobotConfig
from optimizer import TrajectoryOptimizer
from multiverse_optimizer import MasterTrajectoryOptimizer
from validator import validate_trajectory

class ComprehensiveBenchmark:
    def __init__(self, config_path="robot_config.json", output_dir="benchmarks"):
        self.config_path = config_path
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        with open(config_path, 'r') as f:
            self.config_data = json.load(f)
        self.robot_cfg = RobotConfig(self.config_data)
        
        self.results = []

    def generate_random_waypoints(self, profile="complex"):
        """Generate random waypoints based on mission profiles."""
        wps = []
        if profile == "short_sprint":
            num_wps = 3
            x_range, y_range = (0, 1.5), (0, 0.5)
        elif profile == "complex":
            num_wps = random.randint(5, 8)
            x_range, y_range = (0, 2.0), (0, 1.5)
        elif profile == "stress_test":
            num_wps = random.randint(4, 6)
            x_range, y_range = (0, 1.0), (0, 1.0)
        
        for i in range(num_wps):
            wp = {
                "x": random.uniform(*x_range),
                "y": random.uniform(*y_range),
                "heading": random.choice([None, random.uniform(-np.pi, np.pi)]),
                "stop": True if i == 0 or i == num_wps - 1 else random.random() < 0.3
            }
            wps.append(wp)
            
        # For stress tests, force some sharp turns
        if profile == "stress_test":
            for i in range(1, num_wps - 1):
                if random.random() < 0.5:
                    wps[i]["heading"] = (wps[i-1].get("heading", 0) or 0) + np.pi # Force 180 flip
        
        return wps

    def run_benchmark(self, num_runs=10, profiles=["short_sprint", "complex", "stress_test"]):
        print(f"Starting comprehensive benchmark: {num_runs} runs...")
        
        for i in range(num_runs):
            profile = random.choice(profiles)
            print(f"\n--- Run {i+1}/{num_runs} (Profile: {profile}) ---")
            
            wps_json = self.generate_random_waypoints(profile)
            wps_list = [(wp['x'], wp['y'], wp.get('heading')) for wp in wps_json]
            stop_indices = [idx for idx, wp in enumerate(wps_json) if wp.get('stop')]
            
            run_id = f"run_{i:03d}_{profile}"
            run_dir = self.output_dir / run_id
            run_dir.mkdir(exist_ok=True)
            
            # Save waypoints
            with open(run_dir / "waypoints.json", 'w') as f:
                json.dump(wps_json, f, indent=2)
            
            # 1. Simple Optimizer
            print("  Running Simple Optimizer...")
            simple_opt = TrajectoryOptimizer(self.robot_cfg)
            try:
                samples_simple, stats_simple = simple_opt.solve(wps_list, stop_waypoint_indices=stop_indices, verbose=False)
                val_simple = self._validate(samples_simple, run_dir / "traj_simple.traj")
            except Exception as e:
                print(f"    Simple failed: {e}")
                samples_simple, stats_simple, val_simple = None, None, None

            # 2. Multi-Verse Optimizer
            print("  Running Multi-Verse Optimizer...")
            mv_opt = MasterTrajectoryOptimizer(self.robot_cfg, enable_parallel=True, num_workers=4, verbose=False)
            try:
                samples_mv, stats_mv = mv_opt.solve(wps_list, stop_waypoint_indices=stop_indices, verbose=False)
                val_mv = self._validate(samples_mv, run_dir / "traj_mv.traj")
            except Exception as e:
                print(f"    Multi-Verse failed: {e}")
                samples_mv, stats_mv, val_mv = None, None, None
            
            # Record result
            self.results.append({
                "id": run_id,
                "profile": profile,
                "waypoints": wps_json,
                "simple": {"stats": stats_simple, "validation": val_simple},
                "multiverse": {"stats": stats_mv, "validation": val_mv}
            })
            
            # Incremental save
            self.save_results()

    def _validate(self, samples, output_file):
        if not samples: return None
        
        # Save to file for validator
        data = {
            "trajectory": {
                "config": self.config_data.get("robot", {}),
                "samples": samples
            }
        }
        with open(output_file, 'w') as f:
            json.dump(data, f)
            
        try:
            metrics, audit, errors = validate_trajectory(str(output_file), self.config_path, apply_headroom=False)
            return {"metrics": metrics, "audit": audit, "errors": errors}
        except Exception as e:
            print(f"    Validation error: {e}")
            return None

    def save_results(self):
        with open(self.output_dir / "benchmark_results.json", 'w') as f:
            json.dump(self.results, f, indent=2)
            
    def generate_report(self):
        report_path = self.output_dir / "benchmark_report.md"
        
        lines = [
            "# Comprehensive Optimizer Benchmark Report",
            f"Generated on: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            "",
            "## Summary Statistics",
            "| Metric | Simple | Multi-Verse | Delta / Speedup |",
            "| :--- | :--- | :--- | :--- |"
        ]
        
        # Aggregates
        simple_times = [r["simple"]["stats"]["total_time"] for r in self.results if r["simple"]["stats"]]
        mv_times = [r["multiverse"]["stats"]["total_time"] for r in self.results if r["multiverse"]["stats"]]
        
        avg_simple = np.mean(simple_times) if simple_times else 0
        avg_mv = np.mean(mv_times) if mv_times else 0
        speedup = avg_simple / avg_mv if avg_mv > 0 else 0
        
        lines.append(f"| Avg Solve Time | {avg_simple:.3f}s | {avg_mv:.3f}s | {speedup:.2f}x |")
        
        simple_costs = [r["simple"]["stats"]["final_cost"] for r in self.results if r["simple"]["stats"]]
        mv_costs = [r["multiverse"]["stats"]["final_cost"] for r in self.results if r["multiverse"]["stats"]]
        
        avg_cost_s = np.mean(simple_costs) if simple_costs else 0
        avg_cost_mv = np.mean(mv_costs) if mv_costs else 0
        improvement = (1 - avg_cost_mv / avg_cost_s) * 100 if avg_cost_s > 0 else 0
        
        lines.append(f"| Avg Traj Duration | {avg_cost_s:.3f}s | {avg_cost_mv:.3f}s | {improvement:.1f}% improvement |")
        
        # Success Rate
        success_simple = sum(1 for r in self.results if r["simple"]["stats"] and r["simple"]["stats"].get("converged", True))
        success_mv = sum(1 for r in self.results if r["multiverse"]["stats"]) # MV usually always returns something
        
        lines.append(f"| Success Rate | {success_simple}/{len(self.results)} | {success_mv}/{len(self.results)} | |")
        
        lines.append("\n## Heuristic Effectiveness")
        lines.append("| Window | Winning Heuristic | Improvement |")
        lines.append("| :--- | :--- | :--- |")
        
        for run in self.results:
            mv_stats = run["multiverse"]["stats"]
            if mv_stats and mv_stats.get("heuristic_wins"):
                for win in mv_stats["heuristic_wins"]:
                    lines.append(f"| {run['id']}:{win['window']} | {win['heuristic']} | {win['improvement_pct']:.1f}% |")

        lines.append("\n## Physical Validation (Multi-Verse)")
        lines.append("| Run | Max Pos Error | Max Slip | Pass/Fail |")
        lines.append("| :--- | :--- | :--- | :--- |")
        
        for run in self.results:
            v = run["multiverse"]["validation"]
            if v:
                err = v["errors"]["max_pos_error_m"]
                slip = v["audit"]["left_wheel_slip"] + v["audit"]["right_wheel_slip"]
                status = "✅ PASS" if (err < 0.01 and slip < 1e-3) else "❌ FAIL"
                lines.append(f"| {run['id']} | {err:.6f}m | {slip:.6f}N | {status} |")
            else:
                lines.append(f"| {run['id']} | N/A | N/A | N/A |")

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("\n".join(lines))
        
        print(f"\nReport generated: {report_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--runs", type=int, default=10)
    args = parser.parse_args()
    
    benchmark = ComprehensiveBenchmark()
    benchmark.run_benchmark(num_runs=args.runs)
    benchmark.generate_report()
