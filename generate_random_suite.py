import os
import json
import random
import numpy as np
from robot_model import RobotConfig
from multiverse_optimizer import MasterTrajectoryOptimizer
from plotter import plot_trajectory
import matplotlib.pyplot as plt

def generate_random_waypoints(num_waypoints=5, x_range=(0, 1), y_range=(0, 1)):
    wps = []
    for i in range(num_waypoints):
        wp = {
            "x": random.uniform(*x_range),
            "y": random.uniform(*y_range),
            "heading": random.choice([None, random.uniform(-np.pi, np.pi)]),
            "stop": random.choice([True, False]) if i > 0 and i < num_waypoints - 1 else True
        }
        wps.append(wp)
    return wps

def run_suite(num_runs=10, output_dir="random_suite"):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        
    config_path = "robot_config.json"
    with open(config_path, 'r') as f:
        config_data = json.load(f)
    robot_cfg = RobotConfig(config_data)
    
    # Store results for the human reviewer
    suite_data = []
    
    for i in range(num_runs):
        print(f"--- Run {i+1}/{num_runs} ---")
        num_wps = random.randint(3, 8)
        wps_json = generate_random_waypoints(num_wps)
        wps_list = [(wp['x'], wp['y'], wp.get('heading')) for wp in wps_json]
        stop_indices = [idx for idx, wp in enumerate(wps_json) if wp.get('stop')]
        
        # Randomize strategy
        accuracy_weight = random.choice([0.0, 1.0, 2.0, 5.0])
        use_simple = random.random() < 0.2 # 20% simple
        
        run_name = f"run_{i:03d}"
        run_dir = os.path.join(output_dir, run_name)
        if not os.path.exists(run_dir):
            os.makedirs(run_dir)
            
        with open(os.path.join(run_dir, "waypoints.json"), 'w') as f:
            json.dump(wps_json, f, indent=2)
            
        try:
            if use_simple:
                from optimizer import TrajectoryOptimizer
                optimizer = TrajectoryOptimizer(robot_cfg)
            else:
                optimizer = MasterTrajectoryOptimizer(robot_cfg, enable_parallel=True, num_workers=4, verbose=False)
                
            samples = optimizer.solve(
                wps_list, 
                num_samples_per_segment=15, 
                accuracy_weight=accuracy_weight,
                stop_waypoint_indices=stop_indices,
                verbose=False
            )
            
            # Save trajectory
            with open(os.path.join(run_dir, "trajectory.json"), 'w') as f:
                json.dump(samples, f, indent=2)
                
            # Plot
            fig = plot_trajectory(samples, waypoints=wps_list, title=f"Run {i}: a={accuracy_weight}, simple={use_simple}", show=False)
            fig.savefig(os.path.join(run_dir, "plot.png"))
            plt.close(fig)
            
            suite_data.append({
                "id": run_name,
                "accuracy_weight": accuracy_weight,
                "simple": use_simple,
                "status": "success",
                "review": {
                    "is_wrong": False,
                    "explanation": ""
                }
            })
            
        except Exception as e:
            print(f"Run {i} failed: {e}")
            suite_data.append({
                "id": run_name,
                "status": "failed",
                "error": str(e)
            })
            
        # Save the suite metadata incrementally for immediate review
        with open(os.path.join(output_dir, "suite_metadata.json"), 'w') as f:
            json.dump(suite_data, f, indent=2)
        
    print(f"\nSuite generation complete. Review folder: {output_dir}")
    print("Please check suite_metadata.json to log your review comments.")

if __name__ == "__main__":
    run_suite(num_runs=10)
