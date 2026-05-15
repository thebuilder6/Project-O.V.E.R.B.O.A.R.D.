import click
import json
import os
import numpy as np
from robot_model import RobotConfig
from optimizer import TrajectoryOptimizer
from multiverse_optimizer import MasterTrajectoryOptimizer
from plotter import plot_trajectory
from animated_plotter import animate_trajectory
from validator import validate_trajectory
from export import write_controller_file, write_python_file
from convergence_plotter import plot_convergence, animate_convergence, ConvergencePlotter

@click.command()
@click.option('-c', '--config', required=True, type=click.Path(exists=True), help='Path to the configuration file (.chor or .json).')
@click.option('-w', '--waypoints', required=True, type=click.Path(exists=True), help='Path to waypoints JSON file.')
@click.option('-o', '--output', default='output.traj', type=str, help='Output trajectory file path.')
@click.option('-n', '--samples', default=10, type=int, help='Samples per segment.')
@click.option('-a', '--accuracy-weight', default=0.0, type=float, help='Smoothness/accuracy weight (0 = pure time-optimal).')
@click.option('--stop-waypoints', default=None, type=str, help='Comma-separated waypoint indices where robot must stop (e.g., "2,5,7").')
@click.option('--events', default=None, type=str, help='Comma-separated waypoint:event pairs (e.g., "2:lower_arm,5:release").')
@click.option('--validate', is_flag=True, help='Run validation report on the generated trajectory.')
@click.option('--export-format', type=click.Choice(['none', 'controller', 'python'], case_sensitive=False), default='none', help='Export format for controller consumption.')
@click.option('--controller-dt', default=0.02, type=float, help='Fixed timestep for controller export (seconds).')
@click.option('--plot', is_flag=True, help='Plot the resulting trajectory.')
@click.option('--animate', is_flag=True, help='Animate the trajectory in real-time.')
@click.option('--simple', is_flag=True, help='Use simple optimizer instead of Multi-Verse refinement.')
@click.option('--no-parallel', is_flag=True, help='Disable parallel processing for Multi-Verse refinement.')
@click.option('--workers', default=8, type=int, help='Number of parallel workers for Multi-Verse refinement.')
@click.option('--benchmark', is_flag=True, help='Collect comprehensive benchmarking data for whitepaper.')
@click.option('--quiet', '-q', is_flag=True, help='Suppress verbose output (cleaner terminal).')
@click.option('--show-convergence', is_flag=True, help='Show convergence visualization after optimization.')
@click.option('--convergence-mode', type=click.Choice(['parallel', 'best', 'layered'], case_sensitive=False), default='best', help='Convergence visualization mode.')
@click.option('--convergence-animate', is_flag=True, help='Animate convergence instead of static plot.')
@click.option('--convergence-output', type=str, default=None, help='Save convergence plot to file (e.g., convergence.png). If not specified, displays interactively.')
def main(config, waypoints, output, samples, accuracy_weight, stop_waypoints, events, validate, export_format, controller_dt, plot, animate, simple, no_parallel, workers, benchmark, quiet, show_convergence, convergence_mode, convergence_animate, convergence_output):
    """
    FLL Trajectory Optimizer CLI.
    Generates time-optimal trajectories for Lego robots.
    """
    if not quiet:
        click.echo(f"Loading config from {config}...")
    with open(config, 'r') as f:
        config_data = json.load(f)
    
    robot_cfg = RobotConfig(config_data)
    
    # Choose optimizer based on --simple flag
    if simple:
        if not quiet:
            click.echo("Using simple optimizer (legacy mode)")
        optimizer = TrajectoryOptimizer(robot_cfg)
    else:
        parallel = not no_parallel
        if not quiet:
            click.echo(f"Using Multi-Verse optimizer (parallel={parallel}, workers={workers})")
        optimizer = MasterTrajectoryOptimizer(robot_cfg, enable_parallel=parallel, num_workers=workers, verbose=not quiet)
    
    if not quiet:
        click.echo(f"Loading waypoints from {waypoints}...")
    with open(waypoints, 'r') as f:
        wp_data = json.load(f)
    
    # Expected wp_data: list of objects with x, y, and optionally heading and event
    # or list of lists [x, y, heading]
    wps = []
    waypoint_events = {}  # index -> event name
    for i, item in enumerate(wp_data):
        if isinstance(item, dict):
            wps.append((item['x'], item['y'], item.get('heading')))
            if 'event' in item:
                waypoint_events[i] = item['event']
        else:
            # Assume [x, y, heading]
            wps.append((item[0], item[1], item[2] if len(item) > 2 else None))

    if not quiet:
        click.echo(f"Optimizing trajectory through {len(wps)} waypoints (accuracy_weight={accuracy_weight})...")

    # Enable iteration capture if convergence visualization is requested
    capture_iterations = show_convergence

    # Parse stop waypoints
    stop_indices = []
    if stop_waypoints:
        try:
            stop_indices = [int(x.strip()) for x in stop_waypoints.split(',')]
            if not quiet:
                click.echo(f"Stop waypoints at indices: {stop_indices}")
        except ValueError:
            click.echo("Invalid stop waypoints format. Use comma-separated indices (e.g., '2,5,7').")

    # Parse events from CLI (overrides JSON)
    if events:
        try:
            for pair in events.split(','):
                idx_str, event_name = pair.strip().split(':')
                waypoint_events[int(idx_str.strip())] = event_name.strip()
            if not quiet:
                click.echo(f"Events at waypoints: {waypoint_events}")
        except ValueError:
            click.echo("Invalid events format. Use 'index:event' pairs separated by commas (e.g., '2:lower_arm,5:release').")

    # TODO: If benchmark flag is set, collect comprehensive data:
    # - Measure total solve time
    # - Collect solver statistics (iterations, constraint violations)
    # - Store trajectory quality metrics (tortuosity, chattering, etc.)
    # - Save to structured JSON file for whitepaper analysis
    # - Include configuration parameters for reproducibility
    
    samples_data = optimizer.solve(wps, num_samples_per_segment=samples, accuracy_weight=accuracy_weight, stop_waypoint_indices=stop_indices, waypoint_events=waypoint_events, verbose=not quiet, capture_iterations=capture_iterations)
    
    # Construct Choreo-like output
    result = {
        "name": os.path.basename(output).split('.')[0],
        "version": 3,
        "trajectory": {
            "config": config_data.get("config", {}),
            "samples": samples_data
        }
    }
    
    with open(output, 'w') as f:
        json.dump(result, f, indent=1)
    
    if not quiet:
        click.echo(f"Successfully saved trajectory to {output}")

    if validate:
        validate_trajectory(output, config)

    if export_format == 'controller':
        ctrl_output = os.path.splitext(output)[0] + '_controller.json'
        write_controller_file(output, ctrl_output, target_dt=controller_dt,
                              track_width=robot_cfg.track_width)

    if export_format == 'python':
        py_output = os.path.splitext(output)[0] + '.py'
        write_python_file(output, py_output)

    if plot:
        plot_trajectory(samples_data, waypoints=wps,
                        title=f"Trajectory: {os.path.basename(output)}")

    if animate:
        animate_trajectory(samples_data, waypoints=wps,
                          title=f"Trajectory: {os.path.basename(output)}")
    
    if show_convergence:
        if hasattr(optimizer, 'iteration_history') and optimizer.iteration_history:
            if not quiet:
                click.echo(f"Showing convergence visualization ({convergence_mode} mode)...")
            if convergence_animate:
                if convergence_output:
                    animate_convergence(optimizer.iteration_history, waypoints=wps,
                                     title=f"Convergence: {os.path.basename(output)}",
                                     save_path=convergence_output)
                    if not quiet:
                        click.echo(f"Convergence animation saved to {convergence_output}")
                else:
                    animate_convergence(optimizer.iteration_history, waypoints=wps,
                                     title=f"Convergence: {os.path.basename(output)}")
            else:
                if convergence_output:
                    # Save to HTML file
                    plot_convergence(optimizer.iteration_history, mode=convergence_mode, 
                                   waypoints=wps, title=f"Convergence: {os.path.basename(output)}",
                                   output_file=convergence_output)
                    if not quiet:
                        click.echo(f"Convergence plot saved to {convergence_output}")
                else:
                    # Open in browser
                    plot_convergence(optimizer.iteration_history, mode=convergence_mode, 
                                   waypoints=wps, title=f"Convergence: {os.path.basename(output)}")
        else:
            if not quiet:
                click.echo("No iteration history available for convergence visualization.")

if __name__ == '__main__':
    main()
