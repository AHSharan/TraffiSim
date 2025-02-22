"""
run.py
------
Functions to run multiple simulations, collect data, and optionally 
provide a CLI entry point.
"""

import os
import csv

from .intersection import IntersectionSim, ARRIVAL_RATES
from .models import DEFAULT_VEHICLE_TYPES
from .rendering import TrafficRenderer

def run_multiple_simulations(
    N_runs=1,
    csv_filename="simulation_results.csv",
    junction_type="4way",
    multiple_lights=False,
    total_time=300,
    simulation_speed=30,
    save_to_files=True,
    output_folder="simulation_outputs",
    multiple_lanes=False,
    lane_count=2,
    yellow_duration=5,
    all_red_duration=2,
    vehicle_distribution=None,
    india_mode=False,
    show_visuals=True,
    simulate_full_route=True,
    adaptive_signals=True
):
    directions = ['N','E','S','W'] if junction_type=='4way' else ['N','E','W']

    summary_fieldnames = [
        "SimulationRun",
        "JunctionType",
        "MultipleLights",
        "MultipleLanes",
        "LaneCount",
        "TotalVehiclesProcessed",
        "OverallAvgWait",
        "SimulateFullRoute",
    ]
    for d in directions:
        summary_fieldnames.append(f"RedEmpty{d}")
    for vt in DEFAULT_VEHICLE_TYPES.keys():
        summary_fieldnames.append(f"Count{vt.capitalize()}")

    if save_to_files:
        os.makedirs(output_folder, exist_ok=True)
    summary_csv_path = os.path.join(output_folder, csv_filename) if save_to_files else None
    file_exists = (save_to_files and os.path.isfile(summary_csv_path))

    all_results = []
    for run_idx in range(1, N_runs+1):
        print(f"\n=== Starting Simulation Run {run_idx}/{N_runs} ===\n")
        renderer_class = TrafficRenderer if show_visuals else None

        sim = IntersectionSim(
            junction_type=junction_type,
            multiple_lights=multiple_lights,
            total_time=total_time,
            simulation_speed=simulation_speed,
            multiple_lanes=multiple_lanes,
            lane_count=lane_count,
            yellow_duration=yellow_duration,
            all_red_duration=all_red_duration,
            vehicle_distribution=vehicle_distribution,
            india_mode=india_mode,
            show_visuals=show_visuals,
            renderer_class=renderer_class,
            simulate_full_route=simulate_full_route,
            adaptive_signals=adaptive_signals
        )
        sim.run()
        sim.print_statistics()

        if save_to_files:
            run_csv = os.path.join(output_folder, f"run_{run_idx}.csv")
            if sim.per_timestep_data:
                fields = list(sim.per_timestep_data[0].keys())
            else:
                fields = ["TimeStep"]

            with open(run_csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in sim.per_timestep_data:
                    writer.writerow(row)

            summary_row = sim.get_results_dict()
            summary_row["SimulationRun"] = run_idx
            with open(summary_csv_path, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
                if (not file_exists) and run_idx == 1:
                    writer.writeheader()
                writer.writerow(summary_row)
                file_exists = True

        all_results.append(sim.get_results_dict())

    if save_to_files and len(all_results) > 0:
        avg_row = compute_average_row(all_results, directions)
        avg_row["SimulationRun"] = "Average"
        with open(summary_csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
            writer.writerow(avg_row)
        print("Average row appended to summary CSV.")

def compute_average_row(all_results, directions):
    n = len(all_results)
    if n == 0:
        return {}
    sum_tvp = 0
    sum_wait = 0
    sum_red = {d: 0 for d in directions}
    sum_counts = {vt: 0 for vt in DEFAULT_VEHICLE_TYPES.keys()}

    # We'll copy some fields from the last result (assuming they are all same config).
    jt = all_results[-1]["JunctionType"]
    ml = all_results[-1]["MultipleLights"]
    mls = all_results[-1]["MultipleLanes"]
    lc = all_results[-1]["LaneCount"]
    sfr = all_results[-1]["SimulateFullRoute"]

    for r in all_results:
        sum_tvp += r["TotalVehiclesProcessed"]
        sum_wait += r["OverallAvgWait"]
        for d in directions:
            sum_red[d] += r[f"RedEmpty{d}"]
        for vt in DEFAULT_VEHICLE_TYPES.keys():
            key = "Count" + vt.capitalize()
            sum_counts[vt] += r[key]

    avg_row = {
        "JunctionType": jt,
        "MultipleLights": ml,
        "MultipleLanes": mls,
        "LaneCount": lc,
        "SimulateFullRoute": sfr,
        "TotalVehiclesProcessed": sum_tvp / n,
        "OverallAvgWait": sum_wait / n
    }
    for d in directions:
        avg_row[f"RedEmpty{d}"] = sum_red[d] / n
    for vt in DEFAULT_VEHICLE_TYPES.keys():
        key = "Count" + vt.capitalize()
        avg_row[key] = sum_counts[vt] / n
    return avg_row

def main_cli():
    """
    Very minimal CLI for demonstration.
    You can expand with argparse to parse arguments from the command line.
    """
    print("Running a default simulation from CLI for demonstration...")
    run_multiple_simulations(
        N_runs=1,
        csv_filename="cli_results.csv",
        junction_type="4way",
        multiple_lights=False,
        total_time=300,
        simulation_speed=10,
        save_to_files=False,
        multiple_lanes=True,
        lane_count=3,
        india_mode=False,
        show_visuals=False,
        simulate_full_route=True,
        adaptive_signals=True
    )
