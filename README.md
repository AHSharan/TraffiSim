# Traffic Simulation System
## Overview
This project is a Python-based traffic simulation system using Pygame. It models vehicle flow through an intersection,
supporting multiple lanes, different traffic light configurations, and turn-based vehicle movement.
Features

- 4-way and 3-way junctions: Supports both configurations.
- Multiple lanes: Configurable number of lanes per direction.
- Traffic light management: Single or multiple light systems.
- Vehicle behavior simulation: Cars can turn left, right, or go straight.
- Poisson arrival model: Vehicles arrive based on real-world statistical models.
- Dynamic queueing: Vehicles queue up and cross the intersection based on signals.
- Data logging: Stores results of multiple simulation runs in CSV format.

## Requirements
Make sure you have the following dependencies installed:
```
pip install pygame
```
## How to Run
To execute a single run with visualization:
```
python traffic_sim.py
```
To run multiple simulations and save results
```
python traffic_sim.py --multi
```
## Configuration
Modify the parameters in traffic_sim.py to adjust the simulation:

- TOTAL_TIME: Total simulation timesteps.
- ARRIVAL_RATES: Vehicle arrival rates per second.
- CROSSING_TIME: Time a vehicle takes to cross the intersection.
- multiple_lanes: Set to True to enable multiple lanes.
- multiple_lights: Set to True to allow simultaneous green signals.
- lane_count: Number of lanes per direction.

## Logging & Data Output
Simulation outputs are stored in CSV format under the simulation_outputs/ directory. It includes:

- Per timestep data: Vehicle queue sizes, wait times, and traffic light phases.
- Summary statistics: Aggregated results from multiple simulation runs.

## Example Usage
To run multiple simulations with data logging:
python traffic_sim.py --runs 5 --output results.csv

This will run 5 simulations and store results in results.csv.
## Customization

Modify define_signal_phases() to change traffic light logic.
Adjust generate_arrivals() to tweak vehicle arrival behavior.
Implement additional road structures for complex intersections.

## Acknowledgments
This project was developed as part of a research initiative to analyze traffic congestion and optimize signal timings.
