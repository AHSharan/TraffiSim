# TraffiSim
TraffiSim is a Python-based intersection traffic simulator designed to capture various driving behaviors (including Indian driving styles and multi-lane configurations) that might not be fully addressed by large-scale simulators like SUMO. It uses the Intelligent Driver Model (IDM) for longitudinal control, MOBIL for lane changes, and supports both adaptive and fixed traffic signal strategies.

## Table of Contents
- [Why TraffiSim?](#why-traffisim)
- [Features](#features)
- [Technical Overview](#technical-overview)
- [Installation](#installation)
- [Usage](#usage)
    - [Using in Python Scripts](#using-in-python-scripts)
- [Simulation Parameters](#simulation-parameters)
- [Customization](#customization)
- [Comparison to Other Tools (SUMO, etc.)](#comparison-to-other-tools-sumo-etc)
- [Limitations & Future Work](#limitations--future-work)
- [Contributing](#contributing)
- [License](#license)

## Why TraffiSim?

While powerful traffic simulators exist—such as SUMO, MATSim, and Aimsun—they are often complex to install, configure, and extend. In addition, many of these simulators do not provide country-specific behaviors (e.g., Indian driving styles with freer lateral movement, two-wheelers navigating queues side-by-side, etc.) out of the box.

TraffiSim was created to address these needs:

### India-Specific Queueing & Lateral Movement
- Incorporates concepts like India Mode for simulating two-wheelers and small vehicles weaving side-by-side in queues.

### Lightweight & Extensible
- Focuses on intersection-level simulation, making it easy to add new driving rules or tweak existing ones.

### Educational & Rapid Prototyping
- A simpler codebase suitable for teaching traffic engineering or quickly prototyping new signal control strategies.

## Features
- **Multiple Vehicle Types:** Cars, scooters, motorcycles, trucks, buses (with configurable parameters).
- **IDM & MOBIL:** Intelligent Driver Model for speed control, and MOBIL for lane-change decisions.
- **Adaptive or Fixed Signals:** Choose between a simple queue-based adaptive or a time-based fixed cycle strategy.
- **Multi-Lane Support:** Allows 1 to N lanes in each direction.
- **“India Mode”:** Enables side-by-side queueing for two-wheelers and small vehicles.
- **Optionally Simulate Full Routes:** Vehicles can vanish at the intersection center or continue to an exit point.
- **Pygame Visualization:** Observe your simulation in real-time or disable it for headless runs.
- **Data Collection:** Logs queue length, arrival rates, wait times, and more in CSV format for analysis.
## Technical Overview
TraffiSim’s code is organized into several Python modules:

**intersection.py**:  
- Core logic for arrivals, queues, traffic lights, adaptive/fixed signal control, and data recording.  
- Main class: `IntersectionSim`.

**models.py**:  
- Vehicle-level logic, including IDM for longitudinal control, MOBIL for lane changing.  
- Holds default parameters in `DEFAULT_VEHICLE_TYPES`.

**rendering.py**:  
- Uses Pygame for drawing roads, vehicles, signals, and queue lines if `show_visuals=True`.  
- Entirely optional (the simulator can run without visualization).

**run.py**:  
- High-level function `run_multiple_simulations` that can be called multiple times with varying parameters.  
- A minimal `main_cli()` function for a command-line interface.

**utils.py**:  
- Simple helper functions for geometry (e.g., distance calculations) and exit point definitions.

The Intelligent Driver Model (IDM) controls acceleration based on the gap to the lead vehicle, desired speeds, and comfortable deceleration. MOBIL checks whether lane changes are beneficial and safe based on accelerations for the subject vehicle and adjacent vehicles.

## Installation
### A) From Source (Local Development)

Clone the repository:
```bash
git clone https://github.com/AHSharan/TraffiSim.git
cd TraffiSim
```
Install with pip:
```bash
pip install .
```
To include Pygame automatically, install the optional `[rendering]` extras:
```bash
pip install .[rendering]
```

### B) Installing from PyPI
You can install TraffiSim directly from the Python Package Index (PyPI):
```bash
pip install TraffiSim
```
If you want to include Pygame (used for visualization) right away, install the `[rendering]` extra:
```bash
pip install TraffiSim[rendering]
```
Note: If you only need to run headless simulations (no graphics), you don’t need the extra.

Once installed, verify by running:
```bash
traffisim-run
```
This will execute a default simulation with minimal parameters.

You can then use TraffiSim in your own Python scripts:
```python
from traffisim.run import run_multiple_simulations

run_multiple_simulations(
    N_runs=1,
    total_time=300,
    multiple_lanes=True,
    lane_count=3,
    show_visuals=True,  # True requires pygame
    adaptive_signals=True,
)
```

## Usage
### Using in Python Scripts
You can run simulations programmatically:
```python
from traffisim.run import run_multiple_simulations

run_multiple_simulations(
    N_runs=1,
    junction_type="4way",
    total_time=600,
    show_visuals=True,
    multiple_lanes=True,
    lane_count=3,
    adaptive_signals=True,
    india_mode=True,
    save_to_files=False
)
```
Or instantiate `IntersectionSim` directly:
```python
from traffisim.intersection import IntersectionSim
from traffisim.rendering import TrafficRenderer

sim = IntersectionSim(
    junction_type="4way",
    multiple_lanes=True,
    lane_count=2,
    total_time=300,
    show_visuals=True,
    renderer_class=TrafficRenderer,
    india_mode=False,
    adaptive_signals=True
)
sim.run()
```

## Simulation Parameters
Commonly adjusted parameters:

| Parameter             | Default     | Description                                                                 |
|-----------------------|------------|-----------------------------------------------------------------------------|
| `junction_type`       | "4way"     | Intersection type: "4way" or "3way".                                       |
| `multiple_lights`     | False      | Use a single traffic light vs. per-approach lights.                         |
| `total_time`          | 300        | Number of simulation steps.                                                |
| `sim_steps_per_sec`   | 10         | Internal sub-step resolution (10 or 1).                                    |
| `simulation_speed`    | 30         | Frame rate in Pygame visualization.                                        |
| `multiple_lanes`      | False      | Whether each approach has multiple lanes.                                  |
| `lane_count`          | 2          | Number of lanes per approach if multiple lanes are enabled.               |
| `india_mode`          | False      | Allow side-by-side queue for two-wheelers or small vehicles.              |
| `adaptive_signals`    | True       | Use a simple adaptive signal logic vs. fixed-cycle approach.               |
| `simulate_full_route` | True       | Vehicles continue from entry to exit points; if False, they vanish center. |
| `vehicle_distribution`| dict       | Probability distribution for vehicle types (car, scooter, etc.).           |
| `save_to_files`       | True       | Save simulation data to CSV.                                              |
| `output_folder`       | "simulation_outputs" | Directory for saving CSV outputs.                                |

## Customization
**New Vehicle Types**  
Add entries in `DEFAULT_VEHICLE_TYPES` in `models.py` with parameters like `desired_speed`, `max_acceleration`, etc.

**Signal Logic**  
Default adaptive logic chooses phases based on queue length. Edit `choose_new_green()` in `intersection.py`.

**Queue Behaviors (India Mode)**  
For complex lateral movement, modify `reposition_queue()` in `intersection.py`.

**Saving & Analyzing Data**  
Per-step data in `sim.per_timestep_data`; summary in `sim.get_results_dict()`. Customize columns or CSV writing in `run.py`.

## Comparison to Other Tools (SUMO, etc.)
**SUMO:**  
- Large-scale, open-source traffic simulator.  
- TraffiSim focuses on intersection-level details like 2D positioning for Indian driving patterns.

**MATSim / Aimsun:**  
- Large or commercial simulators for multi-agent, city-scale traffic.  
- TraffiSim is simpler and specialized for intersection-based experiments.

## Limitations & Future Work
- **Geometry:** Physical turn radii or collisions not deeply simulated.  
- **No Traffic Signal Optimization:** Basic queue-based adaptive approach.  
- **Single Intersection Focus:** Multi-intersection networks need extensions.  
- **Performance:** High vehicle counts may impact performance.

## Contributing
Ideas for contribution include:
- Machine learning or optimization for signals  
- More realistic India Mode dynamics  
- Network-level expansions  
- Additional vehicle classes (auto-rickshaws, e-bikes)

Submit pull requests or open issues on GitHub.

## License
TraffiSim is under the GNU License.  
Feel free to use, modify, and distribute with attribution.




