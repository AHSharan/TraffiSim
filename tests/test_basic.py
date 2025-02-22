"""
tests/test_basic.py
-------------------
A minimal unit test to ensure that the package can be imported 
and that a short run executes without error.
"""

import unittest
from traffisim.run import run_multiple_simulations

class TestBasicSimulation(unittest.TestCase):
    def test_run_once(self):
        custom_vehicle_distribution = {
        "car": 0.1,
        "scooter": 0.5,
        "motorcycle": 0.2,
        "truck": 0.1,
        "bus": 0.1
        }
        run_multiple_simulations(
            N_runs=1,
            total_time=100,
            show_visuals=True,
            save_to_files=True,
            simulate_full_route=False,
            lane_count=3,
            multiple_lanes=True,
            multiple_lights=False,
            india_mode=True,
            adaptive_signals=True,
            junction_type="4way",
            simulation_speed=0,
            csv_filename="test_results.csv",
            vehicle_distribution=custom_vehicle_distribution
       
            


        )
        # If it completes without errors, we consider that "passed" for now.

if __name__ == "__main__":
    unittest.main()
