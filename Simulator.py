import pygame
import sys
import random
import math
import csv
import os
from datetime import datetime

# ------------------------------
# CONFIGURABLE PARAMETERS (Defaults)
# ------------------------------
TOTAL_TIME = 300               # total simulation timesteps
# 1) LOGIC TIMESTEPS & SPEED
SIM_STEPS_PER_SEC = 10
# Explanation:
#   - By setting this to 1, each "logic" timestep represents 1 real second.
#   - This makes it easier to reason about arrival rates, crossing times, etc.

# 2) ARRIVAL RATES (vehicles/sec) in each direction.
#    Typically, 0.1 -> 0.3 vehicles/sec per approach is moderate traffic.
#    0.3 vehicles/sec is 18 vehicles/min (very busy for one lane).
#    Adjust as needed for rush hour vs. off-peak.
ARRIVAL_RATES = {
    'N': 0.50,  # About 30 vehicles per minute
    'E': 0.60,  # About 36 vehicles per minute 
    'S': 0.50,  # About 30 vehicles per minute
    'W': 0.40   # About 24 vehicles per minute
}


# 3) CROSSING_TIME
CROSSING_TIME = 6
# Explanation:
#   - Each vehicle takes ~4 seconds to fully clear the intersection from the moment it starts crossing.
#   - Real-world times can vary from 2 to 7 seconds depending on speed, intersection size, etc.

# 4) PRINT INTERVAL
PRINT_INTERVAL = 60
MIN_QUEUE_EMPTY = 0

# 6) TIMESERIES_INTERVAL
TURN_PROBABILITIES = {
    "left": 0.3,
    "straight": 0.5,
    "right": 0.2
}

LEFT_TURN_OPEN_PROB = 0.2

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
# Road geometry
ROAD_WIDTH = 100
QUEUE_OFFSET = 140
CAR_SPACING = 20
CAR_RADIUS = 5

# Colors
COLOR_BG = (30, 30, 30)
COLOR_ROAD = (70, 70, 70)
COLOR_TEXT = (255, 255, 255)
COLOR_MAP = {
    'N': (255, 0, 0),
    'E': (0, 0, 255),
    'S': (0, 255, 0),
    'W': (255, 255, 0),
}


# ------------------------------
# VEHICLE CLASS
# ------------------------------
class Vehicle:
    def __init__(self, arrival_time, direction):
        self.arrival_time = arrival_time
        self.direction = direction
        self.state = 'queueing'
        self.start_cross_time = None
        self.finish_time = None
        self.x = 0
        self.y = 0
        self.cross_progress = 0.0
        # When a vehicle starts crossing, we record its starting position.
        self.start_position = None
        # For multilane simulation, store the lane index (if applicable)
        self.lane_index = None
        # NEW: Turn decision for the vehicle ("left", "straight", or "right")
        self.turn_direction = random.choices(
            list(TURN_PROBABILITIES.keys()),
            weights=list(TURN_PROBABILITIES.values())
        )[0]
    
    @property
    def wait_time(self):
        """
        How long from arrival to the moment the vehicle starts crossing.
        """
        if self.start_cross_time is None:
            return None
        return self.start_cross_time - self.arrival_time


# ------------------------------
# INTERSECTION SIMULATION
# ------------------------------
class IntersectionSim:
    def __init__(
        self,
        junction_type="4way",
        multiple_lights=False,
        total_time=TOTAL_TIME,
        sim_steps_per_sec=SIM_STEPS_PER_SEC,
        crossing_time=CROSSING_TIME,
        arrival_rates=None,
        print_interval=PRINT_INTERVAL,
        min_queue_empty=MIN_QUEUE_EMPTY,
        simulation_speed=30,
        multiple_lanes=False,   
        lane_count=2                
    ):
        """
        :param junction_type: "4way" or "3way"
        :param multiple_lights: if True => concurrency (signal phases)
        :param total_time: logic timesteps to simulate
        :param sim_steps_per_sec: how many timesteps per real second
        :param crossing_time: timesteps a vehicle spends crossing
        :param arrival_rates: dict of arrival rates (default if None)
        :param print_interval: how often to print console logs
        :param min_queue_empty: queue <= this => "empty"
        :param simulation_speed: 0 => run fast, else real-time FPS
        :param multiple_lanes: if True, simulate multiple lanes per approach
        :param lane_count: number of lanes per direction (used if multiple_lanes=True)
        """
        if arrival_rates is None:
            arrival_rates = ARRIVAL_RATES
        
        self.junction_type = junction_type
        self.multiple_lights = multiple_lights
        self.total_time = total_time
        self.sim_steps_per_sec = sim_steps_per_sec
        self.crossing_time = crossing_time
        self.arrival_rates = arrival_rates
        self.print_interval = print_interval
        self.min_queue_empty = min_queue_empty
        self.simulation_speed = simulation_speed

        # NEW: multiple lanes settings
        self.multiple_lanes = multiple_lanes
        self.lane_count = lane_count
        
        pygame.init()
        pygame.display.set_caption(f"Traffic Sim ({junction_type}, multiLights={multiple_lights}, multiLanes={multiple_lanes})")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 24)
        
        self.cx = SCREEN_WIDTH // 2
        self.cy = SCREEN_HEIGHT // 2
        
        # Directions in use
        if junction_type == "3way":
            self.directions = ['N', 'E', 'W']
        else:
            self.directions = ['N', 'E', 'S', 'W']
        
        # Set up queues. In single-lane mode, each direction holds a single list.
        # In multi-lane mode, each direction is a list of lanes (each lane is a list of vehicles).
        if self.multiple_lanes:
            self.queues = {d: [[] for _ in range(self.lane_count)] for d in self.directions}
        else:
            self.queues = {d: [] for d in self.directions}
            
        self.crossing_vehicles = []
        self.processed_vehicles = []
        
        self.sim_time = 0
        self.running = True
        
        # Define phases
        self.phases = []
        self.cycle_length = 0
        self.define_signal_phases()
        
        # Track red-with-empty
        self.red_no_car_time = {d: 0 for d in self.directions}
        
        #We store per-timestep data in a list of dicts
        self.per_timestep_data = []  # each entry will be a row {time, qsizeN, qsizeE, ... etc.}

    def define_signal_phases(self):
        if self.junction_type == "4way":
            if not self.multiple_lights:
                self.phases = [
                    (['N'], 30),
                    (['E'], 30),
                    (['S'], 30),
                    (['W'], 30)
                ]
            else:
                # 2-phase concurrency: (N,S), (E,W)
                self.phases = [
                    (['N','S'], 30),
                    (['E','W'], 30)
                ]
        else:
            # 3way T-junction
            if not self.multiple_lights:
                self.phases = [
                    (['N'], 30),
                    (['E'], 30),
                    (['W'], 30)
                ]
            else:
                self.phases = [
                    (['N','E'], 30),
                    (['E','W'], 30),
                    (['W','N'], 30)
                ]
        
        self.cycle_length = sum(dur for _, dur in self.phases)
    
    def run(self):
        sim_step_acc = 0.0
        steps_per_frame = float(self.sim_steps_per_sec) / (self.simulation_speed if self.simulation_speed > 0 else 1.0)
        
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            if self.sim_speed_is_real_time():
                sim_step_acc += steps_per_frame
            
            if self.sim_time < self.total_time:
                if not self.sim_speed_is_real_time():
                    # run as many steps as we can
                    while self.sim_time < self.total_time:
                        self.sim_update()
                else:
                    while sim_step_acc >= 1.0 and self.sim_time < self.total_time:
                        self.sim_update()
                        sim_step_acc -= 1.0
                    # partial crossing movement
                    if self.sim_time < self.total_time:
                        fraction = min(sim_step_acc, 1.0)
                        self.update_crossing_vehicles_fraction(fraction)
            
            self.draw()
            
            if self.sim_speed_is_real_time():
                self.clock.tick(self.simulation_speed)
            
            if self.sim_time >= self.total_time:
                self.running = False
        
        pygame.quit()

    def sim_speed_is_real_time(self):
        return (self.simulation_speed > 0)

    def sim_update(self):
        # 1) arrivals
        self.generate_arrivals(self.sim_time)
        # 2) green directions
        green_dirs = self.get_green_directions(self.sim_time)
        # 3) for each green direction, start crossing vehicles.
        for d in green_dirs:
            self.start_crossing_one_vehicle(d, self.sim_time)
        # 4) update crossing
        self.update_crossing_vehicles_step()
        # 5) track red no-car
        self.track_empty_red_time(green_dirs)
        # 6) print state occasionally
        if self.sim_time % self.print_interval == 0:
            self.print_state(self.sim_time, green_dirs)
        # 7) record timeseries data every TIMESERIES_INTERVAL timesteps
        if (self.sim_time % TIMESERIES_INTERVAL) == 0:
            self.record_timestep_data()
        # inc time
        self.sim_time += 1

    def record_timestep_data(self):
        """
        Capture the queue sizes, average wait times, etc., at the current time step,
        and store them in self.per_timestep_data.
        """
        row = {
            "time": self.sim_time
        }
        # queue sizes: for multilane mode, sum across lanes.
        for d in self.directions:
            if self.multiple_lanes:
                row[f"queue_{d}"] = sum(len(lane) for lane in self.queues[d])
            else:
                row[f"queue_{d}"] = len(self.queues[d])
        
        # average wait times for each direction so far
        for d in self.directions:
            row[f"avg_wait_{d}"] = self.average_wait_time_for_direction(d)
        
        # overall average
        row["overall_avg_wait"] = self.overall_average_wait_time()
        
        self.per_timestep_data.append(row)

    def generate_arrivals(self, t):
        for d in self.directions:
            rate = self.arrival_rates.get(d, 0.0)
            arrivals = self.poisson_random(rate)
            for _ in range(arrivals):
                v = Vehicle(t, d)
                self.place_in_queue(v, d)
    
    def poisson_random(self, rate):
        L = math.exp(-rate)
        p = 1.0
        k = 0
        while p > L:
            p *= random.random()
            k += 1
        return k - 1

    def place_in_queue(self, v, direction):
        if self.multiple_lanes:
            # Choose the lane with the fewest vehicles
            lanes = self.queues[direction]
            lane_index = min(range(self.lane_count), key=lambda i: len(lanes[i]))
            lane = lanes[lane_index]
            lane.append(v)
            v.lane_index = lane_index
            # Set the vehicle's position based on its lane and its order in that lane.
            lane_gap = 15  # lateral gap between lanes
            if direction == 'N':
                base_x = self.cx
                base_y = self.cy - QUEUE_OFFSET
                offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                v.x = base_x + offset
                v.y = base_y - (len(lane)-1) * CAR_SPACING
            elif direction == 'S':
                base_x = self.cx
                base_y = self.cy + QUEUE_OFFSET
                offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                v.x = base_x + offset
                v.y = base_y + (len(lane)-1) * CAR_SPACING
            elif direction == 'E':
                base_x = self.cx + QUEUE_OFFSET
                base_y = self.cy
                offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                v.x = base_x + (len(lane)-1) * CAR_SPACING
                v.y = base_y + offset
            elif direction == 'W':
                base_x = self.cx - QUEUE_OFFSET
                base_y = self.cy
                offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                v.x = base_x - (len(lane)-1) * CAR_SPACING
                v.y = base_y + offset
        else:
            # Single-lane (existing behavior)
            idx = len(self.queues[direction])
            if direction == 'N':
                v.x = self.cx
                v.y = self.cy - QUEUE_OFFSET - idx * CAR_SPACING
            elif direction == 'S':
                v.x = self.cx
                v.y = self.cy + QUEUE_OFFSET + idx * CAR_SPACING
            elif direction == 'E':
                v.x = self.cx + QUEUE_OFFSET + idx * CAR_SPACING
                v.y = self.cy
            elif direction == 'W':
                v.x = self.cx - QUEUE_OFFSET - idx * CAR_SPACING
                v.y = self.cy
            self.queues[direction].append(v)

    def start_crossing_one_vehicle(self, direction, t):
        if self.multiple_lanes:
            # For each lane, if there is a vehicle at the front, try to let it cross.
            lanes = self.queues[direction]
            for lane in lanes:
                if lane:
                    # Peek at the first vehicle in the lane without removing it yet.
                    v = lane[0]
                    if v.turn_direction == "left":
                        # For left turns, check if the left-turn lane is open.
                        if random.random() < LEFT_TURN_OPEN_PROB:
                            # Left-turn lane is open; allow crossing.
                            v = lane.pop(0)
                        else:
                            # Left-turn lane is closed; do not pop vehicle.
                            continue
                    else:
                        # For "straight" or "right" turns, allow crossing.
                        v = lane.pop(0)
                    v.state = 'crossing'
                    v.start_cross_time = t
                    v.finish_time = t + self.crossing_time
                    v.start_position = (v.x, v.y)
                    self.crossing_vehicles.append(v)
        else:
            q = self.queues[direction]
            if q:
                # Peek at the first vehicle
                v = q[0]
                if v.turn_direction == "left":
                    if random.random() < LEFT_TURN_OPEN_PROB:
                        v = q.pop(0)
                    else:
                        # Left-turn lane closed; do not allow crossing.
                        return
                else:
                    v = q.pop(0)
                v.state = 'crossing'
                v.start_cross_time = t
                v.finish_time = t + self.crossing_time
                v.start_position = (v.x, v.y)
                self.crossing_vehicles.append(v)

    def update_crossing_vehicles_step(self):
        finished = []
        for v in self.crossing_vehicles:
            v.cross_progress += 1.0 / float(self.crossing_time)
            if v.cross_progress >= 1.0:
                v.state = 'finished'
                finished.append(v)
        
        for v in finished:
            self.crossing_vehicles.remove(v)
            self.processed_vehicles.append(v)
        
        for d in self.directions:
            self.reposition_queue(d)

    def reposition_queue(self, direction):
        if self.multiple_lanes:
            # Reposition vehicles in each lane for this direction.
            lanes = self.queues[direction]
            lane_gap = 15  # lateral gap between lanes
            for lane_index, lane in enumerate(lanes):
                for idx, v in enumerate(lane):
                    if direction == 'N':
                        base_x = self.cx
                        base_y = self.cy - QUEUE_OFFSET
                        offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                        v.x = base_x + offset
                        v.y = base_y - idx * CAR_SPACING
                    elif direction == 'S':
                        base_x = self.cx
                        base_y = self.cy + QUEUE_OFFSET
                        offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                        v.x = base_x + offset
                        v.y = base_y + idx * CAR_SPACING
                    elif direction == 'E':
                        base_x = self.cx + QUEUE_OFFSET
                        base_y = self.cy
                        offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                        v.x = base_x + idx * CAR_SPACING
                        v.y = base_y + offset
                    elif direction == 'W':
                        base_x = self.cx - QUEUE_OFFSET
                        base_y = self.cy
                        offset = (lane_index - (self.lane_count - 1)/2) * lane_gap
                        v.x = base_x - idx * CAR_SPACING
                        v.y = base_y + offset
        else:
            q = self.queues[direction]
            for idx, v in enumerate(q):
                if direction == 'N':
                    v.x = self.cx
                    v.y = self.cy - QUEUE_OFFSET - idx * CAR_SPACING
                elif direction == 'S':
                    v.x = self.cx
                    v.y = self.cy + QUEUE_OFFSET + idx * CAR_SPACING
                elif direction == 'E':
                    v.x = self.cx + QUEUE_OFFSET + idx * CAR_SPACING
                    v.y = self.cy
                elif direction == 'W':
                    v.x = self.cx - QUEUE_OFFSET - idx * CAR_SPACING
                    v.y = self.cy

    def update_crossing_vehicles_fraction(self, fraction):
        for v in self.crossing_vehicles:
            if v.state == 'crossing' and v.cross_progress < 1.0:
                v.cross_progress += fraction / float(self.crossing_time)
                if v.cross_progress > 1.0:
                    v.cross_progress = 1.0

    def get_green_directions(self, t):
        cycle_pos = t % self.cycle_length
        accum = 0
        for green_dirs, duration in self.phases:
            if cycle_pos < accum + duration:
                return green_dirs
            accum += duration
        return []

    def track_empty_red_time(self, green_dirs):
        for d in self.directions:
            if d not in green_dirs:
                if self.multiple_lanes:
                    qsize = sum(len(lane) for lane in self.queues[d])
                else:
                    qsize = len(self.queues[d])
                if qsize <= self.min_queue_empty:
                    self.red_no_car_time[d] += 1

    def draw(self):
        self.screen.fill(COLOR_BG)
        self.draw_roads()
        # Draw queued vehicles
        for d in self.directions:
            if self.multiple_lanes:
                for lane in self.queues[d]:
                    for v in lane:
                        self.draw_vehicle(v)
            else:
                for v in self.queues[d]:
                    self.draw_vehicle(v)
        # Draw vehicles that are crossing
        for v in self.crossing_vehicles:
            self.draw_crossing_vehicle(v)
        if self.sim_time < self.total_time:
            green_dirs = self.get_green_directions(self.sim_time)
        else:
            green_dirs = []
        txt = f"Time={self.sim_time} Green={green_dirs}"
        self.draw_text(txt, 10, 10)
        pygame.display.flip()

    def draw_roads(self):
        if self.junction_type == "4way":
            pygame.draw.rect(
                self.screen, COLOR_ROAD,
                (0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            )
            pygame.draw.rect(
                self.screen, COLOR_ROAD,
                (self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, SCREEN_HEIGHT)
            )
        else:
            # 3way T-junction
            pygame.draw.rect(
                self.screen, COLOR_ROAD,
                (0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            )
            pygame.draw.rect(
                self.screen, COLOR_ROAD,
                (self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, self.cy + ROAD_WIDTH//2)
            )

    def draw_vehicle(self, v):
        color = COLOR_MAP.get(v.direction, (200, 200, 200))
        pygame.draw.circle(self.screen, color, (int(v.x), int(v.y)), CAR_RADIUS)

    def draw_crossing_vehicle(self, v):
        color = COLOR_MAP.get(v.direction, (200, 200, 200))
        if v.start_position is not None:
            sx, sy = v.start_position
        else:
            sx, sy = self.get_queue_front_position(v.direction)
        ex, ey = self.cx, self.cy
        px = sx + (ex - sx) * v.cross_progress
        py = sy + (ey - sy) * v.cross_progress
        pygame.draw.circle(self.screen, color, (int(px), int(py)), CAR_RADIUS)

    def get_queue_front_position(self, direction):
        if direction == 'N':
            return (self.cx, self.cy - QUEUE_OFFSET)
        elif direction == 'S':
            return (self.cx, self.cy + QUEUE_OFFSET)
        elif direction == 'E':
            return (self.cx + QUEUE_OFFSET, self.cy)
        elif direction == 'W':
            return (self.cx - QUEUE_OFFSET, self.cy)
        return (self.cx, self.cy)

    def draw_text(self, text, x, y):
        surface = self.font.render(text, True, COLOR_TEXT)
        self.screen.blit(surface, (x, y))

    def print_state(self, t, green_dirs):
        print(f"\nTime = {t}, Green = {green_dirs}")
        for d in self.directions:
            if self.multiple_lanes:
                qsize = sum(len(lane) for lane in self.queues[d])
            else:
                qsize = len(self.queues[d])
            avg_wait = self.average_wait_time_for_direction(d)
            print(f"  {d}_queue={qsize}, avg_wait={avg_wait:.2f}")
        for d in self.directions:
            print(f"  {d}_redEmptySoFar={self.red_no_car_time[d]}")

    def average_wait_time_for_direction(self, d):
        done = [v for v in self.processed_vehicles if v.direction == d and v.wait_time is not None]
        if not done:
            return 0.0
        return sum(v.wait_time for v in done) / len(done)

    def overall_average_wait_time(self):
        done = [v for v in self.processed_vehicles if v.wait_time is not None]
        if not done:
            return 0.0
        return sum(v.wait_time for v in done) / len(done)

    def print_statistics(self):
        total_processed = len(self.processed_vehicles)
        avg_wait = self.overall_average_wait_time()
        print("\n=== Simulation Statistics ===")
        print(f"Junction Type: {self.junction_type}")
        print(f"Multiple Lights: {self.multiple_lights}")
        print(f"Multiple Lanes: {self.multiple_lanes} (Lanes per direction: {self.lane_count})")
        print(f"Total Vehicles Processed: {total_processed}")
        print(f"Overall Average Waiting Time: {avg_wait:.2f}")
        print("\nRed-empty stats:")
        for d in self.directions:
            print(f"  {d}: {self.red_no_car_time[d]}")
        print("=============================")

    def get_results_dict(self):
        """
        Return final summary (single-run) metrics for the summary CSV.
        (No timestamps— user doesn't want them.)
        """
        total_processed = len(self.processed_vehicles)
        avg_wait = self.overall_average_wait_time()
        result = {
            "junction_type": self.junction_type,
            "multiple_lights": self.multiple_lights,
            "multiple_lanes": self.multiple_lanes,
            "lane_count": self.lane_count,
            "total_vehicles_processed": total_processed,
            "overall_avg_wait": avg_wait,
        }
        for d in self.directions:
            result[f"red_empty_{d}"] = self.red_no_car_time[d]
        return result


# --------------------------------------
#  MULTIPLE SIMULATIONS & DATA SAVING
# --------------------------------------
def run_multiple_simulations(
    N_runs=5,
    csv_filename="simulation_results.csv",
    junction_type="4way",
    multiple_lights=False,
    total_time=300,
    simulation_speed=0,
    save_to_files=True,
    output_folder="simulation_outputs",
    multiple_lanes=False,   # NEW: toggle for multi-lane simulation
    lane_count=2            # NEW: number of lanes per direction (if multiple_lanes=True)
):
    """
    Run the IntersectionSim N_runs times.
      - If save_to_files=True, we:
        1) Create output_folder if needed
        2) For each run_i:
           - Write a timeseries CSV with data from each 10-timestep interval
           - Append a single summary row to `csv_filename`
        3) Append a final average row
      - If save_to_files=False, no files are created; console only.
    """
    
    directions = ["N", "E", "S", "W"] if junction_type == "4way" else ["N", "E", "W"]
    
    # columns for the single summary CSV
    summary_fieldnames = [
        "junction_type",
        "multiple_lights",
        "multiple_lanes",
        "lane_count",
        "total_vehicles_processed",
        "overall_avg_wait",
    ]
    for d in directions:
        summary_fieldnames.append(f"red_empty_{d}")
    
    if save_to_files:
        os.makedirs(output_folder, exist_ok=True)
    
    single_csv_path = os.path.join(output_folder, csv_filename) if save_to_files else None
    file_exists = False
    if save_to_files and os.path.isfile(single_csv_path):
        file_exists = True
    
    all_results = []
    
    for run_index in range(1, N_runs+1):
        print(f"\n===== Starting Simulation Run {run_index}/{N_runs} =====\n")
        
        sim = IntersectionSim(
            junction_type=junction_type,
            multiple_lights=multiple_lights,
            total_time=total_time,
            simulation_speed=simulation_speed,
            multiple_lanes=multiple_lanes,
            lane_count=lane_count
        )
        sim.run()
        sim.print_statistics()
        
        if save_to_files:
            run_csv_file = os.path.join(output_folder, f"run_{run_index}.csv")
            
            if sim.per_timestep_data:
                timeseries_fieldnames = list(sim.per_timestep_data[0].keys())
            else:
                timeseries_fieldnames = ["time"]
            
            with open(run_csv_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=timeseries_fieldnames)
                writer.writeheader()
                for row in sim.per_timestep_data:
                    writer.writerow(row)
            
            summary_results = sim.get_results_dict()
            with open(single_csv_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
                if (not file_exists) and (run_index == 1):
                    writer.writeheader()
                writer.writerow(summary_results)
                file_exists = True
        
        all_results.append(sim.get_results_dict())
    
    if save_to_files and all_results:
        avg_row = compute_average_row(all_results, directions)
        with open(single_csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
            writer.writerow(avg_row)
        print("\nFinal average row added to summary CSV.\n")


def compute_average_row(all_results, directions):
    n = len(all_results)
    if n == 0:
        return {}
    
    sum_tvp = 0
    sum_wait = 0
    sum_red = {d: 0 for d in directions}
    
    jt = all_results[-1]["junction_type"]
    ml = all_results[-1]["multiple_lights"]
    mlanes = all_results[-1]["multiple_lanes"]
    lane_count = all_results[-1]["lane_count"]
    
    for r in all_results:
        sum_tvp += r["total_vehicles_processed"]
        sum_wait += r["overall_avg_wait"]
        for d in directions:
            sum_red[d] += r[f"red_empty_{d}"]
    
    avg_row = {
        "junction_type": jt,
        "multiple_lights": ml,
        "multiple_lanes": mlanes,
        "lane_count": lane_count,
        "total_vehicles_processed": sum_tvp / n,
        "overall_avg_wait": sum_wait / n,
    }
    for d in directions:
        avg_row[f"red_empty_{d}"] = sum_red[d] / n
    
    return avg_row


# ------------------------------
# EXAMPLE MAIN
# ------------------------------
if __name__ == "__main__":
    # For demonstration: run multiple simulations with timeseries output.
    # Here, we enable the multiple lanes feature and the new turning logic to simulate realistic traffic.
    run_multiple_simulations(
        N_runs=1,
        csv_filename="summary.csv",
        junction_type="4way",
        multiple_lights=True,
        total_time=1000,         # short for demo
        simulation_speed=100,       # fast run
        save_to_files=True,
        output_folder="MySimOutput2",
        multiple_lanes=True,      # Enable multi-lane simulation
        lane_count=2            # Use 2 lanes per direction
    )
