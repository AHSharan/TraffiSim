"""
intersection.py
---------------
Core simulation logic in the IntersectionSim class.
Manages queueing, arrivals, signals, vehicle movements, and statistics.
"""

import random
import math
import csv
import os
import pygame

from .models import Vehicle, DEFAULT_VEHICLE_TYPES, VEHICLE_COLORS
from .utils import dist

LEFT_TURN_OPEN_PROB = 0.5

# Default arrival rates (vehicles/sec)
ARRIVAL_RATES = {
    'N': 0.35,
    'E': 0.25,
    'S': 0.25,
    'W': 0.20
}

# Some default constants for screen size, lane widths, etc.
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800
LANE_WIDTH = 30
ROAD_MARGIN = 10
ROAD_WIDTH = LANE_WIDTH * 4 + ROAD_MARGIN * 2
QUEUE_OFFSET = 200
CAR_SPACING = 25

class IntersectionSim:
    """
    Manages the overall intersection simulation.
    Coordinates vehicle arrivals, traffic signals (adaptive or fixed),
    queues, and data recording.
    """
    def __init__(
        self,
        junction_type="4way",
        multiple_lights=False,
        total_time=300,
        sim_steps_per_sec=10,
        arrival_rates=None,
        print_interval=60,
        min_queue_empty=0,
        simulation_speed=30,
        multiple_lanes=False,
        lane_count=2,
        yellow_duration=5,
        all_red_duration=2,
        vehicle_distribution=None,
        india_mode=False,
        show_visuals=True,
        renderer_class=None,
        simulate_full_route=True,
        adaptive_signals=True
    ):
        if arrival_rates is None:
            arrival_rates = ARRIVAL_RATES

        self.junction_type = junction_type
        self.multiple_lights = multiple_lights
        self.total_time = total_time
        self.sim_steps_per_sec = sim_steps_per_sec
        self.print_interval = print_interval
        self.min_queue_empty = min_queue_empty
        self.simulation_speed = simulation_speed
        self.multiple_lanes = multiple_lanes
        self.lane_count = lane_count
        self.yellow_duration = yellow_duration
        self.all_red_duration = all_red_duration
        self.india_mode = india_mode
        self.show_visuals = show_visuals
        self.simulate_full_route = simulate_full_route
        self.adaptive_signals = adaptive_signals

        self.SCREEN_WIDTH = SCREEN_WIDTH
        self.SCREEN_HEIGHT = SCREEN_HEIGHT

        # Vehicle distribution
        if vehicle_distribution is None:
            self.vehicle_distribution = {
                'car': 0.5,
                'scooter': 0.3,
                'motorcycle': 0.1,
                'truck': 0.05,
                'bus': 0.05
            }
        else:
            self.vehicle_distribution = vehicle_distribution

        self.arrival_rates = arrival_rates
        self.sim_time = 0
        self.running = True

        # Directions for 3way vs 4way
        if junction_type == "3way":
            self.directions = ['N','E','W']
        else:
            self.directions = ['N','E','S','W']

        # Build queues
        if self.multiple_lanes:
            self.queues = {d: [[] for _ in range(self.lane_count)] for d in self.directions}
        else:
            self.queues = {d: [] for d in self.directions}

        self.crossing_vehicles = []
        self.processed_vehicles = []
        self.vehicle_type_counts = {vt: 0 for vt in DEFAULT_VEHICLE_TYPES.keys()}
        self.red_no_car_time = {d: 0 for d in self.directions}
        self.arrivals_count = {d: 0 for d in self.directions}

        # Time-step data recording
        self.per_timestep_data = []

        # If not adaptive, define a fixed cycle
        if not self.adaptive_signals:
            self.phases = []
            self.cycle_length = 0
            self.define_signal_phases()
        else:
            # Adaptive signals
            self.signal_state = "green"   # "green","yellow","all_red"
            self.current_green = None
            self.state_timer = 0
            self.min_green_time = 10
            self.max_green_time = 30

        # Create renderer if desired
        self.renderer = None
        if self.show_visuals and renderer_class:
            self.renderer = renderer_class(self)

    def define_signal_phases(self):
        """
        Define fixed signal phases for the junction.
        Only used if adaptive_signals=False.
        """
        if self.junction_type == "4way":
            if not self.multiple_lights:
                base_green = 30
                self.phases = [
                    {'green': ['N'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration},
                    {'green': ['E'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration},
                    {'green': ['S'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration},
                    {'green': ['W'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration}
                ]
            else:
                base_green = 30
                self.phases = [
                    {'green': ['N','S'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration},
                    {'green': ['E','W'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration,
                     'all_red_duration': self.all_red_duration}
                ]
        else:
            base_green = 30
            self.phases = [
                {'green': ['N'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration,
                 'all_red_duration': self.all_red_duration},
                {'green': ['E'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration,
                 'all_red_duration': self.all_red_duration},
                {'green': ['W'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration,
                 'all_red_duration': self.all_red_duration}
            ]
        self.cycle_length = sum(ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration']
                                for ph in self.phases)

    def update_adaptive_signals(self):
        """
        Adaptive signal logic. Chooses green direction based on queue lengths.
        """
        if self.current_green is None:
            self.choose_new_green()
            self.signal_state = "green"
            return

        self.state_timer -= 1
        if self.state_timer <= 0:
            if self.signal_state == "green":
                self.signal_state = "yellow"
                self.state_timer = self.yellow_duration
            elif self.signal_state == "yellow":
                self.signal_state = "all_red"
                self.state_timer = self.all_red_duration
            elif self.signal_state == "all_red":
                self.choose_new_green()
                self.signal_state = "green"

    def choose_new_green(self):
        """
        Pick the next green direction based on the largest queue 
        (simple adaptive logic).
        """
        max_queue = -1
        candidate = None
        for d in self.directions:
            if self.multiple_lanes:
                q_len = sum(len(lane) for lane in self.queues[d])
            else:
                q_len = len(self.queues[d])
            if q_len > max_queue:
                max_queue = q_len
                candidate = d
        if candidate is None or max_queue == 0:
            candidate = self.directions[0]  # fallback
            green_time = self.min_green_time
        else:
            k = 2  # factor
            green_time = min(self.max_green_time, self.min_green_time + k * max_queue)
        self.current_green = candidate
        self.state_timer = green_time

    def get_signal_state(self, direction, t):
        """
        Return 'green','yellow','red' for a direction at time t in the cycle.
        Only relevant if adaptive_signals=False.
        """
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            g = ph['green_duration']
            y = ph['yellow_duration']
            r = ph['all_red_duration']
            phase_len = g + y + r
            if cycle_pos < accum + phase_len:
                pos_in_ph = cycle_pos - accum
                if direction in ph['green']:
                    if pos_in_ph < g:
                        return "green"
                    elif pos_in_ph < g + y:
                        return "yellow"
                    else:
                        return "red"
                else:
                    return "red"
            accum += phase_len
        return "red"

    def get_green_directions(self, t):
        """
        Return a list of directions that are green at time t 
        if using fixed signals.
        """
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            phase_len = ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration']
            if cycle_pos < accum + phase_len:
                pos_in_ph = cycle_pos - accum
                if pos_in_ph < ph['green_duration']:
                    return ph['green']
                else:
                    return []
            accum += phase_len
        return []

    def run(self):
        """
        Main simulation loop. 
        """
        while self.running:
            if self.renderer is not None:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False

            if self.sim_time < self.total_time:
                self.sim_update()
            else:
                self.running = False

            if self.renderer:
                self.renderer.render()
                if self.simulation_speed > 0:
                    self.renderer.clock.tick(self.simulation_speed)

        if self.renderer:
            pygame.quit()

    def sim_update(self):
        """
        One step update: arrivals, signal updates, queue movement, crossing, stats.
        """
        for d in self.directions:
            self.arrivals_count[d] = 0

        self.generate_arrivals(self.sim_time)

        if self.adaptive_signals:
            self.update_adaptive_signals()
            if self.signal_state == "green":
                green_dirs = [self.current_green]
            else:
                green_dirs = []
        else:
            green_dirs = self.get_green_directions(self.sim_time)

        for d in green_dirs:
            self.start_crossing_one_vehicle(d, self.sim_time)

        self.update_crossing_vehicles()
        self.track_empty_red_time(green_dirs)

        if self.sim_time % self.print_interval == 0:
            self.print_state(self.sim_time, green_dirs)

        self.record_timestep_data()
        self.sim_time += 1

    def generate_arrivals(self, t):
        """
        Generate arrivals according to Poisson process for each direction.
        """
        for d in self.directions:
            rate = self.arrival_rates.get(d, 0)
            arrivals = self.poisson_random(rate)
            self.arrivals_count[d] += arrivals
            for _ in range(arrivals):
                if any(v.direction == d and v.state == 'crossing' for v in self.crossing_vehicles):
                    continue
                vt = random.choices(
                    population=list(self.vehicle_distribution.keys()),
                    weights=list(self.vehicle_distribution.values())
                )[0]
                v = Vehicle(t, d, vt,
                            simulate_full_route=self.simulate_full_route,
                            SCREEN_WIDTH=self.SCREEN_WIDTH,
                            SCREEN_HEIGHT=self.SCREEN_HEIGHT)
                self.vehicle_type_counts[vt] += 1
                self.place_in_queue(v, d)

    def poisson_random(self, rate):
        """
        Basic Poisson random for arrivals: expected value = rate 
        (assuming 1-second intervals).
        """
        L = math.exp(-rate)
        p = 1.0
        k = 0
        while p > L:
            p *= random.random()
            k += 1
        return k - 1

    def place_in_queue(self, v, direction):
        """
        Places a vehicle into the shortest lane (if multiple lanes).
        """
        if self.multiple_lanes:
            L = self.queues[direction]
            lane_index = min(range(self.lane_count), key=lambda i: len(L[i]))
            L[lane_index].append(v)
            v.lane_index = lane_index
        else:
            self.queues[direction].append(v)

    def start_crossing_one_vehicle(self, direction, t):
        """
        Remove exactly one vehicle (front) from the queue if reaction time has passed,
        and start crossing.
        """
        if self.multiple_lanes:
            lanes = self.queues[direction]
            for lane in lanes:
                if lane:
                    front = lane[0]
                    if t - front.arrival_time >= front.reaction_time:
                        if front.turn_direction == 'left':
                            if random.random() >= (LEFT_TURN_OPEN_PROB * front.lane_change_aggressiveness):
                                continue
                            else:
                                front = lane.pop(0)
                        else:
                            front = lane.pop(0)
                        front.state = 'crossing'
                        front.start_cross_time = t
                        front.init_route(self.SCREEN_WIDTH//2, self.SCREEN_HEIGHT//2)
                        self.crossing_vehicles.append(front)
                        break
        else:
            Q = self.queues[direction]
            if Q:
                front = Q[0]
                if t - front.arrival_time >= front.reaction_time:
                    if front.turn_direction == 'left':
                        if random.random() >= LEFT_TURN_OPEN_PROB:
                            return
                        else:
                            front = Q.pop(0)
                    else:
                        front = Q.pop(0)
                    front.state = 'crossing'
                    front.start_cross_time = t
                    front.init_route(self.SCREEN_WIDTH//2, self.SCREEN_HEIGHT//2)
                    self.crossing_vehicles.append(front)

    def update_crossing_vehicles(self):
        """
        Update crossing vehicles (IDM, positions). Remove when done.
        """
        done_list = []
        for v in self.crossing_vehicles:
            is_done = v.update_position(dt=1.0, lead_vehicle=None)
            if is_done:
                done_list.append(v)

        for v in done_list:
            self.crossing_vehicles.remove(v)
            v.state = 'finished'
            self.processed_vehicles.append(v)

        # Reposition queues for visualization
        for d in self.directions:
            self.reposition_queue(d)

    def reposition_queue(self, direction):
        """
        Repositions queued vehicles visually in a line or multi-lane arrangement.
        India mode allows side-by-side for 2-wheelers or smaller vehicles.
        """
        lane_gap = 40
        base_x, base_y = self.SCREEN_WIDTH//2, self.SCREEN_HEIGHT//2

        if self.multiple_lanes:
            for lane_idx, lane in enumerate(self.queues[direction]):
                if self.india_mode:
                    rows = []
                    for veh in lane:
                        if rows:
                            last_row = rows[-1]
                            # Some simplistic logic for side-by-side
                            if (len(last_row) < 3 
                                and all(x.vehicle_type in ['scooter','motorcycle'] for x in last_row)
                                and random.random() < 0.3):
                                last_row.append(veh)
                                veh.row_index = len(rows)-1
                                veh.col_index = len(last_row)-1
                            elif (len(last_row) < 2 
                                  and all(x.vehicle_type in ['scooter','motorcycle','car'] for x in last_row)
                                  and random.random() < 0.5):
                                last_row.append(veh)
                                veh.row_index = len(rows)-1
                                veh.col_index = len(last_row)-1
                            else:
                                rows.append([veh])
                                veh.row_index = len(rows)-1
                                veh.col_index = 0
                        else:
                            rows.append([veh])
                            veh.row_index = 0
                            veh.col_index = 0

                    for row_idx, row in enumerate(rows):
                        offset_lane = (lane_idx - (self.lane_count - 1) / 2.0)*lane_gap
                        for col_idx, veh in enumerate(row):
                            extra_x, extra_y = 0, 0
                            if len(row) == 2:
                                extra_x = -5 if col_idx == 0 else 5
                            elif len(row) == 3:
                                extra_x = -10 + (10*col_idx)
                            if direction == 'N':
                                veh.x = base_x + offset_lane + extra_x
                                veh.y = base_y - QUEUE_OFFSET - row_idx*(CAR_SPACING+5)
                            elif direction == 'S':
                                veh.x = base_x + offset_lane + extra_x
                                veh.y = base_y + QUEUE_OFFSET + row_idx*(CAR_SPACING+5)
                            elif direction == 'E':
                                veh.x = base_x + QUEUE_OFFSET + row_idx*(CAR_SPACING+5)
                                veh.y = base_y + offset_lane + extra_x
                            else: # 'W'
                                veh.x = base_x - QUEUE_OFFSET - row_idx*(CAR_SPACING+5)
                                veh.y = base_y + offset_lane + extra_x
                else:
                    for i, veh in enumerate(lane):
                        offset_lane = (lane_idx - (self.lane_count-1)/2.0)*lane_gap
                        if direction == 'N':
                            veh.x = base_x + offset_lane
                            veh.y = base_y - QUEUE_OFFSET - i*CAR_SPACING
                        elif direction == 'S':
                            veh.x = base_x + offset_lane
                            veh.y = base_y + QUEUE_OFFSET + i*CAR_SPACING
                        elif direction == 'E':
                            veh.x = base_x + QUEUE_OFFSET + i*CAR_SPACING
                            veh.y = base_y + offset_lane
                        else:
                            veh.x = base_x - QUEUE_OFFSET - i*CAR_SPACING
                            veh.y = base_y + offset_lane
        else:
            lane = self.queues[direction]
            if self.india_mode:
                rows = []
                for veh in lane:
                    if rows:
                        last_row = rows[-1]
                        if (len(last_row) < 3 
                            and all(x.vehicle_type in ['scooter','motorcycle','car'] for x in last_row)
                            and random.random() < 0.5):
                            last_row.append(veh)
                            veh.row_index = len(rows)-1
                            veh.col_index = len(last_row)-1
                        else:
                            rows.append([veh])
                            veh.row_index = len(rows)-1
                            veh.col_index = 0
                    else:
                        rows.append([veh])
                        veh.row_index = len(rows)-1
                        veh.col_index = 0
                for row_idx, row in enumerate(rows):
                    for col_idx, veh in enumerate(row):
                        extra_x = 0
                        if len(row) == 2:
                            extra_x = -5 if col_idx == 0 else 5
                        elif len(row) == 3:
                            extra_x = -10 + (10*col_idx)
                        if direction == 'N':
                            veh.x = base_x + extra_x
                            veh.y = base_y - QUEUE_OFFSET - row_idx*(CAR_SPACING+5)
                        elif direction == 'S':
                            veh.x = base_x + extra_x
                            veh.y = base_y + QUEUE_OFFSET + row_idx*(CAR_SPACING+5)
                        elif direction == 'E':
                            veh.x = base_x + QUEUE_OFFSET + row_idx*(CAR_SPACING+5)
                            veh.y = base_y + extra_x
                        else:
                            veh.x = base_x - QUEUE_OFFSET - row_idx*(CAR_SPACING+5)
                            veh.y = base_y + extra_x
            else:
                for i, veh in enumerate(lane):
                    if direction == 'N':
                        veh.x = base_x
                        veh.y = base_y - QUEUE_OFFSET - i*CAR_SPACING
                    elif direction == 'S':
                        veh.x = base_x
                        veh.y = base_y + QUEUE_OFFSET + i*CAR_SPACING
                    elif direction == 'E':
                        veh.x = base_x + QUEUE_OFFSET + i*CAR_SPACING
                        veh.y = base_y
                    else:
                        veh.x = base_x - QUEUE_OFFSET - i*CAR_SPACING
                        veh.y = base_y

    def track_empty_red_time(self, green_dirs):
        """
        Increment counters for time steps when a direction is red AND has no cars in queue.
        """
        for d in self.directions:
            if d not in green_dirs:
                if self.multiple_lanes:
                    size = sum(len(x) for x in self.queues[d])
                else:
                    size = len(self.queues[d])
                if size <= self.min_queue_empty:
                    self.red_no_car_time[d] += 1

    def print_state(self, t, green_dirs):
        if self.adaptive_signals:
            sig_info = f"(Signal: {self.signal_state.upper()} for {self.current_green}, timer={self.state_timer})"
        else:
            sig_info = ""
        print(f"Time={t}, green={green_dirs} {sig_info}")
        for d in self.directions:
            if self.multiple_lanes:
                qsize = sum(len(x) for x in self.queues[d])
            else:
                qsize = len(self.queues[d])
            aw = self.average_wait_time_for_direction(d)
            print(f"  {d} queue={qsize}, avg_wait={aw:.2f}")

    def record_timestep_data(self):
        row = {"TimeStep": self.sim_time}
        for d in self.directions:
            if self.multiple_lanes:
                row[f"Queue{d}"] = sum(len(L) for L in self.queues[d])
            else:
                row[f"Queue{d}"] = len(self.queues[d])
            row[f"Arrivals{d}"] = self.arrivals_count[d]
            row[f"AvgWait{d}"] = self.average_wait_time_for_direction(d)
        row["CrossingCount"] = len(self.crossing_vehicles)
        row["ProcessedCount"] = len(self.processed_vehicles)
        row["OverallAvgWait"] = self.overall_average_wait_time()
        self.per_timestep_data.append(row)

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
        print("\n=== Simulation Stats ===")
        print(f"JunctionType: {self.junction_type}")
        print(f"MultipleLights: {self.multiple_lights}")
        print(f"MultipleLanes: {self.multiple_lanes} (LaneCount={self.lane_count})")
        print(f"SimulateFullRoute: {self.simulate_full_route}")
        print(f"AdaptiveSignals: {self.adaptive_signals}")
        print(f"TotalVehiclesProcessed: {total_processed}")
        print(f"OverallAvgWait: {avg_wait:.2f}")
        print("\nVehicle Type Counts:")
        for vt, cnt in self.vehicle_type_counts.items():
            print(f"  {vt}: {cnt}")
        print("\nRed-empty stats:")
        for d in self.directions:
            print(f"  {d}: {self.red_no_car_time[d]}")
        print("========================")

    def get_results_dict(self):
        total_processed = len(self.processed_vehicles)
        avg_wait = self.overall_average_wait_time()
        result = {
            "JunctionType": self.junction_type,
            "MultipleLights": self.multiple_lights,
            "MultipleLanes": self.multiple_lanes,
            "LaneCount": self.lane_count,
            "TotalVehiclesProcessed": total_processed,
            "OverallAvgWait": avg_wait,
            "SimulateFullRoute": self.simulate_full_route
        }
        for d in self.directions:
            result[f"RedEmpty{d}"] = self.red_no_car_time[d]
        for vt in DEFAULT_VEHICLE_TYPES.keys():
            key = "Count" + vt.capitalize()
            result[key] = self.vehicle_type_counts[vt]
        return result
