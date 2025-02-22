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
TOTAL_TIME = 300
SIM_STEPS_PER_SEC = 10

# Base arrival rates (vehicles/sec) per approach
ARRIVAL_RATES = {
    'N': 0.25,
    'E': 0.25,
    'S': 0.25,
    'W': 0.20
}

CROSSING_TIME = 4  # Not used for final route but retained for reference
PRINT_INTERVAL = 60
MIN_QUEUE_EMPTY = 0
TIMESERIES_INTERVAL = 10

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

ROAD_WIDTH = 100
QUEUE_OFFSET = 140
CAR_SPACING = 20

LEFT_TURN_OPEN_PROB = 0.5

# Default vehicle types
DEFAULT_VEHICLE_TYPES = {
    'car': {
        'desired_speed': 20,
        'max_acceleration': 2.5,
        'comfortable_deceleration': 2.5,
        'minimum_gap': 2.5,
        'lane_change_aggressiveness': 0.7,
        'length': 4.5
    },
    'truck': {
        'desired_speed': 15,
        'max_acceleration': 1.5,
        'comfortable_deceleration': 1.5,
        'minimum_gap': 3.5,
        'lane_change_aggressiveness': 0.5,
        'length': 10.0
    },
    'bus': {
        'desired_speed': 15,
        'max_acceleration': 1.5,
        'comfortable_deceleration': 1.5,
        'minimum_gap': 4.0,
        'lane_change_aggressiveness': 0.5,
        'length': 12.0
    },
    'scooter': {
        'desired_speed': 18,
        'max_acceleration': 3.0,
        'comfortable_deceleration': 3.0,
        'minimum_gap': 1.2,
        'lane_change_aggressiveness': 0.8,
        'length': 2.0
    },
    'motorcycle': {
        'desired_speed': 22,
        'max_acceleration': 3.5,
        'comfortable_deceleration': 3.5,
        'minimum_gap': 1.0,
        'lane_change_aggressiveness': 0.9,
        'length': 2.2
    }
}

# Colors for different vehicle types
VEHICLE_COLORS = {
    'car':        (200, 200, 0),
    'truck':      (180, 100, 50),
    'bus':        (120, 40, 150),
    'scooter':    (40, 220, 220),
    'motorcycle': (255, 100, 100),
}

# ------------------------------
# HELPER FUNCTIONS
# ------------------------------
def dist(a, b):
    """Euclidean distance between points a and b."""
    return math.hypot(a[0] - b[0], a[1] - b[1])

def define_exit_point(cx, cy, direction, turn):
    """
    Return the final exit coordinate for a vehicle that starts in 'direction'
    and picks 'turn' among {'left','straight','right'}.
    Used for full-route mode.
    """
    margin = 50
    if direction == 'N':
        if turn == 'left':
            return (-margin, cy)
        elif turn == 'right':
            return (SCREEN_WIDTH + margin, cy)
        else:
            return (cx, SCREEN_HEIGHT + margin)
    elif direction == 'S':
        if turn == 'left':
            return (SCREEN_WIDTH + margin, cy)
        elif turn == 'right':
            return (-margin, cy)
        else:
            return (cx, -margin)
    elif direction == 'E':
        if turn == 'left':
            return (cx, -margin)
        elif turn == 'right':
            return (cx, SCREEN_HEIGHT + margin)
        else:
            return (-margin, cy)
    elif direction == 'W':
        if turn == 'left':
            return (cx, SCREEN_HEIGHT + margin)
        elif turn == 'right':
            return (cx, -margin)
        else:
            return (SCREEN_WIDTH + margin, cy)
    return (cx, cy)

# ------------------------------
# VEHICLE CLASS
# ------------------------------
class Vehicle:
    """
    Represents a single vehicle, including parameters for speed/acceleration,
    position updates, and optional partial-route logic if simulate_full_route=False.
    """
    def __init__(self, arrival_time, direction, vehicle_type=None, simulate_full_route=True):
        self.arrival_time = arrival_time
        self.direction = direction  # 'N','S','E','W'
        self.turn_direction = random.choices(
            ['left', 'straight', 'right'],
            weights=[0.3, 0.5, 0.2]
        )[0]

        # Vehicle type logic
        if vehicle_type is None:
            vehicle_type = random.choice(list(DEFAULT_VEHICLE_TYPES.keys()))
        self.vehicle_type = vehicle_type
        params = DEFAULT_VEHICLE_TYPES[self.vehicle_type]

        self.desired_speed = params['desired_speed']
        self.max_acceleration = params['max_acceleration']
        self.comfortable_deceleration = params['comfortable_deceleration']
        self.minimum_gap = params['minimum_gap']
        self.lane_change_aggressiveness = params['lane_change_aggressiveness']
        self.length = params['length']

        self.reaction_time = random.uniform(0.8, 1.5)
        self.state = 'queueing'  # 'queueing', 'crossing', or 'finished'

        # For queue positioning
        self.lane_index = None
        self.row_index = None
        self.col_index = None

        # Position & movement
        self.x = 0
        self.y = 0
        self.current_speed = 0

        # Route details
        self.start_position = None
        self.center_position = None
        self.exit_position = None
        self.route_distance = 0
        self.distance_covered = 0

        self.start_cross_time = None
        self.finish_time = None

        # Should we simulate full route or vanish at center?
        self.simulate_full_route = simulate_full_route

    @property
    def wait_time(self):
        if self.start_cross_time is None:
            return None
        return self.start_cross_time - self.arrival_time

    def init_route(self, center_x, center_y):
        """
        Called when vehicle first starts crossing:
          - sets start_position
          - sets center_position
          - sets exit_position (if simulate_full_route=True)
          - calculates total route distance
        """
        self.start_position = (self.x, self.y)
        self.center_position = (center_x, center_y)

        if self.simulate_full_route:
            # Define a real exit point off-screen
            self.exit_position = define_exit_point(center_x, center_y, self.direction, self.turn_direction)
            d1 = dist(self.start_position, self.center_position)
            d2 = dist(self.center_position, self.exit_position)
            self.route_distance = d1 + d2
        else:
            # Only go from start -> center, then disappear
            self.exit_position = self.center_position  # For consistency
            self.route_distance = dist(self.start_position, self.center_position)

        self.distance_covered = 0.0

    def update_position(self, dt=1.0):
        """
        Accelerate (up to desired_speed), move forward, update (x,y).
        Return True if done with route.
        If not simulate_full_route, "done" once we've reached the center.
        """
        # Basic acceleration
        self.current_speed = min(self.current_speed + self.max_acceleration * dt, self.desired_speed)
        dist_step = self.current_speed * dt
        self.distance_covered += dist_step
        frac = self.distance_covered / self.route_distance

        # Once fraction >= 1, route is complete
        if frac >= 1.0:
            frac = 1.0

        # Piecewise linear: start->center, center->exit
        d1 = dist(self.start_position, self.center_position)
        r1 = d1 / self.route_distance

        # If not simulate_full_route, we only have start->center
        # so r1 should be 1.0.  We'll handle both cases the same way,
        # but if simulate_full_route=False, r1 = 1 => only one segment.
        if frac < r1:
            # inbound segment
            subFrac = frac / r1  # goes from 0..1 in first segment
            x0, y0 = self.start_position
            x1, y1 = self.center_position
            self.x = x0 + (x1 - x0) * subFrac
            self.y = y0 + (y1 - y0) * subFrac
        else:
            # outbound segment (only relevant if simulate_full_route=True)
            subFrac = (frac - r1) / (1.0 - r1) if r1 < 1.0 else 1.0
            x1, y1 = self.center_position
            x2, y2 = self.exit_position
            self.x = x1 + (x2 - x1) * subFrac
            self.y = y1 + (y2 - y1) * subFrac

        # Return True if the entire route is complete
        return (frac >= 1.0)


# ------------------------------
# INTERSECTION SIMULATION
# ------------------------------
class IntersectionSim:
    """
    Main simulation logic. The parameter 'simulate_full_route'
    determines if vehicles vanish at the intersection center or
    continue off-screen.
    """
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
        lane_count=2,
        yellow_duration=5,
        all_red_duration=2,
        vehicle_distribution=None,
        india_mode=False,
        show_visuals=True,
        renderer_class=None,
        # New param: let user choose if vehicles disappear at center or go off-screen
        simulate_full_route=True
    ):
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
        self.multiple_lanes = multiple_lanes
        self.lane_count = lane_count
        self.yellow_duration = yellow_duration
        self.all_red_duration = all_red_duration
        self.india_mode = india_mode
        self.show_visuals = show_visuals

        # New: store the route mode
        self.simulate_full_route = simulate_full_route

        # If not specified, default distribution
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

        # Count how many vehicles of each type
        self.vehicle_type_counts = {vt: 0 for vt in DEFAULT_VEHICLE_TYPES.keys()}

        # Directions
        if junction_type == "3way":
            self.directions = ['N', 'E', 'W']
        else:
            self.directions = ['N', 'E', 'S', 'W']

        # Build queue structures
        if self.multiple_lanes:
            self.queues = {d: [[] for _ in range(self.lane_count)] for d in self.directions}
        else:
            self.queues = {d: [] for d in self.directions}

        self.crossing_vehicles = []
        self.processed_vehicles = []

        self.sim_time = 0
        self.running = True

        self.phases = []
        self.cycle_length = 0
        self.define_signal_phases()

        self.red_no_car_time = {d: 0 for d in self.directions}
        self.arrivals_count = {d: 0 for d in self.directions}
        self.per_timestep_data = []

        # Optional rendering
        self.renderer = None
        if self.show_visuals and renderer_class:
            self.renderer = renderer_class(self)

    def define_signal_phases(self):
        """Same logic as before, define phases, compute cycle_length."""
        if self.junction_type == "4way":
            if not self.multiple_lights:
                base_green = 30
                self.phases = [
                    {'green': ['N'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['S'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['W'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
            else:
                base_green = 30
                self.phases = [
                    {'green': ['N', 'S'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E', 'W'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
        else:
            # 3way
            if not self.multiple_lights:
                base_green = 30
                self.phases = [
                    {'green': ['N'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['W'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
            else:
                base_green = 30
                self.phases = [
                    {'green': ['N', 'E'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E', 'W'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['W', 'N'], 'green_duration': base_green, 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
        self.cycle_length = sum(ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration'] for ph in self.phases)

    def get_green_directions(self, t):
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            ph_total = ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration']
            if cycle_pos < accum + ph_total:
                time_in_phase = cycle_pos - accum
                if time_in_phase < ph['green_duration']:
                    return ph['green']
                else:
                    return []
            accum += ph_total
        return []

    def get_signal_state(self, direction, t):
        """Return 'green','yellow','red' for the direction at time t."""
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            g = ph['green_duration']
            y = ph['yellow_duration']
            r = ph['all_red_duration']
            ph_total = g + y + r
            if cycle_pos < accum + ph_total:
                time_in_phase = cycle_pos - accum
                if direction in ph['green']:
                    if time_in_phase < g:
                        return "green"
                    elif time_in_phase < g + y:
                        return "yellow"
                    else:
                        return "red"
                else:
                    return "red"
            accum += ph_total
        return "red"

    def run(self):
        sim_step_acc = 0.0
        steps_per_frame = float(self.sim_steps_per_sec) / (self.simulation_speed if self.simulation_speed > 0 else 1.0)
        while self.running:
            if self.renderer is not None:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        self.running = False

            if self.sim_time < self.total_time:
                if self.sim_speed_is_real_time():
                    sim_step_acc += steps_per_frame
                    while sim_step_acc >= 1.0 and self.sim_time < self.total_time:
                        self.sim_update()
                        sim_step_acc -= 1.0
                    fraction = min(sim_step_acc, 1.0)
                    self.update_crossing_vehicles_fraction(fraction)
                else:
                    # fast-forward
                    while self.sim_time < self.total_time:
                        self.sim_update()
            else:
                self.running = False

            # Render if needed
            if self.renderer is not None:
                self.renderer.render()
                if self.sim_speed_is_real_time():
                    self.renderer.clock.tick(self.simulation_speed)

        if self.renderer:
            pygame.quit()

    def sim_speed_is_real_time(self):
        return (self.simulation_speed > 0)

    def sim_update(self):
        for d in self.directions:
            self.arrivals_count[d] = 0

        self.generate_arrivals(self.sim_time)

        # Let front vehicles cross if light is green
        green_dirs = self.get_green_directions(self.sim_time)
        for d in green_dirs:
            self.start_crossing_one_vehicle(d, self.sim_time)

        self.update_crossing_vehicles_step()

        # Track red-empty
        self.track_empty_red_time(green_dirs)

        # Print occasionally
        if self.sim_time % self.print_interval == 0:
            self.print_state(self.sim_time, green_dirs)

        self.record_timestep_data()
        self.sim_time += 1

    def update_crossing_vehicles_fraction(self, fraction):
        """
        For partial frame updates in real time (fractional dt).
        """
        done_list = []
        for v in self.crossing_vehicles:
            done = v.update_position(dt=fraction)
            if done:
                done_list.append(v)
        for v in done_list:
            self.crossing_vehicles.remove(v)
            v.state = 'finished'
            self.processed_vehicles.append(v)

    def generate_arrivals(self, t):
        for d in self.directions:
            rate = self.arrival_rates.get(d, 0)
            mult = self.time_multiplier(t)
            arrivals = self.poisson_random(rate * mult)
            self.arrivals_count[d] += arrivals
            for _ in range(arrivals):
                vt = random.choices(
                    population=list(self.vehicle_distribution.keys()),
                    weights=list(self.vehicle_distribution.values())
                )[0]
                # Create a vehicle, specifying simulate_full_route from the IntersectionSim
                v = Vehicle(t, d, vehicle_type=vt, simulate_full_route=self.simulate_full_route)
                self.vehicle_type_counts[vt] += 1
                self.place_in_queue(v, d)

    def place_in_queue(self, v, direction):
        if self.multiple_lanes:
            L = self.queues[direction]
            lane_index = min(range(self.lane_count), key=lambda i: len(L[i]))
            L[lane_index].append(v)
            v.lane_index = lane_index
        else:
            self.queues[direction].append(v)

    def poisson_random(self, rate):
        L = math.exp(-rate)
        p = 1.0
        k = 0
        while p > L:
            p *= random.random()
            k += 1
        return k - 1

    def start_crossing_one_vehicle(self, direction, t):
        """
        Move exactly one vehicle from queue into crossing if reaction time is met
        and if left-turn is permissible with some probability.
        """
        if self.multiple_lanes:
            lanes = self.queues[direction]
            for lane in lanes:
                if lane:
                    v = lane[0]
                    if t - v.arrival_time < v.reaction_time:
                        continue
                    if v.turn_direction == 'left':
                        # left-turn logic
                        if random.random() >= LEFT_TURN_OPEN_PROB * v.lane_change_aggressiveness:
                            continue
                        else:
                            v = lane.pop(0)
                    else:
                        v = lane.pop(0)
                    v.state = 'crossing'
                    v.start_cross_time = t
                    v.init_route(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
                    self.crossing_vehicles.append(v)
                    break
        else:
            Q = self.queues[direction]
            if Q:
                v = Q[0]
                if t - v.arrival_time < v.reaction_time:
                    return
                if v.turn_direction == 'left':
                    if random.random() >= LEFT_TURN_OPEN_PROB:
                        return
                    else:
                        v = Q.pop(0)
                else:
                    v = Q.pop(0)
                v.state = 'crossing'
                v.start_cross_time = t
                v.init_route(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
                self.crossing_vehicles.append(v)

    def update_crossing_vehicles_step(self):
        done_list = []
        for v in self.crossing_vehicles:
            done = v.update_position(dt=1.0)
            if done:
                done_list.append(v)
        for v in done_list:
            self.crossing_vehicles.remove(v)
            v.state = 'finished'
            self.processed_vehicles.append(v)

        # Reposition queues
        for d in self.directions:
            self.reposition_queue(d)

    def reposition_queue(self, direction):
        """
        Positions queued vehicles in lines, with optional 'India Mode' for
        pairing two-wheelers side-by-side. Slightly offset per lane if multiple lanes.
        """
        lane_gap = 15
        if self.multiple_lanes:
            for lane_index, lane in enumerate(self.queues[direction]):
                if self.india_mode:
                    rows = []
                    for x in lane:
                        if x.vehicle_type in ['scooter', 'motorcycle']:
                            if rows and all(xx.vehicle_type in ['scooter', 'motorcycle'] for xx in rows[-1]) and len(rows[-1]) < 2:
                                rows[-1].append(x)
                                x.row_index = len(rows) - 1
                                x.col_index = len(rows[-1]) - 1
                            else:
                                rows.append([x])
                                x.row_index = len(rows) - 1
                                x.col_index = 0
                        else:
                            rows.append([x])
                            x.row_index = len(rows) - 1
                            x.col_index = 0
                    for row_idx, row in enumerate(rows):
                        for v in row:
                            base_x, base_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
                            offset_lane = (lane_index - (self.lane_count - 1) / 2) * lane_gap
                            extra = 0
                            if len(row) == 2:
                                extra = -5 if v.col_index == 0 else 5
                            if direction == 'N':
                                v.x = base_x + offset_lane + extra
                                v.y = base_y - QUEUE_OFFSET - row_idx * max(CAR_SPACING, v.minimum_gap)
                            elif direction == 'S':
                                v.x = base_x + offset_lane + extra
                                v.y = base_y + QUEUE_OFFSET + row_idx * max(CAR_SPACING, v.minimum_gap)
                            elif direction == 'E':
                                v.x = base_x + QUEUE_OFFSET + row_idx * max(CAR_SPACING, v.minimum_gap)
                                v.y = base_y + offset_lane + extra
                            elif direction == 'W':
                                v.x = base_x - QUEUE_OFFSET - row_idx * max(CAR_SPACING, v.minimum_gap)
                                v.y = base_y + offset_lane + extra
                else:
                    # single-file
                    for idx, v in enumerate(lane):
                        base_x, base_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
                        offset_lane = (lane_index - (self.lane_count - 1) / 2) * lane_gap
                        if direction == 'N':
                            v.x = base_x + offset_lane
                            v.y = base_y - QUEUE_OFFSET - idx * CAR_SPACING
                        elif direction == 'S':
                            v.x = base_x + offset_lane
                            v.y = base_y + QUEUE_OFFSET + idx * CAR_SPACING
                        elif direction == 'E':
                            v.x = base_x + QUEUE_OFFSET + idx * CAR_SPACING
                            v.y = base_y + offset_lane
                        elif direction == 'W':
                            v.x = base_x - QUEUE_OFFSET - idx * CAR_SPACING
                            v.y = base_y + offset_lane
        else:
            # single-lane
            lane = self.queues[direction]
            if self.india_mode:
                rows = []
                for x in lane:
                    if x.vehicle_type in ['scooter', 'motorcycle']:
                        if rows and all(xx.vehicle_type in ['scooter', 'motorcycle'] for xx in rows[-1]) and len(rows[-1]) < 2:
                            rows[-1].append(x)
                            x.row_index = len(rows) - 1
                            x.col_index = len(rows[-1]) - 1
                        else:
                            rows.append([x])
                            x.row_index = len(rows) - 1
                            x.col_index = 0
                    else:
                        rows.append([x])
                        x.row_index = len(rows) - 1
                        x.col_index = 0
                for row_idx, row in enumerate(rows):
                    base_x, base_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
                    for v in row:
                        extra = -5 if (len(row) == 2 and v.col_index == 0) else (5 if (len(row) == 2 and v.col_index == 1) else 0)
                        if direction == 'N':
                            v.x = base_x + extra
                            v.y = base_y - QUEUE_OFFSET - row_idx * max(CAR_SPACING, v.minimum_gap)
                        elif direction == 'S':
                            v.x = base_x + extra
                            v.y = base_y + QUEUE_OFFSET + row_idx * max(CAR_SPACING, v.minimum_gap)
                        elif direction == 'E':
                            v.x = base_x + QUEUE_OFFSET + row_idx * max(CAR_SPACING, v.minimum_gap)
                            v.y = base_y + extra
                        elif direction == 'W':
                            v.x = base_x - QUEUE_OFFSET - row_idx * max(CAR_SPACING, v.minimum_gap)
                            v.y = base_y + extra
            else:
                for idx, v in enumerate(lane):
                    base_x, base_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
                    if direction == 'N':
                        v.x = base_x
                        v.y = base_y - QUEUE_OFFSET - idx * CAR_SPACING
                    elif direction == 'S':
                        v.x = base_x
                        v.y = base_y + QUEUE_OFFSET + idx * CAR_SPACING
                    elif direction == 'E':
                        v.x = base_x + QUEUE_OFFSET + idx * CAR_SPACING
                        v.y = base_y
                    elif direction == 'W':
                        v.x = base_x - QUEUE_OFFSET - idx * CAR_SPACING
                        v.y = base_y

    def track_empty_red_time(self, green_dirs):
        for d in self.directions:
            if d not in green_dirs:
                if self.multiple_lanes:
                    qsize = sum(len(L) for L in self.queues[d])
                else:
                    qsize = len(self.queues[d])
                if qsize <= self.min_queue_empty:
                    self.red_no_car_time[d] += 1

    def time_multiplier(self, t):
        # Example: heavier traffic between t=100..200
        if 100 <= t <= 200:
            return 1.5
        return 1.0

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

    def print_state(self, t, green_dirs):
        print(f"\nTime={t}, green={green_dirs}")
        for d in self.directions:
            if self.multiple_lanes:
                qsize = sum(len(L) for L in self.queues[d])
            else:
                qsize = len(self.queues[d])
            aw = self.average_wait_time_for_direction(d)
            print(f"  {d}_queue={qsize}, avg_wait={aw:.2f}")
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
        print("\n=== Simulation Stats ===")
        print(f"JunctionType: {self.junction_type}")
        print(f"MultipleLights: {self.multiple_lights}")
        print(f"MultipleLanes: {self.multiple_lanes} (LaneCount={self.lane_count})")
        print(f"SimulateFullRoute: {self.simulate_full_route}")
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
        for vt, cnt in self.vehicle_type_counts.items():
            key = "Count" + vt.capitalize()
            result[key] = cnt
        return result


# ------------------------------
# RENDERING CLASS
# ------------------------------
class TrafficRenderer:
    """
    Basic Pygame rendering of the roads, vehicles, and signals.
    """
    COLOR_BG = (30, 30, 30)
    COLOR_ROAD = (70, 70, 70)
    COLOR_TEXT = (255, 255, 255)
    LINE_COLOR = (100, 100, 100)

    TRAFFIC_LIGHT_COLORS = {
        "red":    (255,  40,  40),
        "yellow": (255, 255,   0),
        "green":  ( 40, 255,  40),
    }

    def __init__(self, sim):
        self.sim = sim
        pygame.init()
        pygame.display.set_caption(f"Traffic Sim (type={sim.junction_type}, MLights={sim.multiple_lights})")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)

        self.cx = SCREEN_WIDTH // 2
        self.cy = SCREEN_HEIGHT // 2

    def render(self):
        self.screen.fill(self.COLOR_BG)
        self.draw_radial_gradient()
        self.draw_roads_with_lane_markings()
        self.draw_divider_line()
        self.draw_traffic_lights()
        self.draw_vehicles()
        self.draw_ui_panel()
        pygame.display.flip()

    def draw_radial_gradient(self):
        center = (self.cx, self.cy)
        maxr = int(math.hypot(self.cx, self.cy))
        step = 14
        for r in range(maxr, 0, -step):
            frac = r / float(maxr)
            color = (
                int(self.COLOR_BG[0] * frac),
                int(self.COLOR_BG[1] * frac),
                int(self.COLOR_BG[2] * frac)
            )
            pygame.draw.circle(self.screen, color, center, r)

    def draw_roads_with_lane_markings(self):
        if self.sim.junction_type == "4way":
            pygame.draw.rect(self.screen, self.COLOR_ROAD,
                             (0, self.cy - ROAD_WIDTH // 2, SCREEN_WIDTH, ROAD_WIDTH))
            pygame.draw.rect(self.screen, self.COLOR_ROAD,
                             (self.cx - ROAD_WIDTH // 2, 0, ROAD_WIDTH, SCREEN_HEIGHT))
        else:
            pygame.draw.rect(self.screen, self.COLOR_ROAD,
                             (0, self.cy - ROAD_WIDTH // 2, SCREEN_WIDTH, ROAD_WIDTH))
            pygame.draw.rect(self.screen, self.COLOR_ROAD,
                             (self.cx - ROAD_WIDTH // 2, 0, ROAD_WIDTH, self.cy + ROAD_WIDTH // 2))

        step = ROAD_WIDTH / 2
        if self.sim.junction_type == "4way":
            for i in range(-1, 2, 2):
                offset = self.cx + i * step / 2
                pygame.draw.line(self.screen, self.LINE_COLOR, (offset, 0), (offset, SCREEN_HEIGHT), 1)
        else:
            for i in range(-1, 2, 2):
                offset = self.cx + i * step / 2
                pygame.draw.line(self.screen, self.LINE_COLOR, (offset, 0),
                                 (offset, self.cy + ROAD_WIDTH // 2), 1)

        for i in range(-1, 2, 2):
            offset = self.cy + i * step / 2
            pygame.draw.line(self.screen, self.LINE_COLOR, (0, offset), (SCREEN_WIDTH, offset), 1)

    def draw_divider_line(self):
        dash_len = 10
        gap_len = 6
        # horizontal center line
        line_y = self.cy
        x = 0
        while x < SCREEN_WIDTH:
            pygame.draw.line(self.screen, (160, 160, 160), (x, line_y),
                             (min(x + dash_len, SCREEN_WIDTH), line_y), 1)
            x += dash_len + gap_len

        # vertical center line (for 4-way)
        if self.sim.junction_type == "4way":
            line_x = self.cx
            y = 0
            while y < SCREEN_HEIGHT:
                pygame.draw.line(self.screen, (160, 160, 160), (line_x, y),
                                 (line_x, min(y + dash_len, SCREEN_HEIGHT)), 1)
                y += dash_len + gap_len

    def draw_traffic_lights(self):
        offsets = {
            'N': (0, -50),
            'S': (0, 50),
            'E': (50, 0),
            'W': (-50, 0)
        }
        r = 8
        for d in self.sim.directions:
            st = self.sim.get_signal_state(d, self.sim.sim_time)
            color = self.TRAFFIC_LIGHT_COLORS[st]
            ox, oy = offsets.get(d, (0, 0))
            cx = self.cx + ox
            cy = self.cy + oy
            pygame.draw.circle(self.screen, (0, 0, 0), (cx, cy), r + 2)
            pygame.draw.circle(self.screen, color, (cx, cy), r)

    def draw_vehicles(self):
        # Queue vehicles
        for d in self.sim.directions:
            if self.sim.multiple_lanes:
                for lane in self.sim.queues[d]:
                    for v in lane:
                        self.draw_single_vehicle(v)
            else:
                for v in self.sim.queues[d]:
                    self.draw_single_vehicle(v)
        # Crossing vehicles
        for v in self.sim.crossing_vehicles:
            self.draw_single_vehicle(v)

    def draw_single_vehicle(self, v):
        color = VEHICLE_COLORS.get(v.vehicle_type, (200, 200, 200))
        shadow_color = (color[0] // 4, color[1] // 4, color[2] // 4)

        if v.vehicle_type in ['car', 'truck', 'bus']:
            self.draw_rect_vehicle(v, color, shadow_color)
        else:
            self.draw_circle_vehicle(v, color, shadow_color)

    def draw_rect_vehicle(self, v, color, shadow_color):
        if v.vehicle_type == 'car':
            w, h = (10, 18)
        elif v.vehicle_type == 'truck':
            w, h = (14, 30)
        else:  # bus
            w, h = (14, 35)
        rect = pygame.Rect(0, 0, w, h)
        rect.center = (int(v.x), int(v.y))

        shadow_rect = rect.copy()
        shadow_rect.x += 2
        shadow_rect.y += 2

        pygame.draw.rect(self.screen, shadow_color, shadow_rect, border_radius=3)
        pygame.draw.rect(self.screen, color, rect, border_radius=3)

    def draw_circle_vehicle(self, v, color, shadow_color):
        radius = 4 if (v.vehicle_type == 'scooter') else 5
        center = (int(v.x), int(v.y))
        shadow_center = (center[0] + 2, center[1] + 2)

        pygame.draw.circle(self.screen, shadow_color, shadow_center, radius)
        pygame.draw.circle(self.screen, color, center, radius)

    def draw_ui_panel(self):
        lines = [
            f"Time= {self.sim.sim_time}/{self.sim.total_time}",
            (f"Speed= {self.sim.simulation_speed} steps/sec"
             if self.sim.sim_speed_is_real_time() else "Speed=Unlimited"),
            f"JunctionType={self.sim.junction_type}, MultiLights={self.sim.multiple_lights}",
            f"MultiLanes={self.sim.multiple_lanes}, LaneCount={self.sim.lane_count}",
            f"IndiaMode={self.sim.india_mode}",
            f"SimulateFullRoute={self.sim.simulate_full_route}",
            "Press ESC or close window to quit."
        ]
        x = 10
        y = 10
        for txt in lines:
            surf = self.font.render(txt, True, (255, 255, 255))
            self.screen.blit(surf, (x, y))
            y += 20

# ------------------------------
# MULTIPLE SIMULATIONS + SAVING
# ------------------------------
def run_multiple_simulations(
    N_runs=5,
    csv_filename="simulation_results.csv",
    junction_type="4way",
    multiple_lights=False,
    total_time=300,
    simulation_speed=0,
    save_to_files=True,
    output_folder="simulation_outputs",
    multiple_lanes=False,
    lane_count=2,
    yellow_duration=5,
    all_red_duration=2,
    vehicle_distribution=None,
    india_mode=False,
    show_visuals=True,
    simulate_full_route=True  # NEW: Option to vanish at center or go off-screen
):
    directions = ['N', 'E', 'S', 'W'] if junction_type == '4way' else ['N', 'E', 'W']

    summary_fieldnames = [
        "SimulationRun",
        "JunctionType",
        "MultipleLights",
        "MultipleLanes",
        "LaneCount",
        "TotalVehiclesProcessed",
        "OverallAvgWait",
        "SimulateFullRoute"
    ]
    for d in directions:
        summary_fieldnames.append(f"RedEmpty{d}")

    for vt in DEFAULT_VEHICLE_TYPES.keys():
        summary_fieldnames.append(f"Count{vt.capitalize()}")

    if save_to_files:
        os.makedirs(output_folder, exist_ok=True)
    summary_csv_path = os.path.join(output_folder, csv_filename) if save_to_files else None
    file_exists = False
    if save_to_files and os.path.isfile(summary_csv_path):
        file_exists = True

    all_results = []
    for run_idx in range(1, N_runs + 1):
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
            simulate_full_route=simulate_full_route
        )
        sim.run()
        sim.print_statistics()

        # Save timeseries
        if save_to_files:
            run_csv = os.path.join(output_folder, f"run_{run_idx}.csv")
            if sim.per_timestep_data:
                timeseries_fieldnames = list(sim.per_timestep_data[0].keys())
            else:
                timeseries_fieldnames = ["TimeStep"]

            with open(run_csv, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=timeseries_fieldnames)
                writer.writeheader()
                for row in sim.per_timestep_data:
                    writer.writerow(row)

            # summary row
            summary_row = sim.get_results_dict()
            summary_row["SimulationRun"] = run_idx
            with open(summary_csv_path, "a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
                if (not file_exists) and run_idx == 1:
                    writer.writeheader()
                writer.writerow(summary_row)
                file_exists = True

        all_results.append(sim.get_results_dict())

    # Optional average row
    if save_to_files and all_results:
        avg_row = compute_average_row(all_results, directions)
        avg_row["SimulationRun"] = "Average"
        with open(summary_csv_path, "a", newline="") as f:
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

    # Assume parameters from last record
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

# ------------------------------
# EXAMPLE MAIN
# ------------------------------
if __name__ == "__main__":
    """
    Example usage: 
      - N_runs=1
      - simulate_full_route=False => vehicles vanish at center
      - or simulate_full_route=True => vehicles exit off screen.
    """
    custom_vehicle_distribution = {
        "car": 0.3,
        "scooter": 0.3,
        "motorcycle": 0.2,
        "truck": 0.1,
        "bus": 0.1
    }

    # Run a single simulation with vehicles disappearing at the center
    # Just change simulate_full_route=True if you want them to travel off screen
    run_multiple_simulations(
        N_runs=1,
        csv_filename="summary.csv",
        junction_type="4way",
        multiple_lights=False,
        total_time=6000,
        simulation_speed=600,
        save_to_files=True,
        output_folder="SimulationV2",
        multiple_lanes=True,
        lane_count=3,
        yellow_duration=2,
        all_red_duration=0,
        vehicle_distribution=custom_vehicle_distribution,
        india_mode=True,
        show_visuals=True,
        simulate_full_route=False  # << KEY OPTION: vanish at center
    )
