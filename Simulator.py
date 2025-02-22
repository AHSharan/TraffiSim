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

PRINT_INTERVAL = 60
MIN_QUEUE_EMPTY = 0
TIMESERIES_INTERVAL = 10

# Increased screen sizes for more lanes
SCREEN_WIDTH = 1000
SCREEN_HEIGHT = 800

# We’ll increase the road width as well to accommodate multiple lanes
# Each lane is ~30 pixels, for example, plus some margins
LANE_WIDTH = 30   # width per lane in the visual
ROAD_MARGIN = 10  # margin on each side
ROAD_WIDTH = LANE_WIDTH * 4 + ROAD_MARGIN * 2  # enough for 4 lanes + margins (can be dynamic if needed)
QUEUE_OFFSET = 200
CAR_SPACING = 25

# Probability for a left-turn to proceed if the lane-change check allows it
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
    margin = 100
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
# VEHICLE CLASS with IDM & MOBIL
# ------------------------------
class Vehicle:
    """
    Represents a single vehicle, using IDM for longitudinal and MOBIL for lane changing.
    Also includes India Mode logic for side-by-side queue placement.
    """
    def __init__(self, arrival_time, direction, vehicle_type=None, simulate_full_route=True):
        self.arrival_time = arrival_time
        self.direction = direction  # 'N', 'S', 'E', 'W'
        # Random turn choice
        self.turn_direction = random.choices(
            ['left', 'straight', 'right'],
            weights=[0.3, 0.5, 0.2]
        )[0]

        # Vehicle type logic
        if vehicle_type is None:
            vehicle_type = random.choice(list(DEFAULT_VEHICLE_TYPES.keys()))
        self.vehicle_type = vehicle_type
        params = DEFAULT_VEHICLE_TYPES[self.vehicle_type]

        # Basic physical parameters
        self.length = params['length']

        # Lane-change aggressiveness for a quick left turn check
        self.lane_change_aggressiveness = params['lane_change_aggressiveness']

        # For queue vs crossing
        self.state = 'queueing'  # 'queueing', 'crossing', 'finished'
        self.start_cross_time = None
        self.finish_time = None

        # Position & movement
        self.x = 0.0
        self.y = 0.0
        self.current_speed = 0.0
        self.distance_covered = 0.0

        # Route details
        self.start_position = None
        self.center_position = None
        self.exit_position = None
        self.route_distance = 0.0
        self.simulate_full_route = simulate_full_route

        # Reaction time
        self.reaction_time = random.uniform(0.8, 1.5)

        # IDM parameters
        self.idm_a0 = 2.0   # max acceleration
        self.idm_b  = 2.5   # comfortable deceleration
        self.idm_v0 = params['desired_speed']  # desired speed from the dictionary
        self.idm_T  = 1.5   # safe time headway
        self.idm_s0 = 2.0   # minimum jam distance
        self.idm_delta = 4.0

        # Politeness factor for MOBIL
        self.politeness = 0.2
        self.delta_a_threshold = 0.2

        # Additional indexing for queue positioning
        self.lane_index = None
        self.row_index = None
        self.col_index = None

        # Assign a driver profile to add some variety
        self.assign_driver_profile()

    @property
    def wait_time(self):
        if self.start_cross_time is None:
            return None
        return self.start_cross_time - self.arrival_time

    def assign_driver_profile(self):
        """
        Randomly assigns an 'aggressive', 'normal', or 'cautious' profile
        to create variability in driving style.
        """
        profile_type = random.choices(
            ["aggressive", "normal", "cautious"],
            weights=[0.2, 0.6, 0.2]
        )[0]

        if profile_type == "aggressive":
            self.idm_a0 = random.uniform(2.5, 3.0)
            self.idm_b  = 2.0
            self.idm_T  = 1.0
            self.idm_v0 *= 1.1  # bump up desired speed by ~10%
            self.politeness = 0.1
            self.reaction_time = random.uniform(0.6, 1.0)
        elif profile_type == "cautious":
            self.idm_a0 = random.uniform(1.0, 1.5)
            self.idm_b  = 2.5
            self.idm_T  = 1.8
            self.idm_v0 *= 0.9  # slow by ~10%
            self.politeness = 0.3
            self.reaction_time = random.uniform(1.5, 2.0)
        else:
            # normal
            pass  # keep defaults

    def init_route(self, cx, cy):
        """
        Called once vehicle starts crossing. We define start/center/exit, compute route distance.
        """
        self.start_position = (self.x, self.y)
        self.center_position = (cx, cy)
        if self.simulate_full_route:
            # Real exit point
            self.exit_position = define_exit_point(cx, cy, self.direction, self.turn_direction)
            d1 = dist(self.start_position, self.center_position)
            d2 = dist(self.center_position, self.exit_position)
            self.route_distance = d1 + d2
        else:
            # Vanish at center
            self.exit_position = self.center_position
            self.route_distance = dist(self.start_position, self.center_position)

        self.distance_covered = 0.0

    def compute_idm_acceleration(self, lead_vehicle):
        """
        Calculate acceleration using IDM, given the lead vehicle in the same lane (if any).
        """
        if lead_vehicle is None:
            # no lead => infinite gap
            s = 1e9
            delta_v = 0.0
        else:
            # gap = distance between centers - half lengths
            s = dist((self.x, self.y), (lead_vehicle.x, lead_vehicle.y)) \
                - 0.5*self.length - 0.5*lead_vehicle.length
            delta_v = self.current_speed - lead_vehicle.current_speed
            if s < 0.1:
                s = 0.1

        s_star = self.idm_s0 + max(
            0, self.current_speed * self.idm_T + (self.current_speed*delta_v)/(2*math.sqrt(self.idm_a0*self.idm_b))
        )

        # free road term
        alpha_free = 1 - pow((self.current_speed / self.idm_v0), self.idm_delta)
        # interaction term
        alpha_int = - pow((s_star / s), 2)

        a = self.idm_a0 * (alpha_free + alpha_int)
        return a

    def check_mobil_lane_change(self, sim, current_lane, target_lane,
                                lead_current, lead_target,
                                rear_current, rear_target):
        """
        Decide if lane change is beneficial by MOBIL:
          (a_new_self - a_old_self) + p * (a_new_rear - a_old_rear) > delta_a_threshold
        """
        a_old_self = self.compute_idm_acceleration(lead_current)
        a_new_self = self.compute_idm_acceleration(lead_target)

        a_old_rear, a_new_rear = 0.0, 0.0
        if rear_current:
            # the lead for rear_current in the old lane
            lead_for_rear_current = sim.get_lead_vehicle_in_lane(current_lane, rear_current)
            a_old_rear = rear_current.compute_idm_acceleration(lead_for_rear_current)
        if rear_target:
            lead_for_rear_target = sim.get_lead_vehicle_in_lane(target_lane, rear_target)
            a_new_rear = rear_target.compute_idm_acceleration(lead_for_rear_target)

        lhs = (a_new_self - a_old_self) + self.politeness*(a_new_rear - a_old_rear)
        if lhs > self.delta_a_threshold:
            # check safety
            if self.is_safe_to_change_lane(lead_target, rear_target):
                return True
        return False

    def is_safe_to_change_lane(self, lead_vehicle, rear_vehicle):
        """
        Very simple check: ensure gap front and gap rear are > 5m.
        """
        safe_gap = 5.0
        # front gap
        if lead_vehicle:
            gap_front = dist((self.x, self.y), (lead_vehicle.x, lead_vehicle.y)) \
                        - 0.5*self.length - 0.5*lead_vehicle.length
            if gap_front < safe_gap:
                return False
        # rear gap
        if rear_vehicle:
            gap_rear = dist((self.x, self.y), (rear_vehicle.x, rear_vehicle.y)) \
                       - 0.5*self.length - 0.5*rear_vehicle.length
            if gap_rear < safe_gap:
                return False
        return True

    def update_position(self, dt=1.0, lead_vehicle=None):
        """
        Called each simulation step (or sub-step). 
        Uses the IDM acceleration to update speed, then moves the vehicle along its route.
        """
        # 1) Compute IDM acceleration
        a = self.compute_idm_acceleration(lead_vehicle)
        # 2) Update speed
        self.current_speed += a*dt
        if self.current_speed < 0.0:
            self.current_speed = 0.0

        # 3) Move the vehicle along route
        dist_step = self.current_speed * dt
        self.distance_covered += dist_step

        frac = self.distance_covered / self.route_distance
        if frac >= 1.0:
            frac = 1.0

        # piecewise: start->center, center->exit
        d1 = dist(self.start_position, self.center_position)
        route_total = self.route_distance
        if route_total < 0.1:
            return True  # edge case if start == center

        if frac < d1/route_total:
            # in the segment from start->center
            sub_frac = frac / (d1 / route_total)
            sx, sy = self.start_position
            cx, cy = self.center_position
            self.x = sx + (cx - sx)*sub_frac
            self.y = sy + (cy - sy)*sub_frac
        else:
            # in the center->exit segment
            ratio_remaining = frac - (d1/route_total)
            segment_len = 1.0 - (d1/route_total)
            sub_frac = ratio_remaining / segment_len if segment_len > 1e-6 else 1.0
            cx, cy = self.center_position
            ex, ey = self.exit_position
            self.x = cx + (ex - cx)*sub_frac
            self.y = cy + (ey - cy)*sub_frac

        # Check if done
        return (frac >= 1.0)


# ------------------------------
# MAIN SIMULATION CLASS
# ------------------------------
class IntersectionSim:
    def __init__(
        self,
        junction_type="4way",
        multiple_lights=False,
        total_time=TOTAL_TIME,
        sim_steps_per_sec=SIM_STEPS_PER_SEC,
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
        simulate_full_route=True
    ):
        if arrival_rates is None:
            arrival_rates = ARRIVAL_RATES
        self.arrival_rates = arrival_rates

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

        # Basic data structures
        if junction_type == "3way":
            self.directions = ['N', 'E', 'W']
        else:
            self.directions = ['N', 'E', 'S', 'W']

        # Build queues
        if self.multiple_lanes:
            # each direction: list of lane_count lists
            self.queues = {d: [[] for _ in range(self.lane_count)] for d in self.directions}
        else:
            self.queues = {d: [] for d in self.directions}

        self.crossing_vehicles = []
        self.processed_vehicles = []

        self.vehicle_type_counts = {vt: 0 for vt in DEFAULT_VEHICLE_TYPES.keys()}
        self.red_no_car_time = {d: 0 for d in self.directions}
        self.arrivals_count = {d: 0 for d in self.directions}

        self.sim_time = 0
        self.running = True

        self.phases = []
        self.cycle_length = 0
        self.define_signal_phases()

        self.per_timestep_data = []

        # Possibly create the renderer
        self.renderer = None
        if self.show_visuals and renderer_class:
            self.renderer = renderer_class(self)

    def define_signal_phases(self):
        # Simplified from your code
        if self.junction_type == "4way":
            if not self.multiple_lights:
                base_green = 30
                self.phases = [
                    {'green': ['N'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['S'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['W'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
            else:
                base_green = 30
                self.phases = [
                    {'green': ['N', 'S'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                    {'green': ['E', 'W'], 'green_duration': base_green,
                     'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
                ]
        else:
            # 3way example
            base_green = 30
            self.phases = [
                {'green': ['N'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                {'green': ['E'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration},
                {'green': ['W'], 'green_duration': base_green,
                 'yellow_duration': self.yellow_duration, 'all_red_duration': self.all_red_duration}
            ]
        self.cycle_length = sum(ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration']
                                for ph in self.phases)

    def get_signal_state(self, direction, t):
        """
        Return 'green','yellow','red' for the direction at time t
        (for visualization).
        """
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            g = ph['green_duration']
            y = ph['yellow_duration']
            r = ph['all_red_duration']
            phase_len = g + y + r
            if cycle_pos < accum + phase_len:
                # we are in this phase
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
        Return a list of directions that are green at time t.
        """
        cycle_pos = t % self.cycle_length
        accum = 0
        for ph in self.phases:
            phase_len = ph['green_duration'] + ph['yellow_duration'] + ph['all_red_duration']
            if cycle_pos < accum + phase_len:
                # found our phase
                pos_in_ph = cycle_pos - accum
                if pos_in_ph < ph['green_duration']:
                    return ph['green']
                else:
                    return []
            accum += phase_len
        return []

    # The main run loop
    def run(self):
        steps_per_frame = float(self.sim_steps_per_sec)
        while self.running:
            if self.renderer is not None:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        self.running = False

            if self.sim_time < self.total_time:
                # We do a simple approach: we call sim_update once per step
                # We'll do partial real-time sync if we want
                self.sim_update()
            else:
                self.running = False

            # Render
            if self.renderer:
                self.renderer.render()
                if self.simulation_speed > 0:
                    # control FPS
                    self.renderer.clock.tick(self.simulation_speed)

        if self.renderer:
            pygame.quit()

    def sim_update(self):
        # 1) zero out arrival counts
        for d in self.directions:
            self.arrivals_count[d] = 0

        # 2) generate arrivals
        self.generate_arrivals(self.sim_time)

        # 3) let front vehicles cross if light is green
        green_dirs = self.get_green_directions(self.sim_time)
        for d in green_dirs:
            self.start_crossing_one_vehicle(d, self.sim_time)

        # 4) update crossing vehicles
        self.update_crossing_vehicles()

        # 5) track red-empty
        self.track_empty_red_time(green_dirs)

        # 6) print occasionally
        if self.sim_time % self.print_interval == 0:
            self.print_state(self.sim_time, green_dirs)

        # 7) record data
        self.record_timestep_data()

        self.sim_time += 1

    def generate_arrivals(self, t):
        for d in self.directions:
            rate = self.arrival_rates.get(d, 0)
            arrivals = self.poisson_random(rate)
            self.arrivals_count[d] += arrivals
            for _ in range(arrivals):
                # pick a vehicle type from distribution
                vt = random.choices(
                    population=list(self.vehicle_distribution.keys()),
                    weights=list(self.vehicle_distribution.values())
                )[0]
                v = Vehicle(t, d, vt, simulate_full_route=self.simulate_full_route)
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
        if self.multiple_lanes:
            L = self.queues[direction]
            # pick the lane with the fewest vehicles
            lane_index = min(range(self.lane_count), key=lambda i: len(L[i]))
            L[lane_index].append(v)
            v.lane_index = lane_index
        else:
            self.queues[direction].append(v)

    def start_crossing_one_vehicle(self, direction, t):
        # remove exactly one from queue if reaction time has passed
        if self.multiple_lanes:
            lanes = self.queues[direction]
            for lane in lanes:
                if lane:
                    front = lane[0]
                    if t - front.arrival_time >= front.reaction_time:
                        # check if left turn is allowed
                        if front.turn_direction == 'left':
                            if random.random() >= (LEFT_TURN_OPEN_PROB * front.lane_change_aggressiveness):
                                continue
                            else:
                                front = lane.pop(0)
                        else:
                            front = lane.pop(0)
                        front.state = 'crossing'
                        front.start_cross_time = t
                        front.init_route(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
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
                    front.init_route(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2)
                    self.crossing_vehicles.append(front)

    def update_crossing_vehicles(self):
        # For each crossing vehicle, do an IDM update
        # We are skipping lane changes in the intersection zone itself
        # If you wanted to model it, you'd need more geometry checks.
        done_list = []
        for v in self.crossing_vehicles:
            # We pass no lead_vehicle here (or do advanced searching).
            # Typically you'd track lane position if you want fully realistic following in intersection.
            is_done = v.update_position(dt=1.0, lead_vehicle=None)
            if is_done:
                done_list.append(v)
        for v in done_list:
            self.crossing_vehicles.remove(v)
            v.state = 'finished'
            self.processed_vehicles.append(v)

        # reposition queues for visualization
        for d in self.directions:
            self.reposition_queue(d)

    def reposition_queue(self, direction):
        """
        Reposition queued vehicles in lines. 
        Extended 'India mode': up to 3 side by side with some probability.
        """
        lane_gap = 40  # spacing between lanes in the UI
        base_x, base_y = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2

        if self.multiple_lanes:
            for lane_idx, lane in enumerate(self.queues[direction]):
                if self.india_mode:
                    # We'll allow up to 3 in a row with some probability
                    # to show a congested "India mode" scenario
                    rows = []
                    for veh in lane:
                        # Decide how many can fit in the last row
                        if rows:
                            last_row = rows[-1]
                            # We'll say there's a ~30% chance we can fit a 3rd vehicle
                            # if all are scooters/motorcycles
                            if (len(last_row) < 3 
                                and all(x.vehicle_type in ['scooter','motorcycle'] for x in last_row)
                                and random.random() < 0.3):
                                last_row.append(veh)
                                veh.row_index = len(rows)-1
                                veh.col_index = len(last_row)-1
                            elif (len(last_row) < 2 
                                  and all(x.vehicle_type in ['scooter','motorcycle','car'] for x in last_row)
                                  and random.random() < 0.5):
                                # maybe we can fit 2 side by side with a car included
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

                    # Now position them
                    for row_idx, row in enumerate(rows):
                        # offset lane
                        offset_lane = (lane_idx - (self.lane_count - 1) / 2.0)*lane_gap
                        for col_idx, veh in enumerate(row):
                            extra_x, extra_y = 0, 0
                            # separate them horizontally if multiple in row
                            if len(row) == 2:
                                extra_x = -5 if col_idx == 0 else 5
                            elif len(row) == 3:
                                # space them out a bit more
                                extra_x = -10 + (10*col_idx)

                            # set position based on direction
                            if direction == 'N':
                                veh.x = base_x + offset_lane + extra_x
                                # move upward in y by row_idx
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
                    # normal single-file
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
                        else: # W
                            veh.x = base_x - QUEUE_OFFSET - i*CAR_SPACING
                            veh.y = base_y + offset_lane
        else:
            # Single-lane
            lane = self.queues[direction]
            if self.india_mode:
                rows = []
                for veh in lane:
                    if rows:
                        last_row = rows[-1]
                        # up to 3 side-by-side if all are small
                        if (len(last_row) < 3 
                            and all(x.vehicle_type in ['scooter','motorcycle'] for x in last_row)
                            and random.random() < 0.3):
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
                # position them
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
                    else: # W
                        veh.x = base_x - QUEUE_OFFSET - i*CAR_SPACING
                        veh.y = base_y

    def track_empty_red_time(self, green_dirs):
        for d in self.directions:
            if d not in green_dirs:
                if self.multiple_lanes:
                    size = sum(len(x) for x in self.queues[d])
                else:
                    size = len(self.queues[d])
                if size <= self.min_queue_empty:
                    self.red_no_car_time[d] += 1

    def print_state(self, t, green_dirs):
        print(f"Time={t}, green={green_dirs}")
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
        return sum(v.wait_time for v in done)/len(done)

    def overall_average_wait_time(self):
        done = [v for v in self.processed_vehicles if v.wait_time is not None]
        if not done:
            return 0.0
        return sum(v.wait_time for v in done)/len(done)

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
# PYGAME RENDERER with bigger roads + basic lane barriers
# ------------------------------
class TrafficRenderer:
    COLOR_BG = (30, 30, 30)
    COLOR_ROAD = (70, 70, 70)
    COLOR_TEXT = (255, 255, 255)
    LINE_COLOR = (140, 140, 140)

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

        self.cx = SCREEN_WIDTH//2
        self.cy = SCREEN_HEIGHT//2

    def render(self):
        self.screen.fill(self.COLOR_BG)
        self.draw_roads()
        self.draw_lane_barriers()
        self.draw_traffic_lights()
        self.draw_vehicles()
        self.draw_ui_panel()
        pygame.display.flip()

    def draw_roads(self):
        """
        Draw a wide cross for a 4way, or a T shape for 3way
        We'll fill with ROAD_COLOR
        """
        # For a 4-way, let's draw wide horizontal and vertical rectangles
        if self.sim.junction_type == '4way':
            # Horizontal
            rect_h = pygame.Rect(0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_h)
            # Vertical
            rect_v = pygame.Rect(self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, SCREEN_HEIGHT)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_v)
        else:
            # 3-way, for simplicity, a T shape:
            # horizontal across entire screen
            rect_h = pygame.Rect(0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_h)
            # vertical from top to center
            rect_v = pygame.Rect(self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, self.cy + ROAD_WIDTH//2)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_v)

    def draw_lane_barriers(self):
        """
        Draw simpler lines or barriers to indicate lane boundaries.
        We'll do dashed lines or solid lines at the edges.
        """
        # For illustration, let's place multiple lines in horizontal and vertical
        # to create lane boundaries. We'll assume lane_count up to 4 or so.

        # Each lane is ~ LANE_WIDTH wide
        # We'll center them around self.cx or self.cy
        # For demonstration, draw dashed lines
        dash_len = 10
        gap_len = 6

        # Horizontal lanes: We'll place them in the vertical strip from (cy - ROAD_WIDTH//2) to (cy + ROAD_WIDTH//2).
        # We step by LANE_WIDTH to draw boundaries.
        top_road = self.cy - (ROAD_WIDTH//2)
        bottom_road = self.cy + (ROAD_WIDTH//2)
        left_road = self.cx - (ROAD_WIDTH//2)
        right_road = self.cx + (ROAD_WIDTH//2)

        # vertical lane boundaries (for horizontal direction)
        # e.g. if ROAD_WIDTH= ~160, we can have up to 4 or 5 lanes
        # We'll do them from x=some offset. But let's only do them in the region of the road.
        lane_count_est = int(ROAD_WIDTH/ LANE_WIDTH)
        for i in range(1, lane_count_est):
            x_line = left_road + i*LANE_WIDTH
            # dash from left to right across entire screen horizontally
            y = 0
            while y < SCREEN_HEIGHT:
                pygame.draw.line(self.screen, self.LINE_COLOR,
                                 (x_line, y), (x_line, min(y+dash_len, SCREEN_HEIGHT)), 2)
                y += dash_len + gap_len

        # horizontal lane boundaries (for vertical direction)
        lane_count_est_v = int(ROAD_WIDTH/ LANE_WIDTH)
        for i in range(1, lane_count_est_v):
            y_line = top_road + i*LANE_WIDTH
            x = 0
            while x < SCREEN_WIDTH:
                pygame.draw.line(self.screen, self.LINE_COLOR,
                                 (x, y_line), (min(x+dash_len, SCREEN_WIDTH), y_line), 2)
                x += dash_len + gap_len

    def draw_traffic_lights(self):
        offsets = {
            'N': (0, -60),
            'S': (0, 60),
            'E': (60, 0),
            'W': (-60, 0)
        }
        r = 9
        for d in self.sim.directions:
            st = self.sim.get_signal_state(d, self.sim.sim_time)
            color = self.TRAFFIC_LIGHT_COLORS[st]
            ox, oy = offsets.get(d, (0, 0))
            cx = self.cx + ox
            cy = self.cy + oy
            # black circle behind
            pygame.draw.circle(self.screen, (0,0,0), (cx, cy), r+2)
            pygame.draw.circle(self.screen, color, (cx, cy), r)

    def draw_vehicles(self):
        # Draw queued vehicles
        for d in self.sim.directions:
            if self.sim.multiple_lanes:
                for lane in self.sim.queues[d]:
                    for v in lane:
                        self.draw_single_vehicle(v)
            else:
                for v in self.sim.queues[d]:
                    self.draw_single_vehicle(v)
        # Draw crossing vehicles
        for v in self.sim.crossing_vehicles:
            self.draw_single_vehicle(v)

    def draw_single_vehicle(self, v):
        color = VEHICLE_COLORS.get(v.vehicle_type, (200,200,200))
        shadow_color = (color[0]//4, color[1]//4, color[2]//4)

        # pick shape
        if v.vehicle_type in ['car', 'truck', 'bus']:
            self.draw_rect_vehicle(v, color, shadow_color)
        else:
            self.draw_circle_vehicle(v, color, shadow_color)

    def draw_rect_vehicle(self, v, color, shadow):
        # approximate pixel dimensions
        if v.vehicle_type == 'car':
            w,h = 12, 20
        elif v.vehicle_type == 'truck':
            w,h = 14, 35
        else:
            w,h = 14, 40  # bus

        rect = pygame.Rect(0,0,w,h)
        rect.center = (int(v.x), int(v.y))

        shadow_rect = rect.copy()
        shadow_rect.x += 2
        shadow_rect.y += 2

        pygame.draw.rect(self.screen, shadow, shadow_rect, border_radius=3)
        pygame.draw.rect(self.screen, color, rect, border_radius=3)

    def draw_circle_vehicle(self, v, color, shadow):
        # scooters ~5 px radius, motorcycle ~6 px
        r = 5 if v.vehicle_type=='scooter' else 6
        center = (int(v.x), int(v.y))
        shadow_c = (center[0]+2, center[1]+2)

        pygame.draw.circle(self.screen, shadow, shadow_c, r)
        pygame.draw.circle(self.screen, color, center, r)

    def draw_ui_panel(self):
        lines = [
            f"Time= {self.sim.sim_time}/{self.sim.total_time}",
            f"Speed= {self.sim.simulation_speed} steps/sec",
            f"JunctionType= {self.sim.junction_type}, MultiLights= {self.sim.multiple_lights}",
            f"MultiLanes= {self.sim.multiple_lanes}, LaneCount= {self.sim.lane_count}",
            f"IndiaMode= {self.sim.india_mode}",
            f"SimulateFullRoute= {self.sim.simulate_full_route}",
            "Close window or press Ctrl+C to quit."
        ]
        x=10
        y=10
        for txt in lines:
            surf = self.font.render(txt, True, (255,255,255))
            self.screen.blit(surf, (x,y))
            y+=20

# ------------------------------
# RUN MULTIPLE SIMULATIONS
# ------------------------------
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
    simulate_full_route=True
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
        "SimulateFullRoute"
    ]
    for d in directions:
        summary_fieldnames.append(f"RedEmpty{d}")
    for vt in DEFAULT_VEHICLE_TYPES.keys():
        summary_fieldnames.append(f"Count{vt.capitalize()}")

    if save_to_files:
        os.makedirs(output_folder, exist_ok=True)
    summary_csv_path = os.path.join(output_folder, csv_filename) if save_to_files else None
    file_exists = (save_to_files and os.path.isfile(summary_csv_path))

    all_results=[]
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
            simulate_full_route=simulate_full_route
        )
        sim.run()
        sim.print_statistics()

        # timeseries CSV
        if save_to_files:
            run_csv = os.path.join(output_folder, f"run_{run_idx}.csv")
            if sim.per_timestep_data:
                fields = list(sim.per_timestep_data[0].keys())
            else:
                fields = ["TimeStep"]
            with open(run_csv,'w',newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                for row in sim.per_timestep_data:
                    writer.writerow(row)

            # summary row
            summary_row = sim.get_results_dict()
            summary_row["SimulationRun"] = run_idx
            with open(summary_csv_path,'a',newline='') as f:
                writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
                if (not file_exists) and run_idx==1:
                    writer.writeheader()
                writer.writerow(summary_row)
                file_exists=True

        all_results.append(sim.get_results_dict())

    # Optionally compute average row
    if save_to_files and len(all_results)>0:
        avg_row = compute_average_row(all_results, directions)
        avg_row["SimulationRun"] = "Average"
        with open(summary_csv_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=summary_fieldnames)
            writer.writerow(avg_row)
        print("Average row appended to summary CSV.")


def compute_average_row(all_results, directions):
    n = len(all_results)
    if n==0:
        return {}
    sum_tvp=0
    sum_wait=0
    sum_red={d:0 for d in directions}
    sum_counts={vt:0 for vt in DEFAULT_VEHICLE_TYPES.keys()}
    jt = all_results[-1]["JunctionType"]
    ml = all_results[-1]["MultipleLights"]
    mls= all_results[-1]["MultipleLanes"]
    lc = all_results[-1]["LaneCount"]
    sfr= all_results[-1]["SimulateFullRoute"]

    for r in all_results:
        sum_tvp+=r["TotalVehiclesProcessed"]
        sum_wait+=r["OverallAvgWait"]
        for d in directions:
            sum_red[d]+= r[f"RedEmpty{d}"]
        for vt in DEFAULT_VEHICLE_TYPES.keys():
            key = "Count"+vt.capitalize()
            sum_counts[vt]+=r[key]

    avg_row={
        "JunctionType": jt,
        "MultipleLights": ml,
        "MultipleLanes": mls,
        "LaneCount": lc,
        "SimulateFullRoute": sfr,
        "TotalVehiclesProcessed": sum_tvp/n,
        "OverallAvgWait": sum_wait/n
    }
    for d in directions:
        avg_row[f"RedEmpty{d}"] = sum_red[d]/n
    for vt in DEFAULT_VEHICLE_TYPES.keys():
        key = "Count"+vt.capitalize()
        avg_row[key] = sum_counts[vt]/n
    return avg_row


# ------------------------------
# EXAMPLE MAIN
# ------------------------------
if __name__ == "__main__":
    # Example usage
    custom_vehicle_distribution = {
        "car": 0.3,
        "scooter": 0.3,
        "motorcycle": 0.2,
        "truck": 0.1,
        "bus": 0.1
    }

    run_multiple_simulations(
        N_runs=1,
        csv_filename="simulation_results.csv",
        junction_type="4way",
        multiple_lights=False,
        total_time=6000,
        simulation_speed=60,
        save_to_files=True,
        output_folder="SimulationV3",
        multiple_lanes=True,
        lane_count=3,
        yellow_duration=5,
        all_red_duration=2,
        vehicle_distribution=custom_vehicle_distribution,
        india_mode=True,
        show_visuals=True,
        simulate_full_route=True
    )
