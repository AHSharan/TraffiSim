"""
models.py
---------
Vehicle model and default vehicle parameters.
Implements IDM & MOBIL logic.
"""

import random
import math
from .utils import dist, define_exit_point

# Default vehicle parameters
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

# Colors for different vehicle types (for rendering usage)
VEHICLE_COLORS = {
    'car':        (200, 200, 0),
    'truck':      (180, 100, 50),
    'bus':        (120, 40, 150),
    'scooter':    (40, 220, 220),
    'motorcycle': (255, 100, 100),
}

class Vehicle:
    """
    Represents a single vehicle, using IDM for longitudinal and MOBIL for lane changing.
    Also includes India Mode logic for side-by-side queue placement (if enabled).
    """
    def __init__(self, arrival_time, direction, vehicle_type=None,
                 simulate_full_route=True,
                 SCREEN_WIDTH=1000, SCREEN_HEIGHT=800):
        self.arrival_time = arrival_time
        self.direction = direction  # 'N', 'S', 'E', 'W'
        
        # Random turn choice
        self.turn_direction = random.choices(
            ['left', 'straight', 'right'],
            weights=[0.34, 0.33, 0.33]
        )[0]

        # Vehicle type logic
        if vehicle_type is None:
            vehicle_type = random.choice(list(DEFAULT_VEHICLE_TYPES.keys()))
        self.vehicle_type = vehicle_type
        params = DEFAULT_VEHICLE_TYPES[self.vehicle_type]

        # Basic physical parameters
        self.length = params['length']
        self.lane_change_aggressiveness = params['lane_change_aggressiveness']

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
        self.SCREEN_WIDTH = SCREEN_WIDTH
        self.SCREEN_HEIGHT = SCREEN_HEIGHT

        # Reaction time
        self.reaction_time = random.uniform(0.8, 1.5)

        # IDM parameters
        self.idm_a0 = 2.0   
        self.idm_b  = 2.5   
        self.idm_v0 = params['desired_speed'] 
        self.idm_T  = 1.5   
        self.idm_s0 = 2.0   
        self.idm_delta = 4.0

        # Politeness factor for MOBIL
        self.politeness = 0.2
        self.delta_a_threshold = 0.2

        # Additional indexing for queue positioning
        self.lane_index = None
        self.row_index = None
        self.col_index = None

        # Assign a driver profile to add variety
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
            weights=[0.3, 0.5, 0.2]
        )[0]

        if profile_type == "aggressive":
            self.idm_a0 = random.uniform(2.5, 3.0)
            self.idm_b  = 2.0
            self.idm_T  = 1.0
            self.idm_v0 *= 1.3  # bump up desired speed ~30% for effect
            self.politeness = 0.1
            self.reaction_time = random.uniform(0.6, 1.0)
        elif profile_type == "cautious":
            self.idm_a0 = random.uniform(1.0, 1.5)
            self.idm_b  = 2.5
            self.idm_T  = 1.8
            self.idm_v0 *= 0.9  # reduce desired speed
            self.politeness = 0.3
            self.reaction_time = random.uniform(1.5, 2.0)
        else:
            # 'normal' -> keep defaults
            pass

    def init_route(self, cx, cy):
        """
        Called once vehicle starts crossing. We define start/center/exit, compute route distance.
        """
        self.start_position = (self.x, self.y)
        self.center_position = (cx, cy)
        if self.simulate_full_route:
            # Real exit point
            self.exit_position = define_exit_point(
                cx, cy, self.direction, self.turn_direction,
                self.SCREEN_WIDTH, self.SCREEN_HEIGHT
            )
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
            0, self.current_speed * self.idm_T +
            (self.current_speed*delta_v)/(2*math.sqrt(self.idm_a0*self.idm_b))
        )

        alpha_free = 1 - pow((self.current_speed / self.idm_v0), self.idm_delta)
        alpha_int  = - pow((s_star / s), 2)
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
            lead_for_rear_current = sim.get_lead_vehicle_in_lane(current_lane, rear_current)
            a_old_rear = rear_current.compute_idm_acceleration(lead_for_rear_current)
        if rear_target:
            lead_for_rear_target = sim.get_lead_vehicle_in_lane(target_lane, rear_target)
            a_new_rear = rear_target.compute_idm_acceleration(lead_for_rear_target)

        lhs = (a_new_self - a_old_self) + self.politeness * (a_new_rear - a_old_rear)
        if lhs > self.delta_a_threshold:
            # check safety
            if self.is_safe_to_change_lane(lead_target, rear_target):
                return True
        return False

    def is_safe_to_change_lane(self, lead_vehicle, rear_vehicle):
        """
        Very simple check: ensure gap front and gap rear are > 3.0m (adjust as desired).
        """
        safe_gap = 3.0
        if lead_vehicle:
            gap_front = dist((self.x, self.y), (lead_vehicle.x, lead_vehicle.y)) \
                        - 0.5*self.length - 0.5*lead_vehicle.length
            if gap_front < safe_gap:
                return False
        if rear_vehicle:
            gap_rear = dist((self.x, self.y), (rear_vehicle.x, rear_vehicle.y)) \
                       - 0.5*self.length - 0.5*rear_vehicle.length
            if gap_rear < safe_gap:
                return False
        return True

    def update_position(self, dt=1.0, lead_vehicle=None):
        """
        Update speed via IDM acceleration and move the vehicle along its route.
        Returns True if vehicle has completed its route.
        """
        a = self.compute_idm_acceleration(lead_vehicle)
        self.current_speed += a * dt
        if self.current_speed < 0.0:
            self.current_speed = 0.0

        dist_step = self.current_speed * dt
        self.distance_covered += dist_step

        if self.route_distance < 0.1:
            return True  # edge case if start == center

        frac = self.distance_covered / self.route_distance
        if frac >= 1.0:
            frac = 1.0

        d1 = dist(self.start_position, self.center_position)
        route_total = self.route_distance
        if frac < d1 / route_total:
            # in the segment start->center
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

        return (frac >= 1.0)
