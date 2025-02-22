"""
rendering.py
------------
Optional Pygame renderer for TraffiSim.
"""

import pygame
from .models import VEHICLE_COLORS
from .intersection import SCREEN_WIDTH, SCREEN_HEIGHT, LANE_WIDTH, ROAD_WIDTH, ROAD_MARGIN
from .intersection import QUEUE_OFFSET, CAR_SPACING

class TrafficRenderer:
    """
    Class responsible for rendering the traffic simulation using Pygame.
    """
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
        """
        Initialize the TrafficRenderer with a simulation instance.
        
        Args:
            sim (IntersectionSim): The simulation instance to render.
        """
        self.sim = sim
        pygame.init()
        pygame.display.set_caption(f"TraffiSim (type={sim.junction_type}, MLights={sim.multiple_lights})")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont(None, 22)

        self.cx = SCREEN_WIDTH // 2
        self.cy = SCREEN_HEIGHT // 2

    def render(self):
        """
        Render the current state of the simulation.
        """
        self.screen.fill(self.COLOR_BG)
        self.draw_roads()
        self.draw_lane_barriers()
        self.draw_traffic_lights()
        self.draw_vehicles()
        self.draw_ui_panel()
        pygame.display.flip()

    def draw_roads(self):
        """
        Draw the intersection roads as rectangles.
        """
        if self.sim.junction_type == '4way':
            rect_h = pygame.Rect(0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_h)
            rect_v = pygame.Rect(self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, SCREEN_HEIGHT)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_v)
        else:
            rect_h = pygame.Rect(0, self.cy - ROAD_WIDTH//2, SCREEN_WIDTH, ROAD_WIDTH)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_h)
            rect_v = pygame.Rect(self.cx - ROAD_WIDTH//2, 0, ROAD_WIDTH, self.cy + ROAD_WIDTH//2)
            pygame.draw.rect(self.screen, self.COLOR_ROAD, rect_v)

    def draw_lane_barriers(self):
        """
        Draw dashed lane lines.
        """
        dash_len = 10
        gap_len = 6

        top_road = self.cy - (ROAD_WIDTH//2)
        bottom_road = self.cy + (ROAD_WIDTH//2)
        left_road = self.cx - (ROAD_WIDTH//2)
        right_road = self.cx + (ROAD_WIDTH//2)

        lane_count_est = int(ROAD_WIDTH / LANE_WIDTH)
        for i in range(1, lane_count_est):
            x_line = left_road + i*LANE_WIDTH
            y = 0
            while y < SCREEN_HEIGHT:
                pygame.draw.line(self.screen, self.LINE_COLOR,
                                 (x_line, y), (x_line, min(y+dash_len, SCREEN_HEIGHT)), 2)
                y += dash_len + gap_len

        lane_count_est_v = int(ROAD_WIDTH / LANE_WIDTH)
        for i in range(1, lane_count_est_v):
            y_line = top_road + i*LANE_WIDTH
            x = 0
            while x < SCREEN_WIDTH:
                pygame.draw.line(self.screen, self.LINE_COLOR,
                                 (x, y_line), (min(x+dash_len, SCREEN_WIDTH), y_line), 2)
                x += dash_len + gap_len

    def draw_traffic_lights(self):
        """
        Draw traffic lights at the intersection.
        """
        offsets = {
            'N': (0, -60),
            'S': (0, 60),
            'E': (60, 0),
            'W': (-60, 0)
        }
        r = 9
        for d in self.sim.directions:
            if self.sim.adaptive_signals:
                if self.sim.current_green == d and self.sim.signal_state == "green":
                    st = "green"
                elif self.sim.current_green == d and self.sim.signal_state == "yellow":
                    st = "yellow"
                else:
                    st = "red"
            else:
                st = self.sim.get_signal_state(d, self.sim.sim_time)

            color = self.TRAFFIC_LIGHT_COLORS[st]
            ox, oy = offsets.get(d, (0,0))
            cx = self.cx + ox
            cy = self.cy + oy

            pygame.draw.circle(self.screen, (0,0,0), (cx, cy), r+2)
            pygame.draw.circle(self.screen, color, (cx, cy), r)

    def draw_vehicles(self):
        """
        Draw all vehicles in the simulation.
        """
        for d in self.sim.directions:
            if self.sim.multiple_lanes:
                for lane in self.sim.queues[d]:
                    for v in lane:
                        self.draw_single_vehicle(v)
            else:
                for v in self.sim.queues[d]:
                    self.draw_single_vehicle(v)
        for v in self.sim.crossing_vehicles:
            self.draw_single_vehicle(v)

    def draw_single_vehicle(self, v):
        """
        Draw a single vehicle.
        
        Args:
            v (Vehicle): The vehicle to draw.
        """
        color = VEHICLE_COLORS.get(v.vehicle_type, (200,200,200))
        shadow_color = (color[0]//4, color[1]//4, color[2]//4)

        if v.vehicle_type in ['car', 'truck', 'bus']:
            self.draw_rect_vehicle(v, color, shadow_color)
        else:
            self.draw_circle_vehicle(v, color, shadow_color)

    def draw_rect_vehicle(self, v, color, shadow):
        """
        Draw a rectangular vehicle (car, truck, bus).
        
        Args:
            v (Vehicle): The vehicle to draw.
            color (tuple): The color of the vehicle.
            shadow (tuple): The shadow color of the vehicle.
        """
        if v.vehicle_type == 'car':
            w, h = 12, 20
        elif v.vehicle_type == 'truck':
            w, h = 14, 35
        else:
            w, h = 14, 40  # bus

        rect = pygame.Rect(0, 0, w, h)
        rect.center = (int(v.x), int(v.y))
        shadow_rect = rect.copy()
        shadow_rect.x += 2
        shadow_rect.y += 2

        pygame.draw.rect(self.screen, shadow, shadow_rect, border_radius=3)
        pygame.draw.rect(self.screen, color, rect, border_radius=3)

    def draw_circle_vehicle(self, v, color, shadow):
        """
        Draw a circular vehicle (scooter, motorcycle).
        
        Args:
            v (Vehicle): The vehicle to draw.
            color (tuple): The color of the vehicle.
            shadow (tuple): The shadow color of the vehicle.
        """
        r = 5 if v.vehicle_type=='scooter' else 6
        center = (int(v.x), int(v.y))
        shadow_c = (center[0]+2, center[1]+2)

        pygame.draw.circle(self.screen, shadow, shadow_c, r)
        pygame.draw.circle(self.screen, color, center, r)

    def draw_ui_panel(self):
        """
        Draw the UI panel with simulation information.
        """
        if self.sim.adaptive_signals:
            sig_text = f"Signal: {self.sim.signal_state.upper()} for {self.sim.current_green} (timer: {self.sim.state_timer})"
        else:
            sig_text = ""
        lines = [
            f"Time= {self.sim.sim_time}/{self.sim.total_time}",
            f"Speed= {self.sim.simulation_speed} steps/sec",
            f"JunctionType= {self.sim.junction_type}, MultiLights= {self.sim.multiple_lights}",
            f"MultiLanes= {self.sim.multiple_lanes}, LaneCount= {self.sim.lane_count}",
            f"IndiaMode= {self.sim.india_mode}",
            f"SimulateFullRoute= {self.sim.simulate_full_route}",
            sig_text,
            "Close window or press Ctrl+C to quit."
        ]
        x = 10
        y = 10
        for txt in lines:
            surf = self.font.render(txt, True, (255,255,255))
            self.screen.blit(surf, (x, y))
            y += 20
