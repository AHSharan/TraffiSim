"""
utils.py
--------
Helper functions for TraffiSim.
"""

import math

def dist(a, b):
    """
    Euclidean distance between points a and b.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])

def define_exit_point(cx, cy, direction, turn, SCREEN_WIDTH, SCREEN_HEIGHT):
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
