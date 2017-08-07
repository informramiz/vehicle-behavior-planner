from __future__ import division
from collections import namedtuple
from math import sqrt,exp
import pdb

TrajectoryData = namedtuple("TrajectoryData", [
        "proposed_lane",
        "avg_speed",
        "max_acceleration",
        "rms_acceleration",
        "closest_approach",
        "end_distance_to_goal",
        "end_lanes_from_goal",
        "collides",
        ])

def calculate_cost(vehicle, trajectory, predictions):
    #TODO: implement
    return 0
