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
"""
Filter predicted trajectories whose first prediction is in in given lane
"""
def filter_predictions_by_lane(predictions, lane):
    filtered = {}
    for v_id, trajectory in predictions.items():
        #-1 is vehicle_id for ego vehicle so we don't want to consider
        #ego vehicle predictions if any
        if trajectory[0]['lane'] == lane and v_id != -1:
            filtered[v_id] = trajectory;

    return filtered
