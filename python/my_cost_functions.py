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
Detects collision between ego vehicle and other vehicle based on
ego vehicle (s, v) and other vehicle (s_now, s_previous)
"""
def check_collision(ego_vehicle, other_vehicle_s_previous, other_vehicle_s_now):
    ego_vehicle_s = ego_vehicle.s
    ego_vehicle_v = ego_vehicle.v

    #calculate other vehicle speed
    #which is: (s_previous - s_now)/dt but as dt=1 so
    other_vehicle_v = other_vehicle_s_now - other_vehicle_s_now

    #There are 3 cases in which vehicle can collide

    #CASE-1: Other vehicle was behind ego vehicle previously but then
    #accelerated and now on same position as ego_vehicle
    if other_vehicle_s_previous < ego_vehicle_s:
        if other_vehicle_s_now >= ego_vehicle_s:
            return True

    #CASE-2: Other vehicle ahead of ego vehicle but then decelerated
    #resulting in collision
    if other_vehicle_s_previous > ego_vehicle_s:
        if other_vehicle_s_now <= ego_vehicle_s:
            return True

    #CASE-3: Ego vehicle is right beind other vehicle
    #and other vehicle has same or less speed than ego vehicle
    if other_vehicle_s_previous == ego_vehicle_s:
        if other_vehicle_v <= ego_vehicle_v:
            return True

    return False

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
