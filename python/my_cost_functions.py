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

# timesteps buffer, vehicles should be these many timesteps away
DESIRED_BUFFER = 1.5
# timesteps we consider for calculating cost
PLANNING_HORIZON = 2

#Weights for each cost function
COLLISION = 10 ** 6
DANGER = 10 ** 5
REACH_GOAL = 10 ** 5
COMFORT = 10 ** 4
EFFICIENCY = 10 ** 2


"""
Penalizes lane changes AWAY from the goal lane and rewards
lane changes TOWARDS the goal lane.
"""
def lane_change_cost(vehicle, trajectory, predictions, trajectory_data):
    #this is the lane distance from goal lane that this new trajectory is proposing
    proposed_distance_to_goal_lane = trajectory_data.end_lanes_from_goal
    #this is the lane distance from goal lane that is before taking this trajectory
    previous_distance_to_goal_lane = trajectory[0].lane

    cost = 0.0
    #if new distance is greater than previous then penalize otherwise reward
    if proposed_distance_to_goal_lane > previous_distance_to_goal_lane:
        cost = COMFORT
    else:
        #reward as we are moving closer to goal lane
        cost = -COMFORT

    if cost != 0:
        print("!! \n \ncost for lane change is {}\n\n".format(cost))

    return cost

def distance_to_goal_lane(vehicle, trajectory, predictions, trajectory_data):
    #distance to goal
    distance = abs(trajectory_data.end_distance_to_goal)
    #we don't want distance to be below 1.0
    distance = max(distance, 1.0)

    #calculate time left to reach goal/cover remaining distance
    time_to_goal = float(distance) / trajectory_data.avg_speed

    #the less time we have to reach goal the more difficult it will be to
    #achieve goal lane
    cost = (5.0 * trajectory_data.end_lanes_from_goal / time_to_goal)

    #multiply cost with weight
    return cost * REACH_GOAL

def collision_cost(vehicle, trajectory, predictions, trajectory_data):
    #if there is no collision then no cost
    if trajectory_data.collides == False:
        return 0.0;

    #find out time left till collision
    time_til_collision = trajectory_data.collides['at']
    #find cost, use exp function to get range between [0, 1]
    exponent = (float(time_til_collision)) ** 2
    cost = exp(-exponent)

    return COLLISION * cost;

def buffer_cost(vehicle, trajectory, predictions, trajectory_data):
    #check if there is any distance between ego vehicle and next or not
    if trajectory_data.closest_approach == 0:
        #we are very close to collision
        return 10 * DANGER

    #calculate timesteps remaining to cover remaining distance to next vehicle
    timesteps_away = trajectory_data.closest_approach / trajectory_data.avg_speed

    #timesteps remaining is more than desired buffer timesteps
    if timesteps_away > DESIRED_BUFFER:
        return 0.0

    #define cost function
    cost = 1.0 - (timesteps_away / DESIRED_BUFFER) ** 2
    return cost * DANGER

def inefficiency_cost(vehicle, trajectory, predictions, trajectory_data):
    speed_diff = (vehicle.target_speed - trajectory_data.avg_speed)
    cost = (float(speed_diff) / vehicle.target_speed) ** 2

    return cost * EFFICIENCY

def calculate_cost(vehicle, trajectory, predictions):
    #calculate some common helper data for cost functions
    data = get_helper_data(vehicle, trajectory, predictions)
    #all of our cost functions
    cfs = [lane_change_cost, distance_to_goal_lane, collision_cost, buffer_cost, inefficiency_cost]

    cost = 0.0
    #go through each cost function and find cost
    for cf in cfs:
        new_cost = cf(vehicle, trajectory, predictions, data)
        cost += new_cost

    return cost

"""
Calculates helper TrajectoryData(
	proposed_lane,
	avg_speed,
	max_accel,
	rms_acceleration,
	closest_approach,
	end_distance_to_goal,
	end_lanes_from_goal,
	collides)
"""
def get_helper_data(vehicle, trajectory, predictions):
    #define a shorthand alias for trajectory
    t = trajectory

    #REMEMBER: trajectory[0] is current state (lane, s, v, a, state)
	#of vehicle and not predicted trajectory state. Predicted trajectory
	#state starts from trajectory[1]
    current_state_snapshot = t[0]

    first_snapshot = t[1]
    last_snapshot = t[-1]

    #lane in which vehicle starts this trajectory
    proposed_lane = first_snapshot.lane

    #calculate distance from goal_s after this trajectory is taken
    end_distance_to_goal = vehicle.goal_s - last_snapshot.s
    #calculate distance from goal_lane after this trajectory is taken
    end_lanes_from_goal = abs(vehicle.goal_lane - last_snapshot.lane)

    #calculate ego vehicle's avg speed during this trajectory
    #for calculating speed, we need timestes count which is same
    #as length of trajectory as each snapshot in trajectory represents
    #one timestep
    dt = float(len(trajectory))
    avg_speed = (last_snapshot.s - current_state_snapshot.s) / dt

    #only keep predictions which are in proposed_lane
    filtered_predictions = filter_predictions_by_lane(predictions, proposed_lane)

    #intialize a bunch of variables that we want to calculate
    #distance to closest vehicle
    closest_approach = 999999
    #list of accelerations in all snapshots of given trajectory
    accels = []
    #variable to hold collision information if any
    collides = False

    #go through each trajectory snapshot for PLANNING_HORIZON
    for i, snapshot in enumerate(trajectory[1:PLANNING_HORIZON+1], 1):
        accels.append(snapshot.a)

        #compare this snapshot with every other vehicle in same lane and
        #at same timestep `i` to find closest one and check if collision is possible
        for v_id, v_preds in filtered_predictions.items():
            #get the corresponding prediction from current vehicle predictions
            v_pred_now = v_preds[i]
            #get 1 timestep before prediction as well for calculating speed
            #and check collision etc.
            v_pred_previous = v_preds[i-1]

            #check if collision is possible
            if check_collision(snapshot, v_pred_previous['s'], v_pred_now['s']):
                collides = True
                collides = {"at": i}

            #find distance between ego_vehicle and other vehicle v_id
            distance = abs(snapshot.s - v_pred_now['s'])
            #if this vehicle is more closer than previous one then
            #update closest approach distance
            if distance < closest_approach:
                closest_approach = distance

        #END FOR LOOP filtered_predictions
    #END FOR LOOP trajectory

    #find max acceleration
    max_acceleration = max(accels, key=lambda a: abs(a))
    #calculate root means squared acceleration
    sqaured_a = [a**2 for a in accels]
    mean_squared_a = float(sum(sqaured_a)) / len(sqaured_a)
    root_mean_squared_a = sqrt(mean_squared_a)

    return TrajectoryData(
        proposed_lane,
        avg_speed,
        max_acceleration,
        mean_squared_a,
        closest_approach,
        end_distance_to_goal,
        end_lanes_from_goal,
        collides)
"""
Detects collision between ego vehicle and other vehicle based on
ego vehicle (s, v) and other vehicle (s_now, s_previous)
"""
def check_collision(ego_vehicle, other_vehicle_s_previous, other_vehicle_s_now):
    ego_vehicle_s = ego_vehicle.s
    ego_vehicle_v = ego_vehicle.v

    #calculate other vehicle speed
    #which is: (s_previous - s_now)/dt but as dt=1 so
    other_vehicle_v = other_vehicle_s_now - other_vehicle_s_previous

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
