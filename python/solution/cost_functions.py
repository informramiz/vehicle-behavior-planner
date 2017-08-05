from __future__ import division
from collections import namedtuple
from math import sqrt,exp
import pdb

TrajectoryData = namedtuple("TrajectoryData", [
	'proposed_lane',
	'avg_speed',
	'max_acceleration',
	'rms_acceleration',
	'closest_approach',
	'end_distance_to_goal',
	'end_lanes_from_goal',
	'collides',
	])

# priority levels for costs
COLLISION  = 10 ** 6
DANGER     = 10 ** 5
REACH_GOAL = 10 ** 5
COMFORT    = 10 ** 4
EFFICIENCY = 10 ** 2

DESIRED_BUFFER = 1.5 # timesteps
PLANNING_HORIZON = 2

DEBUG = False
# DEBUG = True

"""
Penalizes lane changes AWAY from the goal lane and rewards
lane changes TOWARDS the goal lane.
"""
def change_lane_cost(vehicle, trajectory, predictions, data):
	"""
	Penalizes lane changes AWAY from the goal lane and rewards
	lane changes TOWARDS the goal lane.
	"""
	#Find how far we will be from goal_lane based on lane we will be at (end_lanes_from_goal)
	#if given trajectory is taken
	proposed_lanes = data.end_lanes_from_goal
	#what is the lane number before this trajectory is taken
	cur_lanes = trajectory[0].lane

	cost = 0
	#check if resultant lane number after taking given trajectory is
	#far from lane number before taking given trajectory. If it is far
	#then we need to penalize this trajectory as we are moving away from
	#goal lane
	if proposed_lanes > cur_lanes:
		cost = COMFORT

	#check if resultant lane number after taking given trajectory is
	#less from lane number before taking given trajectory. If it is less
	#then we need to reward (negative cost) this trajectory as we are moving towards the
	#goal lane
	if proposed_lanes < cur_lanes:
		cost = -COMFORT

	if cost != 0:
		print("!! \n \ncost for lane change is {}\n\n".format(cost))
	return cost

"""
Penalizes state trajectory based on how far it is from goal_lane and time required
to reach the goal_s
"""
def distance_from_goal_lane(vehicle, trajectory, predictions, data):
	"""

	"""
	#distance from goal_s
	distance = abs(data.end_distance_to_goal)
	#Make it 1.0 if it is less than 1.0
	distance = max(distance,1.0)
	#find time to reach goal_s based on avg speed and distance left to reach goa_s
	time_to_goal = float(distance) / data.avg_speed
	#distance from goal_lane
	lanes = data.end_lanes_from_goal
	#find a cost function (multiplier) value based distance from goal_lane and time to reach goal_lane
	#it should be in way so that if time_to_goal is greater then multiplier is less
	#otherwise bigger and vice versal for distance to goal_lane (lanes)
	multiplier = float(5 * lanes / time_to_goal)

	#multiply cost function value (multiplier) with its weight REACH_GOAL
	cost = multiplier * REACH_GOAL

	return cost
	pass

"""
We want to reach goal_s efficiently and quickly so by maintaining a target speed
if possible. This function penalizes trajectories that are not efficient in achieving
target speed
"""
def inefficiency_cost(vehicle, trajectory, predictions, data):
	#extract vehicle avg speed and target speed
	speed = data.avg_speed
	target_speed = vehicle.target_speed

	#find the difference between vehicle's current speed and target_speed
	diff = target_speed - speed

	#define cost function (multiplier) that
	#penalize more if difference is greater or less if difference is less
	pct = float(diff) / target_speed
	multiplier = pct ** 2
	#multiple cost function value with its weight EFFICIENCY
	return multiplier * EFFICIENCY
	pass

"""
If there is collision chance then penalize this trajectory based on how
close vehicle is to collision
"""
def collision_cost(vehicle, trajectory, predictions, data ):
	#check if collision is going to happen at any predicted timestep
	if data.collides:
		#extract timestep on which collision is going to happen
		time_til_collision = data.collides['at']
		#define cost function (multiplier) which penalizes based on
		#how close we are to the collision
		exponent = (float(time_til_collision) ) ** 2
		mult = exp(-exponent)

		#multiple cost function value with its weight COLLISION
		return mult * COLLISION
	return 0

"""
We want to keep a minimum buffer distance from vehicles ahead
This cost function penalizes trajectories that don't keep a buffer distance
"""
def buffer_cost(vehicle, trajectory, predictions, data):
	#extract closest approach to next leading after this trajectory is taken
	closest = data.closest_approach
	#if there is no buffer distance left then penalize maximum
	if closest == 0:
		return 10 * DANGER

	#find timesteps till we leave no buffer distance
	timesteps_away = closest / data.avg_speed
	#if timesteps to leave no buffere distance are greater than DESIRED_BUFFER timesteps
	#then there is no need to penalize
	if timesteps_away > DESIRED_BUFFER:
		return 0.0

	#otherwise penalize based on how small are we timesteps_away from covering buffer distance
	multiplier = 1.0 - (timesteps_away / DESIRED_BUFFER)**2

	#multiply cost function value with its weight DANGER
	return multiplier * DANGER
	pass

def calculate_cost(vehicle, trajectory, predictions, verbose=False):
	trajectory_data = get_helper_data(vehicle, trajectory, predictions)
	cost = 0.0
	for cf in [
		distance_from_goal_lane,
		inefficiency_cost,
		collision_cost,
		buffer_cost,
		change_lane_cost]:
		new_cost = cf(vehicle, trajectory, predictions, trajectory_data)
		if DEBUG or verbose:
			print("{} has cost {} for lane {}".format(cf.__name__, new_cost, trajectory[-1].lane))
			# pdb.set_trace()
		cost += new_cost
	return cost

	pass

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
def get_helper_data(vehicle,trajectory,predictions):
	print("trajectory: ",  trajectory)

	#REMEMBER: trajectory[0] is current state (lane, s, v, a, state)
	#of vehicle and not predicted trajectory state. Predicted trajectory
	#state starts from trajectory[1]

	#define alias for trajectory
	t = trajectory

	#as first snapshot of trajectory was not predicted but current state
	#of vehicle
	current_snapshot = t[0]

	#extract first and last trajectory snapshots
	first = t[1]
	last = t[-1]

	#find distance left when current trajectory is completed
	#which is distance between trajectory's last snapshot's `s` coordinate
	#and goal_s
	end_distance_to_goal = vehicle.goal_s - last.s

	#find distance of trajectory's last snapshot's lane from goal lane
	#this is is how far we are from goal lane if current trajectory is taken
	end_lanes_from_goal = abs(vehicle.goal_lane - last.lane)
	#find number of timesteps corresponding to trajectory
	dt = float(len(trajectory))

	#proposed_lane is the lane this trajectory starts from
	proposed_lane = first.lane

	#calculate avg speed before taking this trajectory and after trajectory
	#has been taken
	avg_speed = (last.s - current_snapshot.s) / dt

	# initialize a bunch of variables
	accels = []
	closest_approach = 999999
	collides = False
	#???? Feels wrong
	last_snap = trajectory[0]

	#extract predictions that are in proposed_lane at timestep 0 and are not of ego vehicle
	filtered = filter_predictions_by_lane(predictions, proposed_lane)

	#go through trajectory snapshots starting from index 1 (because index 0 is original state)
	#upto PLANNING_HORIZON timesteps
	for i, snapshot in enumerate(trajectory[1:PLANNING_HORIZON+1], 1):
		#unpack values from snapshot
		lane, s, v, a = unpack_snapshot(snapshot)

		#append extract `a` value to list of accels.
		accels.append(a)

		#go through filtered predictions
		for v_id, v in filtered.items():
			#extract the prediction corresponding to current trajectory timestep
			state = v[i]
			#extract the previous prediction corresponding to current trajectory timestep
			last_state = v[i-1]

			#check if given vehicle and other vehicle collides
			#based on trajectory's current snapshot and other vehicle's corresponding
			#current and previous position (last_state['s'], state['s'])
			vehicle_collides = check_collision(snapshot, last_state['s'], state['s'])

			#if vehicles do collided then set collision flag
			#true and add collision trajectory index/timestep to collides dictionary
			if vehicle_collides:
				collides = True
				collides = {"at" : i}

			#find distance between trajectory's current snapshot's `s` and
			#and corresponding predicted `s` of other vehicle
			dist = abs(state['s'] - s)
			#if distance is under closest_approach then
			#mark this distance a closest_approach instead
			if dist < closest_approach:
				closest_approach = dist

		#END FOR LOOP filtered.items()

		#save currently visited snapshot for later use, if any
		last_snap = snapshot

	#END FOR LOOP enumerate(trajectory[1:PLANNING_HORIZON+1])

	#choose max acceleration from the list of acceleration (accels)
	#made by extracting each from snapshots in trajectory. We will use this
	#in other cost functions
	max_accel = max(accels, key=lambda a: abs(a))

	#Calculate Mean Squared acceleration (sum(A(i)^2) / N ) )

	#1. Take sqaure of each value
	rms_accels = [a**2 for a in accels]
	#2. find total elements count (N) to find mean
	num_accels = len(rms_accels)
	#find mean of squared acceleration values
	rms_acceleration = float(sum(rms_accels)) / num_accels

	return TrajectoryData(
		proposed_lane,
		avg_speed,
		max_accel,
		rms_acceleration,
		closest_approach,
		end_distance_to_goal,
		end_lanes_from_goal,
		collides)


def check_collision(snapshot, s_previous, s_now):
	#extract given vehicle's s and v from snapshot
	s = snapshot.s
	v = snapshot.v
	#calculate speed of other vehicle based its predicted current s (s_now)
	#and predicted previous s (s_previous)
	v_target = s_now - s_previous

	#CASE: Other vehicle might be accelerating
	#if other vehicle's previous position (s_previous) was less than given
	#current vehicle's current position (s)
	if s_previous < s:
		#and if other vehicle's current position (s_now) is greater than or equal
		#to current vehicle's current position (s) that means other vehicle
		#is going to collide with current/given vehicle
		if s_now >= s:
			return True
		else:
			return False

	#Case: Other vehicle might be decelerating
	#if other vehicle's s_previous was greater than given vehicle's s
	#if it is decelerating then its current position (s_now) might match
	#given/current vehicle's position
	if s_previous > s:
		if s_now <= s:
			return True
		else:
			return False

	#CASE: If other vehicle's s_previous was same as current vehicle's `s`
	#then if other vehicle's speed is greater than current vehicle's speed
	#then collision can be avoided, otherwise both will collide
	if s_previous == s:
		if v_target > v:
			return False
		else:
			return True
	raise ValueError


def unpack_snapshot(snapshot):
	s = snapshot
	return s.lane, s.s, s.v, s.a

"""
Filter predictions that are in given lane
"""
def filter_predictions_by_lane(predictions, lane):
	filtered = {}
	for v_id, predicted_traj in predictions.items():
		if predicted_traj[0]['lane'] == lane and v_id != -1:
			filtered[v_id] = predicted_traj
	return filtered
