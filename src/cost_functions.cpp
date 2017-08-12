/*
 * cost_functions.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#include "cost_functions.h"

CostFunctions::CostFunctions()
: COLLISION(pow(10, 6)), DANGER(pow(10, 5)), REACH_GOAL(pow(10, 5)),
  COMFORT(pow(10, 4)), EFFICIENCY(pow(10, 2)) {
  cost_functions_pointers = {&CostFunctions::change_lane_cost,
      &CostFunctions::distance_from_goal_lane_cost,
      &CostFunctions::inefficiency_cost,
      &CostFunctions::collision_cost};
}

CostFunctions::~CostFunctions() {
  // TODO
}

/**
 * Calculates cost for changing lane based on trajectory data and predictions
 */
double CostFunctions::change_lane_cost(const Vehicle &vehicle,
                                       const map<int, vector<vector<int> > > &predictios,
                                       const vector<Snapshot> &trajectory,
                                       const TrajectoryData &data) {
  //as trajectory[0] is current state of vehicle and not predicted state
  //and data.proposed_lane is the lane for which we needs to calculate cost for

  double cost = 0;
  //check if new proposed lane is far from goal lane, as compared to current lane
  // is to goal lane
  if (data.end_distance_from_goal_lane >  trajectory[0].lane) {
    //penalize as we want to stay close to goal lane
    cost = COMFORT;
  }

  //check if new proposed lane is close to goal lane, as compared to current lane
  // is to goal lane
  if (data.end_distance_from_goal_lane >  trajectory[0].lane) {
    //reward as we want to stay close to goal lane
    cost = -COMFORT;
  }

  if (cost != 0) {
    printf("!! \n \ncost for lane change is %f\n\n", cost);
  }

  return cost;
}

/**
 * Calculates cost for how far we are from goal lane and based on
 * in how many timesteps we will reach to goal position.
 */
double CostFunctions::distance_from_goal_lane_cost(const Vehicle &vehicle,
                                                   const map<int, vector<vector<int> > > &predictions,
                                                   const vector<Snapshot> &trajectory,
                                                   const TrajectoryData &data) {

  //we don't want distance below 1.0, so if it less then pick 1.0
  int distance = std::max(data.end_distance_from_goal_lane, 1);

  //calculate in how many timesteps we will reach goal position
  //given that we know avg_speed of vehicle
  double timesteps_away = data.end_distance_to_goal / data.avg_speed;
  //find a cost function (multiplier) value based distance from goal_lane and time to reach goal_lane
  //it should be in way so that if time_to_goal is greater then multiplier is less
  //otherwise bigger and vice versal for distance to goal_lane (lanes)
  //as distance from goal lane is a small value so multiply it with a constant
  //to normalize it with timesteps value
  double cost = (5.0 * distance) / timesteps_away;

  //multiple cost with its weight
  cost *= REACH_GOAL;

  return cost;
}

/**
 * Calculates cost based on how far vehicle's velocity is from target velocity
 */
double CostFunctions::inefficiency_cost(const Vehicle &vehicle,
                                        const map<int, vector<vector<int> > > &predictions,
                                        const vector<Snapshot> &trajectory,
                                        const TrajectoryData &data) {
  double cost = (vehicle.target_speed - data.avg_speed) / vehicle.target_speed;
  return pow(cost, 2) * EFFICIENCY;
}

/**
 * Calculates cost based on whether collision is detected or not
 */
double CostFunctions::collision_cost(const Vehicle &vehicle,
                      const map<int, vector<vector<int> > > &predictions,
                      const vector<Snapshot> &trajectory,
                      const TrajectoryData &data) {
  if (!data.is_collision_detected) {
    return 0;
  }

  int exponent = pow(data.collides_at, 2);
  //increases if exponent is small or decreases if exponent is large
  //as we want more cost of time till collision is small
  double cost = exp(-exponent);

  return cost * COLLISION;
}

double CostFunctions::calculate_cost(const Vehicle &vehicle,
                                     const map<int, vector<vector<int> > > &predictions,
                                     const vector<Snapshot> &trajectory) {
  TrajectoryData data = calculate_helper_data(vehicle, predictions, trajectory);

  double cost = 0;
  for (int i = 0; i < cost_functions_pointers.size(); ++i) {
    (this->*cost_functions_pointers[i])(vehicle, predictions, trajectory, data);
  }

  return cost;
}

/**
Calculates helper TrajectoryData(
  proposed_lane,
  avg_speed,
  max_accel,
  rms_acceleration,
  closest_approach,
  end_distance_to_goal,
  end_lanes_from_goal,
  is_collision_detected,
  )

  @param  vehicle,      vehicle for which trajectory is being planned
  @param  predictions,  predictions about other vehicles trajectories
  @param  trajectory,   trajectory for which to calculate cost for

  @returns  TrajectoryData,   A passive object @see TrajectoryData
                              containing some helpful results about trajectory given
 */
TrajectoryData CostFunctions::calculate_helper_data(const Vehicle &vehicle,
                                                    const map<int, vector<vector<int> > > &predictions,
                                                    const vector<Snapshot> &trajectory) {

  //REMEMBER: trajectory[0] is current state (lane, s, v, a, state)
  //of vehicle and not predicted trajectory state. Predicted trajectory
  //state starts from trajectory[1]
  Snapshot current_state_snapshot = trajectory[0];
  Snapshot first_snapshot = trajectory[1];
  Snapshot last_snapshot = trajectory[trajectory.size() - 1];

  //the lane to move on for this trajectory
  int proposed_lane = first_snapshot.lane;

  //number of timesteps (dt) for which this trajectory is predicted
  //which is same as number of snapshots in trajectory as each snaphsot
  //is for a single timestep
  int dt = trajectory.size();

  //avg speed during this speed -> avg_speed = distance/time
  double avg_speed = (last_snapshot.s - current_state_snapshot.s) / (double)dt;

  //distance left from goal position
  int end_distance_from_goal = vehicle.goal_s - last_snapshot.s;
  //distance of current lane from goal lane
  int end_distance_from_goal_lane = abs(vehicle.goal_lane - last_snapshot.lane);

  //we are only interested in predictions that are in proposed lane, so filter only those
  map<int, vector<vector<int> > > filtered_preds = filter_predictions_by_lane(predictions, proposed_lane);

  //variables that we needs to calculate
  bool is_collision_detected = false;
  int collision_detected_at_timestep = -1;
  int closest_approach = 999999;
  vector<double> accelerations;

  for (int i = 1; i < PLANNING_HORIZON + 1; ++i) {

    //save acceleration value for current trajectory snapshot
    accelerations.push_back(trajectory[i].a);

    map<int, vector<vector<int> > >::const_iterator preds_itr = predictions.begin();
    while (preds_itr != predictions.end()) {
      //map key is vehicle id
      int v_id = preds_itr->first;
      //map value is a list: [s, lane]
      vector<vector<int> > predictions = preds_itr->second;

      //extract other vehicle's prediction corresponding to current timestep i
      vector<int> pred_now = predictions[i];
      //extract previous prediction
      vector<int> pred_previous = predictions[i-1];

      //check if there is a collision between current trajectory snapshot of vehicle and
      //other vehicle
      if(check_collision(trajectory[i], pred_now[0], pred_previous[0])) {
        //mark collision timestep
        is_collision_detected = true;
        collision_detected_at_timestep = i;
      }

      //find distance between ego vehicle's current snapshot and other vehicle's
      //corresponding prediction
      int distance = abs(pred_now[0] - trajectory[i].s);

      //check if this distance is less than other closest approach
      //distance that we already have, we need to keep track of closest distance to any vehicle
      if (distance < closest_approach) {
        closest_approach = distance;
      }

      //move to predictions of next vehicle
      preds_itr++;

    } //END WHILE LOOP
  } //END FOR LOOP

  //find max acceleration, either negative or positive acceleration
  //as acceleration can be negative or positive and we need max based on
  //value and not sign so we will define a lambad function along with
  //STL's max_element function to compare absolute values
  double max_acceleration = *std::max_element(accelerations.begin(), accelerations.end(),
                                              [](const double &lhs, const double &rhs) {
    return abs(lhs) < abs(rhs);
  });

  //calculate mean squared acceleration
  double squared_sum_acceleration = std::accumulate(accelerations.begin(), accelerations.end(), 0.0,
                                                    [](const double &lhs, const double &rhs) {
    //we need squared sum so sqaure each value
    return pow(lhs, 2) + pow(rhs, 2);
  });

  //calculate mean of sqaured sums
  double mean_squared_acceleration = squared_sum_acceleration / (float)accelerations.size();


  return TrajectoryData(proposed_lane,
      avg_speed,
      max_acceleration,
      mean_squared_acceleration,
      closest_approach,
      end_distance_from_goal,
      end_distance_from_goal_lane,
      is_collision_detected,
      collision_detected_at_timestep);
}

/**
 * Detects if there is a collision between two vehicles based on (s, v)
 */
bool CostFunctions::check_collision(const Snapshot& snapshot, int other_vehicle_s_now, int other_vehicle_s_previous) {
  //calculate other vehicle speed
  //which is: (s_previous - s_now)/dt but as dt=1 so
  double other_vehicle_v = other_vehicle_s_now - other_vehicle_s_previous;

  //there are 3 cases in which collision can happen

  //CASE-1
  //Other vehicle was behind ego vehicle but in next timestep it is at same time step
  //due to acceleration may be
  if (other_vehicle_s_previous < snapshot.s) {
    if (other_vehicle_s_now >= snapshot.s) {
      return true;
    }
  }

  //CASE-2
  //Other vehicle was ahead of ego vehicle in previous timestep and in next timestep
  //both vehicles are at same position
  if (other_vehicle_s_previous > snapshot.s) {
    if (other_vehicle_s_now <= snapshot.s) {
      return true;
    }
  }

  //CASE-3
  //Ego vehicle is now at same position as was other vehicle in previous time step so
  //in this case if other vehicle's speed is less than the speed of ego vehicle
  //then collision is imminent
  if (other_vehicle_s_previous == snapshot.s) {
    if (other_vehicle_v <= snapshot.v) {
      return true;
    }
  }

  return false;
}

/**
 * Filter predictions and only keep predictions whose first prediction is in given lane
 */
map<int, vector<vector<int> > > CostFunctions::filter_predictions_by_lane(const map<int, vector<vector<int> > > &predictions,
                                                                          int lane) {
  map<int, vector<vector<int> > > filtered_preds;

  map<int, vector<vector<int> > >::const_iterator preds_itr = predictions.begin();
  while(preds_itr != predictions.end()) {
    //map key is vehicle id
    int v_id = preds_itr->first;
    //map value is a list: [s, lane]
    vector<vector<int> > v_preds = preds_itr->second;

    //check first lane of each prediction and also make sure it not ego vehicle (ego vehicle has id -1)
    if (v_preds[0][1] == lane && v_id != -1) {
      filtered_preds[v_id] = v_preds;
    }

    //move to preds
    preds_itr++;
  }

  return filtered_preds;
}

