/*
 * Vehicle.cpp
 *
 *  Created on: Jul 29, 2017
 *      Author: ramiz
 */

#include <cassert>
#include <iostream>
#include <algorithm>
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>
#include "cost_functions.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  state = "CS";
  max_acceleration = -1;

}

Vehicle::~Vehicle() {
}

// TODO - Implement this method.
void Vehicle::update_state(map<int, vector<vector<int> > > predictions) {
  /*
   Updates the "state" of the vehicle by assigning one of the
   following values to 'self.state':

   "KL" - Keep Lane
   - The vehicle will attempt to drive its target speed, unless there is
   traffic in front of it, in which case it will slow down.

   "LCL" or "LCR" - Lane Change Left / Right
   - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
   behavior for the "KL" state in the new lane.

   "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
   - The vehicle will find the nearest vehicle in the adjacent lane which is
   BEHIND itself and will adjust speed to try to get behind that vehicle.

   INPUTS
   - predictions
   A dictionary. The keys are ids of other vehicles and the values are arrays
   where each entry corresponds to the vehicle's predicted location at the
   corresponding timestep. The FIRST element in the array gives the vehicle's
   current position. Example (showing a car with id 3 moving at 2 m/s):

   {
   3 : [
   {"s" : 4, "lane": 0},
   {"s" : 6, "lane": 0},
   {"s" : 8, "lane": 0},
   {"s" : 10, "lane": 0},
   ]
   }

   */
  state = get_state(predictions);
}

/**
 * @returns new state for vehicle based on predictions of other vehicles given
 */
string Vehicle::get_state(const map<int, vector<vector<int> > > &predictions) {
  vector<string> possible_states = {"KL", "LCL", "LCR"};

  //check if we are in left-most-lane
  if (this->lane == 0) {
    //we are in left-most lane so Lane Change Left (LCL)
    //is not possible, remove it from possible states
    possible_states.erase(possible_states.begin() + 1);
  }

  //check if we are in right most lane
  if (this->lane == (this->lanes_available - 1)) {
    //we are in right-most lane so Lane Change Left (LCR)
    //is not possible, remove it from possible states
    possible_states.erase(possible_states.begin() + 2);
  }

  vector<double> costs;
  CostFunctions cost_functions;
  for(int i = 0; i < possible_states.size(); ++i) {
    //find a rough trajectory to reach this state
    vector<Snapshot> trajectory = trajectory_for_state(possible_states[i], predictions);
    //find cost for found trajectory
    double cost = cost_functions.calculate_cost(*this, predictions, trajectory);
    costs.push_back(cost);
  }

  //find index of min cost which is also index of corresponding state
  int min_cost_state_index = std::distance(costs.begin(), std::min_element(costs.begin(), costs.end()));

  //return min cost state
  return possible_states[min_cost_state_index];
}

/**
 * @param state, state for which trajectory to calculate
 * @param predictions, map of predictions of other vehicle's predicted trajectories
 * @returns Returns trajectory to for reaching given state
 */
vector<Snapshot> Vehicle::trajectory_for_state(const string &state,
                                               const map<int, vector<vector<int> > > &predictions,
                                               int horizon) {
  //make a copy of predictions are we are going to change it
  map<int, vector<vector<int> > > predictions_copy = predictions;

  //make a vector to hold trajectory
  vector<Snapshot> trajectory;
  //take snapshot of current vehicle state
  Snapshot current_state_snapshot = take_current_state_snapshot();

  //add current state snapshot to trajectory
  trajectory.push_back(current_state_snapshot);

  //for prediction time horizon
  for (int i = 0; i < horizon; ++i) {
    //reset vehicle state to its original state (current_state_snapshot)
    restore_state_from_snapshot(current_state_snapshot);

    //pretend to be in new (passed as param) state
    this->state = state;

    //update vehicle to passed state by changing (lane, a) based on given state and
    //predictions of other vehicles
    realize_state(predictions_copy);

    //validate lane number
    assert(0 <= this->lane && this->lane < this->lanes_available);

    //update current vehicle (s, v) for delta_t=1, assuming constant acceleration
    increment();

    //add current state snapshot to trajectory
    trajectory.push_back(take_current_state_snapshot());

    //As one timestep has been finished so we
    //need to remove first prediction for each vehicle
    //so that all vehicles predictions start from the next timestep
    remove_first_prediction_for_each(predictions_copy);
  }

  //reset state to original state (current_state_snapshot)
  restore_state_from_snapshot(current_state_snapshot);

  return trajectory;
}

void Vehicle::remove_first_prediction_for_each(map<int, vector<vector<int> > > &predictions) {
  map<int, vector<vector<int> > >::iterator iter = predictions.begin();

  while (iter != predictions.end()) {
    //map value (second element in iterator) is vector with each vector value a vector: [s, lane]
    //take a reference to this second vector object so that we can update it
    vector<vector<int> > &v_preds = iter->second;

    //remove first vector of this prediction
    v_preds.erase(v_preds.begin());

    //move to next vehicle predictions
    iter++;
  }
}

/**
 * Takes snapshot of current state of vehicle and saves it into snapshot object
 * @return  Snapshot object containing current state of vehicle
 */
Snapshot Vehicle::take_current_state_snapshot() {
  return Snapshot(this->lane,
      this->s,
      this->v,
      this->a,
      this->state);
}
/**
 * Sets current state of vehicle to that of saved in passed snapshot
 * @param snapshot  snapshot containing state of vehicle to restore from
 */
void Vehicle::restore_state_from_snapshot(const Snapshot& snapshot) {
  this->lane = snapshot.lane;
  this->s = snapshot.s;
  this->v = snapshot.v;
  this->a = snapshot.a;
  this->state = snapshot.state;
}

void Vehicle::configure(vector<int> road_data) {
  /*
   Called by simulator before simulation begins. Sets various
   parameters which will impact the ego vehicle.
   vector<int> ego_config = { SPEED_LIMIT, num_lanes, goal_s, goal_lane,
        MAX_ACCEL };
   */
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}

string Vehicle::display() {

  ostringstream oss;

  oss << "s:    " << this->s << "\n";
  oss << "lane: " << this->lane << "\n";
  oss << "v:    " << this->v << "\n";
  oss << "a:    " << this->a << "\n";

  return oss.str();
}

void Vehicle::increment(int dt) {

  this->s += this->v * dt;
  this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

  /*
   Predicts state of vehicle in t seconds (assuming constant acceleration)
   */
  int s = this->s + this->v * t + this->a * t * t / 2;
  int v = this->v + this->a * t;
  return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

  /*
   Simple collision detection.
   */
  vector<int> check1 = state_at(at_time);
  vector<int> check2 = other.state_at(at_time);
  return (check1[0] == check2[0]) && (abs(check1[1] - check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

  Vehicle::collider collider_temp;
  collider_temp.collision = false;
  collider_temp.time = -1;

  for (int t = 0; t < timesteps + 1; t++) {
    if (collides_with(other, t)) {
      collider_temp.collision = true;
      collider_temp.time = t;
      return collider_temp;
    }
  }

  return collider_temp;
}

void Vehicle::realize_state(map<int, vector<vector<int> > > predictions) {

  /*
   Given a state, realize it by adjusting acceleration and lane.
   Note - lane changes happen instantaneously.
   */
  string state = this->state;
  if (state.compare("CS") == 0) {
    realize_constant_speed();
  } else if (state.compare("KL") == 0) {
    realize_keep_lane(predictions);
  } else if (state.compare("LCL") == 0) {
    realize_lane_change(predictions, "L");
  } else if (state.compare("LCR") == 0) {
    realize_lane_change(predictions, "R");
  } else if (state.compare("PLCL") == 0) {
    realize_prep_lane_change(predictions, "L");
  } else if (state.compare("PLCR") == 0) {
    realize_prep_lane_change(predictions, "R");
  }

}

void Vehicle::realize_constant_speed() {
  a = 0;
}

int Vehicle::_max_accel_for_lane(map<int, vector<vector<int> > > predictions,
                                 int lane, int s) {

  int delta_v_til_target = target_speed - v;
  int max_acc = min(max_acceleration, delta_v_til_target);

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > in_front;
  while (it != predictions.end()) {

    int v_id = it->first;

    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] > s)) {
      in_front.push_back(v);

    }
    it++;
  }

  if (in_front.size() > 0) {
    int min_s = 1000;
    vector<vector<int>> leading = { };
    for (int i = 0; i < in_front.size(); i++) {
      if ((in_front[i][0][1] - s) < min_s) {
        min_s = (in_front[i][0][1] - s);
        leading = in_front[i];
      }
    }

    int next_pos = leading[1][1];
    int my_next = s + this->v;
    int separation_next = next_pos - my_next;
    int available_room = separation_next - preferred_buffer;
    max_acc = min(max_acc, available_room);
  }

  return max_acc;

}

void Vehicle::realize_keep_lane(map<int, vector<vector<int> > > predictions) {
  this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int, vector<vector<int> > > predictions,
                                  string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  this->lane += delta;
  int lane = this->lane;
  int s = this->s;
  this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(
    map<int, vector<vector<int> > > predictions, string direction) {
  int delta = -1;
  if (direction.compare("L") == 0) {
    delta = 1;
  }
  int lane = this->lane + delta;

  map<int, vector<vector<int> > >::iterator it = predictions.begin();
  vector<vector<vector<int> > > at_behind;
  while (it != predictions.end()) {
    int v_id = it->first;
    vector<vector<int> > v = it->second;

    if ((v[0][0] == lane) && (v[0][1] <= this->s)) {
      at_behind.push_back(v);

    }
    it++;
  }
  if (at_behind.size() > 0) {

    int max_s = -1000;
    vector<vector<int> > nearest_behind = { };
    for (int i = 0; i < at_behind.size(); i++) {
      if ((at_behind[i][0][1]) > max_s) {
        max_s = at_behind[i][0][1];
        nearest_behind = at_behind[i];
      }
    }
    int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    int delta_v = this->v - target_vel;
    int delta_s = this->s - nearest_behind[0][1];
    if (delta_v != 0) {

      int time = -2 * delta_s / delta_v;
      int a;
      if (time == 0) {
        a = this->a;
      } else {
        a = delta_v / time;
      }
      if (a > this->max_acceleration) {
        a = this->max_acceleration;
      }
      if (a < -this->max_acceleration) {
        a = -this->max_acceleration;
      }
      this->a = a;
    } else {
      int my_min_acc = max(-this->max_acceleration, -delta_s);
      this->a = my_min_acc;
    }

  }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

  vector<vector<int> > predictions;
  for (int i = 0; i < horizon; i++) {
    vector<int> check1 = state_at(i);
    vector<int> lane_s = { check1[0], check1[1] };
    predictions.push_back(lane_s);
  }
  return predictions;

}

