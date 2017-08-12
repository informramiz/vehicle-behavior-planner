/*
 * Vehicle.h
 *
 *  Created on: Jul 29, 2017
 *      Author: ramiz
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include "snapshot.h"

using namespace std;

class Vehicle {
public:

  struct collider {

    bool collision; // is there a collision?
    int time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  int s;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  /**
   * Constructor
   */
  Vehicle(int lane, int s, int v, int a);

  /**
   * Destructor
   */
  virtual ~Vehicle();

  void update_state(map<int, vector<vector<int> > > predictions);

  void configure(vector<int> road_data);

  string display() const;

  void increment(int dt=1);

  vector<int> state_at(int t);

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector<vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int, vector<vector<int> > > predictions, int lane,
                          int s);

  void realize_keep_lane(map<int, vector<vector<int> > > predictions);

  void realize_lane_change(map<int, vector<vector<int> > > predictions,
                           string direction);

  void realize_prep_lane_change(map<int, vector<vector<int> > > predictions,
                                string direction);

  vector<vector<int> > generate_predictions(int horizon);

private:
  /**
   * Takes snapshot of current state of vehicle and saves it into snapshot object
   * @returns  Snapshot object containing current state of vehicle
   */
  Snapshot take_current_state_snapshot();
  /**
   * Sets current state of vehicle to that of saved in passed snapshot
   * @param snapshot  snapshot containing state of vehicle to restore from
   */
  void restore_state_from_snapshot(const Snapshot& snapshot);

  /**
   * @param predictions,  map of predictions of other vehicle's predicted trajectories
   * @returns   new state for vehicle based on predictions of other vehicles given
   */
  string get_state(const map<int, vector<vector<int> > > &predictions);

  /**
   * @param state, state for which trajectory to calculate
   * @param predictions, map of predictions of other vehicle's predicted trajectories
   * @returns Returns trajectory to for reaching given state
   */
  vector<Snapshot> trajectory_for_state(const string &state,
      const map<int, vector<vector<int> > > &predictions,
      int horizon = 5);

  void remove_first_prediction_for_each(map<int, vector<vector<int> > > &predictions);

};

#endif
