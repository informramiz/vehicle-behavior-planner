/*
 * trajectory_data.h
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#ifndef TRAJECTORY_DATA_H_
#define TRAJECTORY_DATA_H_

/**
 * A passive object to hold helpful data extracted from trajectory
 * when calculating cost for given trajectory
 */
struct TrajectoryData {
  //proposed lane for new trajectory
  int proposed_lane;
  //avg speed for given trajectory
  double avg_speed;
  //max acceleration for given trajectory
  double max_acceleration;
  //mean squared acceleration for given trajectory
  double mean_squared_acceleration;
  //double distance to closes vehicle, we have to keep a buffer distance other traffic
  double closest_approach;
  //when this trajectory is taken, how much distance from goal position will be left
  //at the end of this trajectory
  double end_distance_to_goal;
  //at the end of new trajectory how far is the lane vehicle is in from goal lane
  int end_distance_from_goal_lane;
  //flag for collision detection
  bool is_collision_detected;
  //variable to hold timestep at which collision happens, if any
  int collides_at;

  TrajectoryData(int proposed_lane,
                 double avg_speed,
                 double max_acceleration,
                 double mean_squared_acceleration,
                 double closes_approach,
                 double end_distance_to_goal,
                 double end_distance_from_goal_lane,
                 double is_collision_detected,
                 int collides_at) {

    this->proposed_lane = proposed_lane;
    this->avg_speed = avg_speed;
    this->max_acceleration = max_acceleration;
    this->mean_squared_acceleration = mean_squared_acceleration;
    this->closest_approach = closes_approach;
    this->end_distance_to_goal = end_distance_to_goal;
    this->end_distance_from_goal_lane = end_distance_from_goal_lane;
    this->is_collision_detected = is_collision_detected;
    this->collides_at = collides_at;
  }
};


#endif /* TRAJECTORY_DATA_H_ */
