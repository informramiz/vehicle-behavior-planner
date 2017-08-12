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

}

CostFunctions::~CostFunctions() {
  // TODO
}

double CostFunctions::CalculateCost(const Vehicle &vehicle,
                                    const map<int, vector<vector<int> > > &predictios,
                                    const vector<Snapshot> &trajectory) {
  //TODO
  return 0;
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

