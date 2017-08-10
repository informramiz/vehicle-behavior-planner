/*
 * cost_functions.h
 *  Class responsible for calculating cost for given trajectory, based on predictions and ego
 *  vehicle state.
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_

#include <cmath>
#include <map>
#include <iterator>
#include "vehicle.h"
#include "snapshot.h"
#include "trajectory_data.h"

using namespace std;

class CostFunctions {
public:
  CostFunctions();
  virtual ~CostFunctions();

  /**
   * @param vehicle, Ego vehicle for which to calculate cost of trajectory
   * @param predictions,  predicted trajectories of other nearby vehicles on road
   * @param trajectory,   trajectory for which to calculate cost of
   */
  double CalculateCost(const Vehicle &vehicle,
                       const map<int, vector<vector<int> > > &predictios,
                       const vector<Snapshot> &trajectory);

  /*
   * Constants
   */

  //time horizon based on which we want to decide
  //cost of trajectory
  const int PLANNING_HORIZON = 2;
  //timesteps buffer, vehicles should be these many timesteps away
  const double BUFFER_TIME = 1.5;

  //weights for each category of cost functions
  //Safety:
  const double COLLISION;
  const double DANGER;
  //Reach goal
  const double REACH_GOAL;
  //COMFORT
  const double COMFORT;
  //Efficiency
  const double EFFICIENCY;


};

#endif /* COST_FUNCTIONS_H_ */
