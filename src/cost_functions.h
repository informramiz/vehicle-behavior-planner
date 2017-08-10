/*
 * cost_functions.h
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

using namespace std;

class CostFunctions {
public:
  CostFunctions();
  virtual ~CostFunctions();
  double CalculateCost(const Vehicle &vehicle, map<int, vector<vector<int> > > &predictios, vector<Snapshot> &trajectory);

  //time horizon based on which we want to decide
  //cost of trajectory
  const int PLANNING_HORIZON = 2;
  //timesteps buffer, vehicles should be these many timesteps away
  const double BUFFER_TIME = 1.5;

  //weights for each category of cost functions
  //Safety:
  const double COLLISION;
  const double DANGER;
  //COMFORT
  const double COMFORT;
  //Reach goal
  const double REACH_GOAL;
  //Efficiency
  const double EFFICIENCY;


};

#endif /* COST_FUNCTIONS_H_ */
