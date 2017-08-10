/*
 * cost_functions.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#include "cost_functions.h"

CostFunctions::CostFunctions()
  : COLLISION(pow(10, 6)), DANGER(pow(10, 5)), REACH_GOAL(pow(10, 5)),
    COMFORT(pow(10, 4)), EFFICIENCY(pow(10, 2)){

}

CostFunctions::~CostFunctions() {
  // TODO
}

double CostFunctions::CalculateCost(const Vehicle &vehicle, map<int, vector<vector<int> > > &predictios, vector<Snapshot> &trajectory) {
  //TODO
  return 0;
}

