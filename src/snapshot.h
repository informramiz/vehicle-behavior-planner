/*
 * snapshot.h
 *
 *  Created on: Aug 10, 2017
 *      Author: ramiz
 */

#ifndef SNAPSHOT_H_
#define SNAPSHOT_H_

#include <string>
using namespace std;

/**
 * A passive object to save a snapshot of state of vehicle
 * at any given time
 */
struct Snapshot {
  int lane;
  int s;
  double v;
  double a;
  string state;

 Snapshot(int lane, int s, double v, double a, string state) {
   this->lane = lane;
   this->s = s;
   this->v = v;
   this->a = a;
   this->state = state;
 }
};



#endif /* SNAPSHOT_H_ */
