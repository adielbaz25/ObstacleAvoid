/*
 * WayPointManager.h
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

#include <vector>
#include <iostream>
using namespace std;

typedef pair<int, int> MyCell;

class WayPointManager {
public:
  virtual ~WayPointManager();
  vector<MyCell> WalkGrid(MyCell source, MyCell target);
  WayPointManager();
};

#endif /* WAYPOINTS_H_ */
