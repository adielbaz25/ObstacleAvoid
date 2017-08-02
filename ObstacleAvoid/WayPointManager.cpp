/*
 * WayPointManager.cpp
 *
 * Author: Adi Elbaz 206257313
 *         Yuval Ron 313584187
 */

#include "WayPointManager.h"
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

WayPointManager::~WayPointManager() {
  // TODO Auto-generated destructor stub
}

vector<MyCell> WayPointManager::WalkGrid(MyCell source, MyCell target){
  double sourcex = source.first + 0.5;
  double sourcey = source.second + 0.5;
  double targetx = source.first + 0.5;
  double targety = source.second + 0.5;

  vector<MyCell> vec;
  int nx = abs(targetx - sourcex);
  int ny = abs(targety - sourcey);
  int step_x = targetx > sourcex ? 1 : -1;
  int step_y = targety > sourcey ? 1 : -1;
  vec.push_back(MyCell((int)sourcex, (int)sourcey));
  for(int ix = 0, iy = 0; ix < nx || iy < ny;)
  {
    if((double)ix / nx < (double)iy / ny){
      sourcex += step_x;
      ix++;
    }
    else {
      sourcey += step_y;
      iy++;
    }
    vec.push_back(MyCell((int)sourcex, (int)sourcey));
  }
  return vec;
}

WayPointManager::WayPointManager() {
  // TODO Auto-generated constructor stub

}

