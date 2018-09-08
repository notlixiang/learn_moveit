#ifndef RRTPLANNER_H
#define RRTPLANNER_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <boost/smart_ptr.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
//#include "constants.h"
#include <rrt.h>
// #include <typeinfo.h>
#include "obstacles.h"
class Obstacles;

using namespace std;
using namespace Eigen;


class RRT_PLANNER : public RRT {
 public:
  Obstacles *obstacles;
  bool isCollisionFree(_type_position &p1, _type_position &p2);
  void clearAll();
  void setStepSize(int step);
  void setMaxIterations(int iter);
  RRT_PLANNER(_type_position startPosFo, _type_position endPosFo, int maxIterFo,
              float endDistThreshold, float stepSizeFo, float neiHoodSizeFo,
              _type_position space_max_limit_fo,
              _type_position space_min_limit_fo);
};

#endif  // RRT_H
