#include "rrtplanner.h"

bool RRT_PLANNER::isCollisionFree(_type_position &p1, _type_position &p2) {
  bool temp = !(obstacles->isSegmentInObstacle(p1, p2));
  return temp;
}

RRT_PLANNER::RRT_PLANNER(_type_position startPosFo, _type_position endPosFo,
                         int maxIterFo, float endDistThreshold,
                         float stepSizeFo, float neiHoodSizeFo,
                         _type_position space_max_limit_fo,
                         _type_position space_min_limit_fo)
    : RRT(startPosFo, endPosFo, maxIterFo, endDistThreshold, stepSizeFo,
          neiHoodSizeFo, space_max_limit_fo, space_min_limit_fo) {
  obstacles = new Obstacles;
}

void RRT_PLANNER::clearAll() {
  obstacles->obstacles.clear();
  obstacles->obstacles.resize(0);
  deleteNodes(root);
  deleteNodes(root1);
  deleteNodes(root2);
  nodes.clear();
  nodes.resize(0);
  path.clear();
  path.resize(0);
  nodes1.clear();
  nodes1.resize(0);
  path1.clear();
  path1.resize(0);
  nodes2.clear();
  nodes2.resize(0);
  path2.clear();
  path2.resize(0);
}

void RRT_PLANNER::setStepSize(int step) {
  step_size = step;
  NEIHOOD_SIZE = step_size * 5;
}

void RRT_PLANNER::setMaxIterations(int iter) { max_iter = iter; }
