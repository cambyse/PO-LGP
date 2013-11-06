#ifndef _HEADER_GUARD_RRT_PLANNER_H_
#define _HEADER_GUARD_RRT_PLANNER_H_

#include <Core/array.h>

struct MotionProblem;

namespace ors { 
  struct Graph;
  struct RRTPlanner {
    private:
      struct sRRTPlanner *s;
    public:
      Graph *G;
      MotionProblem& problem;

      RRTPlanner(ors::Graph* G, MotionProblem &problem, double stepsize);

      arr getTrajectoryTo(arr, double prec);
  };
}




#endif // _HEADER_GUARD_RRT_PLANNER_H_


