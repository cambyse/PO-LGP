#ifndef _HEADER_GUARD_RRT_PLANNER_H_
#define _HEADER_GUARD_RRT_PLANNER_H_

#include <Core/array.h>

struct MotionProblem;
struct OpenGL;

namespace ors { 
  struct Graph;
  struct RRTPlanner {
    private:
      struct sRRTPlanner *s;
    public:
      Graph *G;                 ///< the graph to plan in
      MotionProblem& problem;   ///< the MotionProblem gives the feasibility test for new states

      arr joint_max, joint_min; ///< in which range are the joints allowed (boundaries for the sample space)

      RRTPlanner(ors::Graph* G, MotionProblem &problem, double stepsize);

      arr getTrajectoryTo(const arr& target, OpenGL* gl = NULL); ///< returns the trajectory created by the RRT
  };
}




#endif // _HEADER_GUARD_RRT_PLANNER_H_


