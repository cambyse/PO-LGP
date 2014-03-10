#include "rrt_planner.h"

#include <Ors/ors.h>
#include <Algo/rrt.h>
#include <Motion/motion.h>

#include <Gui/opengl.h>
#include <Gui/plot.h>

#include <devTools/logging.h>
SET_LOG(rrt_planner, DEBUG)

namespace ors {
  struct sRRTPlanner {
    RRTPlanner *p;
    RRT rrt;

    sRRTPlanner(RRTPlanner *p, RRT rrt, bool verbose) : p(p), rrt(rrt), verbose(verbose) { };

<<<<<<< HEAD
    bool growTowards(RRT& growing, RRT& passive);

    bool isFeasible(const arr& q);
=======
    bool growTowards(RRT& growing, RRT& passive, ors::KinematicWorld &G);
>>>>>>> master

    uint success_growing;
    uint success_passive;

    bool verbose;
  };
}

bool ors::sRRTPlanner::isFeasible(const arr& q) {
  arr phi, J_x, J_v;
  p->problem.setState(q, NoArr);
  return p->problem.getTaskCosts(phi, J_x, J_v, 0);
}

bool ors::sRRTPlanner::growTowards(RRT& growing, RRT& passive) {
  arr q;
  if(rnd.uni()<.5) {
    q = p->joint_min + rand(p->problem.world.getJointStateDimension(), 1) % ( p->joint_max - p->joint_min );
    q.reshape(q.d0);
  }
  else { 
    q = passive.getRandomNode();
  }
  arr proposal;
  growing.getProposalTowards(proposal, q);

  bool feasible = isFeasible(proposal);
  if (feasible) { 
    growing.add(proposal);
    arr tmp_prop;
    double d = passive.getProposalTowards(tmp_prop, proposal);

    if (d < growing.getStepsize()) {
      growing.getProposalTowards(tmp_prop, proposal); // to actually get the latest point
      success_growing = growing.getNearest();
      success_passive = passive.getNearest();
      return true;
    }
  } 
  return false;
}

arr buildTrajectory(RRT& rrt, uint node, bool forward) {
  arr q;
  uint N = rrt.getNode(node).N;
  uint i = 1; // this is not 0, because we do "do...while"
  do {
    q.append(rrt.getNode(node));
    node = rrt.getParent(node);
    
    ++i;
  }
  while(node);
  // append the root node
  q.append(rrt.getNode(0));

  q.reshape(i, N);
  if (forward) {
   q.reverseRows();   
  }

  return q;
}
    
ors::RRTPlanner::RRTPlanner(ors::KinematicWorld *G, MotionProblem &problem, double stepsize, bool verbose) : 
  s(new ors::sRRTPlanner(this, RRT(G->getJointState(), stepsize), verbose)), G(G), problem(problem) {
    joint_min = zeros(G->getJointStateDimension(), 1);
    joint_max = ones(G->getJointStateDimension(), 1);
  }

void drawRRT(RRT rrt) {
  for(uint i=1; i < rrt.getNumberNodes(); ++i) {
    arr line;
    line.append(rrt.getNode(i)); line.reshape(1, line.N);
    line.append(rrt.getNode(rrt.getParent(i)));
    plotLine(line);
  }
}

arr ors::RRTPlanner::getTrajectoryTo(const arr& target, int max_iter) {
  arr q;

  if (!s->isFeasible(target))
    return arr(0);

  RRT target_rrt(target, s->rrt.getStepsize());

  bool found = false;
  uint node0 = 0, node1 = 0;

  int iter = 0;
  while(!found) {
    found = s->growTowards(s->rrt, target_rrt);
    if(found) {
      node0 = s->success_growing;
      node1 = s->success_passive;
      break;
    }

    found = s->growTowards(target_rrt, s->rrt);
    if(found) {
      node0 = s->success_passive;
      node1 = s->success_growing;
      break;
    }
    if (s->verbose && iter % 20 == 0) std::cout << "." << std::flush;
    if (max_iter && iter >= max_iter) return arr(0);
    iter++;
  }
  if (s->verbose) std::cout << std::endl;

  arr q0 = buildTrajectory(s->rrt, node0, true);
  arr q1 = buildTrajectory(target_rrt, node1, false);

  // add trajectories
  q.append(q0);
  q.append(q1);
  q.reshape(q0.d0 + q1.d0, q0.d1);

  return q;
}


