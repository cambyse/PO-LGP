#include "rrt_planner.h"

#include <Ors/ors.h>
#include <Algo/rrt.h>
#include <Motion/motion.h>

#include <Gui/opengl.h>
#include <Gui/plot.h>

#include <retired/devTools/logging.h>
#include <retired/devTools/logging.cpp>
SET_LOG(rrt_planner, DEBUG)

namespace ors {
  struct sRRTPlanner {
    RRTPlanner *p;
    RRT rrt;

    sRRTPlanner(RRTPlanner *p, RRT rrt) : p(p), rrt(rrt) { };

    bool growTowards(RRT& growing, RRT& passive, ors::Graph &G);

    uint success_growing;
    uint success_passive;
  };
}


bool ors::sRRTPlanner::growTowards(RRT& growing, RRT& passive, ors::Graph &G) {
  arr q;
  if(rnd.uni()<.5) {
    q = p->joint_min + rand(G.getJointStateDimension(), 1) % ( p->joint_max - p->joint_min );
    q.reshape(q.d0);
  }
  else { 
    passive.getRandomNode(q);
  }

  growing.getProposalTowards(q);

  G.setJointState(q);
  G.calcBodyFramesFromJoints();

  ors::Graph *G_t = p->problem.ors;
  p->problem.ors = &G;
  arr phi, J_x, J_v;
  p->problem.swift->computeProxies(G);
  bool feasible = p->problem.getTaskCosts(phi, J_x, J_v, 0);
  p->problem.ors = G_t;

  if (feasible) {

    growing.add(q);
    double d = passive.getProposalTowards(q);

    if (d < growing.getStepsize()) {
      growing.getProposalTowards(q); // to actually get the latest point
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
    
ors::RRTPlanner::RRTPlanner(ors::Graph *G, MotionProblem &problem, double stepsize) : 
  s(new ors::sRRTPlanner(this, RRT(G->getJointState(), stepsize))), G(G), problem(problem) {
    joint_min = zeros(G->getJointStateDimension(), 1);
    joint_max = ones(G->getJointStateDimension(), 1);
  }

arr ors::RRTPlanner::getTrajectoryTo(const arr& target, const double prec) {
  ors::Graph *copy = G->newClone();
  arr q;

  RRT target_rrt(target, s->rrt.getStepsize());

  bool found = false;

  uint node0 = 0, node1 = 0;

  while(!found) {
    found = s->growTowards(s->rrt, target_rrt, *copy);
    if(found) {
      node0 = s->success_growing;
      node1 = s->success_passive;
      break;
    }

    found = s->growTowards(target_rrt, s->rrt, *copy);
    if(found) {
      node0 = s->success_passive;
      node1 = s->success_growing;
      break;
    }
  }
  delete copy;

  arr q0 = buildTrajectory(s->rrt, node0, true);
  arr q1 = buildTrajectory(target_rrt, node1, false);

  // add trajectories
  q.append(q0);
  q.append(q1);
  q.reshape(q0.d0 + q1.d0, q0.d1);

  return q;
}

// don't delete. might be handy:
//void ors::RRTPlanner::plotRRT(OpenGL* gl) {
  //gl->add(glDrawPlot, &plotModule);

  //for(uint i=1; i < s->rrt.getNumberNodes(); ++i) {
    //arr line;
    //line.append(s->rrt.getNode(i)); line.reshape(1, line.N);
    //line.append(s->rrt.getNode(s->rrt.getParent(i)));
    //plotLine(line);
  //}
//}


