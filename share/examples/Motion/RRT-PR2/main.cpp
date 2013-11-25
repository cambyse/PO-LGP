#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_default.h>
#include <Ors/ors.h>
#include <gtest/gtest.h>
#include <Gui/opengl.h>
#include <ctime>

#include <devTools/logging.h>
SET_LOG(main, DEBUG);

arr create_endpose(ors::Graph& G) {
  MotionProblem P(&G);

  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  arr x = P.x0;
  keyframeOptimizer(x, P, false, 2);

  return x;
}

arr create_rrt_trajectory(ors::Graph& G, arr& target) {
  double stepsize = MT::getParameter<double>("rrt_stepsize", .005);

  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);
  arr q = { 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003 };
  planner.joint_max = q + ones(q.N, 1);
  planner.joint_min = q - ones(q.N, 1);
  std::cout << "Planner initialized" <<std::endl;
  
  return planner.getTrajectoryTo(target);
}

arr optimize_trajectory(ors::Graph& G, arr& init_trajectory) {
  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();
  P.H_rate_diag = pr2_reasonable_W();
  P.T = init_trajectory.d0-1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e1);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  MotionProblemFunction MF(P);
  arr x = init_trajectory;
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=40, useAdaptiveDamping=false, damping=1e-0, maxStep=1.));
  return x;
}

void show_trajectory(ors::Graph& G, OpenGL& gl, arr& trajectory, const char* title) {
  arr start;
  G.getJointState(start);
  displayTrajectory(trajectory, trajectory.d0, G, gl, title);
  gl.watch();
  G.setJointState(start);
}

int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  int seed = time(NULL);

  rnd.seed(seed);

  ors::Graph G("world.ors");
  makeConvexHulls(G.shapes);

  OpenGL gl;
  bindOrsToOpenGL(G, gl);

  arr start = G.getJointState();
  std::cout << "q = " << start << std::endl;

  arr target = create_endpose(G);
  G.setJointState(start);

  std::cout << "target = " << target << std::endl;

  arr rrt_trajectory = create_rrt_trajectory(G, target);
  show_trajectory(G, gl, rrt_trajectory, "RRT");

  arr opt_trajectory = optimize_trajectory(G, rrt_trajectory);
  show_trajectory(G, gl, opt_trajectory, "optimized");

  return 0;
}

