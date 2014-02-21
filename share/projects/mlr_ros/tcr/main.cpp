#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/pr2_heuristics.h>
#include <Motion/motionHeuristics.h>
#include <Motion/taskMap_proxy.h>
#include <Motion/taskMap_default.h>
#include <Core/array_t.h>
#include <Core/util_t.h>
#include <Ors/ors.h>
#include <Gui/opengl.h>
#include <ctime>

#include <devTools/logging.h>
SET_LOG(main, DEBUG);

arr create_endpose(ors::KinematicWorld& G, double col_prec, double pos_prec, arr& start) {
  DEBUG_VAR(main, G.qdot);
  MotionProblem P(G);

  P.loadTransitionParameters();
  P.H_rate_diag = MT::getParameter<arr>("Hratediag");

  cout << pr2_get_shapes(G) << endl;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = MT::getParameter<uintA>("agent_shapes");
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, col_prec);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.world.getBodyByName("target")->X.pos), pos_prec);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  keyframeOptimizer(start, P, true, 2);

  return start;
}

arr create_rrt_trajectory(ors::KinematicWorld& G, arr& target) {
  double stepsize = MT::getParameter<double>("rrt_stepsize", .005);

  // create MotionProblem
  MotionProblem P(G);
  P.loadTransitionParameters();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = MT::getParameter<uintA>("agent_shapes");
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);
  planner.joint_max = MT::getParameter<arr>("joint_max");
  planner.joint_min = MT::getParameter<arr>("joint_min");
  std::cout << "Planner initialized" <<std::endl;
  
  return planner.getTrajectoryTo(target);
}

arr optimize_trajectory(ors::KinematicWorld& G, const arr& init_trajectory) {
  // create MotionProblem
  MotionProblem P(G);
  P.loadTransitionParameters();
  P.H_rate_diag = MT::getParameter<arr>("Hratediag");
  P.T = init_trajectory.d0-1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = pr2_get_shapes(G);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e1);

  c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.world.getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e2);

  MotionProblemFunction MF(P);
  arr x = init_trajectory;
  optNewton(x, Convert(MF), OPT(verbose=2, stopIters=40, useAdaptiveDamping=false, damping=1e-0, maxStep=1.));
  DEBUG_VAR(main, x);
  return x;
}

void show_trajectory(ors::KinematicWorld& G, OpenGL& gl, arr& trajectory, const char* title) {
  arr start;
  G.getJointState(start);
  displayTrajectory(trajectory, trajectory.d0, G, title);
  gl.watch();
  G.setJointState(start);
}

int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  ros::init(argc, argv, "Controller");
  int seed = time(NULL);

  rnd.seed(seed);
  

  ors::KinematicWorld G(MT::getParameter<MT::String>("orsFile"));
  makeConvexHulls(G.shapes);


  OpenGL gl;
  bindOrsToOpenGL(G, gl);

  arr start = G.getJointState();

  std::cout << "q = " << start << std::endl;

  arr opt_start = start;
  opt_start(0) = G.getBodyByName("target")->X.pos.x;
  opt_start(1) = G.getBodyByName("target")->X.pos.y;
  //opt_start(2) = (rand()/(double) RAND_MAX) * 2 * M_PI - M_PI;

  DEBUG_VAR(main, G.qdot);
  arr target_t = create_endpose(G, 1e0, 1e3, opt_start);
  arr target = create_endpose(G, 1e3, 0, target_t);
  G.setJointState(start);

  std::cout << "target = " << target << std::endl;

  arr rrt_trajectory = create_rrt_trajectory(G, target);
  //show_trajectory(G, gl, rrt_trajectory, "RRT");

  arr opt_trajectory = optimize_trajectory(G, rrt_trajectory);
  show_trajectory(G, gl, opt_trajectory, "optimized");

  return 0;
}

