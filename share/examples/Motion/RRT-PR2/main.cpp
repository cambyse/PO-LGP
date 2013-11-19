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

  TaskCost *c = P.addTaskMap("position", new DefaultTaskMap(posTMT, G, "tip1", ors::Vector(0, 0, .0)));
  P.setInterpolatingCosts(c, MotionProblem::finalOnly, ARRAY(P.ors->getBodyByName("target")->X.pos), 1e2);
  P.setInterpolatingVelCosts(c, MotionProblem::finalOnly, ARRAY(0.,0.,0.), 1e1);

  //uintA shapes = ARRAY<uint>(P.ors->getBodyByName("l_gripper_l_finger_tip_link")->shapes(0)->index);
  //c = P.addTaskMap("proxyColls",
                   //new ProxyTaskMap(allVersusListedPTMT, shapes, .2, true));
  //P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e1);

  arr x = P.x0;
  keyframeOptimizer(x, P, false, 2);

  return x;
}

int main(int argc, char** argv) {
  MT::initCmdLine(argc,argv);
  int seed = time(NULL);

  arr trajectory;  
  arr start;
  arr target;

  ors::Graph G;

  double stepsize; // RRT stepsize
  double eps;      // eps environment size

  rnd.seed(seed);
  stepsize = .005;
  eps = .001;
  G.init("world.ors");
  makeConvexHulls(G.shapes);

  // create MotionProblem
  MotionProblem P(&G);
  P.loadTransitionParameters();

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = ARRAY<uint>(P.ors->getBodyByName("l_gripper_l_finger_tip_link")->shapes(0)->index);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);
  arr q = { 0.1, 0.999998, 0.500003, 0.999998, 1.5, -2, 0, 0.500003, 0, 0 };
  planner.joint_max = q + ones(q.N, 1);
  planner.joint_min = q - ones(q.N, 1);

  std::cout << "Planner initialized" <<std::endl;
  start = G.getJointState();

  target = create_endpose(G);
  std::cout << "q = " << start << std::endl;
  std::cout << "target = " << target << std::endl;

  trajectory = planner.getTrajectoryTo(target);

  //std::cout << "Trajectory:" << std::endl;
  //for(uint i=0; i<trajectory.d0; ++i)
    //cout << i << " : " << trajectory[i] << std::endl;

  OpenGL gl;
  bindOrsToOpenGL(G, gl);
  displayTrajectory(trajectory, trajectory.d0, G, gl, "RRT");
  gl.watch();

  return 0;
}

