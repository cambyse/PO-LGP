#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/taskMap_proxy.h>
#include <Ors/ors.h>
#include <gtest/gtest.h>
#include <Gui/opengl.h>
#include <ctime>

int main() {
  int seed = time(NULL);

  arr trajectory;  
  arr start;
  arr target;

  ors::Graph G;

  double stepsize; // RRT stepsize
  double eps;      // eps environment size

  rnd.seed(seed);
  stepsize = .05;
  eps = .001;
  G.init("world.ors");
  makeConvexHulls(G.shapes);

  // create MotionProblem
  MotionProblem P(&G);

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = ARRAY<uint>(P.ors->getBodyByName("l_gripper_l_finger_tip_link")->shapes(0)->index);
  TaskCost *c = P.addTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, {0.}, 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);

  //planner.joint_max = { 6, 6, 0.};
  //planner.joint_min = { -6, -6, 2. };

  std::cout << "Planner initialized" <<std::endl;
  start = G.getJointState();
  std::cout << "q = " << start << std::endl;

  int q_dim = G.getJointStateDimension();
  target = rand(1, q_dim)*2.*M_PI-ones(1, q_dim)*M_PI;
  target.reshape(q_dim);
  std::cout << "target = " << target << std::endl;

  trajectory = planner.getTrajectoryTo(target, eps);

  //std::cout << "Trajectory:" << std::endl;
  //for(uint i=0; i<trajectory.d0; ++i)
    //cout << i << " : " << trajectory[i] << std::endl;

  OpenGL gl;
  bindOrsToOpenGL(G, gl);
  displayTrajectory(trajectory, trajectory.d0, G, gl, "RRT");
  gl.watch();

  return 0;
}

