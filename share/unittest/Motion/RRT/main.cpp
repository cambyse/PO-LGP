#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/taskMap_proxy.h>
#include <Ors/ors.h>
#include <gtest/gtest.h>
#include <Gui/opengl.h>
#include <ctime>

TEST(AlgosTest, testRRT) {
  rnd.seed(time(NULL));
  double eps = .001; 
  double stepsize = .1;
  ors::Graph G("world_complex.ors");

  // create MotionProblem
  MotionProblem P(&G);
  P.T = 1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = ARRAY<uint>(P.ors->getBodyByName("endeff")->shapes(0)->index);
  TaskCost *c = P.addCustomTaskMap("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  c->y_threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);

  std::cout << "Planner initialized" <<std::endl;
  arr q = G.getJointState();
  std::cout << "q = " << q << std::endl;

  arr target = ARRAY(G.getBodyByName("target")->X.pos);
  std::cout << "target = " << target << std::endl;

  double end_prec = 1e-1;

  arr traj = planner.getTrajectoryTo(target, end_prec);

  std::cout << "Trajectory:" << std::endl;
  for(uint i=0; i<traj.d0; ++i)
    cout << i << " : " << traj[i] << std::endl;

  OpenGL gl;
  bindOrsToOpenGL(G, gl);
  MT::wait();
  displayTrajectory(traj, traj.d0, G, gl, "RRT");

  // check for correct start point
  EXPECT_TRUE( norm(traj[0] - q) <= end_prec);

  // check for goal reaching
  EXPECT_TRUE( norm(traj[traj.d0-1] - target) <= end_prec);

  for(uint i=0; i<traj.d0; ++i) {
    // check for small enough steps
    if(i>0) {
      EXPECT_LE(norm(traj[i] - traj[i-1]), stepsize + eps) << "i = " << i;  
    }

    // check for no collisions
    G.setJointState(traj[i]);
    arr penetration;
    G.getPenetrationState(penetration);
    EXPECT_LE(sum(penetration), 0);
  }
}


GTEST_API_ int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
