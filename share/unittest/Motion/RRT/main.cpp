#include <Motion/rrt_planner.h>
#include <Motion/motion.h>
#include <Motion/taskMap_proxy.h>
#include <Ors/ors.h>
#include <gtest/gtest.h>
#include <Gui/opengl.h>
#include <ctime>

int seed = time(NULL);

class RRTPlannerTest : public ::testing::Test {
  protected:
    arr trajectory;  
    arr start;
    arr target;

    ors::KinematicWorld G;

    double stepsize; // RRT stepsize
    double eps;      // eps environment size

    RRTPlannerTest();
};

RRTPlannerTest::RRTPlannerTest() {
  rnd.seed(seed);
  stepsize = .05;
  eps = .001;
  G.init("world_complex.ors");

  // create MotionProblem
  MotionProblem P(&G);
  P.T = 1;

  // add a collision cost with threshold 0 to avoid collisions
  uintA shapes = ARRAY<uint>(P.world.getBodyByName("endeff")->shapes(0)->index);
  TaskCost *c = P.addTask("proxyColls", new ProxyTaskMap(allVersusListedPTMT, shapes, .01, true));
  P.setInterpolatingCosts(c, MotionProblem::constant, ARRAY(0.), 1e-0);
  c->threshold = 0;

  ors::RRTPlanner planner(&G, P, stepsize);

  planner.joint_max = { 6, 6, 1.};
  planner.joint_min = { -6, -6, 1. };

  std::cout << "Planner initialized" <<std::endl;
  start = G.getJointState();
  std::cout << "q = " << start << std::endl;

  target = ARRAY(G.getBodyByName("target")->X.pos);
  std::cout << "target = " << target << std::endl;

  //OpenGL gl;
  //bindOrsToOpenGL(G, gl);
  trajectory = planner.getTrajectoryTo(target);

  //std::cout << "Trajectory:" << std::endl;
  //for(uint i=0; i<trajectory.d0; ++i)
    //cout << i << " : " << trajectory[i] << std::endl;

  //displayTrajectory(trajectory, trajectory.d0, G, gl, "RRT");
  //gl.watch();
}

GTEST_TEST_F(RRTPlannerTest, testRRTEndPoints) {

  // check for correct start point
  EXPECT_TRUE( length(trajectory[0] - start) <= eps);

  // check for goal reaching
  EXPECT_TRUE( length(trajectory[trajectory.d0-1] - target) <= eps);
}

GTEST_TEST_F(RRTPlannerTest, testRRTStepSize) {
  for(uint i=0; i<trajectory.d0; ++i) {
    // check for small enough steps
    if(i>0) {
      EXPECT_LE( length(trajectory[i] - trajectory[i-1]), stepsize + eps) << "i = " << i;  
    }
  }
}

GTEST_TEST_F(RRTPlannerTest, testRRTCollisionFree) {
  for(uint i=0; i<trajectory.d0; ++i) {
    // check for no collisions
    G.setJointState(trajectory[i]);
    arr penetration;
    G.getPenetrationState(penetration);
    EXPECT_LE(sum(penetration), 0);
  }
}


GTEST_API_ int main(int argc, char** argv) {
  rnd.seed(time(NULL));
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
