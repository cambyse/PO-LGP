#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Motion/motion.h>
//#include <Motion/motionHeuristics.h>
#include <Motion/taskMaps.h>

#include <Motion/komo.h>


#include <Ors/ors_swift.h>

//===========================================================================

void TEST(UsingSpecs){
  Graph specs("specsPush.g");
  KOMO komo(specs);
  komo.reset();
//  komo.MP->reportFeatures(true);
  komo.run();
  komo.MP->costReport(true);
  for(;;)
    komo.displayTrajectory();
}

//===========================================================================

void TEST(UsingKomo){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);

  komo.setTiming(5., 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e2);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e2);
  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities(-1., -1., 1e-1);

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", sumOfSqrTT, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", sumOfSqrTT, ARR(.0), 1e3);

  komo.setKS_placeOn(2., true, "obj1", "table", true);

  komo.setPosition(3.8, 4., "obj1", "table", sumOfSqrTT, ARR(.4, .0, .1), 1e1);

  komo.setKS_placeOn(4., true, "obj1", "table", false);

  //velocities
  komo.setTask(2.-.15, 2., new TaskMap_Default(posDiffTMT, W, "endeff"), sumOfSqrTT, ARR(.1,0,0), 1e2, 1);
  komo.setTask(4., 4.+.15, new TaskMap_Default(posDiffTMT, W, "endeff"), sumOfSqrTT, ARR(-.1,0,0), 1e2, 1);

  //keep distance
//  komo.setTask(1.5, 4., new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector),
//                                            true),          sumOfSqrTT, ARR(.2), 1e3);
//  komo.setTask(2., 4., new TaskMap_GJK(W, "endeff", "obj1", true, true), eqTT, ARR(-.15), 1e2, 0);
  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), sumOfSqrTT, ARR(-.1,0,0), 1e2);
  //push align
//  komo.setTask(2., 4., new TaskMap_PushConsistent(W, "obj1", "endeff"), sumOfSqrTT, ARR(0,0,0), 1e3);

  //no collisions
  komo.setTask(0., 1.9, new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e3);
//  komo.setTask(4.5, -1., new TaskMap_Proxy(allPTMT, uintA(), .03), sumOfSqrTT, NoArr, 1e2);


  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);
  komo.MP->costReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testUsingSpecs();

  testUsingKomo();

  return 0;
}
