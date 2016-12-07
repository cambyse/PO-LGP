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

  komo.setTiming(3, 40, 10., 2, false);
  komo.setSquaredFixJointVelocities(-1., -1., 1e2);
  komo.setSquaredFixSwitchVelocities(-1., -1., 1e2);
  komo.setSquaredQAccelerations();

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", sumOfSqrTT, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", sumOfSqrTT, ARR(.0), 1e3);

  komo.setKS_placeOn(1., true, "obj1", "table", true);

  komo.setPosition(2.8, 3., "obj1", "table", sumOfSqrTT, ARR(.4, .0, .1), 1e3);
  komo.setTask(1., 3., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), sumOfSqrTT, ARR(-.1,0,0), 1e2);

  komo.setTask(1., 3., new TaskMap_PushConsistent(W, "obj1", "endeff"), eqTT, ARR(0,0,0), 1e3);

//      WatchWorkspace(1., 2., "obj1");
//  komo.setGrasp(1., "humanR", "blue");
//  //  komo.setPlace(1.8, "humanR", "blue", "tableC");
//  komo.setPlace(1.8, "humanR", "blue", "red");

//  komo.setGrasp(1.3, "humanL", "yellow");
//  //  komo.setPlace(2.1, "humanL", "yellow", "tableC");
//  komo.setPlace(2.1, "humanL", "yellow", "blue");


  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);
  komo.MP->costReport(true);

  while(komo.displayTrajectory(0, true));
}

//===========================================================================


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  testUsingSpecs();

//  testUsingKomo();

  return 0;
}
