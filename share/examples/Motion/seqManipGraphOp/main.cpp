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
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e2);
  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities(-1., -1., 1e-1);

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", OT_sumOfSqr, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", OT_sumOfSqr, ARR(.0), 1e3);

  komo.setKS_placeOn(2., true, "obj1", "table", true);

  komo.setPosition(3.8, 4., "obj1", "table", OT_sumOfSqr, ARR(.4, .0, .1), 1e1);

  komo.setKS_placeOn(4., true, "obj1", "table", false);

  //velocities
  komo.setTask(2.-.15, 2., new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, ARR(.1,0,0), 1e2, 1);
  komo.setTask(4., 4.+.15, new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, ARR(-.1,0,0), 1e2, 1);

  //keep distance
//  komo.setTask(1.5, 4., new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector),
//                                            true),          OT_sumOfSqr, ARR(.2), 1e3);
//  komo.setTask(2., 4., new TaskMap_GJK(W, "endeff", "obj1", true, true), OT_eq, ARR(-.15), 1e2, 0);
  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), OT_sumOfSqr, ARR(-.1,0,0), 1e2);
  //push align
//  komo.setTask(2., 4., new TaskMap_PushConsistent(W, "obj1", "endeff"), OT_sumOfSqr, ARR(0,0,0), 1e3);

  //no collisions
  komo.setTask(0., 1.9, new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e3);
//  komo.setTask(4.5, -1., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);


  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);
  komo.MP->costReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

void testSlide(){
  mlr::KinematicWorld W("model_slide.g");

  KOMO komo;
  komo.setModel(W);

  komo.setTiming(2., 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e2);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e2);
  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities(-1., -1., 1e-1);

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", OT_sumOfSqr, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", OT_sumOfSqr, ARR(.0), 1e3);

#if 0
  komo.setKS_placeOn(2., true, "obj1", "table", true);
//#else
  komo.setKS_slider(1., true, "obj1", "slider", "table", true);
#endif

//  komo.setHoldStill(1., 4., "slider1", 1e2);
//  komo.setHoldStill(1., 4., "slider2", 1e4);
//  komo.setHoldStill(-1., 1., "obj1", 1e4);
//  komo.setKS_slider(1., true, "obj1", "slider2", "table", true);

  komo.setKinematicSwitch(1., true, "delete", "table", "slider1");
  komo.setKinematicSwitch(1., true, "transXYPhiZero", "table", "slider1");

  komo.setKinematicSwitch(1., true, "delete", "table", "obj1");
  mlr::Transformation rel = 0;
  rel.addRelativeTranslation( 0., 0., .12);
  komo.setKinematicSwitch(1., true, "hingeZZero", "slider2", "obj1", rel );
//  komo.setKinematicSwitch(1., true, "sliderMechanism", "table", "obj1", rel );


  komo.setPosition(1.8, 2., "obj1", "table", OT_sumOfSqr, ARR(.4, -.2, .12), 1e2);

//  komo.setKS_placeOn(4., true, "obj1", "table", false);

//  //velocities
//  komo.setTask(2.-.15, 2., new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, ARR(.1,0,0), 1e2, 1);
//  komo.setTask(4., 4.+.15, new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, ARR(-.1,0,0), 1e2, 1);

//  //keep distance
////  komo.setTask(1.5, 4., new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector),
////                                            true),          OT_sumOfSqr, ARR(.2), 1e3);
////  komo.setTask(2., 4., new TaskMap_GJK(W, "endeff", "obj1", true, true), OT_eq, ARR(-.15), 1e2, 0);
//  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), OT_sumOfSqr, ARR(-.1,0,0), 1e2);
//  //push align
////  komo.setTask(2., 4., new TaskMap_PushConsistent(W, "obj1", "endeff"), OT_sumOfSqr, ARR(0,0,0), 1e3);

//  //no collisions
//  komo.setTask(0., 1.9, new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e3);
////  komo.setTask(4.5, -1., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);

  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);

  ofstream fil("z.x");
  for(mlr::KinematicWorld* c:komo.MP->configurations) fil <<c->q <<endl;
  gnuplot("plot 'z.x' us 1,'' us 2,'' us 3,'' us 4,'' us 5,'' us 6");

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testUsingSpecs();

//  testUsingKomo();

  testSlide();

  return 0;
}
