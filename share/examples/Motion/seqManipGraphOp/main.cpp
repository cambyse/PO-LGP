#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>
#include <KOMO/komo.h>


#include <Kin/kin_swift.h>

//===========================================================================

void TEST(UsingSpecs){
  Graph specs("specsPush.g");
  KOMO komo(specs);
  komo.reset();
//  komo.MP->reportFeatures(true);
  komo.run();
  cout <<komo.getReport(true) <<endl;
  for(;;)
    komo.displayTrajectory();
}

//===========================================================================

void testNonSliderSlide(){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);

  komo.setTiming(5., 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);
  komo.setSquaredQAccelerations();
//  komo.setSquaredQVelocities(-1., -1., 1e-1);

//  komo.setPosition(.5, 2.5, "obj1", "endeffWorkspace", OT_sumOfSqr, NoArr, 1e-1);
//  komo.setTouch(1., 3., "endeff", "obj1", OT_sumOfSqr, {.0}, 1e3);

  komo.setKS_placeOn(2., true, "obj1", "table", true);
//  komo.setKS_slider(2., true, "obj1", "table", true);

  komo.setPosition(3.8, 5., "obj1", "target", OT_sumOfSqr, {}, 1e1);

  komo.setKS_placeOn(4., true, "obj1", "table", false);

  //velocities
//  komo.setTask(2.-.15, 2., new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, {.1,0,0}, 1e2, 1);
//  komo.setTask(4., 4.+.15, new TaskMap_Default(posDiffTMT, W, "endeff"), OT_sumOfSqr, {-.1,0,0}, 1e2, 1);

  //keep distance
//  komo.setTask(1.5, 4., new TaskMap_LinTrans(new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector),
//                                            true),          OT_sumOfSqr, {.2}, 1e3);
//  komo.setTask(2., 4., new TaskMap_GJK(W, "endeff", "obj1", true, true), OT_eq, {-.15}, 1e2, 0);
  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "endeff", NoVector, "obj1", NoVector), OT_sumOfSqr, {-.1,0,0}, 1e2);
  //push align
//  komo.setTask(2., 4., new TaskMap_PushConsistent(W, "obj1", "endeff"), OT_sumOfSqr, {0,0,0}, 1e3);

  //no collisions
  komo.setTask(0., 1.9, new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);
//  komo.setTask(4.5, -1., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e2);


  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

void testToolSlide(){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);
  komo.useJointGroups({"armL", "base"}, false);

  komo.setTiming(5., 20, 5., 2, true);
  komo.setSquaredFixJointVelocities(-1., -1., 1e3);
  komo.setSquaredFixSwitchedObjects(-1., -1., 1e3);
  komo.setSquaredQAccelerations();

  komo.deactivateCollisions("coll_hand_r", "stick");
  komo.activateCollisions("obj1", "stick");

  komo.setGrasp(1., "pr2R", "stick_handle", 0);

//  komo.setSlowAround(1., .1);
  komo.setKS_slider(2., true, "obj1", "slider1", "table");

  komo.setPosition(3.8, 5., "obj1", "target", OT_sumOfSqr, {}, 1e2);

  komo.setKS_placeOn(4., true, "obj1", "table", false);

  komo.setTask(1.7, 1.7, new TaskMap_Default(posDiffTMT, W, "stick_eff", NoVector, "obj1"), OT_sumOfSqr, {0,0,.2}, 1e2);

  komo.setTask(2., 4., new TaskMap_Default(vecAlignTMT, W, "stick_eff", -Vector_y, "slider1b", Vector_x), OT_sumOfSqr, {1.}, 1e2);
  komo.setTask(2., 4., new TaskMap_Default(vecAlignTMT, W, "stick_eff", Vector_z, NULL, Vector_z), OT_sumOfSqr, {1.}, 1e2);
  komo.setTask(2., 4., new TaskMap_Default(posDiffTMT, W, "stick_eff", NoVector, "slider1b", {.12, .0, .0}), OT_sumOfSqr, {}, 1e2);

  komo.setTask(0., 5., new TaskMap_Proxy(allPTMT, uintA(), .03), OT_sumOfSqr, NoArr, 1e5);

  komo.reset();
  komo.run();

  cout <<komo.getReport(true);
//  komo.reportProxies();

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

  komo.setKS_slider(1., true, "obj1", "slider1", "table");

  komo.setPosition(1.8, 2., "obj1", "table", OT_sumOfSqr, {.4, -.2, .12}, 1e2);

  komo.reset();
  komo.run();
//      komo.checkGradients();

  cout <<komo.getReport(true);

//  ofstream fil("z.x");
//  for(mlr::KinematicWorld* c:komo.MP->configurations) fil <<c->q <<endl;
//  gnuplot("plot 'z.x' us 1,'' us 2,'' us 3,'' us 4,'' us 5,'' us 6");

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testUsingSpecs();

//  testUsingKomo();
  testToolSlide();

//  testSlide();

  return 0;
}
