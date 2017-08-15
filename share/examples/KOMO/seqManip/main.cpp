#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>
#include <KOMO/komo.h>


#include <Kin/kin_swift.h>

//===========================================================================

void testSlide(){
  mlr::KinematicWorld W("model_slide.g");

  KOMO komo;
  komo.setModel(W);
  komo.setPathOpt(2., 20, 5.);

  komo.setKS_slider(1., true, "obj1", "slider1", "table");
  komo.setPosition(1.8, 2., "obj1", "table", OT_sumOfSqr, {.4, -.2, .12}, 1e2);

  komo.reset();
  komo.reportProblem();

  komo.run();
  komo.checkGradients();

  komo.getReport(true);

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

void testToolSlide(){
  mlr::KinematicWorld W("model.g");

  KOMO komo;
  komo.setModel(W);
  komo.useJointGroups({"armL", "base"}, false);

  komo.setPathOpt(5., 20, 5.);

  komo.deactivateCollisions("coll_hand_r", "stick_coll");
  komo.deactivateCollisions("coll_hand_r", "table");
//  komo.activateCollisions("obj1", "stick");

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
  komo.reportProblem();
  komo.run();

  komo.getReport(true);
//  komo.reportProxies();

  while(komo.displayTrajectory(.1, true));
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testSlide();

  testToolSlide();

  return 0;
}
