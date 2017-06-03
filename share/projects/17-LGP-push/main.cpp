#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>
#include <KOMO/komo.h>

#include <Kin/kin_swift.h>

#include <LGP/optLGP.h>

//===========================================================================

void testToolSlide(){
  mlr::KinematicWorld W("kin.g");

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

struct PushDemo{
  mlr::KinematicWorld kin;
  FOL_World fol;
  PushDemo(){
    kin.init("kin.g");
    fol.init("fol.g");

    //-- prepare logic world
    fol.addObject("obj1");
    fol.addObject("stick");
    fol.addFact({"table","table1"});
//    fol.addAgent("pr2L");
    fol.addAgent("baxterL");
    fol.addAgent("baxterR");
//    fol.addAgent("handL");
//    fol.addAgent("handR");

    FILE("z.kin.start") <<kin;
    FILE("z.fol.start") <<kin;
  }
};

void testLGP_player(){
  PushDemo demo;

  OptLGP opt(demo.kin, demo.fol);

  opt.player({"p","3","p","2","p","0","p","1","x"});

//  opt.run(1, true);
  mlr::wait(.1);
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testToolSlide();

  testLGP_player();

  return 0;
}
