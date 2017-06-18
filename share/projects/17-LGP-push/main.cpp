#include <Core/util.tpp>
#include <Gui/opengl.h>

#include <Kin/taskMaps.h>
#include <KOMO/komo.h>

#include <Kin/kin_swift.h>

#include <LGP/optLGP.h>
#include <Logic/fol_mcts_world.h> //TODO: rename to folWorld.h

#include <Roopi/roopi.h>

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

void roopiInterface(){
  Roopi R;

  auto lgp = R.newLGP();

  lgp->setKinematics("kin-stickHandover.g");
  lgp->setLogic("fol.g");
  FILE("z.fol") <<lgp->fol().KB <<endl;

  //-- prepare logic world
  lgp->fol().addObject("obj1");
  lgp->fol().addObject("stick");
  lgp->fol().addFact({"table","table1"});
  lgp->fol().addFact({"table","obj1"});
    //    fol.addAgent("pr2L");
  lgp->fol().addAgent("baxterL");
  lgp->fol().addAgent("baxterR");
  FILE("z.fol") <<lgp->fol().KB <<endl;
  lgp->fol().addAgent("stickTip");
//  lgp->fol().addAgent("obj1");
    //    fol.addAgent("handL");
    //    fol.addAgent("handR");

  lgp->fol().addTerminalRule({{"pushing", "obj1"}});

#if 1
  OptLGP opt(lgp->kin(), lgp->fol());
  opt.optFixedSequence("(grasp baxterR stick) (grasp stickTip obj1)");
#else
//  lgp->fixLogicSequence("(grasp baxterR stick) (handover baxterR stick baxterL) (grasp stickTip obj1)");
  lgp->fixLogicSequence("(grasp baxterR stick) (grasp stickTip obj1)");
//  lgp->fixLogicSequence("(grasp baxterR stick) (place baxterR stick table1) (grasp baxterL obj1) (grasp obj1 stick) ");
//
// (place stickTip obj1 table1) (grasp baxterR obj1) (place baxterR obj1 table1)");

//  lgp->fixLogicSequence("(grasp baxterR stick) (activate_pushing stick obj1 table1) (grasp baxterL obj1) (place baxterL obj1 table1) ");


  lgp->start();

  R.wait(+lgp);

#endif
  R.wait();
}

//===========================================================================

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

//  testToolSlide();

//  testLGP_player();

  roopiInterface();
  return 0;
}
